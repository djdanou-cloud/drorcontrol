//! Serial MAVLink link.
//!
//! Uses the `serialport` crate with one blocking reader thread and one blocking writer
//! thread per port. Non-standard baud rates on macOS go through `IOSSIOSPEED` (done by
//! `serialport`). [`discovery`] adds port listing, board hints, hotplug and auto-baud.

pub mod discovery;

use std::io::{self, Read, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, SystemTime};

use link_core::{LinkEvent, LinkHandle, LinkId};
use mavlink_codec::{Frame, FrameParser};
use serialport::SerialPort;
use tokio::sync::mpsc;

/// Read timeout: how long a read blocks before the reader re-checks for shutdown.
pub const READ_TIMEOUT: Duration = Duration::from_millis(20);
/// First wait before reopening a port that failed (unplugged radio, reset autopilot).
pub const REOPEN_BACKOFF_MIN: Duration = Duration::from_millis(250);
/// Longest wait between reopen attempts. Kept short: a USB device re-enumerates in about a
/// second, and every second without a link is a second without control.
pub const REOPEN_BACKOFF_MAX: Duration = Duration::from_secs(2);
/// Consecutive empty reads (no data, no timeout) treated as a hang-up.
const EOF_READS: u32 = 50;

fn open(path: &str, baud: u32) -> io::Result<Box<dyn SerialPort>> {
    serialport::new(path, baud)
        .timeout(READ_TIMEOUT)
        .open()
        .map_err(io::Error::other)
}

/// Open `path` at `baud` (8N1, no flow control) and start the link.
///
/// The first open must succeed (a wrong path is an error). After that the link is kept:
/// when the device goes away (USB unplugged, autopilot reset) the link reports
/// `Status { up: false }`, reopens `path` with back-off, and reports `Status { up: true }`
/// when it is back. Frames queued while the port was closed are discarded, never sent late
/// (the command pipeline retransmits within each command's lifetime, ADR-0009).
pub async fn spawn_serial(
    id: LinkId,
    path: &str,
    baud: u32,
    events: mpsc::Sender<LinkEvent>,
) -> io::Result<LinkHandle> {
    let port = open(path, baud)?;
    let (handle, outbound) = LinkHandle::new(id, format!("serial {path} @ {baud}"));
    let _ = events.send(LinkEvent::Added(handle.clone())).await;
    let path = path.to_string();
    tokio::spawn(async move {
        supervise(id, port, Some((path, baud)), outbound, events).await;
    });
    Ok(handle)
}

/// Start a link over an already-open port (also used by tests with pseudo-terminals).
/// Without a path it cannot be reopened: on an I/O error the link is removed.
pub async fn spawn_port(
    id: LinkId,
    name: String,
    port: Box<dyn SerialPort>,
    events: mpsc::Sender<LinkEvent>,
) -> io::Result<LinkHandle> {
    let (handle, outbound) = LinkHandle::new(id, name);
    let _ = events.send(LinkEvent::Added(handle.clone())).await;
    tokio::spawn(async move {
        supervise(id, port, None, outbound, events).await;
    });
    Ok(handle)
}

/// How one open-port session ended.
enum SessionEnd {
    /// Every handle was dropped: the hub forgot the link.
    Shutdown,
    /// The port failed (unplugged, reset); the text says why.
    Lost(String),
}

async fn supervise(
    id: LinkId,
    first: Box<dyn SerialPort>,
    reopen: Option<(String, u32)>,
    mut outbound: mpsc::Receiver<Frame>,
    events: mpsc::Sender<LinkEvent>,
) {
    let mut port = Some(first);
    let mut backoff = REOPEN_BACKOFF_MIN;
    let mut last_detail = String::new();
    loop {
        if let Some(p) = port.take() {
            let detail = match run_session(id, p, &mut outbound, &events).await {
                Ok(SessionEnd::Shutdown) => break,
                Ok(SessionEnd::Lost(e)) | Err(e) => e,
            };
            let _ = events
                .send(LinkEvent::Status {
                    link: id,
                    up: false,
                    detail: detail.clone(),
                })
                .await;
            last_detail = detail;
            backoff = REOPEN_BACKOFF_MIN;
        }
        let Some((path, baud)) = &reopen else { break };

        // Wait before reopening; frames sent meanwhile are dropped (never sent late), and a
        // dropped handle ends the link promptly.
        let sleep = tokio::time::sleep(backoff);
        tokio::pin!(sleep);
        loop {
            tokio::select! {
                _ = &mut sleep => break,
                f = outbound.recv() => if f.is_none() {
                    let _ = events.send(LinkEvent::Removed(id)).await;
                    return;
                },
            }
        }
        let (p, b) = (path.clone(), *baud);
        match tokio::task::spawn_blocking(move || open(&p, b)).await {
            Ok(Ok(p)) => {
                while outbound.try_recv().is_ok() {}
                let _ = events
                    .send(LinkEvent::Status {
                        link: id,
                        up: true,
                        detail: format!("reopened {path}"),
                    })
                    .await;
                last_detail.clear();
                port = Some(p);
            }
            Ok(Err(e)) => {
                // Report a new reason once, not every attempt.
                let detail = format!("reopening {path}: {e}");
                if detail != last_detail {
                    let _ = events
                        .send(LinkEvent::Status {
                            link: id,
                            up: false,
                            detail: detail.clone(),
                        })
                        .await;
                    last_detail = detail;
                }
                backoff = (backoff * 2).min(REOPEN_BACKOFF_MAX);
            }
            Err(_) => break, // runtime shutting down
        }
    }
    let _ = events.send(LinkEvent::Removed(id)).await;
}

/// Run reader and writer threads over one open port until it fails or the link is dropped.
async fn run_session(
    id: LinkId,
    mut port: Box<dyn SerialPort>,
    outbound: &mut mpsc::Receiver<Frame>,
    events: &mpsc::Sender<LinkEvent>,
) -> Result<SessionEnd, String> {
    port.set_timeout(READ_TIMEOUT).map_err(|e| e.to_string())?;
    let mut writer = port.try_clone().map_err(|e| e.to_string())?;
    let stop = Arc::new(AtomicBool::new(false));
    let (end_tx, mut end_rx) = mpsc::unbounded_channel::<String>();

    let rx_events = events.clone();
    let rx_stop = stop.clone();
    let rx_end = end_tx.clone();
    std::thread::Builder::new()
        .name(format!("serial-rx-{}", id.0))
        .spawn(move || {
            let mut parser = FrameParser::new();
            let mut buf = [0u8; 1024];
            let mut empty = 0u32;
            while !rx_stop.load(Ordering::Relaxed) {
                match port.read(&mut buf) {
                    Ok(0) => {
                        empty += 1;
                        if empty >= EOF_READS {
                            let _ = rx_end.send("end of file (device hung up)".into());
                            return;
                        }
                    }
                    Ok(n) => {
                        empty = 0;
                        parser.push(&buf[..n]);
                        while let Some(frame) = parser.next_frame() {
                            let ev = LinkEvent::Frame {
                                link: id,
                                frame,
                                rx_time: SystemTime::now(),
                            };
                            if rx_events.blocking_send(ev).is_err() {
                                return;
                            }
                        }
                    }
                    Err(e)
                        if e.kind() == io::ErrorKind::TimedOut
                            || e.kind() == io::ErrorKind::Interrupted =>
                    {
                        empty = 0;
                    }
                    Err(e) => {
                        let _ = rx_end.send(e.to_string());
                        return;
                    }
                }
            }
        })
        .map_err(|e| e.to_string())?;

    // The writer thread ends when `to_writer` is dropped (end of this session).
    let (to_writer, from_session) = std::sync::mpsc::channel::<Frame>();
    std::thread::Builder::new()
        .name(format!("serial-tx-{}", id.0))
        .spawn(move || {
            for frame in from_session {
                if let Err(e) = writer.write_all(frame.as_bytes()) {
                    let _ = end_tx.send(e.to_string());
                    return;
                }
            }
        })
        .map_err(|e| e.to_string())?;

    let end = loop {
        tokio::select! {
            f = outbound.recv() => match f {
                Some(f) => { let _ = to_writer.send(f); }
                None => break SessionEnd::Shutdown,
            },
            Some(e) = end_rx.recv() => break SessionEnd::Lost(e),
        }
    };
    stop.store(true, Ordering::Relaxed);
    Ok(end)
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use mavlink_codec::{Header, Message, Version, encode, messages::Heartbeat};
    use serialport::TTYPort;

    #[tokio::test(flavor = "multi_thread")]
    async fn moves_frames_both_ways_over_a_pty() {
        let (mut far, near) = TTYPort::pair().expect("pty pair");
        far.set_timeout(Duration::from_secs(2)).unwrap();
        let (ev_tx, mut ev_rx) = mpsc::channel(16);
        let h = spawn_port(LinkId(5), "pty".into(), Box::new(near), ev_tx)
            .await
            .unwrap();
        assert!(matches!(ev_rx.recv().await, Some(LinkEvent::Added(_))));

        let f = encode(
            Version::V2,
            Header {
                seq: 3,
                sysid: 1,
                compid: 1,
            },
            &Message::Heartbeat(Heartbeat::default()),
        )
        .unwrap();
        far.write_all(b"\x00\xff").unwrap();
        far.write_all(f.as_bytes()).unwrap();
        match tokio::time::timeout(Duration::from_secs(5), ev_rx.recv())
            .await
            .unwrap()
        {
            Some(LinkEvent::Frame { link, frame, .. }) => {
                assert_eq!((link, frame), (LinkId(5), f.clone()))
            }
            other => panic!("unexpected {other:?}"),
        }

        assert!(h.send(f.clone()));
        let mut got = vec![0u8; f.as_bytes().len()];
        tokio::task::spawn_blocking(move || far.read_exact(&mut got).map(|_| got))
            .await
            .unwrap()
            .map(|got| assert_eq!(got, f.as_bytes()))
            .unwrap();
    }

    fn hb(seq: u8) -> mavlink_codec::Frame {
        encode(
            Version::V2,
            Header {
                seq,
                sysid: 1,
                compid: 1,
            },
            &Message::Heartbeat(Heartbeat::default()),
        )
        .unwrap()
    }

    /// A pseudo-terminal whose slave is reachable at `link` (a symlink standing in for the
    /// device path, e.g. `/dev/cu.usbmodem1101`). Returns the master side.
    fn plug(link: &std::path::Path) -> TTYPort {
        let (mut far, near) = TTYPort::pair().expect("pty pair");
        far.set_timeout(Duration::from_secs(2)).unwrap();
        let slave = near.name().expect("pty name");
        drop(near); // the link opens it by path
        let _ = std::fs::remove_file(link);
        std::os::unix::fs::symlink(slave, link).unwrap();
        far
    }

    async fn next(ev: &mut mpsc::Receiver<LinkEvent>) -> LinkEvent {
        tokio::time::timeout(Duration::from_secs(5), ev.recv())
            .await
            .expect("link event within 5 s")
            .expect("events open")
    }

    /// Unplugging the device must not end the link: it is reopened when the device returns,
    /// and frames queued during the outage are not sent late.
    #[tokio::test(flavor = "multi_thread")]
    async fn reopens_after_unplug_and_drops_stale_frames() {
        let dir = std::env::temp_dir().join(format!("link-serial-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let dev = dir.join("radio");
        let mut far = plug(&dev);
        let (ev_tx, mut ev_rx) = mpsc::channel(64);
        let h = spawn_serial(LinkId(7), dev.to_str().unwrap(), 115_200, ev_tx)
            .await
            .unwrap();
        assert!(matches!(next(&mut ev_rx).await, LinkEvent::Added(_)));
        far.write_all(hb(1).as_bytes()).unwrap();
        assert!(matches!(next(&mut ev_rx).await, LinkEvent::Frame { frame, .. } if frame == hb(1)));

        // Unplug: the link reports down but stays.
        drop(far);
        let _ = std::fs::remove_file(&dev);
        match next(&mut ev_rx).await {
            LinkEvent::Status {
                link, up: false, ..
            } => assert_eq!(link, LinkId(7)),
            other => panic!("expected link down, got {other:?}"),
        }
        assert!(!h.is_closed());
        // Queued during the outage: must never reach the vehicle.
        h.send(hb(2));
        tokio::time::sleep(Duration::from_millis(600)).await;
        h.send(hb(3));

        // Replug (a new device node at the same path): reopened, frames flow both ways.
        let mut far = plug(&dev);
        loop {
            match next(&mut ev_rx).await {
                LinkEvent::Status { up: true, .. } => break,
                LinkEvent::Status { up: false, .. } => {} // a failed attempt before the replug
                other => panic!("expected link up, got {other:?}"),
            }
        }
        far.write_all(hb(4).as_bytes()).unwrap();
        assert!(
            matches!(next(&mut ev_rx).await, LinkEvent::Frame { link: LinkId(7), frame, .. } if frame == hb(4))
        );
        assert!(h.send(hb(5)));
        let want = hb(5);
        let mut got = vec![0u8; want.as_bytes().len()];
        let got = tokio::task::spawn_blocking(move || far.read_exact(&mut got).map(|_| got))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(
            got,
            want.as_bytes(),
            "first bytes after replug are the fresh frame"
        );

        // Dropping the handle ends the link.
        drop(h);
        loop {
            if let LinkEvent::Removed(id) = next(&mut ev_rx).await {
                assert_eq!(id, LinkId(7));
                break;
            }
        }
        let _ = std::fs::remove_dir_all(&dir);
    }
}
