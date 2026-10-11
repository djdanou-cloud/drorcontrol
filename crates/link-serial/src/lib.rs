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
/// `Status { up: false }`, reopens it with back-off, and reports `Status { up: true }` when
/// it is back. Frames queued while the port was closed are discarded, never sent late (the
/// command pipeline retransmits within each command's lifetime, ADR-0009).
///
/// A USB device can come back under another name (another USB port, or macOS renumbering).
/// If `path` does not return, the link follows the device by its USB identity — vendor,
/// product and serial number — but only when exactly one port carries it (see
/// [`resolve_path`]).
pub async fn spawn_serial(
    id: LinkId,
    path: &str,
    baud: u32,
    events: mpsc::Sender<LinkEvent>,
) -> io::Result<LinkHandle> {
    let port = open(path, baud)?;
    let original = path.to_string();
    // Remember who the device is while it is present.
    let key = discovery::list_ports()
        .ok()
        .and_then(|ports| ports.into_iter().find(|p| p.path == original))
        .and_then(|p| stable_key(&p));
    Ok(spawn_reopening(
        id,
        format!("serial {path} @ {baud}"),
        port,
        Box::new(move || {
            let target = match discovery::list_ports() {
                Ok(ports) => resolve_path(&original, key.as_deref(), &ports)
                    .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "device not present"))?,
                // Listing failed: try the original path.
                Err(_) => original.clone(),
            };
            open(&target, baud).map(|p| (p, target))
        }),
        events,
    )
    .await)
}

/// Identity that survives re-enumeration: only `vid:pid:serial` (devices with a serial
/// number). Without one, two identical adapters cannot be told apart, so there is none.
fn stable_key(port: &discovery::PortInfo) -> Option<String> {
    port.usb.as_ref()?.serial.as_ref()?;
    Some(port.device_key())
}

/// Where to reopen a device last seen at `original` with identity `key`, given the ports
/// present now. The original path wins if it is back. Otherwise the device is followed by
/// `key`, but only if exactly one present port carries it: cheap adapters sometimes share a
/// serial number, and attaching the vehicle link to the wrong radio is worse than waiting.
pub fn resolve_path(
    original: &str,
    key: Option<&str>,
    ports: &[discovery::PortInfo],
) -> Option<String> {
    if ports.iter().any(|p| p.path == original) {
        return Some(original.to_string());
    }
    let key = key?;
    let mut matches = ports
        .iter()
        .filter(|p| stable_key(p).as_deref() == Some(key));
    match (matches.next(), matches.next()) {
        (Some(p), None) => Some(p.path.clone()),
        _ => None,
    }
}

/// Opens the device again after a failure: the port and a description of what was opened
/// (its path), shown in the link status.
pub type Opener = Box<dyn FnMut() -> io::Result<(Box<dyn SerialPort>, String)> + Send>;

/// Start a link over `first` that calls `reopen` (with back-off) whenever the port fails,
/// as [`spawn_serial`] does with the port's path.
pub async fn spawn_reopening(
    id: LinkId,
    name: String,
    first: Box<dyn SerialPort>,
    reopen: Opener,
    events: mpsc::Sender<LinkEvent>,
) -> LinkHandle {
    let (handle, outbound) = LinkHandle::new(id, name.clone());
    let _ = events.send(LinkEvent::Added(handle.clone())).await;
    tokio::spawn(async move {
        supervise(id, name, first, Some(reopen), outbound, events).await;
    });
    handle
}

/// Start a link over an already-open port (also used by tests with pseudo-terminals).
/// Without a path it cannot be reopened: on an I/O error the link is removed.
pub async fn spawn_port(
    id: LinkId,
    name: String,
    port: Box<dyn SerialPort>,
    events: mpsc::Sender<LinkEvent>,
) -> io::Result<LinkHandle> {
    let (handle, outbound) = LinkHandle::new(id, name.clone());
    let _ = events.send(LinkEvent::Added(handle.clone())).await;
    tokio::spawn(async move {
        supervise(id, name, port, None, outbound, events).await;
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
    name: String,
    first: Box<dyn SerialPort>,
    mut reopen: Option<Opener>,
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
        let Some(mut opener) = reopen.take() else {
            break;
        };

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
        let attempt = tokio::task::spawn_blocking(move || {
            let r = opener();
            (opener, r)
        })
        .await;
        let Ok((opener, result)) = attempt else {
            break; // runtime shutting down
        };
        reopen = Some(opener);
        match result {
            Ok((p, opened)) => {
                while outbound.try_recv().is_ok() {}
                let _ = events
                    .send(LinkEvent::Status {
                        link: id,
                        up: true,
                        detail: format!("reopened {name}: {opened}"),
                    })
                    .await;
                last_detail.clear();
                port = Some(p);
            }
            Err(e) => {
                // Report a new reason once, not every attempt.
                let detail = format!("reopening {name}: {e}");
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

    /// A pseudo-terminal pair: the master is the "device" side the test drives; the slave is
    /// what the link opens. (On macOS a pty cannot be opened by path through `serialport`:
    /// setting the baud rate uses `IOSSIOSPEED`, which ptys reject.)
    fn device() -> (TTYPort, TTYPort) {
        let (mut far, near) = TTYPort::pair().expect("pty pair");
        far.set_timeout(Duration::from_secs(2)).unwrap();
        (far, near)
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
        // Plugged devices waiting to be opened; empty = unplugged.
        let (plug, plugged) = std::sync::mpsc::channel::<TTYPort>();
        let plugged = std::sync::Mutex::new(plugged);
        let opener: Opener = Box::new(move || match plugged.lock().unwrap().try_recv() {
            Ok(p) => Ok((Box::new(p) as Box<dyn SerialPort>, "pty".to_string())),
            Err(_) => Err(io::Error::new(io::ErrorKind::NotFound, "no such device")),
        });
        let (mut far, near) = device();
        let (ev_tx, mut ev_rx) = mpsc::channel(64);
        let h = spawn_reopening(LinkId(7), "radio".into(), Box::new(near), opener, ev_tx).await;
        assert!(matches!(next(&mut ev_rx).await, LinkEvent::Added(_)));
        far.write_all(hb(1).as_bytes()).unwrap();
        assert!(matches!(next(&mut ev_rx).await, LinkEvent::Frame { frame, .. } if frame == hb(1)));

        // Unplug: the link reports down but stays.
        drop(far);
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

        // Replug: reopened, frames flow both ways.
        let (mut far, near) = device();
        plug.send(near).unwrap();
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
    }

    fn port(path: &str, serial: Option<&str>) -> discovery::PortInfo {
        discovery::PortInfo {
            path: path.into(),
            usb: Some(discovery::UsbInfo {
                vid: 0x0403,
                pid: 0x6015,
                serial: serial.map(Into::into),
                manufacturer: None,
                product: None,
            }),
            hint: None,
        }
    }

    #[test]
    fn follows_a_device_that_moved_only_when_unambiguous() {
        let key = "0403:6015:RFD1";
        let radio_back = [port("/dev/cu.usbserial-1", Some("RFD1"))];
        let radio_moved = [port("/dev/cu.usbserial-7", Some("RFD1"))];
        // The original path is back: use it.
        assert_eq!(
            resolve_path("/dev/cu.usbserial-1", Some(key), &radio_back).as_deref(),
            Some("/dev/cu.usbserial-1")
        );
        // Same radio under another name: follow it.
        assert_eq!(
            resolve_path("/dev/cu.usbserial-1", Some(key), &radio_moved).as_deref(),
            Some("/dev/cu.usbserial-7")
        );
        // Not present yet: wait.
        assert_eq!(resolve_path("/dev/cu.usbserial-1", Some(key), &[]), None);
        // A different radio appeared: never take it.
        let other = [port("/dev/cu.usbserial-7", Some("RFD2"))];
        assert_eq!(resolve_path("/dev/cu.usbserial-1", Some(key), &other), None);
        // Two ports with the same identity (adapters sharing a serial): refuse to guess.
        let twins = [
            port("/dev/cu.usbserial-7", Some("RFD1")),
            port("/dev/cu.usbserial-8", Some("RFD1")),
        ];
        assert_eq!(resolve_path("/dev/cu.usbserial-1", Some(key), &twins), None);
        // No serial number when first seen: no identity, only the original path.
        assert_eq!(stable_key(&port("/dev/cu.usbserial-1", None)), None);
        let anon = [port("/dev/cu.usbserial-7", None)];
        assert_eq!(resolve_path("/dev/cu.usbserial-1", None, &anon), None);
    }
}
