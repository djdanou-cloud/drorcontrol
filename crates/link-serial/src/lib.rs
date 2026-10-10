//! Serial MAVLink link.
//!
//! Uses the `serialport` crate with one blocking reader thread and one blocking writer
//! thread per port. Non-standard baud rates on macOS go through `IOSSIOSPEED` (done by
//! `serialport`). [`discovery`] adds port listing, board hints, hotplug and auto-baud.

pub mod discovery;

use std::io::{self, Read, Write};
use std::time::{Duration, SystemTime};

use link_core::{LinkEvent, LinkHandle, LinkId};
use mavlink_codec::FrameParser;
use serialport::SerialPort;
use tokio::sync::mpsc;

/// Read timeout: how long a read blocks before the reader re-checks for shutdown.
pub const READ_TIMEOUT: Duration = Duration::from_millis(20);

/// Open `path` at `baud` (8N1, no flow control) and start the link.
pub async fn spawn_serial(
    id: LinkId,
    path: &str,
    baud: u32,
    events: mpsc::Sender<LinkEvent>,
) -> io::Result<LinkHandle> {
    let port = serialport::new(path, baud)
        .timeout(READ_TIMEOUT)
        .open()
        .map_err(io::Error::other)?;
    spawn_port(id, format!("serial {path} @ {baud}"), port, events).await
}

/// Start a link over an already-open port (also used by tests with pseudo-terminals).
pub async fn spawn_port(
    id: LinkId,
    name: String,
    mut port: Box<dyn SerialPort>,
    events: mpsc::Sender<LinkEvent>,
) -> io::Result<LinkHandle> {
    port.set_timeout(READ_TIMEOUT).map_err(io::Error::other)?;
    let mut writer = port.try_clone().map_err(io::Error::other)?;
    let (handle, mut outbound) = LinkHandle::new(id, name);
    let _ = events.send(LinkEvent::Added(handle.clone())).await;

    let rx_events = events.clone();
    std::thread::Builder::new()
        .name(format!("serial-rx-{}", id.0))
        .spawn(move || {
            let mut parser = FrameParser::new();
            let mut buf = [0u8; 1024];
            loop {
                match port.read(&mut buf) {
                    Ok(0) => {}
                    Ok(n) => {
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
                            || e.kind() == io::ErrorKind::Interrupted => {}
                    Err(e) => {
                        let _ = rx_events.blocking_send(LinkEvent::Status {
                            link: id,
                            up: false,
                            detail: e.to_string(),
                        });
                        let _ = rx_events.blocking_send(LinkEvent::Removed(id));
                        return;
                    }
                }
                if rx_events.is_closed() {
                    return;
                }
            }
        })?;

    std::thread::Builder::new()
        .name(format!("serial-tx-{}", id.0))
        .spawn(move || {
            while let Some(frame) = outbound.blocking_recv() {
                if let Err(e) = writer.write_all(frame.as_bytes()) {
                    let _ = events.blocking_send(LinkEvent::Status {
                        link: id,
                        up: false,
                        detail: e.to_string(),
                    });
                    return;
                }
            }
        })?;
    Ok(handle)
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
}
