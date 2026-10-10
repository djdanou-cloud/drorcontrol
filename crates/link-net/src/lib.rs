//! UDP and TCP MAVLink links.
//!
//! | Kind | Typical use | Behaviour |
//! |------|-------------|-----------|
//! | [`spawn_udp`] `Server` | GCS port 14550 | Learns peers from what it receives; sends to every peer heard in the last [`PEER_TTL`] |
//! | [`spawn_udp`] `Client` | Companion computer, SITL `--out` | Sends to a fixed address; accepts replies |
//! | [`spawn_tcp_client`] | SITL (`tcp:5760`), radio bridges | Reconnects with back-off; discards queued frames on reconnect |
//! | [`spawn_tcp_server`] | Third-party tools connecting to us | Each accepted client is its own link |
//!
//! Every link registers itself with the hub by sending [`LinkEvent::Added`].

use std::collections::HashMap;
use std::io;
use std::net::SocketAddr;
use std::time::{Duration, Instant};

use link_core::{LinkEvent, LinkHandle, LinkId, LinkIds, PumpEnd, forward_frames, pump_stream};
use mavlink_codec::FrameParser;
use tokio::net::{TcpListener, TcpStream, UdpSocket};
use tokio::sync::mpsc;

/// UDP peers silent for longer than this stop receiving our frames.
pub const PEER_TTL: Duration = Duration::from_secs(10);
/// Longest wait between TCP reconnect attempts.
pub const MAX_BACKOFF: Duration = Duration::from_secs(5);

/// UDP link mode.
#[derive(Debug, Clone, Copy)]
pub enum UdpMode {
    /// Bind `bind` and talk to whoever sends to us.
    Server {
        /// Local address, e.g. `0.0.0.0:14550`.
        bind: SocketAddr,
    },
    /// Send to `remote` from an ephemeral port.
    Client {
        /// Remote address.
        remote: SocketAddr,
    },
}

/// Start a UDP link. Returns its handle and the local address actually bound.
pub async fn spawn_udp(
    id: LinkId,
    mode: UdpMode,
    events: mpsc::Sender<LinkEvent>,
) -> io::Result<(LinkHandle, SocketAddr)> {
    let (sock, fixed, name) = match mode {
        UdpMode::Server { bind } => (
            UdpSocket::bind(bind).await?,
            None,
            format!("udp-server {bind}"),
        ),
        UdpMode::Client { remote } => {
            let any: SocketAddr = if remote.is_ipv4() {
                "0.0.0.0:0"
            } else {
                "[::]:0"
            }
            .parse()
            .expect("literal");
            (
                UdpSocket::bind(any).await?,
                Some(remote),
                format!("udp-client {remote}"),
            )
        }
    };
    let local = sock.local_addr()?;
    let (handle, mut outbound) = LinkHandle::new(id, name);
    let _ = events.send(LinkEvent::Added(handle.clone())).await;

    tokio::spawn(async move {
        let mut peers: HashMap<SocketAddr, (FrameParser, Instant)> = HashMap::new();
        let mut buf = vec![0u8; 65_536];
        loop {
            tokio::select! {
                r = sock.recv_from(&mut buf) => match r {
                    Ok((n, from)) => {
                        let (parser, seen) = peers.entry(from).or_insert_with(|| (FrameParser::new(), Instant::now()));
                        *seen = Instant::now();
                        if !forward_frames(id, parser, &buf[..n], &events).await {
                            break;
                        }
                    }
                    // ICMP port-unreachable surfaces as an error on some platforms; keep going.
                    Err(e) if e.kind() == io::ErrorKind::ConnectionReset => {}
                    Err(e) => {
                        let _ = events.send(LinkEvent::Status { link: id, up: false, detail: e.to_string() }).await;
                        break;
                    }
                },
                f = outbound.recv() => {
                    let Some(frame) = f else { break };
                    match fixed {
                        Some(remote) => {
                            let _ = sock.send_to(frame.as_bytes(), remote).await;
                        }
                        None => {
                            peers.retain(|_, (_, seen)| seen.elapsed() <= PEER_TTL);
                            for addr in peers.keys() {
                                let _ = sock.send_to(frame.as_bytes(), addr).await;
                            }
                        }
                    }
                }
            }
        }
        let _ = events.send(LinkEvent::Removed(id)).await;
    });
    Ok((handle, local))
}

/// Start a TCP client link that keeps reconnecting to `remote` until the hub drops it.
///
/// Frames queued while disconnected are discarded on reconnect: a command typed during a
/// link outage must not fire minutes later.
pub async fn spawn_tcp_client(
    id: LinkId,
    remote: SocketAddr,
    events: mpsc::Sender<LinkEvent>,
) -> LinkHandle {
    let (handle, mut outbound) = LinkHandle::new(id, format!("tcp-client {remote}"));
    let _ = events.send(LinkEvent::Added(handle.clone())).await;
    tokio::spawn(async move {
        let mut backoff = Duration::from_millis(250);
        loop {
            match TcpStream::connect(remote).await {
                Ok(stream) => {
                    let _ = stream.set_nodelay(true);
                    backoff = Duration::from_millis(250);
                    while outbound.try_recv().is_ok() {}
                    let _ = events
                        .send(LinkEvent::Status {
                            link: id,
                            up: true,
                            detail: format!("connected to {remote}"),
                        })
                        .await;
                    let end = pump_stream(id, stream, &mut outbound, &events).await;
                    let detail = match &end {
                        PumpEnd::Eof => "peer closed".to_string(),
                        PumpEnd::Error(e) => e.to_string(),
                        PumpEnd::Shutdown => break,
                    };
                    let _ = events
                        .send(LinkEvent::Status {
                            link: id,
                            up: false,
                            detail,
                        })
                        .await;
                }
                Err(e) => {
                    let _ = events
                        .send(LinkEvent::Status {
                            link: id,
                            up: false,
                            detail: e.to_string(),
                        })
                        .await;
                }
            }
            // Wait before retrying, but notice shutdown (handle dropped) promptly.
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
            backoff = (backoff * 2).min(MAX_BACKOFF);
        }
        let _ = events.send(LinkEvent::Removed(id)).await;
    });
    handle
}

/// Listen on `bind`; every accepted connection becomes a new link (ids from `ids`).
/// Returns the bound address.
pub async fn spawn_tcp_server(
    ids: LinkIds,
    bind: SocketAddr,
    events: mpsc::Sender<LinkEvent>,
) -> io::Result<SocketAddr> {
    let listener = TcpListener::bind(bind).await?;
    let local = listener.local_addr()?;
    tokio::spawn(async move {
        while let Ok((stream, peer)) = listener.accept().await {
            let _ = stream.set_nodelay(true);
            let id = ids.next();
            let (handle, mut outbound) =
                LinkHandle::new(id, format!("tcp-server {local} <- {peer}"));
            if events.send(LinkEvent::Added(handle)).await.is_err() {
                break;
            }
            let events = events.clone();
            tokio::spawn(async move {
                pump_stream(id, stream, &mut outbound, &events).await;
                let _ = events.send(LinkEvent::Removed(id)).await;
            });
        }
    });
    Ok(local)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::{Frame, Header, Message, Version, encode, messages::Heartbeat};
    use tokio::io::AsyncReadExt;
    use tokio::time::timeout;

    const T: Duration = Duration::from_secs(5);

    fn hb(seq: u8) -> Frame {
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

    async fn status(ev: &mut mpsc::Receiver<LinkEvent>, want_up: bool) {
        loop {
            match timeout(T, ev.recv()).await.unwrap().unwrap() {
                LinkEvent::Status { up, .. } if up == want_up => return,
                _ => continue,
            }
        }
    }

    #[tokio::test]
    async fn tcp_client_reconnects_and_discards_stale_frames() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let (ev_tx, mut ev_rx) = mpsc::channel(64);
        let h = spawn_tcp_client(LinkId(1), addr, ev_tx).await;

        let (first, _) = listener.accept().await.unwrap();
        status(&mut ev_rx, true).await;
        drop(first); // peer goes away
        status(&mut ev_rx, false).await;

        // Queued while disconnected: must not be sent after reconnecting.
        assert!(h.send(hb(1)));
        let (mut second, _) = timeout(T, listener.accept()).await.unwrap().unwrap();
        status(&mut ev_rx, true).await;
        assert!(h.send(hb(2)));
        let mut got = vec![0u8; hb(2).as_bytes().len()];
        timeout(T, second.read_exact(&mut got))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(
            got,
            hb(2).as_bytes(),
            "first frame after reconnect is the fresh one"
        );
    }

    #[tokio::test]
    async fn udp_client_sends_to_remote_and_receives_replies() {
        let remote = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let (ev_tx, mut ev_rx) = mpsc::channel(64);
        let (h, _) = spawn_udp(
            LinkId(2),
            UdpMode::Client {
                remote: remote.local_addr().unwrap(),
            },
            ev_tx,
        )
        .await
        .unwrap();
        assert!(matches!(ev_rx.recv().await, Some(LinkEvent::Added(_))));

        assert!(h.send(hb(7)));
        let mut buf = [0u8; 300];
        let (n, from) = timeout(T, remote.recv_from(&mut buf))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(&buf[..n], hb(7).as_bytes());

        remote.send_to(hb(8).as_bytes(), from).await.unwrap();
        match timeout(T, ev_rx.recv()).await.unwrap().unwrap() {
            LinkEvent::Frame { link, frame, .. } => assert_eq!((link, frame), (LinkId(2), hb(8))),
            other => panic!("unexpected {other:?}"),
        }
    }
}
