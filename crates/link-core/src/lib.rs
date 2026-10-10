//! Types shared by every MAVLink link (serial, UDP, TCP, …) and the router hub.
//!
//! A link is a task (or thread pair) that turns bytes into [`Frame`]s for the hub and
//! writes frames the hub hands it. Links report everything through [`LinkEvent`]s on a
//! single channel; the hub sends to a link through its [`LinkHandle`].

use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::time::SystemTime;

use mavlink_codec::{Frame, FrameParser};
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};
use tokio::sync::mpsc;

/// Outbound queue depth per link. A full queue drops frames (counted) rather than
/// stalling the hub behind a slow radio.
pub const OUTBOUND_QUEUE: usize = 256;

/// Identifies one link (one TCP client, one UDP socket, one serial port, …).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct LinkId(pub u32);

impl fmt::Display for LinkId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "link#{}", self.0)
    }
}

/// Hands out unique link ids; clone it into every component that creates links.
#[derive(Debug, Clone, Default)]
pub struct LinkIds(Arc<AtomicU32>);

impl LinkIds {
    /// New allocator starting at 1.
    pub fn new() -> Self {
        LinkIds(Arc::new(AtomicU32::new(1)))
    }

    /// Next unused id.
    pub fn next(&self) -> LinkId {
        LinkId(self.0.fetch_add(1, Ordering::Relaxed))
    }
}

/// Everything a link reports to the hub.
// Frames are kept inline (~290 bytes) on purpose: boxing would cost a heap
// allocation per frame on the hot path.
#[allow(clippy::large_enum_variant)]
#[derive(Debug)]
pub enum LinkEvent {
    /// A new link exists (e.g. a TCP client connected); route frames to it.
    Added(LinkHandle),
    /// The link is gone for good; forget its routes.
    Removed(LinkId),
    /// Connection state change for a persistent link (e.g. TCP client reconnecting).
    Status {
        /// Which link.
        link: LinkId,
        /// True when the transport is usable.
        up: bool,
        /// Human-readable detail (peer address, error text).
        detail: String,
    },
    /// A valid frame arrived.
    Frame {
        /// Which link it arrived on.
        link: LinkId,
        /// The frame.
        frame: Frame,
        /// Host wall-clock time of receipt (for `.tlog`).
        rx_time: SystemTime,
    },
}

/// Counters a link keeps about its outbound side.
#[derive(Debug, Default)]
pub struct LinkCounters {
    /// Frames accepted for sending.
    pub queued: AtomicU64,
    /// Frames dropped because the outbound queue was full or the link was closed.
    pub dropped: AtomicU64,
}

/// The hub's way to send frames to one link.
#[derive(Debug, Clone)]
pub struct LinkHandle {
    /// Link id.
    pub id: LinkId,
    /// Short description, e.g. `udp-server 0.0.0.0:14550`.
    pub name: String,
    tx: mpsc::Sender<Frame>,
    counters: Arc<LinkCounters>,
}

impl LinkHandle {
    /// Create a handle and the receiver the link task drains.
    pub fn new(id: LinkId, name: impl Into<String>) -> (LinkHandle, mpsc::Receiver<Frame>) {
        let (tx, rx) = mpsc::channel(OUTBOUND_QUEUE);
        let h = LinkHandle {
            id,
            name: name.into(),
            tx,
            counters: Arc::default(),
        };
        (h, rx)
    }

    /// Queue a frame without waiting. Returns false (and counts a drop) if the queue is
    /// full or the link has shut down.
    pub fn send(&self, frame: Frame) -> bool {
        match self.tx.try_send(frame) {
            Ok(()) => {
                self.counters.queued.fetch_add(1, Ordering::Relaxed);
                true
            }
            Err(_) => {
                self.counters.dropped.fetch_add(1, Ordering::Relaxed);
                false
            }
        }
    }

    /// Outbound counters.
    pub fn counters(&self) -> &LinkCounters {
        &self.counters
    }

    /// True once the link task has dropped its receiver.
    pub fn is_closed(&self) -> bool {
        self.tx.is_closed()
    }
}

/// Feed `bytes` to `parser` and forward every complete frame as a [`LinkEvent::Frame`].
/// Returns false if the hub is gone.
pub async fn forward_frames(
    link: LinkId,
    parser: &mut FrameParser,
    bytes: &[u8],
    events: &mpsc::Sender<LinkEvent>,
) -> bool {
    parser.push(bytes);
    while let Some(frame) = parser.next_frame() {
        let ev = LinkEvent::Frame {
            link,
            frame,
            rx_time: SystemTime::now(),
        };
        if events.send(ev).await.is_err() {
            return false;
        }
    }
    true
}

/// Why [`pump_stream`] returned.
#[derive(Debug)]
pub enum PumpEnd {
    /// Peer closed the stream.
    Eof,
    /// I/O error.
    Error(std::io::Error),
    /// The hub dropped the event channel or the link's handle (shutdown).
    Shutdown,
}

/// Drive a byte stream (TCP socket, async serial port, …): parse incoming bytes into
/// frames for the hub and write outbound frames, until EOF, error or shutdown.
pub async fn pump_stream<S>(
    link: LinkId,
    stream: S,
    outbound: &mut mpsc::Receiver<Frame>,
    events: &mpsc::Sender<LinkEvent>,
) -> PumpEnd
where
    S: AsyncRead + AsyncWrite + Unpin,
{
    let (mut rd, mut wr) = tokio::io::split(stream);
    // Strict: frames of unknown ids cannot be checksum-verified, so noise could pass as
    // data. Pass-through for foreign dialects can be added per link when needed.
    let mut parser = FrameParser::new();
    let mut buf = vec![0u8; 4096];
    loop {
        tokio::select! {
            r = rd.read(&mut buf) => match r {
                Ok(0) => return PumpEnd::Eof,
                Ok(n) => {
                    if !forward_frames(link, &mut parser, &buf[..n], events).await {
                        return PumpEnd::Shutdown;
                    }
                }
                Err(e) => return PumpEnd::Error(e),
            },
            f = outbound.recv() => match f {
                Some(frame) => {
                    if let Err(e) = wr.write_all(frame.as_bytes()).await {
                        return PumpEnd::Error(e);
                    }
                }
                None => return PumpEnd::Shutdown,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::{Header, Message, Version, encode, messages::Heartbeat};

    fn hb() -> Frame {
        let m = Message::Heartbeat(Heartbeat {
            custom_mode: 9,
            ..Default::default()
        });
        encode(
            Version::V2,
            Header {
                seq: 1,
                sysid: 1,
                compid: 1,
            },
            &m,
        )
        .unwrap()
    }

    #[test]
    fn ids_are_unique() {
        let ids = LinkIds::new();
        let a = ids.clone().next();
        assert_ne!(a, ids.next());
    }

    #[test]
    fn full_queue_drops_and_counts() {
        let (h, _rx) = LinkHandle::new(LinkId(1), "test");
        for _ in 0..OUTBOUND_QUEUE {
            assert!(h.send(hb()));
        }
        assert!(!h.send(hb()));
        assert_eq!(h.counters().dropped.load(Ordering::Relaxed), 1);
    }

    #[tokio::test]
    async fn pump_parses_and_writes() {
        let (ours, mut theirs) = tokio::io::duplex(1024);
        let (h, mut out_rx) = LinkHandle::new(LinkId(3), "duplex");
        let (ev_tx, mut ev_rx) = mpsc::channel(16);
        let task =
            tokio::spawn(async move { pump_stream(LinkId(3), ours, &mut out_rx, &ev_tx).await });

        theirs.write_all(b"junk").await.unwrap();
        theirs.write_all(hb().as_bytes()).await.unwrap();
        match ev_rx.recv().await.unwrap() {
            LinkEvent::Frame { link, frame, .. } => assert_eq!((link, frame), (LinkId(3), hb())),
            other => panic!("unexpected {other:?}"),
        }

        assert!(h.send(hb()));
        let mut got = vec![0u8; hb().as_bytes().len()];
        theirs.read_exact(&mut got).await.unwrap();
        assert_eq!(got, hb().as_bytes());

        drop(theirs);
        assert!(matches!(task.await.unwrap(), PumpEnd::Eof));
    }
}
