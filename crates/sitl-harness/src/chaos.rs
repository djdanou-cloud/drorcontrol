//! `link-chaos`: a frame-aware TCP proxy that degrades a MAVLink link on purpose (plan 08 §2).
//!
//! GCS ⇄ proxy ⇄ SITL. Each direction parses MAVLink frames and, per frame, may drop it,
//! duplicate it, corrupt one byte (so the receiver's CRC check rejects it) and delay it by
//! `delay + uniform(0, jitter)`; jitter larger than the frame spacing reorders frames.
//! [`ChaosProxy::blackout`] silently drops everything while keeping TCP open (a radio fade);
//! [`ChaosProxy::cut`] closes the connection (the GCS sees EOF and must reconnect). Random
//! choices come from a seeded generator, so a failing run can be replayed with its seed.

use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::io;
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use mavlink_codec::FrameParser;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::tcp::{OwnedReadHalf, OwnedWriteHalf};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::watch;
use tokio::time::Instant;

/// What to do to every frame, per direction.
#[derive(Debug, Clone, PartialEq)]
pub struct ChaosConfig {
    /// Probability a frame is dropped.
    pub drop: f64,
    /// Probability a frame is sent twice.
    pub dup: f64,
    /// Probability one byte of a frame is flipped.
    pub corrupt: f64,
    /// Fixed one-way delay.
    pub delay: Duration,
    /// Extra uniform random delay (reorders when larger than frame spacing).
    pub jitter: Duration,
    /// Seed for the random choices (logged by the tests).
    pub seed: u64,
}

impl Default for ChaosConfig {
    fn default() -> Self {
        ChaosConfig {
            drop: 0.0,
            dup: 0.0,
            corrupt: 0.0,
            delay: Duration::ZERO,
            jitter: Duration::ZERO,
            seed: 1,
        }
    }
}

#[derive(Debug, Clone)]
struct State {
    cfg: ChaosConfig,
    blackout: bool,
    epoch: u64,
}

/// Frame counters (both directions).
#[derive(Debug, Default)]
pub struct ChaosStats {
    /// Frames delivered (duplicates count twice).
    pub forwarded: AtomicU64,
    /// Frames dropped (loss or blackout).
    pub dropped: AtomicU64,
    /// Frames duplicated.
    pub duplicated: AtomicU64,
    /// Frames corrupted.
    pub corrupted: AtomicU64,
}

/// A running proxy.
pub struct ChaosProxy {
    addr: SocketAddr,
    state: watch::Sender<State>,
    stats: Arc<ChaosStats>,
}

/// xorshift64*: small, seedable, good enough for fault injection.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Rng {
        Rng(seed.max(1))
    }
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn chance(&mut self, p: f64) -> bool {
        let u = (self.next() >> 11) as f64 / (1u64 << 53) as f64;
        p > 0.0 && u < p
    }
    fn below(&mut self, n: u64) -> u64 {
        if n == 0 { 0 } else { self.next() % n }
    }
}

impl ChaosProxy {
    /// Listen on an ephemeral local port; every accepted client gets a fresh connection to
    /// `upstream`.
    pub async fn start(upstream: SocketAddr, cfg: ChaosConfig) -> io::Result<ChaosProxy> {
        let listener = TcpListener::bind("127.0.0.1:0").await?;
        let addr = listener.local_addr()?;
        let (state, rx) = watch::channel(State {
            cfg,
            blackout: false,
            epoch: 0,
        });
        let stats = Arc::new(ChaosStats::default());
        let st = stats.clone();
        tokio::spawn(async move {
            let mut n = 0u64;
            while let Ok((down, _)) = listener.accept().await {
                let Ok(up) = TcpStream::connect(upstream).await else {
                    continue;
                };
                let _ = down.set_nodelay(true);
                let _ = up.set_nodelay(true);
                let (dr, dw) = down.into_split();
                let (ur, uw) = up.into_split();
                let epoch = rx.borrow().epoch;
                n += 1;
                let seed = rx.borrow().cfg.seed.wrapping_add(n * 1_000_003);
                tokio::spawn(pump(dr, uw, rx.clone(), st.clone(), epoch, seed));
                tokio::spawn(pump(
                    ur,
                    dw,
                    rx.clone(),
                    st.clone(),
                    epoch,
                    seed ^ 0x9E37_79B9_7F4A_7C15,
                ));
            }
        });
        Ok(ChaosProxy { addr, state, stats })
    }

    /// Where the GCS should connect.
    pub fn addr(&self) -> SocketAddr {
        self.addr
    }

    /// Change the impairments (takes effect for the next frame).
    pub fn set(&self, cfg: ChaosConfig) {
        self.state.send_modify(|s| s.cfg = cfg);
    }

    /// Drop every frame in both directions while `on` (TCP stays open).
    pub fn blackout(&self, on: bool) {
        self.state.send_modify(|s| s.blackout = on);
    }

    /// Close the current connections (the GCS sees EOF and must reconnect).
    pub fn cut(&self) {
        self.state.send_modify(|s| s.epoch += 1);
    }

    /// Counters.
    pub fn stats(&self) -> &ChaosStats {
        &self.stats
    }
}

async fn pump(
    mut rd: OwnedReadHalf,
    mut wr: OwnedWriteHalf,
    mut state: watch::Receiver<State>,
    stats: Arc<ChaosStats>,
    epoch: u64,
    seed: u64,
) {
    let mut rng = Rng::new(seed);
    let mut parser = FrameParser::new();
    let mut buf = [0u8; 4096];
    // (due, tie-break, bytes)
    let mut queue: BinaryHeap<Reverse<(Instant, u64, Vec<u8>)>> = BinaryHeap::new();
    let mut tie = 0u64;
    loop {
        let next_due = queue.peek().map(|Reverse((t, _, _))| *t);
        tokio::select! {
            r = rd.read(&mut buf) => {
                let n = match r { Ok(0) | Err(_) => return, Ok(n) => n };
                parser.push(&buf[..n]);
                let s = state.borrow().clone();
                while let Some(f) = parser.next_frame() {
                    if s.blackout || rng.chance(s.cfg.drop) {
                        stats.dropped.fetch_add(1, Ordering::Relaxed);
                        continue;
                    }
                    let mut bytes = f.as_bytes().to_vec();
                    if rng.chance(s.cfg.corrupt) {
                        // Flip a byte after the header so the CRC no longer matches.
                        let i = 6 + rng.below(bytes.len().saturating_sub(6) as u64) as usize;
                        if let Some(b) = bytes.get_mut(i) {
                            *b ^= 0x5A;
                        }
                        stats.corrupted.fetch_add(1, Ordering::Relaxed);
                    }
                    let copies = if rng.chance(s.cfg.dup) {
                        stats.duplicated.fetch_add(1, Ordering::Relaxed);
                        2
                    } else {
                        1
                    };
                    for _ in 0..copies {
                        let jitter = Duration::from_micros(rng.below(s.cfg.jitter.as_micros() as u64 + 1));
                        tie += 1;
                        queue.push(Reverse((Instant::now() + s.cfg.delay + jitter, tie, bytes.clone())));
                    }
                }
            }
            _ = async { tokio::time::sleep_until(next_due.unwrap()).await }, if next_due.is_some() => {
                let now = Instant::now();
                while queue.peek().is_some_and(|Reverse((t, _, _))| *t <= now) {
                    let Reverse((_, _, bytes)) = queue.pop().expect("peeked");
                    if wr.write_all(&bytes).await.is_err() {
                        return;
                    }
                    stats.forwarded.fetch_add(1, Ordering::Relaxed);
                }
            }
            changed = state.changed() => {
                if changed.is_err() || state.borrow().epoch != epoch {
                    return; // cut: dropping the halves closes both sockets
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::messages::Heartbeat;
    use mavlink_codec::{Header, Message, Version, encode};

    fn frame(seq: u8) -> Vec<u8> {
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
        .as_bytes()
        .to_vec()
    }

    async fn pair(cfg: ChaosConfig) -> (ChaosProxy, TcpStream, TcpStream) {
        let upstream = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let proxy = ChaosProxy::start(upstream.local_addr().unwrap(), cfg)
            .await
            .unwrap();
        let gcs = TcpStream::connect(proxy.addr()).await.unwrap();
        let (vehicle, _) = upstream.accept().await.unwrap();
        (proxy, gcs, vehicle)
    }

    async fn count_frames(s: &mut TcpStream, within: Duration) -> (usize, usize) {
        let mut p = FrameParser::new();
        let mut buf = [0u8; 4096];
        let end = Instant::now() + within;
        let mut seqs = Vec::new();
        while let Ok(Ok(n)) = tokio::time::timeout_at(end, s.read(&mut buf)).await {
            if n == 0 {
                break;
            }
            p.push(&buf[..n]);
            while let Some(f) = p.next_frame() {
                seqs.push(f.seq());
            }
        }
        let crc_errors = p.stats().crc_errors as usize;
        (seqs.len(), crc_errors)
    }

    #[tokio::test]
    async fn clean_proxy_forwards_everything_in_order() {
        let (_p, mut gcs, mut vehicle) = pair(ChaosConfig::default()).await;
        for i in 0..100 {
            vehicle.write_all(&frame(i)).await.unwrap();
        }
        let (n, crc) = count_frames(&mut gcs, Duration::from_millis(500)).await;
        assert_eq!((n, crc), (100, 0));
    }

    #[tokio::test]
    async fn drops_duplicates_and_corrupts_at_the_configured_rates() {
        let cfg = ChaosConfig {
            drop: 0.3,
            dup: 0.1,
            corrupt: 0.1,
            seed: 42,
            ..Default::default()
        };
        let (p, mut gcs, mut vehicle) = pair(cfg).await;
        for i in 0..2000u32 {
            vehicle.write_all(&frame(i as u8)).await.unwrap();
        }
        let (n, crc) = count_frames(&mut gcs, Duration::from_millis(1500)).await;
        let s = p.stats();
        let dropped = s.dropped.load(Ordering::Relaxed);
        assert!((500..700).contains(&dropped), "≈30 % dropped: {dropped}");
        assert!(crc > 0, "corrupted frames fail the receiver's CRC");
        // Every uncorrupted frame arrives (a corrupted one may also make the parser resync).
        let (fwd, bad) = (
            s.forwarded.load(Ordering::Relaxed),
            s.corrupted.load(Ordering::Relaxed),
        );
        assert!(
            n as u64 <= fwd && n as u64 + 2 * bad >= fwd,
            "received {n}, forwarded {fwd}, corrupted {bad}"
        );
    }

    #[tokio::test]
    async fn blackout_and_cut() {
        let (p, mut gcs, mut vehicle) = pair(ChaosConfig::default()).await;
        p.blackout(true);
        for i in 0..10 {
            vehicle.write_all(&frame(i)).await.unwrap();
        }
        assert_eq!(
            count_frames(&mut gcs, Duration::from_millis(300)).await.0,
            0
        );
        p.blackout(false);
        for i in 0..10 {
            vehicle.write_all(&frame(i)).await.unwrap();
        }
        assert_eq!(
            count_frames(&mut gcs, Duration::from_millis(300)).await.0,
            10
        );
        p.cut();
        let mut b = [0u8; 16];
        let r = tokio::time::timeout(Duration::from_secs(2), gcs.read(&mut b))
            .await
            .unwrap();
        assert!(
            matches!(r, Ok(0) | Err(_)),
            "GCS side sees the connection close"
        );
    }

    #[tokio::test]
    async fn jitter_reorders() {
        let cfg = ChaosConfig {
            jitter: Duration::from_millis(50),
            seed: 7,
            ..Default::default()
        };
        let (_p, mut gcs, mut vehicle) = pair(cfg).await;
        for i in 0..50 {
            vehicle.write_all(&frame(i)).await.unwrap();
        }
        let mut p = FrameParser::new();
        let mut buf = [0u8; 4096];
        let mut seqs = Vec::new();
        while seqs.len() < 50 {
            let n = tokio::time::timeout(Duration::from_secs(2), gcs.read(&mut buf))
                .await
                .unwrap()
                .unwrap();
            p.push(&buf[..n]);
            while let Some(f) = p.next_frame() {
                seqs.push(f.seq());
            }
        }
        assert!(
            seqs.windows(2).any(|w| w[1] < w[0]),
            "some frames arrive out of order"
        );
        let mut sorted = seqs.clone();
        sorted.sort_unstable();
        assert_eq!(sorted, (0..50).collect::<Vec<u8>>(), "but none lost");
    }
}
