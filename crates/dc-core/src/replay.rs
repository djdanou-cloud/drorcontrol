//! `.tlog` replay as a link (`tlog:PATH[@SPEED]`): a recorded session played back with its
//! original timing, so the core, the API and the UI can be exercised without a vehicle.
//!
//! A replay is receive-only: frames the core sends to it are discarded (and counted as
//! queued, then dropped here). The core allows a replay only as its sole link (see
//! [`crate::Core::connect`]), so recorded traffic can never be relayed to a real vehicle.
//! Frames carrying our own system and component id were outbound when recorded; the router
//! already drops those as loops.

use std::io::{self, BufReader};
use std::path::{Path, PathBuf};
use std::time::Duration;

use link_core::{LinkEvent, LinkHandle, LinkId};
use recorder::{TlogReader, TlogRecord, from_unix_micros};
use tokio::sync::mpsc;
use tokio::time::Instant;

/// Start replaying `path` at `speed` × real time. Fails at once if the file cannot be
/// opened; a bad record later ends the replay with a link status naming it.
pub(crate) async fn spawn_replay(
    id: LinkId,
    path: &Path,
    speed: u32,
    events: mpsc::Sender<LinkEvent>,
) -> io::Result<LinkHandle> {
    let file = std::fs::File::open(path)?;
    let path: PathBuf = path.to_path_buf();
    let (handle, mut outbound) =
        LinkHandle::new(id, format!("tlog-replay {} x{speed}", path.display()));
    let _ = events.send(LinkEvent::Added(handle.clone())).await;

    // Read on a thread (file I/O is blocking), pace on the runtime.
    let (rec_tx, mut rec_rx) = mpsc::channel::<io::Result<TlogRecord>>(256);
    std::thread::Builder::new()
        .name("tlog-replay".into())
        .spawn(move || {
            let mut r = TlogReader::new(BufReader::new(file));
            loop {
                let next = r.next_record().transpose();
                let end = !matches!(next, Some(Ok(_)));
                if let Some(rec) = next
                    && rec_tx.blocking_send(rec).is_err()
                {
                    return;
                }
                if end {
                    return;
                }
            }
        })?;

    tokio::spawn(async move {
        let status = |up: bool, detail: String| LinkEvent::Status {
            link: id,
            up,
            detail,
        };
        let _ = events
            .send(status(true, format!("replaying {}", path.display())))
            .await;
        let mut origin: Option<(u64, Instant)> = None;
        let mut frames = 0u64;
        let detail = loop {
            let rec = tokio::select! {
                r = rec_rx.recv() => r,
                // A replay cannot be commanded: drain and discard what the core sends.
                f = outbound.recv() => match f {
                    Some(_) => continue,
                    None => return, // shutdown
                },
            };
            let rec = match rec {
                Some(Ok(rec)) => rec,
                Some(Err(e)) => break format!("replay stopped after {frames} records: {e}"),
                None => break format!("replay finished ({frames} records)"),
            };
            let (t0, start) = *origin.get_or_insert((rec.unix_us, Instant::now()));
            let offset = Duration::from_micros(rec.unix_us.saturating_sub(t0)) / speed;
            let due = start + offset;
            loop {
                tokio::select! {
                    _ = tokio::time::sleep_until(due) => break,
                    f = outbound.recv() => if f.is_none() { return },
                }
            }
            let ev = LinkEvent::Frame {
                link: id,
                frame: rec.frame,
                rx_time: from_unix_micros(rec.unix_us),
            };
            if events.send(ev).await.is_err() {
                return;
            }
            frames += 1;
        };
        let _ = events.send(status(false, detail)).await;
        // Keep the link (it is down, like a vehicle that went silent) until shutdown.
        while outbound.recv().await.is_some() {}
    });
    Ok(handle)
}
