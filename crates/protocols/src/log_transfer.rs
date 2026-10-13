//! Dataflash log listing and download over MAVLink (sans-I/O): `LOG_REQUEST_LIST` →
//! `LOG_ENTRY`, `LOG_REQUEST_DATA` → `LOG_DATA`, then `LOG_REQUEST_END`.
//!
//! Behaviour of the vehicle side, from ArduPilot 4.7.1 `AP_Logger_MAVLinkLogTransfer.cpp`:
//! * every log message is ignored while armed ("Disarm for log download");
//! * a data request streams `count` bytes from `ofs` in 90-byte `LOG_DATA` packets and ends
//!   early at end of file (a packet shorter than 90 bytes); while a transfer is running,
//!   further data requests on the same link are dropped silently, so gaps are re-requested
//!   only after the stream has gone quiet;
//! * with no logs, the listing is a single `LOG_ENTRY` with id 0 and `num_logs` 0.
//!
//! `LOG_ERASE` (it deletes every log on the vehicle) is not part of these transfers: it is
//! `dc_core::Core::erase_logs`, behind the opt-in `log-erase` setting.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use mavlink_codec::Message;
use mavlink_codec::messages::{LogRequestData, LogRequestEnd, LogRequestList};

/// Bytes per `LOG_DATA`.
pub const LOG_DATA_LEN: u32 = 90;

/// One log on the vehicle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LogEntry {
    /// Log number (1-based).
    pub id: u16,
    /// Size, bytes.
    pub size: u32,
    /// UTC of the log, s since the Unix epoch (0 if unknown).
    pub time_utc: u32,
}

/// Retry policy.
#[derive(Debug, Clone, Copy)]
pub struct LogTransferConfig {
    /// Silence before asking again (or for the next gap).
    pub quiet: Duration,
    /// Rounds without progress before giving up.
    pub max_stalls: u32,
    /// Bytes asked for per data request.
    pub window: u32,
}

impl Default for LogTransferConfig {
    fn default() -> Self {
        LogTransferConfig {
            quiet: Duration::from_millis(500),
            max_stalls: 15,
            window: LOG_DATA_LEN * 1000,
        }
    }
}

/// Outcome so far.
#[derive(Debug, Clone, PartialEq)]
pub enum LogTransferStatus {
    /// Running.
    Running,
    /// Done.
    Done,
    /// Gave up; the text says why.
    Failed(String),
}

/// Listing of the logs on a vehicle.
#[derive(Debug)]
pub struct LogList {
    target: (u8, u8),
    cfg: LogTransferConfig,
    entries: BTreeMap<u16, LogEntry>,
    last: Option<u16>,
    last_activity: Instant,
    stalls: u32,
    progress_at_last_round: usize,
    status: LogTransferStatus,
}

impl LogList {
    /// Start; returns the `LOG_REQUEST_LIST` to send.
    pub fn start(target: (u8, u8), cfg: LogTransferConfig, now: Instant) -> (Self, Message) {
        let l = LogList {
            target,
            cfg,
            entries: BTreeMap::new(),
            last: None,
            last_activity: now,
            stalls: 0,
            progress_at_last_round: 0,
            status: LogTransferStatus::Running,
        };
        let req = l.request(0, u16::MAX);
        (l, req)
    }

    fn request(&self, start: u16, end: u16) -> Message {
        Message::LogRequestList(LogRequestList {
            target_system: self.target.0,
            target_component: self.target.1,
            start,
            end,
        })
    }

    /// Feed a message from the target.
    pub fn on_message(&mut self, msg: &Message, now: Instant) {
        let Message::LogEntry(e) = msg else { return };
        if self.status != LogTransferStatus::Running {
            return;
        }
        self.last_activity = now;
        if e.num_logs == 0 {
            self.last = Some(0);
        } else {
            self.last = Some(e.last_log_num);
            if e.id >= 1 {
                self.entries.insert(
                    e.id,
                    LogEntry {
                        id: e.id,
                        size: e.size,
                        time_utc: e.time_utc,
                    },
                );
            }
        }
        if self.missing().is_empty() {
            self.status = LogTransferStatus::Done;
        }
    }

    fn missing(&self) -> Vec<u16> {
        match self.last {
            None => vec![0],
            Some(last) => (1..=last)
                .filter(|i| !self.entries.contains_key(i))
                .collect(),
        }
    }

    /// Advance timers; returns a request to send, if due.
    pub fn poll(&mut self, now: Instant) -> Vec<Message> {
        if self.status != LogTransferStatus::Running
            || now.duration_since(self.last_activity) < self.cfg.quiet
        {
            return Vec::new();
        }
        if self.entries.len() <= self.progress_at_last_round {
            self.stalls += 1;
            if self.stalls > self.cfg.max_stalls {
                self.status = LogTransferStatus::Failed("no reply to LOG_REQUEST_LIST".into());
                return Vec::new();
            }
        } else {
            self.stalls = 0;
        }
        self.progress_at_last_round = self.entries.len();
        self.last_activity = now;
        let missing = self.missing();
        match (self.last, missing.first(), missing.last()) {
            (Some(_), Some(&a), Some(&b)) => vec![self.request(a, b)],
            _ => vec![self.request(0, u16::MAX)],
        }
    }

    /// Status.
    pub fn status(&self) -> &LogTransferStatus {
        &self.status
    }

    /// Logs found, by id.
    pub fn entries(&self) -> Vec<LogEntry> {
        self.entries.values().copied().collect()
    }
}

/// Download of one log.
#[derive(Debug)]
pub struct LogDownload {
    target: (u8, u8),
    id: u16,
    cfg: LogTransferConfig,
    size: u32,
    data: Vec<u8>,
    /// Received 90-byte blocks.
    have: Vec<bool>,
    received: u32,
    last_activity: Instant,
    stalls: u32,
    progress_at_last_round: u32,
    status: LogTransferStatus,
}

impl LogDownload {
    /// Start downloading `entry`; returns the first `LOG_REQUEST_DATA`.
    pub fn start(
        target: (u8, u8),
        entry: LogEntry,
        cfg: LogTransferConfig,
        now: Instant,
    ) -> (Self, Vec<Message>) {
        let blocks = entry.size.div_ceil(LOG_DATA_LEN) as usize;
        let mut d = LogDownload {
            target,
            id: entry.id,
            cfg,
            size: entry.size,
            data: vec![0; entry.size as usize],
            have: vec![false; blocks],
            received: 0,
            last_activity: now,
            stalls: 0,
            progress_at_last_round: 0,
            status: LogTransferStatus::Running,
        };
        if entry.size == 0 {
            d.status = LogTransferStatus::Done;
            return (d, vec![request_end(target)]);
        }
        let first = d.next_request();
        (d, first.into_iter().collect())
    }

    /// From the first missing block to the last missing one within `window` bytes, in one
    /// request: after loss the gaps are scattered, and one request per gap would cost a
    /// quiet period each. Blocks already held are sent again and ignored.
    fn next_request(&self) -> Option<Message> {
        let start = self.have.iter().position(|h| !h)?;
        let limit = (start + (self.cfg.window / LOG_DATA_LEN) as usize).min(self.have.len());
        let last = (start..limit)
            .rev()
            .find(|&b| !self.have[b])
            .unwrap_or(start);
        let ofs = start as u32 * LOG_DATA_LEN;
        let count = ((last - start + 1) as u32 * LOG_DATA_LEN).min(self.size - ofs);
        Some(Message::LogRequestData(LogRequestData {
            target_system: self.target.0,
            target_component: self.target.1,
            id: self.id,
            ofs,
            count,
        }))
    }

    /// Feed a message from the target. Returns what to send at once (the end request when
    /// complete).
    pub fn on_message(&mut self, msg: &Message, now: Instant) -> Vec<Message> {
        let Message::LogData(p) = msg else {
            return Vec::new();
        };
        if p.id != self.id || self.status != LogTransferStatus::Running {
            return Vec::new();
        }
        self.last_activity = now;
        let count = u32::from(p.count).min(LOG_DATA_LEN);
        if p.ofs % LOG_DATA_LEN != 0 || p.ofs >= self.size.max(1) && count > 0 {
            return Vec::new();
        }
        // A short packet is the end of the file: the log is shorter than listed.
        if count < LOG_DATA_LEN && p.ofs + count < self.size {
            self.truncate(p.ofs + count);
        }
        let block = (p.ofs / LOG_DATA_LEN) as usize;
        if block < self.have.len() && !self.have[block] {
            let end = (p.ofs + count).min(self.size) as usize;
            self.data[p.ofs as usize..end].copy_from_slice(&p.data[..end - p.ofs as usize]);
            self.have[block] = true;
            self.received += (end - p.ofs as usize) as u32;
        }
        if self.have.iter().all(|h| *h) {
            self.status = LogTransferStatus::Done;
            return vec![request_end(self.target)];
        }
        Vec::new()
    }

    fn truncate(&mut self, size: u32) {
        self.size = size;
        self.data.truncate(size as usize);
        self.have.truncate(size.div_ceil(LOG_DATA_LEN) as usize);
        self.received = self
            .have
            .iter()
            .enumerate()
            .filter(|(_, h)| **h)
            .map(|(b, _)| (size - b as u32 * LOG_DATA_LEN).min(LOG_DATA_LEN))
            .sum();
    }

    /// Advance timers: after a quiet period, ask for the next gap.
    pub fn poll(&mut self, now: Instant) -> Vec<Message> {
        if self.status != LogTransferStatus::Running
            || now.duration_since(self.last_activity) < self.cfg.quiet
        {
            return Vec::new();
        }
        if self.received <= self.progress_at_last_round {
            self.stalls += 1;
            if self.stalls > self.cfg.max_stalls {
                self.status = LogTransferStatus::Failed(format!(
                    "log {}: no progress at {} of {} bytes",
                    self.id, self.received, self.size
                ));
                return vec![request_end(self.target)];
            }
        } else {
            self.stalls = 0;
        }
        self.progress_at_last_round = self.received;
        self.last_activity = now;
        self.next_request().into_iter().collect()
    }

    /// Status.
    pub fn status(&self) -> &LogTransferStatus {
        &self.status
    }

    /// (bytes received, size).
    pub fn progress(&self) -> (u32, u32) {
        (self.received, self.size)
    }

    /// The log, once [`LogTransferStatus::Done`].
    pub fn data(&self) -> Option<&[u8]> {
        (self.status == LogTransferStatus::Done).then_some(&self.data[..])
    }
}

/// `LOG_REQUEST_END`: the vehicle resumes normal logging.
pub fn request_end(target: (u8, u8)) -> Message {
    Message::LogRequestEnd(LogRequestEnd {
        target_system: target.0,
        target_component: target.1,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::messages::{LogData, LogEntry as Entry};

    fn entry(id: u16, num: u16, size: u32) -> Message {
        Message::LogEntry(Entry {
            id,
            num_logs: num,
            last_log_num: num,
            time_utc: 0,
            size,
        })
    }

    #[test]
    fn listing_fills_gaps_and_handles_no_logs() {
        let t0 = Instant::now();
        let (mut l, req) = LogList::start((1, 1), LogTransferConfig::default(), t0);
        assert!(matches!(req, Message::LogRequestList(r) if r.start == 0 && r.end == u16::MAX));
        l.on_message(&entry(1, 3, 100), t0);
        l.on_message(&entry(3, 3, 300), t0);
        let again = l.poll(t0 + Duration::from_millis(600));
        assert!(matches!(&again[..], [Message::LogRequestList(r)] if r.start == 2 && r.end == 2));
        l.on_message(&entry(2, 3, 200), t0);
        assert_eq!(l.status(), &LogTransferStatus::Done);
        assert_eq!(
            l.entries().iter().map(|e| e.size).collect::<Vec<_>>(),
            [100, 200, 300]
        );

        let (mut l, _) = LogList::start((1, 1), LogTransferConfig::default(), t0);
        l.on_message(&entry(0, 0, 0), t0);
        assert_eq!(
            (l.status(), l.entries().len()),
            (&LogTransferStatus::Done, 0)
        );
    }

    fn data(id: u16, ofs: u32, bytes: &[u8]) -> Message {
        let mut d = [0u8; 90];
        d[..bytes.len()].copy_from_slice(bytes);
        Message::LogData(LogData {
            id,
            ofs,
            count: bytes.len() as u8,
            data: d,
        })
    }

    #[test]
    fn download_re_requests_lost_blocks() {
        let t0 = Instant::now();
        let file: Vec<u8> = (0..400u32).map(|i| (i % 251) as u8).collect();
        let e = LogEntry {
            id: 2,
            size: 400,
            time_utc: 0,
        };
        let (mut d, first) = LogDownload::start((1, 1), e, LogTransferConfig::default(), t0);
        assert!(matches!(&first[..], [Message::LogRequestData(r)] if r.ofs == 0 && r.count == 400));
        // Blocks 0, 2, 4 arrive; 1 and 3 are lost.
        for b in [0u32, 2, 4] {
            let lo = (b * 90) as usize;
            let hi = (lo + 90).min(400);
            assert!(d.on_message(&data(2, b * 90, &file[lo..hi]), t0).is_empty());
        }
        assert!(
            d.poll(t0 + Duration::from_millis(100)).is_empty(),
            "stream still going"
        );
        // One request spans both gaps (blocks 1 to 3); block 2 comes again and is ignored.
        let gap = d.poll(t0 + Duration::from_millis(600));
        assert!(matches!(&gap[..], [Message::LogRequestData(r)] if r.ofs == 90 && r.count == 270));
        d.on_message(&data(2, 90, &file[90..180]), t0);
        d.on_message(&data(2, 180, &[0xEE; 90]), t0);
        let end = d.on_message(&data(2, 270, &file[270..360]), t0);
        assert!(matches!(&end[..], [Message::LogRequestEnd(_)]));
        assert_eq!(d.data().unwrap(), &file[..]);
    }

    #[test]
    fn a_short_packet_ends_the_file_early() {
        let t0 = Instant::now();
        let e = LogEntry {
            id: 1,
            size: 1000,
            time_utc: 0,
        };
        let (mut d, _) = LogDownload::start((1, 1), e, LogTransferConfig::default(), t0);
        d.on_message(&data(1, 0, &[7; 90]), t0);
        let end = d.on_message(&data(1, 90, &[8; 10]), t0);
        assert!(matches!(&end[..], [Message::LogRequestEnd(_)]));
        assert_eq!(d.progress(), (100, 100));
        assert_eq!(d.data().unwrap().len(), 100);
    }

    #[test]
    fn gives_up_without_progress() {
        let t0 = Instant::now();
        let cfg = LogTransferConfig {
            max_stalls: 2,
            ..Default::default()
        };
        let e = LogEntry {
            id: 1,
            size: 180,
            time_utc: 0,
        };
        let (mut d, _) = LogDownload::start((1, 1), e, cfg, t0);
        let mut t = t0;
        let mut last = Vec::new();
        for _ in 0..4 {
            t += Duration::from_secs(1);
            last = d.poll(t);
        }
        assert!(matches!(d.status(), LogTransferStatus::Failed(_)));
        assert!(matches!(&last[..], [Message::LogRequestEnd(_)]) || last.is_empty());
    }
}
