//! MAVLink parameter protocol (sans-I/O).
//!
//! Download: send `PARAM_REQUEST_LIST`; collect `PARAM_VALUE` by index; when the stream goes
//! quiet, re-request the missing indices with `PARAM_REQUEST_READ`, a few at a time, until
//! all `param_count` values are present or the retry budget runs out.
//!
//! Set: send `PARAM_SET`; done only when a `PARAM_VALUE` echoes the name with the new value
//! (read-back verification, plan 04 §7); retried, then reported as failed.
//!
//! ArduPilot uses C-cast encoding: `param_value` holds the numeric value as a float.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use mavlink_codec::Message;
use mavlink_codec::enums::MavParamType;
use mavlink_codec::messages::{ParamRequestList, ParamRequestRead, ParamSet, ParamValue};

/// One parameter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Param {
    /// Value (C-cast encoding).
    pub value: f32,
    /// `MAV_PARAM_TYPE`.
    pub ty: u8,
    /// Index on the vehicle.
    pub index: u16,
}

impl Param {
    /// True for integer parameter types.
    pub fn is_integer(&self) -> bool {
        !matches!(
            MavParamType(self.ty),
            MavParamType::REAL32 | MavParamType::REAL64
        )
    }

    /// Value formatted like Mission Planner `.param` files: integers without decimals,
    /// floats in the shortest form that round-trips.
    pub fn display(&self) -> String {
        if self.is_integer() {
            format!("{}", self.value.round() as i64)
        } else {
            format!("{}", self.value)
        }
    }
}

/// Encode a parameter name into the 16-byte `param_id` field.
pub fn encode_id(name: &str) -> [u8; 16] {
    let mut id = [0u8; 16];
    let b = name.as_bytes();
    let n = b.len().min(16);
    id[..n].copy_from_slice(&b[..n]);
    id
}

/// Decode `param_id` (NUL-terminated unless all 16 bytes are used).
pub fn decode_id(id: &[u8; 16]) -> String {
    let end = id.iter().position(|&b| b == 0).unwrap_or(16);
    String::from_utf8_lossy(&id[..end]).into_owned()
}

/// Tuning knobs for the download.
#[derive(Debug, Clone)]
pub struct DownloadConfig {
    /// Silence after which missing indices are re-requested.
    pub quiet: Duration,
    /// Re-requests sent per round.
    pub batch: usize,
    /// Rounds without progress before giving up.
    pub max_stalls: u32,
    /// No first `PARAM_VALUE` within this time: give up.
    pub first_reply: Duration,
}

impl Default for DownloadConfig {
    fn default() -> Self {
        DownloadConfig {
            quiet: Duration::from_millis(500),
            batch: 10,
            max_stalls: 10,
            first_reply: Duration::from_secs(5),
        }
    }
}

/// Progress of a download.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DownloadStatus {
    /// Still running: `received` of `total` (0 until the first reply).
    Running {
        /// Unique parameters received.
        received: usize,
        /// `param_count` reported by the vehicle.
        total: usize,
    },
    /// All parameters received.
    Complete,
    /// Gave up.
    Failed,
}

/// Full parameter download from one component.
#[derive(Debug)]
pub struct ParamDownload {
    target: (u8, u8),
    cfg: DownloadConfig,
    started: Instant,
    last_rx: Option<Instant>,
    last_request: Instant,
    total: Option<u16>,
    by_index: BTreeMap<u16, (String, Param)>,
    stalls: u32,
    progress_at_last_round: usize,
    failed: bool,
}

impl ParamDownload {
    /// Start a download; returns the state machine and the `PARAM_REQUEST_LIST` to send.
    pub fn start(target: (u8, u8), cfg: DownloadConfig, now: Instant) -> (Self, Message) {
        let req = Message::ParamRequestList(ParamRequestList {
            target_system: target.0,
            target_component: target.1,
        });
        let d = ParamDownload {
            target,
            cfg,
            started: now,
            last_rx: None,
            last_request: now,
            total: None,
            by_index: BTreeMap::new(),
            stalls: 0,
            progress_at_last_round: 0,
            failed: false,
        };
        (d, req)
    }

    /// Feed a message from the target component (others are ignored by the caller).
    pub fn on_message(&mut self, msg: &Message, now: Instant) {
        let Message::ParamValue(p) = msg else { return };
        self.last_rx = Some(now);
        if p.param_count > 0 && p.param_count != u16::MAX {
            self.total = Some(p.param_count);
        }
        if p.param_index != u16::MAX && self.total.is_none_or(|t| p.param_index < t) {
            let param = Param {
                value: p.param_value,
                ty: p.param_type,
                index: p.param_index,
            };
            self.by_index
                .insert(p.param_index, (decode_id(&p.param_id), param));
        }
    }

    /// Advance timers; returns requests to send now.
    pub fn poll(&mut self, now: Instant) -> Vec<Message> {
        if self.failed || self.is_complete() {
            return Vec::new();
        }
        let Some(total) = self.total else {
            if now.duration_since(self.started) > self.cfg.first_reply {
                self.failed = true;
            }
            return Vec::new();
        };
        let quiet_since = self.last_rx.unwrap_or(self.started).max(self.last_request);
        if now.duration_since(quiet_since) < self.cfg.quiet {
            return Vec::new();
        }
        if self.by_index.len() <= self.progress_at_last_round {
            self.stalls += 1;
            if self.stalls > self.cfg.max_stalls {
                self.failed = true;
                return Vec::new();
            }
        } else {
            self.stalls = 0;
        }
        self.progress_at_last_round = self.by_index.len();
        self.last_request = now;
        (0..total)
            .filter(|i| !self.by_index.contains_key(i))
            .take(self.cfg.batch)
            .map(|i| {
                Message::ParamRequestRead(ParamRequestRead {
                    target_system: self.target.0,
                    target_component: self.target.1,
                    param_index: i as i16,
                    param_id: [0; 16],
                })
            })
            .collect()
    }

    /// True when every index `0..param_count` is present.
    pub fn is_complete(&self) -> bool {
        self.total
            .is_some_and(|t| self.by_index.len() == usize::from(t))
    }

    /// Current status.
    pub fn status(&self) -> DownloadStatus {
        if self.is_complete() {
            DownloadStatus::Complete
        } else if self.failed {
            DownloadStatus::Failed
        } else {
            DownloadStatus::Running {
                received: self.by_index.len(),
                total: self.total.map_or(0, usize::from),
            }
        }
    }

    /// Indices still missing (empty until `param_count` is known).
    pub fn missing(&self) -> Vec<u16> {
        self.total
            .map(|t| (0..t).filter(|i| !self.by_index.contains_key(i)).collect())
            .unwrap_or_default()
    }

    /// Parameters received so far, by name.
    pub fn params(&self) -> BTreeMap<String, Param> {
        self.by_index.values().cloned().collect()
    }
}

/// Outcome of a parameter write.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SetStatus {
    /// Waiting for the read-back.
    Pending,
    /// Vehicle reports the requested value.
    Verified(Param),
    /// Vehicle answered with a different value (e.g. clamped or rejected), or never answered.
    Failed(Option<Param>),
}

/// Write one parameter and verify it by read-back.
#[derive(Debug)]
pub struct ParamWrite {
    request: Message,
    name: String,
    value: f32,
    sent_at: Instant,
    attempts: u32,
    status: SetStatus,
}

/// Seconds to wait for each read-back, and attempts before failing.
pub const SET_TIMEOUT: Duration = Duration::from_secs(1);
/// Attempts before a write is reported as failed.
pub const SET_ATTEMPTS: u32 = 3;

impl ParamWrite {
    /// Start a write; returns the state machine and the `PARAM_SET` to send.
    pub fn start(
        target: (u8, u8),
        name: &str,
        value: f32,
        ty: u8,
        now: Instant,
    ) -> (Self, Message) {
        let request = Message::ParamSet(ParamSet {
            target_system: target.0,
            target_component: target.1,
            param_id: encode_id(name),
            param_value: value,
            param_type: ty,
        });
        let w = ParamWrite {
            request: request.clone(),
            name: name.to_string(),
            value,
            sent_at: now,
            attempts: 1,
            status: SetStatus::Pending,
        };
        (w, request)
    }

    /// Feed a message from the target component.
    pub fn on_message(&mut self, msg: &Message) {
        let Message::ParamValue(ParamValue {
            param_id,
            param_value,
            param_type,
            param_index,
            ..
        }) = msg
        else {
            return;
        };
        if self.status != SetStatus::Pending || decode_id(param_id) != self.name {
            return;
        }
        let p = Param {
            value: *param_value,
            ty: *param_type,
            index: *param_index,
        };
        // Integers travel as floats: compare after the same rounding the vehicle applies.
        let same = if p.is_integer() {
            p.value.round() == self.value.round()
        } else {
            p.value == self.value
        };
        self.status = if same {
            SetStatus::Verified(p)
        } else {
            SetStatus::Failed(Some(p))
        };
    }

    /// Advance timers; returns a retry to send, if due.
    pub fn poll(&mut self, now: Instant) -> Option<Message> {
        if self.status != SetStatus::Pending || now.duration_since(self.sent_at) < SET_TIMEOUT {
            return None;
        }
        if self.attempts >= SET_ATTEMPTS {
            self.status = SetStatus::Failed(None);
            return None;
        }
        self.attempts += 1;
        self.sent_at = now;
        Some(self.request.clone())
    }

    /// Current status.
    pub fn status(&self) -> SetStatus {
        self.status
    }
}

/// Render parameters as a Mission Planner-compatible `.param` file (`NAME,VALUE` lines).
pub fn to_param_file(params: &BTreeMap<String, Param>) -> String {
    params
        .iter()
        .map(|(n, p)| format!("{n},{}\n", p.display()))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn value(name: &str, v: f32, index: u16, count: u16) -> Message {
        Message::ParamValue(ParamValue {
            param_id: encode_id(name),
            param_value: v,
            param_type: MavParamType::REAL32.0,
            param_count: count,
            param_index: index,
        })
    }

    #[test]
    fn ids_round_trip_including_full_16_chars() {
        assert_eq!(decode_id(&encode_id("ATC_RAT_RLL_P")), "ATC_RAT_RLL_P");
        assert_eq!(
            decode_id(&encode_id("SIXTEEN_CHARS_XX")),
            "SIXTEEN_CHARS_XX"
        );
    }

    #[test]
    fn download_recovers_lost_values() {
        let t0 = Instant::now();
        let (mut d, req) = ParamDownload::start((1, 1), DownloadConfig::default(), t0);
        assert!(matches!(req, Message::ParamRequestList(_)));
        // 5 params; indices 1 and 3 are lost on the link.
        for (i, n) in [(0, "A"), (2, "C"), (4, "E")] {
            d.on_message(&value(n, i as f32, i, 5), t0);
        }
        assert_eq!(
            d.status(),
            DownloadStatus::Running {
                received: 3,
                total: 5
            }
        );
        assert!(
            d.poll(t0 + Duration::from_millis(100)).is_empty(),
            "not quiet yet"
        );

        let retry = d.poll(t0 + Duration::from_millis(600));
        let asked: Vec<i16> = retry
            .iter()
            .map(|m| match m {
                Message::ParamRequestRead(r) => r.param_index,
                _ => panic!(),
            })
            .collect();
        assert_eq!(asked, [1, 3]);
        d.on_message(&value("B", 1.0, 1, 5), t0 + Duration::from_millis(700));
        d.on_message(&value("D", 3.0, 3, 5), t0 + Duration::from_millis(700));
        assert_eq!(d.status(), DownloadStatus::Complete);
        assert_eq!(
            d.params().keys().cloned().collect::<Vec<_>>(),
            ["A", "B", "C", "D", "E"]
        );
    }

    #[test]
    fn download_gives_up() {
        let t0 = Instant::now();
        let cfg = DownloadConfig {
            max_stalls: 2,
            ..Default::default()
        };
        let (mut d, _) = ParamDownload::start((1, 1), cfg.clone(), t0);
        d.poll(t0 + Duration::from_secs(6));
        assert_eq!(d.status(), DownloadStatus::Failed, "no reply at all");

        let (mut d, _) = ParamDownload::start((1, 1), cfg, t0);
        d.on_message(&value("A", 0.0, 0, 2), t0);
        let mut t = t0;
        for _ in 0..5 {
            t += Duration::from_secs(1);
            d.poll(t);
        }
        assert_eq!(d.status(), DownloadStatus::Failed);
        assert_eq!(d.missing(), [1]);
    }

    #[test]
    fn write_verifies_readback_and_retries() {
        let t0 = Instant::now();
        let (mut w, _) = ParamWrite::start((1, 1), "FS_THR_ENABLE", 1.0, MavParamType::INT8.0, t0);
        w.on_message(&value("OTHER", 1.0, 0, 1));
        assert_eq!(w.status(), SetStatus::Pending);
        assert!(
            w.poll(t0 + Duration::from_millis(1100)).is_some(),
            "retry after timeout"
        );
        let mut v = value("FS_THR_ENABLE", 1.0, 7, 900);
        if let Message::ParamValue(p) = &mut v {
            p.param_type = MavParamType::INT8.0;
        }
        w.on_message(&v);
        assert!(matches!(w.status(), SetStatus::Verified(p) if p.index == 7));

        let (mut w, _) = ParamWrite::start((1, 1), "X", 5.0, MavParamType::REAL32.0, t0);
        w.on_message(&value("X", 4.0, 0, 1));
        assert!(
            matches!(w.status(), SetStatus::Failed(Some(_))),
            "vehicle clamped the value"
        );

        let (mut w, _) = ParamWrite::start((1, 1), "Y", 5.0, MavParamType::REAL32.0, t0);
        let mut t = t0;
        for _ in 0..4 {
            t += SET_TIMEOUT;
            w.poll(t);
        }
        assert_eq!(w.status(), SetStatus::Failed(None));
    }

    #[test]
    fn param_file_formats_integers_and_floats() {
        let mut m = BTreeMap::new();
        m.insert(
            "A_INT".into(),
            Param {
                value: 3.0,
                ty: MavParamType::INT32.0,
                index: 0,
            },
        );
        m.insert(
            "B_FLT".into(),
            Param {
                value: 0.135,
                ty: MavParamType::REAL32.0,
                index: 1,
            },
        );
        assert_eq!(to_param_file(&m), "A_INT,3\nB_FLT,0.135\n");
    }
}
