//! Telemetry rates per link: link profiles and the request that sets them on the vehicle
//! (plan 09 Step 1).
//!
//! ArduPilot keeps stream rates per link (MAVLink channel), so each link gets its own
//! [`LinkProfile`] and its own [`StreamRequest`], sent on that link only:
//!
//! 1. [`LinkProfile::Constrained`] first stops the parameter-driven streams (`SRx_*`, `MAVn_*`
//!    on 4.7) with `REQUEST_DATA_STREAM` (all, stop). Copter keeps that in RAM only
//!    (`GCS_MAVLINK::persist_streamrates`), so nothing is saved on the vehicle.
//! 2. Each message is set with `MAV_CMD_SET_MESSAGE_INTERVAL`, one at a time: `COMMAND_ACK`
//!    does not say which message it answers.
//! 3. Each accepted rate is read back with `MAV_CMD_GET_MESSAGE_INTERVAL`
//!    (`MESSAGE_INTERVAL`). ArduPilot stores whole milliseconds (`set_message_interval`
//!    truncates), so a read-back within 1 ms of the request counts as set.
//!
//! The "ignore stream rate" link option (`SERIALx_OPTIONS` bit 12 on 4.6, `MAVn_OPTIONS` bit 2
//! on 4.7) makes ArduPilot ignore step 1 only; the read-back shows what the link really runs.

use std::time::{Duration, Instant};

use mavlink_codec::Message;
use mavlink_codec::enums::{MavCmd, MavResult};
use mavlink_codec::messages::{CommandLong, RequestDataStream};

/// How much telemetry a link is asked to carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum LinkProfile {
    /// USB, IP and SITL: the core's configured rates (`CoreConfig::stream_rates`).
    #[default]
    Full,
    /// SiK / RFD900x telemetry radios at 57 600 baud.
    High,
    /// ExpressLRS MAVLink mode or Airport, Crossfire MAVLink: about 0.1–1.25 kB/s down.
    Constrained,
}

/// `HIGH` rates: (message id, Hz). Worst case ≈ 0.9 kB/s with MAVLink 2 framing.
pub const HIGH_RATES: &[(u32, f32)] = &[
    (30, 4.0),  // ATTITUDE
    (33, 3.0),  // GLOBAL_POSITION_INT
    (74, 2.0),  // VFR_HUD
    (1, 1.0),   // SYS_STATUS
    (24, 1.0),  // GPS_RAW_INT
    (245, 1.0), // EXTENDED_SYS_STATE
    (193, 1.0), // EKF_STATUS_REPORT
    (42, 1.0),  // MISSION_CURRENT
    (242, 0.2), // HOME_POSITION
    (246, 1.0), // ADSB_VEHICLE
    (147, 1.0), // BATTERY_STATUS
    (241, 1.0), // VIBRATION
    (168, 0.5), // WIND
    (65, 0.5),  // RC_CHANNELS
    (36, 0.5),  // SERVO_OUTPUT_RAW
    (132, 1.0), // DISTANCE_SENSOR
    (136, 0.5), // TERRAIN_REPORT
    (162, 1.0), // FENCE_STATUS
    (133, 1.0), // TERRAIN_REQUEST
];

/// `CONSTRAINED` rates. Worst case ≈ 0.32 kB/s with MAVLink 2 framing, heartbeat included:
/// under the 500 B/s ArduPilot's ELRS simulator models (`SIM_ELRS`). No ADS-B, vibration,
/// wind, terrain or rangefinder streams.
pub const CONSTRAINED_RATES: &[(u32, f32)] = &[
    (30, 2.0),  // ATTITUDE
    (33, 1.0),  // GLOBAL_POSITION_INT
    (74, 1.0),  // VFR_HUD
    (1, 0.5),   // SYS_STATUS
    (24, 0.5),  // GPS_RAW_INT
    (245, 0.5), // EXTENDED_SYS_STATE
    (193, 0.2), // EKF_STATUS_REPORT
    (42, 0.5),  // MISSION_CURRENT
    (242, 0.1), // HOME_POSITION
    (147, 0.5), // BATTERY_STATUS
    (162, 0.5), // FENCE_STATUS
    (65, 0.2),  // RC_CHANNELS (RSSI / LQ)
];

impl LinkProfile {
    /// Every profile, richest first.
    pub const ALL: [LinkProfile; 3] = [
        LinkProfile::Full,
        LinkProfile::High,
        LinkProfile::Constrained,
    ];

    /// Name in link strings (`?profile=…`) and settings.
    pub fn key(self) -> &'static str {
        match self {
            LinkProfile::Full => "full",
            LinkProfile::High => "high",
            LinkProfile::Constrained => "constrained-elrs",
        }
    }

    /// The profile from its [`key`](Self::key).
    pub fn from_key(key: &str) -> Option<LinkProfile> {
        Self::ALL.into_iter().find(|p| p.key() == key)
    }

    /// The rates to request; `full` is the core's configured list.
    pub fn rates(self, full: &[(u32, f32)]) -> &[(u32, f32)] {
        match self {
            LinkProfile::Full => full,
            LinkProfile::High => HIGH_RATES,
            LinkProfile::Constrained => CONSTRAINED_RATES,
        }
    }

    /// Stop the parameter-driven streams before setting the profile's own.
    pub fn stops_other_streams(self) -> bool {
        self == LinkProfile::Constrained
    }

    /// `TERRAIN_DATA` frames per second when answering a `TERRAIN_REQUEST` on this link
    /// (`None` = no pacing). A request asks for up to 56 frames of 55 bytes; sent at once
    /// that is about 3 kB, six seconds of a 500 B/s link.
    pub fn terrain_frames_per_s(self) -> Option<f32> {
        match self {
            LinkProfile::Full => None,
            // ≈ 550 B/s of a 57 600-baud radio's ≈ 5.7 kB/s.
            LinkProfile::High => Some(10.0),
            // ≈ 110 B/s. The profile asks for no TERRAIN_REQUEST, so this only matters when
            // the vehicle sends one anyway.
            LinkProfile::Constrained => Some(2.0),
        }
    }

    /// The next leaner profile, if any.
    pub fn leaner(self) -> Option<LinkProfile> {
        match self {
            LinkProfile::Full => Some(LinkProfile::High),
            LinkProfile::High => Some(LinkProfile::Constrained),
            LinkProfile::Constrained => None,
        }
    }
}

/// Worst-case bytes per second for `rates` with MAVLink 2 framing (10-byte header, 2-byte
/// CRC, unsigned) and full-length payloads, plus one heartbeat per second. `max_len(id)` is
/// the message's maximum payload length (`None` = unknown, counted as 0).
pub fn budget(rates: &[(u32, f32)], max_len: impl Fn(u32) -> Option<usize>) -> f32 {
    const OVERHEAD: f32 = 12.0;
    let heartbeat = max_len(0).unwrap_or(9) as f32 + OVERHEAD;
    heartbeat
        + rates
            .iter()
            .map(|&(id, hz)| (max_len(id).unwrap_or(0) as f32 + OVERHEAD) * hz)
            .sum::<f32>()
}

/// Timing.
#[derive(Debug, Clone, Copy)]
pub struct StreamRequestConfig {
    /// Wait this long for each `COMMAND_ACK` / `MESSAGE_INTERVAL`.
    pub timeout: Duration,
    /// Sends per command before giving up on it.
    pub attempts: u32,
    /// Pause after `REQUEST_DATA_STREAM` (it has no answer).
    pub settle: Duration,
}

impl Default for StreamRequestConfig {
    fn default() -> Self {
        StreamRequestConfig {
            timeout: Duration::from_millis(1500),
            attempts: 3,
            settle: Duration::from_millis(300),
        }
    }
}

/// What happened to one message's rate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RateOutcome {
    /// Accepted and read back at the requested interval (within 1 ms).
    Set,
    /// Accepted, but the read-back differs: the vehicle runs it at this interval (µs; −1 =
    /// not streaming).
    Different(i32),
    /// Accepted; the read-back never came.
    NotReadBack,
    /// Refused with this `MAV_RESULT`.
    Refused(u8),
    /// No `COMMAND_ACK` after every attempt.
    NoAnswer,
}

/// One message's result.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RateResult {
    /// Message id.
    pub msgid: u32,
    /// Requested rate, Hz.
    pub hz: f32,
    /// Outcome.
    pub outcome: RateOutcome,
}

impl RateResult {
    /// Requested interval in µs.
    pub fn interval_us(&self) -> i32 {
        (1e6 / self.hz).round() as i32
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    Stop { until: Instant },
    Set { index: usize },
    Read { index: usize },
    Done,
}

/// Sets a list of message rates on one vehicle and reads them back. Sans-I/O: feed it every
/// message from the target and call [`poll`](Self::poll) regularly; send what it returns on
/// the link the rates are for.
#[derive(Debug, Clone)]
pub struct StreamRequest {
    target: (u8, u8),
    cfg: StreamRequestConfig,
    rates: Vec<(u32, f32)>,
    results: Vec<RateResult>,
    phase: Phase,
    attempts: u32,
    deadline: Instant,
}

impl StreamRequest {
    /// Start. Returns the machine and the first messages to send.
    pub fn start(
        target: (u8, u8),
        rates: &[(u32, f32)],
        stop_others: bool,
        cfg: StreamRequestConfig,
        now: Instant,
    ) -> (StreamRequest, Vec<Message>) {
        let mut r = StreamRequest {
            target,
            cfg,
            rates: rates.to_vec(),
            results: rates
                .iter()
                .map(|&(msgid, hz)| RateResult {
                    msgid,
                    hz,
                    outcome: RateOutcome::NoAnswer,
                })
                .collect(),
            phase: Phase::Done,
            attempts: 0,
            deadline: now,
        };
        let out = if stop_others {
            r.phase = Phase::Stop {
                until: now + cfg.settle,
            };
            vec![Message::RequestDataStream(RequestDataStream {
                target_system: target.0,
                target_component: target.1,
                req_stream_id: 0, // MAV_DATA_STREAM_ALL
                req_message_rate: 0,
                start_stop: 0,
            })]
        } else {
            r.begin(Phase::Set { index: 0 }, now)
        };
        (r, out)
    }

    /// Results so far, in request order.
    pub fn results(&self) -> &[RateResult] {
        &self.results
    }

    /// Every rate has an outcome.
    pub fn is_done(&self) -> bool {
        self.phase == Phase::Done
    }

    fn command(&self, cmd: MavCmd, msgid: u32, interval_us: f32) -> Message {
        Message::CommandLong(CommandLong {
            target_system: self.target.0,
            target_component: self.target.1,
            command: cmd.0,
            confirmation: self.attempts.saturating_sub(1).min(255) as u8,
            param1: msgid as f32,
            param2: interval_us,
            ..Default::default()
        })
    }

    /// Enter `phase` at its first message still to do, and send it.
    fn begin(&mut self, phase: Phase, now: Instant) -> Vec<Message> {
        self.phase = phase;
        self.attempts = 0;
        // Read only what was accepted.
        while let Phase::Read { index } = self.phase {
            match self.results.get(index) {
                Some(r) if r.outcome == RateOutcome::NotReadBack => break,
                Some(_) => self.phase = Phase::Read { index: index + 1 },
                None => self.phase = Phase::Done,
            }
        }
        match self.phase {
            Phase::Set { index } if index >= self.rates.len() => {
                self.begin(Phase::Read { index: 0 }, now)
            }
            Phase::Set { .. } | Phase::Read { .. } => self.send_current(now),
            _ => Vec::new(),
        }
    }

    fn send_current(&mut self, now: Instant) -> Vec<Message> {
        self.attempts += 1;
        self.deadline = now + self.cfg.timeout;
        match self.phase {
            Phase::Set { index } => {
                let r = self.results[index];
                vec![self.command(
                    MavCmd::SET_MESSAGE_INTERVAL,
                    r.msgid,
                    r.interval_us() as f32,
                )]
            }
            Phase::Read { index } => {
                vec![self.command(MavCmd::GET_MESSAGE_INTERVAL, self.results[index].msgid, 0.0)]
            }
            _ => Vec::new(),
        }
    }

    /// A message from the target.
    pub fn on_message(&mut self, msg: &Message, now: Instant) -> Vec<Message> {
        match (self.phase, msg) {
            (Phase::Set { index }, Message::CommandAck(a))
                if a.command == MavCmd::SET_MESSAGE_INTERVAL.0 =>
            {
                if a.result == MavResult::IN_PROGRESS.0 {
                    return Vec::new();
                }
                self.results[index].outcome = if a.result == MavResult::ACCEPTED.0 {
                    RateOutcome::NotReadBack
                } else {
                    RateOutcome::Refused(a.result)
                };
                self.begin(Phase::Set { index: index + 1 }, now)
            }
            (Phase::Read { index }, Message::MessageInterval(m))
                if u32::from(m.message_id) == self.results[index].msgid =>
            {
                let r = &mut self.results[index];
                let want = r.interval_us();
                r.outcome = if m.interval_us > 0 && (m.interval_us - want).abs() < 1000 {
                    RateOutcome::Set
                } else {
                    RateOutcome::Different(m.interval_us)
                };
                self.begin(Phase::Read { index: index + 1 }, now)
            }
            _ => Vec::new(),
        }
    }

    /// Time passes: retransmit or move on.
    pub fn poll(&mut self, now: Instant) -> Vec<Message> {
        match self.phase {
            Phase::Stop { until } if now >= until => self.begin(Phase::Set { index: 0 }, now),
            Phase::Set { index } | Phase::Read { index } if now >= self.deadline => {
                if self.attempts < self.cfg.attempts {
                    return self.send_current(now);
                }
                // Out of attempts: the outcome stays NoAnswer (set) or NotReadBack (read).
                match self.phase {
                    Phase::Set { .. } => self.begin(Phase::Set { index: index + 1 }, now),
                    _ => self.begin(Phase::Read { index: index + 1 }, now),
                }
            }
            _ => Vec::new(),
        }
    }

    /// One line per message that is not [`RateOutcome::Set`], for the operator.
    pub fn problems(&self) -> Vec<String> {
        self.results
            .iter()
            .filter(|r| r.outcome != RateOutcome::Set)
            .map(|r| {
                let what = match r.outcome {
                    RateOutcome::Set => unreachable!(),
                    RateOutcome::Different(-1) => {
                        "accepted, but the vehicle does not send it".into()
                    }
                    RateOutcome::Different(us) => format!(
                        "accepted, but it runs at {:.2} Hz",
                        if us > 0 { 1e6 / us as f32 } else { 0.0 }
                    ),
                    RateOutcome::NotReadBack => "accepted, not read back".into(),
                    RateOutcome::Refused(res) => format!("refused (MAV_RESULT {res})"),
                    RateOutcome::NoAnswer => "no answer".into(),
                };
                format!("message {} at {} Hz: {what}", r.msgid, r.hz)
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::messages::{CommandAck, MessageInterval};

    fn ack(cmd: MavCmd, result: MavResult) -> Message {
        Message::CommandAck(CommandAck {
            command: cmd.0,
            result: result.0,
            ..Default::default()
        })
    }

    fn interval(id: u16, us: i32) -> Message {
        Message::MessageInterval(MessageInterval {
            message_id: id,
            interval_us: us,
        })
    }

    fn sent(out: &[Message]) -> Vec<(u16, u32, f32)> {
        out.iter()
            .map(|m| match m {
                Message::CommandLong(c) => (c.command, c.param1 as u32, c.param2),
                Message::RequestDataStream(_) => (66, 0, 0.0),
                other => panic!("{other:?}"),
            })
            .collect()
    }

    #[test]
    fn profiles_have_keys_and_fit_their_budgets() {
        for p in LinkProfile::ALL {
            assert_eq!(LinkProfile::from_key(p.key()), Some(p));
        }
        assert_eq!(LinkProfile::from_key("elrs"), None);
        // Maximum payload lengths of the messages used (common.xml / standard.xml).
        let len = |id| {
            Some(match id {
                0 => 9,
                30 | 33 => 28,
                74 => 20,
                1 => 43,
                24 => 52,
                245 => 2,
                193 => 26,
                42 => 6,
                242 => 60,
                147 => 54,
                162 => 9,
                65 => 42,
                _ => return None,
            })
        };
        let b = budget(CONSTRAINED_RATES, len);
        assert!((b - 318.0).abs() < 1.0, "{b}");
        assert!(b < 0.8 * 500.0, "fits SIM_ELRS 500 B/s with headroom");
        assert_eq!(LinkProfile::Full.leaner(), Some(LinkProfile::High));
        assert_eq!(LinkProfile::Constrained.leaner(), None);
        assert!(LinkProfile::Constrained.stops_other_streams());
        assert_eq!(LinkProfile::Full.terrain_frames_per_s(), None);
        assert!(LinkProfile::Constrained.terrain_frames_per_s().unwrap() * 55.0 < 0.25 * 500.0);
        assert!(!LinkProfile::Full.stops_other_streams());
    }

    #[test]
    fn sets_one_at_a_time_then_reads_back() {
        let t = Instant::now();
        let rates = [(30, 2.0), (33, 3.0), (999, 1.0)];
        let (mut r, out) =
            StreamRequest::start((1, 1), &rates, true, StreamRequestConfig::default(), t);
        assert_eq!(sent(&out), [(66, 0, 0.0)], "stop the SR streams first");
        assert!(r.poll(t + Duration::from_millis(100)).is_empty());
        let t1 = t + Duration::from_millis(300);
        assert_eq!(sent(&r.poll(t1)), [(511, 30, 500_000.0)]);
        // An ACK for another command is not ours.
        assert!(
            r.on_message(&ack(MavCmd::COMPONENT_ARM_DISARM, MavResult::ACCEPTED), t1)
                .is_empty()
        );
        let out = r.on_message(&ack(MavCmd::SET_MESSAGE_INTERVAL, MavResult::ACCEPTED), t1);
        assert_eq!(sent(&out), [(511, 33, 333_333.0)]);
        let out = r.on_message(&ack(MavCmd::SET_MESSAGE_INTERVAL, MavResult::ACCEPTED), t1);
        assert_eq!(sent(&out), [(511, 999, 1_000_000.0)]);
        // Unknown message: ArduPilot denies it.
        let out = r.on_message(&ack(MavCmd::SET_MESSAGE_INTERVAL, MavResult::DENIED), t1);
        assert_eq!(sent(&out), [(510, 30, 0.0)], "read back the accepted ones");
        let out = r.on_message(&interval(30, 500_000), t1);
        assert_eq!(sent(&out), [(510, 33, 0.0)]);
        // ArduPilot stores whole milliseconds: 333 ms for 333 333 µs is the same rate.
        assert!(r.on_message(&interval(33, 333_000), t1).is_empty());
        assert!(r.is_done());
        let o: Vec<_> = r.results().iter().map(|x| x.outcome).collect();
        assert_eq!(
            o,
            [
                RateOutcome::Set,
                RateOutcome::Set,
                RateOutcome::Refused(MavResult::DENIED.0)
            ]
        );
        assert_eq!(
            r.problems(),
            ["message 999 at 1 Hz: refused (MAV_RESULT 2)"]
        );
    }

    #[test]
    fn retries_then_gives_up_and_reports() {
        let t = Instant::now();
        let cfg = StreamRequestConfig::default();
        let (mut r, out) = StreamRequest::start((1, 1), &[(30, 2.0), (74, 1.0)], false, cfg, t);
        assert_eq!(sent(&out), [(511, 30, 500_000.0)]);
        let mut now = t;
        for attempt in 2..=3 {
            now += cfg.timeout;
            let out = r.poll(now);
            assert_eq!(sent(&out), [(511, 30, 500_000.0)], "attempt {attempt}");
            let Message::CommandLong(c) = &out[0] else {
                panic!()
            };
            assert_eq!(c.confirmation, attempt - 1);
        }
        now += cfg.timeout;
        assert_eq!(
            sent(&r.poll(now)),
            [(511, 74, 1_000_000.0)],
            "gives up on 30"
        );
        let out = r.on_message(&ack(MavCmd::SET_MESSAGE_INTERVAL, MavResult::ACCEPTED), now);
        assert_eq!(sent(&out), [(510, 74, 0.0)]);
        // Read back as "not streaming": the vehicle took the command but sends nothing.
        r.on_message(&interval(74, -1), now);
        assert!(r.is_done());
        assert_eq!(
            r.problems(),
            [
                "message 30 at 2 Hz: no answer",
                "message 74 at 1 Hz: accepted, but the vehicle does not send it"
            ]
        );
    }

    #[test]
    fn in_progress_waits_and_a_lost_read_back_is_reported() {
        let t = Instant::now();
        let cfg = StreamRequestConfig {
            attempts: 1,
            ..Default::default()
        };
        let (mut r, _) = StreamRequest::start((1, 1), &[(30, 4.0)], false, cfg, t);
        assert!(
            r.on_message(
                &ack(MavCmd::SET_MESSAGE_INTERVAL, MavResult::IN_PROGRESS),
                t
            )
            .is_empty()
        );
        r.on_message(&ack(MavCmd::SET_MESSAGE_INTERVAL, MavResult::ACCEPTED), t);
        // A read-back for another message is ignored.
        assert!(r.on_message(&interval(33, 250_000), t).is_empty());
        assert!(r.poll(t + cfg.timeout).is_empty());
        assert!(r.is_done());
        assert_eq!(r.results()[0].outcome, RateOutcome::NotReadBack);
        // The running rate differs from the request: reported with the rate it runs at.
        let (mut r, _) = StreamRequest::start((1, 1), &[(30, 4.0)], false, cfg, t);
        r.on_message(&ack(MavCmd::SET_MESSAGE_INTERVAL, MavResult::ACCEPTED), t);
        r.on_message(&interval(30, 100_000), t);
        assert_eq!(
            r.problems(),
            ["message 30 at 4 Hz: accepted, but it runs at 10.00 Hz"]
        );
    }
}
