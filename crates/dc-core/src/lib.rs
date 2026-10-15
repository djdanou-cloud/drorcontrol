//! DrorControl headless core.
//!
//! [`Core`] owns the router hub, the links, the vehicle state ([`Fleet`]), the GCS
//! heartbeat and the `.tlog` recorder, and exposes protocol operations (parameter download,
//! stream-rate requests). The desktop UI, `dc-cli`, the gRPC daemon and CI test pilots all
//! drive the same `Core`.
//!
//! Threading: link I/O and routing run on tokio tasks; vehicle state is published as an
//! immutable snapshot (`arc-swap`), so readers (the UI at frame rate) never block ingest.
//!
//! Liveness: a watchdog classifies every vehicle as [`VehicleLink::Connected`], `Stale` or
//! `Lost` from its heartbeat age (plan 08 §4.3) and raises [`CoreEvent::VehicleLink`] on
//! every transition. TCP client links reconnect by themselves (`link-net`).
//!
//! Commands: every vehicle command goes through [`protocols::command::CommandPolicy`]
//! (plan 08 §4, ADR-0009): [`Core::request_command`] → confirmation card →
//! [`Core::submit_command`]. The core retransmits until acknowledged or verified, floods every
//! link while the vehicle's link is not CONNECTED, never sends after the command's lifetime,
//! reports [`CoreEvent::Command`] on every change and appends each step to a hash-chained
//! audit log ([`CoreConfig::audit_log`]).

use std::collections::{BTreeMap, HashMap};
use std::fmt;
use std::io;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::str::FromStr;
use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};
use std::time::{Duration, Instant, SystemTime};

use arc_swap::ArcSwap;
use link_core::{LinkId, LinkIds};
use link_net::{UdpMode, spawn_tcp_client, spawn_tcp_server, spawn_udp};
use mav_router::{HubHandle, HubOutput, LinkPolicy, RouterConfig, spawn_hub};
pub use mav_router::{HubSnapshot, RouterStats};
use mavlink_codec::enums::{MavAutopilot, MavCmd, MavState, MavType};
use mavlink_codec::messages::{CommandLong, Heartbeat};
use mavlink_codec::{Frame, Header, Message, Version, encode};
use protocols::audit::{self, AuditLog};
/// Command types for [`Core::request_command`] and [`Core::submit_command`].
pub use protocols::command::{
    Alert, AlertLevel, Approval, Command, CommandId, CommandState, CommandStatus, PolicyConfig,
    Refusal, Request, Simplified, VehicleFacts,
};
use protocols::command::{AuditEvent, CommandPolicy, LinkInfo, LinkState, Target};
pub use protocols::log_transfer::LogEntry;
use protocols::log_transfer::{LogDownload, LogList, LogTransferConfig, LogTransferStatus};
/// Mission types for [`Core::upload_items`], [`Core::download_mission`] and
/// [`Core::download_items`].
pub use protocols::mission::{Mismatch, MissionItem, MissionType, TransferStatus};
use protocols::mission::{MissionDownload, TransferConfig};
pub use protocols::params::snapshot::ParamSnapshot;
use protocols::params::{DownloadConfig, DownloadStatus, ParamDownload};
/// Parameter values, reports and saved baselines for parameter write/rollback clients.
pub use protocols::params::{Param, SetStatus};
use tokio::sync::broadcast;

pub use vehicle::{
    AutopilotVersion, Battery, Extended, Firmware, FirmwareSource, Fleet, Home, LandedState,
    MissionProgress, MissionState, RadioStatus, Rangefinder, RcInput, ServoOutputs, StatusText,
    TerrainReport, Traffic, VehicleState, Vibration, VtolState, Wind,
};
pub use vehicle::{FenceBreach, FenceStatus, Threat, preflight};
pub use vehicle::{NavHealth, NavThresholds, PositionTrust, TESTED_FIRMWARE};

pub mod annunciator;
mod parameters;
mod replay;
mod vehicle_changes;
pub use parameters::{
    ARMED_WRITE_WARNING, ArmedWrite, PARAM_HEARTBEAT_MAX_AGE, ParamOperationError, ParamReport,
    ParamRisk, ParamWriteResult, parameter_risk,
};
pub use vehicle_changes::{
    ARMED_FENCE_RALLY_WARNING, ARMED_MISSION_UPLOAD_WARNING, ERASE_NOT_VERIFIED_HINT, EraseReport,
    VehicleChange, VehicleChangeError, VehicleChangeSettings,
};

/// How to reach a vehicle. Parsed from strings such as `tcp:127.0.0.1:5760`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LinkSpec {
    /// `tcp:HOST:PORT` — connect (and keep reconnecting) to a TCP server, e.g. SITL.
    Tcp(SocketAddr),
    /// `tcpin:ADDR:PORT` — accept TCP clients.
    TcpIn(SocketAddr),
    /// `udp:ADDR:PORT` — listen for UDP (GCS port 14550 style).
    Udp(SocketAddr),
    /// `udpout:HOST:PORT` — send UDP to a fixed address.
    UdpOut(SocketAddr),
    /// `serial:PATH:BAUD` — serial port.
    Serial {
        /// Device path, e.g. `/dev/ttyACM0`, `/dev/cu.usbmodem1101`, `COM3`.
        path: String,
        /// Baud rate.
        baud: u32,
    },
    /// `tlog:PATH[@SPEED]` — replay a recorded `.tlog` at SPEED × real time (default 1).
    /// Receive-only, and only as the core's sole link.
    Tlog {
        /// The recording.
        path: std::path::PathBuf,
        /// Playback speed, 1..=100.
        speed: u32,
    },
}

/// What kind of links a core has opened: a replay never shares the core with a live link,
/// so recorded traffic cannot be relayed to a vehicle and replayed data cannot be mistaken
/// for live data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum LinkMode {
    #[default]
    None,
    Live,
    Replay,
}

impl FromStr for LinkSpec {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, String> {
        let (kind, rest) = s
            .split_once(':')
            .ok_or_else(|| format!("`{s}`: expected KIND:ADDRESS"))?;
        let addr = || {
            rest.parse::<SocketAddr>()
                .map_err(|e| format!("`{rest}`: {e}"))
        };
        Ok(match kind {
            "tcp" => LinkSpec::Tcp(addr()?),
            "tcpin" => LinkSpec::TcpIn(addr()?),
            "udp" => LinkSpec::Udp(addr()?),
            "udpout" => LinkSpec::UdpOut(addr()?),
            "tlog" => {
                let (path, speed) = match rest.rsplit_once('@') {
                    Some((p, s)) => (
                        p,
                        s.parse::<u32>()
                            .ok()
                            .filter(|s| (1..=100).contains(s))
                            .ok_or_else(|| format!("replay speed `{s}`: expected 1..=100"))?,
                    ),
                    None => (rest, 1),
                };
                if path.is_empty() {
                    return Err("tlog:PATH[@SPEED]".into());
                }
                LinkSpec::Tlog {
                    path: path.into(),
                    speed,
                }
            }
            "serial" => {
                let (path, baud) = rest.rsplit_once(':').ok_or("serial:PATH:BAUD")?;
                LinkSpec::Serial {
                    path: path.to_string(),
                    baud: baud.parse().map_err(|_| format!("bad baud `{baud}`"))?,
                }
            }
            _ => {
                return Err(format!(
                    "unknown link kind `{kind}` (tcp, tcpin, udp, udpout, serial, tlog)"
                ));
            }
        })
    }
}

impl fmt::Display for LinkSpec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LinkSpec::Tcp(a) => write!(f, "tcp:{a}"),
            LinkSpec::TcpIn(a) => write!(f, "tcpin:{a}"),
            LinkSpec::Udp(a) => write!(f, "udp:{a}"),
            LinkSpec::UdpOut(a) => write!(f, "udpout:{a}"),
            LinkSpec::Serial { path, baud } => write!(f, "serial:{path}:{baud}"),
            LinkSpec::Tlog { path, speed: 1 } => write!(f, "tlog:{}", path.display()),
            LinkSpec::Tlog { path, speed } => write!(f, "tlog:{}@{speed}", path.display()),
        }
    }
}

/// Frames received and lost from one (system, component), from MAVLink sequence numbers.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct StreamQuality {
    /// Frames received since the first one.
    pub received: u64,
    /// Frames missing from the sequence since the first one.
    pub lost: u64,
    /// Received and lost over the last [`QUALITY_WINDOW`].
    pub recent_received: u32,
    /// See `recent_received`.
    pub recent_lost: u32,
}

impl StreamQuality {
    /// Loss over the recent window, percent (`None` before any frame in it).
    pub fn recent_loss_pct(&self) -> Option<f32> {
        let n = self.recent_received + self.recent_lost;
        (n > 0).then(|| self.recent_lost as f32 * 100.0 / n as f32)
    }
}

/// Window for [`StreamQuality::recent_loss_pct`].
pub const QUALITY_WINDOW: Duration = Duration::from_secs(10);

/// Frames received and lost from one MAVLink sender's sequence numbers (sans-I/O; also used
/// on recordings).
#[derive(Debug, Default, Clone, Copy)]
pub struct SeqCount {
    last: Option<u8>,
    /// Frames counted (duplicates excluded).
    pub received: u64,
    /// Frames missing from the sequence.
    pub lost: u64,
}

impl SeqCount {
    /// Count a frame with sequence number `seq`. Returns `None` for a duplicate, else the
    /// frames newly lost before it and whether it arrived late (one earlier loss taken back).
    pub fn on_seq(&mut self, seq: u8) -> Option<(u32, bool)> {
        // `last` is the newest sequence number seen. A frame ahead of it by n means n - 1
        // frames lost; a frame behind it (by less than 128) arrived late, after being counted
        // lost, so that loss is taken back; the same number again is a duplicate.
        let (lost, late) = match self.last {
            None => (0, false),
            Some(last) => {
                let ahead = seq.wrapping_sub(last);
                match ahead {
                    0 => return None,
                    1..=127 => (u32::from(ahead - 1), false),
                    _ => (0, true),
                }
            }
        };
        if !late {
            self.last = Some(seq);
        }
        self.received += 1;
        self.lost = (self.lost + u64::from(lost)).saturating_sub(u64::from(late));
        Some((lost, late))
    }

    /// Lost as a percentage of received plus lost (`None` before any frame).
    pub fn loss_pct(&self) -> Option<f64> {
        let n = self.received + self.lost;
        (n > 0).then(|| self.lost as f64 * 100.0 / n as f64)
    }
}

/// Sequence tracking for one (system, component).
#[derive(Debug, Default)]
struct SeqTracker {
    count: SeqCount,
    /// Per-second buckets: (start, received, lost).
    buckets: std::collections::VecDeque<(Instant, u32, u32)>,
}

impl SeqTracker {
    fn on_frame(&mut self, seq: u8, now: Instant) {
        let Some((lost, late)) = self.count.on_seq(seq) else {
            return; // duplicate
        };
        match self.buckets.back_mut() {
            Some(b) if now.duration_since(b.0) < Duration::from_secs(1) => {
                b.1 += 1;
                b.2 = (b.2 + lost).saturating_sub(u32::from(late));
            }
            _ => self.buckets.push_back((now, 1, lost)),
        }
        while self
            .buckets
            .front()
            .is_some_and(|b| now.duration_since(b.0) > QUALITY_WINDOW)
        {
            self.buckets.pop_front();
        }
    }

    fn quality(&self, now: Instant) -> StreamQuality {
        let mut q = StreamQuality {
            received: self.count.received,
            lost: self.count.lost,
            ..Default::default()
        };
        let recent = self
            .buckets
            .iter()
            .filter(|b| now.duration_since(b.0) <= QUALITY_WINDOW);
        (q.recent_received, q.recent_lost) = recent.fold((0, 0), |(r, l), b| (r + b.1, l + b.2));
        q
    }
}

/// A change in mission progress.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionChange {
    /// The vehicle reported reaching this item (`MISSION_ITEM_REACHED`).
    ItemReached(u16),
    /// The item being executed changed.
    Current(u16),
    /// The mission state changed (e.g. to Active, Paused or Complete).
    State(MissionState),
}

/// Camera geotags kept per vehicle.
pub const GEOTAG_LIMIT: usize = 20_000;

/// One photo the autopilot reported taking (`CAMERA_FEEDBACK`, ArduPilot).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geotag {
    /// Host time of receipt.
    pub received: SystemTime,
    /// Image time from the autopilot, µs since the Unix epoch (0 if unknown).
    pub time_usec: u64,
    /// Camera and image index.
    pub cam_idx: u8,
    /// See `cam_idx`.
    pub img_idx: u16,
    /// Position, degrees.
    pub lat_deg: f64,
    /// See `lat_deg`.
    pub lon_deg: f64,
    /// Altitude above MSL and above home, m.
    pub alt_msl_m: f32,
    /// See `alt_msl_m`.
    pub alt_rel_m: f32,
    /// Camera attitude, degrees (earth frame; yaw true, 0–360).
    pub roll_deg: f32,
    /// See `roll_deg`.
    pub pitch_deg: f32,
    /// See `roll_deg`.
    pub yaw_deg: f32,
    /// `CAMERA_FEEDBACK_FLAGS` (0 photo, 1 video, 2 bad exposure, 3 closed loop, 4 open loop).
    pub flags: u8,
}

/// STATUSTEXT lines kept per vehicle (Mission Planner's "Messages" tab).
pub const MESSAGE_HISTORY: usize = 500;

/// One STATUSTEXT line as received.
#[derive(Debug, Clone, PartialEq)]
pub struct LoggedText {
    /// Host time of receipt.
    pub time: SystemTime,
    /// Sending component.
    pub compid: u8,
    /// Severity and text.
    pub text: StatusText,
}

/// Heartbeat-based link state of one vehicle (plan 08 §4.3).
///
/// Informational: it drives annunciation and command *delivery* (retransmission), never
/// whether a command is sent ([ADR-0009](../../docs/plan/adr/0009-link-state-never-blocks-commands.md)).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleLink {
    /// Heartbeat younger than [`CoreConfig::link_stale`].
    Connected,
    /// No heartbeat for `link_stale`..`link_lost`: instruments dim, values show their age.
    Stale,
    /// No heartbeat for longer than [`CoreConfig::link_lost`]: values are invalid.
    Lost,
}

impl fmt::Display for VehicleLink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            VehicleLink::Connected => "CONNECTED",
            VehicleLink::Stale => "STALE",
            VehicleLink::Lost => "LOST",
        })
    }
}

/// Allowed range for [`CoreConfig::link_stale`] ("within safe bounds", plan 08 §4.3).
pub const LINK_STALE_RANGE: (Duration, Duration) =
    (Duration::from_millis(200), Duration::from_secs(5));
/// Upper bound for [`CoreConfig::link_lost`]; it must also exceed `link_stale`.
pub const LINK_LOST_MAX: Duration = Duration::from_secs(30);
/// Allowed mode-change lifetime (ADR-0011: strict 2–3 s; an unanswered mode change is dropped
/// and the operator alerted, never left to execute late).
pub const MODE_LIFETIME_RANGE: (Duration, Duration) =
    (Duration::from_secs(2), Duration::from_secs(3));
/// Frames waiting for the `.tlog` writer before [`CoreEvent::Recorder`] warns (~15 MB).
pub const RECORDER_BACKLOG_WARN: usize = 50_000;

/// Classify a vehicle by the age of its last autopilot heartbeat.
pub fn vehicle_link(v: &VehicleState, now: Instant, cfg: &CoreConfig) -> VehicleLink {
    match &v.heartbeat {
        Some(hb) if hb.age(now) < cfg.link_stale => VehicleLink::Connected,
        Some(hb) if hb.age(now) < cfg.link_lost => VehicleLink::Stale,
        _ => VehicleLink::Lost,
    }
}

/// Core configuration.
#[derive(Debug, Clone)]
pub struct CoreConfig {
    /// Our MAVLink system id.
    pub sysid: u8,
    /// Our component id.
    pub compid: u8,
    /// Write every frame to a `.tlog` file at this path.
    pub tlog: Option<PathBuf>,
    /// Telemetry requested from each new vehicle: `(message id, rate Hz)`.
    pub stream_rates: Vec<(u32, f32)>,
    /// No heartbeat for this long: [`VehicleLink::Stale`] (default 1.5 s).
    pub link_stale: Duration,
    /// No heartbeat for this long: [`VehicleLink::Lost`] (default 5 s).
    pub link_lost: Duration,
    /// Command policy tuning (token lifetime, command lifetimes, retransmission).
    pub policy: PolicyConfig,
    /// Append the hash-chained command audit log here. An existing file is verified and
    /// continued; a broken chain refuses to start (tamper evidence).
    pub audit_log: Option<PathBuf>,
    /// Navigation-health thresholds (hazard H16; defaults mirror ArduPilot's EKF check).
    pub nav: NavThresholds,
    /// Opt-in vehicle-change settings at start (all off by default); changed at run time
    /// with [`Core::set_vehicle_change`].
    pub vehicle_changes: VehicleChangeSettings,
}

impl CoreConfig {
    /// Reject settings outside safe bounds.
    pub fn validate(&self) -> Result<(), String> {
        let (lo, hi) = LINK_STALE_RANGE;
        if !(lo..=hi).contains(&self.link_stale) {
            return Err(format!(
                "link_stale {:?} outside {lo:?}..={hi:?}",
                self.link_stale
            ));
        }
        if self.link_lost <= self.link_stale || self.link_lost > LINK_LOST_MAX {
            return Err(format!(
                "link_lost {:?} must be > link_stale and <= {LINK_LOST_MAX:?}",
                self.link_lost
            ));
        }
        let (lo, hi) = MODE_LIFETIME_RANGE;
        if !(lo..=hi).contains(&self.policy.lifetime_mode) {
            return Err(format!(
                "policy.lifetime_mode {:?} outside {lo:?}..={hi:?} (ADR-0011)",
                self.policy.lifetime_mode
            ));
        }
        if let Some((id, hz)) = self
            .stream_rates
            .iter()
            .find(|(_, hz)| !(hz.is_finite() && *hz > 0.0))
        {
            return Err(format!(
                "stream rate for message {id}: {hz} Hz is not a positive rate"
            ));
        }
        Ok(())
    }
}

impl Default for CoreConfig {
    fn default() -> Self {
        CoreConfig {
            sysid: 255,
            compid: 190,
            tlog: None,
            // ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD, SYS_STATUS, GPS_RAW_INT,
            // EXTENDED_SYS_STATE (landed / VTOL state), EKF_STATUS_REPORT (navigation health),
            // MISSION_CURRENT (mission progress), HOME_POSITION (sent on change anyway; the
            // slow interval covers a missed one)
            stream_rates: vec![
                (30, 10.0),
                (33, 5.0),
                (74, 5.0),
                (1, 2.0),
                (24, 2.0),
                (245, 2.0),
                (193, 2.0),
                (42, 1.0),
                (242, 0.2),
                // ADSB_VEHICLE: ArduPilot sends one aircraft per interval and nothing without
                // traffic (AP_ADSB::send_adsb_vehicle), so this costs nothing until there is some.
                (246, 5.0),
                // BATTERY_STATUS: ArduPilot sends one battery per interval, round robin.
                (147, 2.0),
                // VIBRATION, WIND
                (241, 2.0),
                (168, 1.0),
                // RC_CHANNELS, SERVO_OUTPUT_RAW (setup screens, motor checks; 1 Hz keeps slow
                // radios light)
                (65, 1.0),
                (36, 1.0),
                // DISTANCE_SENSOR (rangefinders; nothing is sent without one)
                (132, 2.0),
                // TERRAIN_REPORT
                (136, 1.0),
                // FENCE_STATUS (ArduPilot sends it only while a fence is enabled)
                (162, 1.0),
                // TERRAIN_REQUEST: ArduPilot schedules it like a stream message and sends one
                // only while it lacks terrain (AP_Terrain::send_request, at most every 2 s)
                (133, 1.0),
            ],
            link_stale: Duration::from_millis(1500),
            link_lost: Duration::from_secs(5),
            policy: PolicyConfig::default(),
            audit_log: None,
            nav: NavThresholds::default(),
            vehicle_changes: VehicleChangeSettings::default(),
        }
    }
}

/// Things worth telling the user about.
#[derive(Debug, Clone, PartialEq)]
pub enum CoreEvent {
    /// Mission progress changed (from `MISSION_CURRENT` / `MISSION_ITEM_REACHED`).
    Mission {
        /// System id.
        sysid: u8,
        /// What changed.
        change: MissionChange,
    },
    /// First heartbeat from a vehicle.
    VehicleSeen {
        /// System id.
        sysid: u8,
    },
    /// `STATUSTEXT`.
    StatusText {
        /// System id.
        sysid: u8,
        /// Text and severity.
        text: StatusText,
    },
    /// A vehicle's heartbeat link changed state (plan 08 §4.3). Raised on every
    /// transition, including back to `Connected`.
    VehicleLink {
        /// System id.
        sysid: u8,
        /// New state.
        state: VehicleLink,
        /// Age of the last heartbeat when the change was detected.
        heartbeat_age: Duration,
    },
    /// The `.tlog` recorder has a problem: a write failed (recording stopped) or it is
    /// falling behind.
    Recorder {
        /// True if recording stopped.
        stopped: bool,
        /// What happened.
        detail: String,
    },
    /// A command changed state (plan 08 §4.2, ADR-0009). `status.summary()` is the
    /// operator-facing line.
    Command(Box<CommandStatus>),
    /// Something the operator must be told (SR-05): e.g. a mode change that was not
    /// confirmed within its 2–3 s lifetime is a Warning; the UI shows it with audio.
    Alert {
        /// System id.
        sysid: u8,
        /// Level (Warning / Caution / Advisory).
        level: AlertLevel,
        /// Operator-facing text.
        text: String,
    },
    /// The annunciator says to sound an alert now (new, escalated or back after its quiet
    /// time; see [`annunciator`]). The UI plays the tone or speech for `level`.
    Sound {
        /// System id.
        sysid: u8,
        /// Annunciator id, e.g. `FENCE`.
        id: String,
        /// Level to sound.
        level: annunciator::AlertLevel,
    },
    /// A link went up/down.
    Link {
        /// Link id.
        link: LinkId,
        /// Up?
        up: bool,
        /// Detail.
        detail: String,
    },
}

/// Per aircraft (ICAO): the last threat level and its name, to alert on changes.
type TrafficSeen = HashMap<u32, (vehicle::Threat, String)>;

/// Outcome of [`Core::upload_mission`].
#[derive(Debug, Clone, PartialEq)]
pub struct MissionReport {
    /// The upload transfer itself (its final ACK can be lost even when the mission arrived).
    pub upload: TransferStatus,
    /// The read-back download.
    pub readback: TransferStatus,
    /// The mission as the vehicle holds it (item 0 is its home).
    pub on_vehicle: Vec<MissionItem>,
    /// Differences between the draft and the vehicle (item 0, home, not compared).
    pub mismatches: Vec<Mismatch>,
}

impl MissionReport {
    /// The vehicle holds exactly the uploaded mission.
    pub fn verified(&self) -> bool {
        self.readback == TransferStatus::Done && self.mismatches.is_empty()
    }
}

/// A decoded frame as seen by protocol drivers.
#[derive(Debug, Clone)]
pub struct Incoming {
    /// Sender system.
    pub sysid: u8,
    /// Sender component.
    pub compid: u8,
    /// Decoded message.
    pub msg: Message,
}

/// Parameter download failure.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamError {
    /// Indices never received.
    pub missing: Vec<u16>,
    /// Parameters that did arrive.
    pub partial: usize,
}

impl fmt::Display for ParamError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.partial == 0 {
            write!(
                f,
                "no PARAM_VALUE received (wrong target, link or SYSID_MYGCS?)"
            )
        } else {
            write!(
                f,
                "parameter download incomplete: {} received, {} missing",
                self.partial,
                self.missing.len()
            )
        }
    }
}

impl std::error::Error for ParamError {}

/// Open (or verify and continue) the audit log and write records on a dedicated thread, so
/// disk latency never blocks command delivery.
fn start_audit_writer(
    path: &std::path::Path,
    events: broadcast::Sender<CoreEvent>,
) -> io::Result<std::sync::mpsc::Sender<(SystemTime, AuditEvent)>> {
    let last = match std::fs::File::open(path) {
        Ok(f) => audit::last_hash(io::BufReader::new(f)).map_err(|b| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "audit log {} is broken at line {} ({}); move it aside to start a new one",
                    path.display(),
                    b.line,
                    b.reason
                ),
            )
        })?,
        Err(e) if e.kind() == io::ErrorKind::NotFound => audit::GENESIS.to_string(),
        Err(e) => return Err(e),
    };
    let file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)?;
    let path = path.to_path_buf();
    let (tx, rx) = std::sync::mpsc::channel::<(SystemTime, AuditEvent)>();
    std::thread::Builder::new()
        .name("audit-log".into())
        .spawn(move || {
            let mut log = AuditLog::resume(file, last);
            for (wall, e) in rx {
                let us = recorder::unix_micros(wall);
                if let Err(err) = log.append(us, &e) {
                    let _ = events.send(CoreEvent::Recorder {
                        stopped: true,
                        detail: format!(
                            "audit log {}: {err}; command auditing stopped",
                            path.display()
                        ),
                    });
                    return;
                }
            }
        })?;
    Ok(tx)
}

/// The headless core. Cheap to clone; all clones share state.
#[derive(Clone)]
pub struct Core {
    cfg: Arc<CoreConfig>,
    hub: HubHandle,
    ids: LinkIds,
    seq: Arc<AtomicU8>,
    fleet: Arc<ArcSwap<Fleet>>,
    incoming: broadcast::Sender<Arc<Incoming>>,
    events: broadcast::Sender<CoreEvent>,
    policy: Arc<std::sync::Mutex<CommandPolicy>>,
    audit: Option<std::sync::mpsc::Sender<(SystemTime, AuditEvent)>>,
    /// Last navigation trust per vehicle, to alert on changes.
    nav_trust: Arc<std::sync::Mutex<HashMap<u8, PositionTrust>>>,
    /// Terrain served to vehicles that ask (`TERRAIN_REQUEST`); none by default.
    terrain: Arc<std::sync::Mutex<Option<Arc<protocols::terrain::Terrain>>>>,
    /// Terrain sub-grids sent, and requested ones not covered by the terrain.
    terrain_counts: Arc<(std::sync::atomic::AtomicU64, std::sync::atomic::AtomicU64)>,
    /// Annunciator per vehicle, fed by every [`CoreEvent::Alert`].
    annunciators: Arc<std::sync::Mutex<HashMap<u8, annunciator::Annunciator>>>,
    /// Last traffic threat per vehicle and aircraft (ICAO), with its name for "clear".
    traffic_seen: Arc<std::sync::Mutex<HashMap<u8, TrafficSeen>>>,
    /// Last fence state per vehicle (outside, breach count), to alert on breaches.
    fence_seen: Arc<std::sync::Mutex<HashMap<u8, (bool, u16)>>>,
    /// Recent STATUSTEXT per vehicle (kept outside the fleet, which is cloned per message).
    messages: Arc<std::sync::Mutex<HashMap<u8, std::collections::VecDeque<LoggedText>>>>,
    /// MAVLink sequence tracking per (system, component).
    sequences: Arc<std::sync::Mutex<HashMap<(u8, u8), SeqTracker>>>,
    /// Camera geotags per vehicle (outside the fleet for the same reason).
    geotags: Arc<std::sync::Mutex<HashMap<u8, Vec<Geotag>>>>,
    parameters: Arc<parameters::ParameterState>,
    /// Held across opening a link, so concurrent connects cannot both pass the check.
    link_mode: Arc<tokio::sync::Mutex<LinkMode>>,
    /// Opt-in vehicle-change settings in force.
    vehicle_changes: Arc<std::sync::Mutex<VehicleChangeSettings>>,
}

impl Core {
    /// Start the core on the current tokio runtime.
    pub async fn start(cfg: CoreConfig) -> io::Result<Core> {
        cfg.validate()
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
        let router = RouterConfig {
            sysid: cfg.sysid,
            compid: cfg.compid,
            ..Default::default()
        };
        let (hub, outputs) = spawn_hub(router, LinkPolicy::relay_all());
        let (incoming, _) = broadcast::channel(4096);
        let (events, _) = broadcast::channel(1024);
        let audit = match &cfg.audit_log {
            Some(path) => Some(start_audit_writer(path, events.clone())?),
            None => None,
        };
        let policy = Arc::new(std::sync::Mutex::new(CommandPolicy::new(
            cfg.policy.clone(),
        )));
        let vehicle_changes = Arc::new(std::sync::Mutex::new(cfg.vehicle_changes));
        let core = Core {
            cfg: Arc::new(cfg),
            hub,
            ids: LinkIds::new(),
            seq: Arc::default(),
            fleet: Arc::new(ArcSwap::from_pointee(Fleet::new())),
            incoming,
            events,
            policy,
            audit,
            nav_trust: Arc::default(),
            fence_seen: Arc::default(),
            traffic_seen: Arc::default(),
            annunciators: Arc::default(),
            terrain: Arc::default(),
            terrain_counts: Arc::default(),
            messages: Arc::default(),
            geotags: Arc::default(),
            sequences: Arc::default(),
            parameters: Arc::default(),
            link_mode: Arc::default(),
            vehicle_changes,
        };

        // Recorder: lossless, on a blocking thread so disk latency never touches routing.
        let mut tap = outputs.tap;
        if let Some(path) = core.cfg.tlog.clone() {
            let file = std::fs::File::create(&path)?;
            let events = core.events.clone();
            tokio::task::spawn_blocking(move || {
                let stop = |e: io::Error| {
                    let _ = events.send(CoreEvent::Recorder {
                        stopped: true,
                        detail: format!("{}: {e}; recording stopped", path.display()),
                    });
                };
                let mut w = recorder::TlogWriter::new(file);
                let mut last_flush = Instant::now();
                let mut warned = false;
                while let Some(t) = tap.blocking_recv() {
                    if let Err(e) = w.write(t.time, &t.frame) {
                        return stop(e);
                    }
                    if last_flush.elapsed() >= Duration::from_secs(1) {
                        if let Err(e) = w.flush() {
                            return stop(e);
                        }
                        last_flush = Instant::now();
                    }
                    let backlog = tap.len();
                    if backlog > RECORDER_BACKLOG_WARN && !warned {
                        warned = true;
                        let _ = events.send(CoreEvent::Recorder {
                            stopped: false,
                            detail: format!("{backlog} frames waiting to be written (slow disk?)"),
                        });
                    } else if backlog < RECORDER_BACKLOG_WARN / 2 {
                        warned = false;
                    }
                }
                if let Err(e) = w.flush() {
                    stop(e);
                }
            });
        } else {
            tokio::spawn(async move { while tap.recv().await.is_some() {} });
        }

        // Dispatcher: decode, update the fleet snapshot, publish to protocol drivers. Link
        // notifications come first so a telemetry backlog never delays a "link down".
        let c = core.clone();
        let mut app = outputs.app;
        let mut links = outputs.links;
        tokio::spawn(async move {
            loop {
                let out = tokio::select! {
                    biased;
                    Some(n) = links.recv() => n,
                    Some(f) = app.recv() => f,
                    else => break,
                };
                match out {
                    HubOutput::Frame { frame, .. } => c.dispatch(&frame).await,
                    HubOutput::LinkStatus { link, up, detail } => {
                        let _ = c.events.send(CoreEvent::Link { link, up, detail });
                    }
                    HubOutput::LinkAdded { .. } | HubOutput::LinkRemoved(_) => {}
                }
            }
        });

        // Link watchdog: heartbeat age -> CONNECTED / STALE / LOST, events on transitions.
        // Sampling every 100 ms could miss a state that lasted less than that (a LOST that
        // ends just after the threshold), so the gap between consecutive heartbeats is also
        // checked and any skipped states are reported in order before the recovery.
        let c = core.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_millis(100));
            let mut last: BTreeMap<u8, (VehicleLink, Instant)> = BTreeMap::new();
            loop {
                tick.tick().await;
                let now = Instant::now();
                for v in c.fleet().vehicles() {
                    let Some(hb) = v.heartbeat.as_ref() else {
                        continue;
                    };
                    let state = vehicle_link(v, now, &c.cfg);
                    let mut report = Vec::new();
                    if let Some(&(prev, prev_at)) = last.get(&v.sysid) {
                        let gap = hb.at.saturating_duration_since(prev_at);
                        if hb.at != prev_at {
                            // A new heartbeat ended a gap: states the sampler skipped.
                            if prev == VehicleLink::Connected && gap >= c.cfg.link_stale {
                                report.push((VehicleLink::Stale, gap));
                            }
                            if prev != VehicleLink::Lost && gap >= c.cfg.link_lost {
                                report.push((VehicleLink::Lost, gap));
                            }
                        }
                        let shown = report.last().map_or(prev, |r| r.0);
                        if shown != state {
                            report.push((state, hb.age(now)));
                        }
                    }
                    last.insert(v.sysid, (state, hb.at));
                    for (state, heartbeat_age) in report {
                        let ev = CoreEvent::VehicleLink {
                            sysid: v.sysid,
                            state,
                            heartbeat_age,
                        };
                        let _ = c.events.send(ev);
                    }
                }
            }
        });

        // Command delivery (ADR-0009): retransmit, flood, expire; 50 ms resolution.
        let c = core.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_millis(50));
            loop {
                tick.tick().await;
                let now = Instant::now();
                let (out, changed) = {
                    let mut p = c.policy();
                    p.poll(now, |sysid| c.link_info(sysid, now))
                };
                c.deliver(out, changed).await;
            }
        });

        // GCS heartbeat, 1 Hz: tells vehicles a GCS is present (GCS failsafe).
        let c = core.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(1));
            loop {
                tick.tick().await;
                let hb = Message::Heartbeat(Heartbeat {
                    r#type: MavType::GCS.0,
                    autopilot: MavAutopilot::INVALID.0,
                    base_mode: 0,
                    custom_mode: 0,
                    system_status: MavState::ACTIVE.0,
                    mavlink_version: 3,
                });
                if !c.send(&hb).await {
                    break;
                }
            }
        });
        Ok(core)
    }

    async fn dispatch(&self, frame: &Frame) {
        let (sysid, compid) = (frame.sysid(), frame.compid());
        let now = Instant::now();
        self.sequences
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .entry((sysid, compid))
            .or_default()
            .on_frame(frame.seq(), now);
        let Ok(msg) = frame.decode() else { return };
        self.observe_parameter_safety(sysid, compid, &msg, now);
        let first_heartbeat = matches!(msg, Message::Heartbeat(ref h) if h.r#type != MavType::GCS.0)
            && self
                .fleet
                .load()
                .get(sysid)
                .is_none_or(|v| v.heartbeat.is_none());
        let mission_before = self
            .fleet
            .load()
            .get(sysid)
            .and_then(|v| v.mission.as_ref().map(|m| (m.value.current, m.value.state)));
        let mut text = None;
        self.fleet.rcu(|f| {
            let mut f = Fleet::clone(f);
            text = f.apply(sysid, compid, &msg, now);
            f
        });
        // Commands: acknowledgements and state-based verification.
        let changed = match &msg {
            Message::CommandAck(ack) => {
                let facts = self.facts(sysid);
                self.policy()
                    .on_ack(Target { sysid, compid }, ack, &facts, now)
            }
            Message::Heartbeat(_) if compid == vehicle::AUTOPILOT_COMPID => {
                let facts = self.facts(sysid);
                self.policy().on_state(sysid, &facts, now)
            }
            _ => Vec::new(),
        };
        if !changed.is_empty() {
            self.deliver(Vec::new(), changed).await;
        }
        if let Message::TerrainRequest(r) = &msg
            && compid == vehicle::AUTOPILOT_COMPID
        {
            self.answer_terrain(r);
        }
        if matches!(msg, Message::AdsbVehicle(_)) {
            self.check_traffic(sysid);
        }
        if compid == vehicle::AUTOPILOT_COMPID && matches!(msg, Message::FenceStatus(_)) {
            self.check_fence(sysid);
        }
        if compid == vehicle::AUTOPILOT_COMPID
            && matches!(msg, Message::EkfStatusReport(_) | Message::GpsRawInt(_))
        {
            self.check_nav(sysid, now);
        }
        if first_heartbeat && compid == vehicle::AUTOPILOT_COMPID {
            let _ = self.events.send(CoreEvent::VehicleSeen { sysid });
            self.request_streams(sysid, compid).await;
            self.request_banner(sysid, compid);
            self.request_version(sysid, compid);
        }
        if let Message::CameraFeedback(c) = &msg {
            self.record_geotag(sysid, c);
        }
        if compid == vehicle::AUTOPILOT_COMPID {
            let mission_event = |change| {
                let _ = self.events.send(CoreEvent::Mission { sysid, change });
            };
            match &msg {
                Message::MissionItemReached(r) => mission_event(MissionChange::ItemReached(r.seq)),
                Message::MissionCurrent(_) => {
                    let now_m = self
                        .fleet()
                        .get(sysid)
                        .and_then(|v| v.mission.as_ref().map(|m| (m.value.current, m.value.state)));
                    if let Some((cur, state)) = now_m {
                        if mission_before.map(|b| b.0) != Some(cur) {
                            mission_event(MissionChange::Current(cur));
                        }
                        if mission_before.map(|b| b.1) != Some(state) {
                            mission_event(MissionChange::State(state));
                        }
                    }
                }
                _ => {}
            }
        }
        if let Some(text) = text {
            // The firmware's version banner confirms it: from now on mode changes are named.
            if compid == vehicle::AUTOPILOT_COMPID
                && vehicle::Firmware::from_banner(&text.text).is_some()
                && let Some(fw) = self.fleet().get(sysid).and_then(|v| v.confirmed_firmware())
            {
                self.policy().set_mode_names(
                    sysid,
                    fw.modes().into_iter().map(|(n, m)| (n, m.to_string())),
                );
            }
            {
                let mut m = self.messages.lock().unwrap_or_else(|e| e.into_inner());
                let q = m.entry(sysid).or_default();
                if q.len() == MESSAGE_HISTORY {
                    q.pop_front();
                }
                q.push_back(LoggedText {
                    time: SystemTime::now(),
                    compid,
                    text: text.clone(),
                });
            }
            let _ = self.events.send(CoreEvent::StatusText { sysid, text });
        }
        let _ = self
            .incoming
            .send(Arc::new(Incoming { sysid, compid, msg }));
    }

    /// Configuration in use.
    pub fn config(&self) -> &CoreConfig {
        &self.cfg
    }

    fn policy(&self) -> std::sync::MutexGuard<'_, CommandPolicy> {
        // A panic while holding the lock cannot leave the policy half-updated in a way that
        // matters more than losing command control, so keep going.
        self.policy.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Last-known vehicle state, as the command policy sees it.
    pub fn facts(&self, sysid: u8) -> VehicleFacts {
        let fleet = self.fleet();
        let v = fleet.get(sysid);
        let hb = v.and_then(|v| v.heartbeat.as_ref());
        let now = Instant::now();
        VehicleFacts {
            armed: hb.map(|h| h.value.armed()),
            custom_mode: hb.map(|h| h.value.custom_mode),
            landed_state: v
                .and_then(|v| v.extended.as_ref())
                .map(|e| (e.value.landed.to_mavlink(), e.age(now))),
        }
    }

    /// Link state and heartbeat age of `sysid` for the command policy. A vehicle never heard
    /// is LOST with no heartbeat; commands are still allowed (ADR-0009).
    pub fn link_info(&self, sysid: u8, now: Instant) -> LinkInfo {
        let fleet = self.fleet();
        let v = fleet.get(sysid).filter(|v| v.heartbeat.is_some());
        let state = match v.map(|v| vehicle_link(v, now, &self.cfg)) {
            Some(VehicleLink::Connected) => LinkState::Connected,
            Some(VehicleLink::Stale) => LinkState::Stale,
            Some(VehicleLink::Lost) | None => LinkState::Lost,
        };
        let heartbeat_age = v.and_then(|v| v.heartbeat.as_ref()).map(|h| h.age(now));
        LinkInfo {
            state,
            heartbeat_age,
        }
    }

    fn target(sysid: u8) -> Target {
        Target {
            sysid,
            compid: vehicle::AUTOPILOT_COMPID,
        }
    }

    /// Name shown in confirmation cards and required as the class D typed phrase.
    pub fn set_vehicle_name(&self, sysid: u8, name: &str) {
        self.policy().set_vehicle_name(sysid, name);
    }

    /// Step 1 of a command: validate, check lockouts, and for class C/D get the confirmation
    /// card and token. Never refused because of the link (ADR-0009).
    pub async fn request_command(&self, sysid: u8, command: Command) -> Result<Request, Refusal> {
        let now = Instant::now();
        let (facts, link) = (self.facts(sysid), self.link_info(sysid, now));
        let simplified = self.vehicle_changes().simplified();
        let r = {
            let mut p = self.policy();
            p.set_simplified(simplified);
            p.request(Self::target(sysid), command, &facts, link, now)
        };
        self.deliver(Vec::new(), Vec::new()).await;
        r
    }

    /// Step 2: submit with the operator's approval; transmission starts at once.
    pub async fn submit_command(
        &self,
        sysid: u8,
        command: Command,
        approval: Approval,
    ) -> Result<CommandId, Refusal> {
        let now = Instant::now();
        let (facts, link) = (self.facts(sysid), self.link_info(sysid, now));
        let simplified = self.vehicle_changes().simplified();
        let r = {
            let mut p = self.policy();
            p.set_simplified(simplified);
            p.submit(Self::target(sysid), command, approval, &facts, link, now)
        };
        match r {
            Ok((id, out, changed)) => {
                self.deliver(out, changed).await;
                Ok(id)
            }
            Err(e) => {
                self.deliver(Vec::new(), Vec::new()).await;
                Err(e)
            }
        }
    }

    /// Stop retransmitting `id`. The vehicle may already have acted on it.
    pub async fn cancel_command(&self, id: CommandId) -> bool {
        let s = self.policy().cancel(id, Instant::now());
        let found = s.is_some();
        self.deliver(Vec::new(), s.into_iter().collect()).await;
        found
    }

    /// Current status of `id`.
    pub fn command_status(&self, id: CommandId) -> Option<CommandStatus> {
        self.policy().status(id, Instant::now())
    }

    /// Write one record to the audit log (if configured), e.g. a parameter change.
    pub(crate) fn audit_note(&self, sysid: u8, text: String) {
        if let Some(tx) = &self.audit {
            let _ = tx.send((
                SystemTime::now(),
                AuditEvent {
                    at: Instant::now(),
                    sysid,
                    id: None,
                    text,
                },
            ));
        }
    }

    /// Transmit, publish status changes and write audit records.
    async fn deliver(&self, out: Vec<protocols::command::Outgoing>, changed: Vec<CommandStatus>) {
        for o in out {
            let seq = self.seq.fetch_add(1, Ordering::Relaxed);
            let header = Header {
                seq,
                sysid: self.cfg.sysid,
                compid: self.cfg.compid,
            };
            if let Ok(frame) = encode(Version::V2, header, &o.msg) {
                if o.flood {
                    self.hub.send_all(frame).await;
                } else {
                    self.hub.send(frame).await;
                }
            }
        }
        for s in changed {
            if let Some(a) = s.alert() {
                self.alert(s.target.sysid, a.level, a.text);
            }
            let _ = self.events.send(CoreEvent::Command(Box::new(s)));
        }
        let events = self.policy().drain_audit();
        if let Some(tx) = &self.audit {
            // Instant -> wall clock for the record.
            let (now_i, now_w) = (Instant::now(), SystemTime::now());
            for e in events {
                let wall = now_w - now_i.saturating_duration_since(e.at);
                let _ = tx.send((wall, e));
            }
        }
    }

    /// Open a link.
    ///
    /// A `.tlog` replay must be the only link: it is refused once any other link was opened,
    /// and every other link is refused while a replay is open.
    pub async fn connect(&self, spec: &LinkSpec) -> io::Result<LinkId> {
        let want = if matches!(spec, LinkSpec::Tlog { .. }) {
            LinkMode::Replay
        } else {
            LinkMode::Live
        };
        let mut mode = self.link_mode.lock().await;
        match (*mode, want) {
            (LinkMode::None, _) | (LinkMode::Live, LinkMode::Live) => {}
            (LinkMode::Replay, _) | (LinkMode::Live, _) => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "a .tlog replay must be the core's only link: use a separate core \
                     (dc-cli process) for replay and for live links",
                ));
            }
        }
        let id = self.open_link(spec).await?;
        // Recorded only once the link is open: a replay file that does not exist must not
        // lock the core out of live links.
        *mode = want;
        Ok(id)
    }

    async fn open_link(&self, spec: &LinkSpec) -> io::Result<LinkId> {
        let ev = self.hub.events.clone();
        Ok(match spec {
            LinkSpec::Tlog { path, speed } => {
                replay::spawn_replay(self.ids.next(), path, *speed, ev)
                    .await?
                    .id
            }
            LinkSpec::Tcp(a) => spawn_tcp_client(self.ids.next(), *a, ev).await.id,
            LinkSpec::TcpIn(a) => {
                spawn_tcp_server(self.ids.clone(), *a, ev).await?;
                LinkId(0)
            }
            LinkSpec::Udp(a) => {
                spawn_udp(self.ids.next(), UdpMode::Server { bind: *a }, ev)
                    .await?
                    .0
                    .id
            }
            LinkSpec::UdpOut(a) => {
                spawn_udp(self.ids.next(), UdpMode::Client { remote: *a }, ev)
                    .await?
                    .0
                    .id
            }
            LinkSpec::Serial { path, baud } => {
                link_serial::spawn_serial(self.ids.next(), path, *baud, ev)
                    .await?
                    .id
            }
        })
    }

    /// Links, routes and router counters (`None` once the hub has stopped).
    pub async fn hub_snapshot(&self) -> Option<HubSnapshot> {
        self.hub.snapshot().await
    }

    /// Send a message from us (MAVLink 2, our ids, next sequence number).
    pub async fn send(&self, msg: &Message) -> bool {
        let seq = self.seq.fetch_add(1, Ordering::Relaxed);
        let header = Header {
            seq,
            sysid: self.cfg.sysid,
            compid: self.cfg.compid,
        };
        match encode(Version::V2, header, msg) {
            Ok(frame) => self.hub.send(frame).await,
            Err(_) => false,
        }
    }

    /// Latest vehicle state (wait-free snapshot).
    pub fn fleet(&self) -> Arc<Fleet> {
        self.fleet.load_full()
    }

    /// Heartbeat link state of `sysid` now, or `None` if it never sent an autopilot heartbeat.
    pub fn vehicle_link(&self, sysid: u8) -> Option<VehicleLink> {
        let fleet = self.fleet();
        let v = fleet.get(sysid).filter(|v| v.heartbeat.is_some())?;
        Some(vehicle_link(v, Instant::now(), &self.cfg))
    }

    /// Frames received and lost from each component of `sysid`, by component id.
    pub fn stream_quality(&self, sysid: u8) -> Vec<(u8, StreamQuality)> {
        let now = Instant::now();
        self.sequences
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .iter()
            .filter(|((s, _), _)| *s == sysid)
            .map(|((_, c), t)| (*c, t.quality(now)))
            .collect()
    }

    fn record_geotag(&self, sysid: u8, c: &mavlink_codec::messages::CameraFeedback) {
        let g = Geotag {
            received: SystemTime::now(),
            time_usec: c.time_usec,
            cam_idx: c.cam_idx,
            img_idx: c.img_idx,
            lat_deg: f64::from(c.lat) * 1e-7,
            lon_deg: f64::from(c.lng) * 1e-7,
            alt_msl_m: c.alt_msl,
            alt_rel_m: c.alt_rel,
            roll_deg: c.roll,
            pitch_deg: c.pitch,
            yaw_deg: c.yaw,
            flags: c.flags,
        };
        let mut all = self.geotags.lock().unwrap_or_else(|e| e.into_inner());
        let list = all.entry(sysid).or_default();
        // A repeated report of the same image replaces the earlier one.
        if let Some(old) = list
            .iter_mut()
            .rev()
            .find(|o| (o.cam_idx, o.img_idx) == (g.cam_idx, g.img_idx))
        {
            *old = g;
        } else if list.len() < GEOTAG_LIMIT {
            list.push(g);
        }
    }

    /// Photos `sysid` reported (CAMERA_FEEDBACK), in order, at most [`GEOTAG_LIMIT`].
    pub fn geotags(&self, sysid: u8) -> Vec<Geotag> {
        self.geotags
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&sysid)
            .cloned()
            .unwrap_or_default()
    }

    /// The last [`MESSAGE_HISTORY`] STATUSTEXT lines from `sysid`, oldest first.
    pub fn messages(&self, sysid: u8) -> Vec<LoggedText> {
        self.messages
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&sysid)
            .map(|q| q.iter().cloned().collect())
            .unwrap_or_default()
    }

    /// Decoded messages as they arrive (for protocol drivers and tools).
    pub fn subscribe(&self) -> broadcast::Receiver<Arc<Incoming>> {
        self.incoming.subscribe()
    }

    /// User-facing events.
    pub fn events(&self) -> broadcast::Receiver<CoreEvent> {
        self.events.subscribe()
    }

    /// Wait for the first vehicle (autopilot heartbeat); returns its system id.
    pub async fn wait_for_vehicle(&self, timeout: Duration) -> Option<u8> {
        let mut rx = self.events.subscribe();
        let pick = |f: &Fleet| {
            f.vehicles()
                .find(|v| v.heartbeat.is_some())
                .map(|v| v.sysid)
        };
        if let Some(id) = pick(&self.fleet()) {
            return Some(id);
        }
        tokio::time::timeout(timeout, async {
            loop {
                match rx.recv().await {
                    Ok(CoreEvent::VehicleSeen { sysid }) => return Some(sysid),
                    Ok(_) | Err(broadcast::error::RecvError::Lagged(_)) => {
                        if let Some(id) = pick(&self.fleet()) {
                            return Some(id);
                        }
                    }
                    Err(broadcast::error::RecvError::Closed) => return None,
                }
            }
        })
        .await
        .ok()
        .flatten()
    }

    /// Navigation health of `sysid` now (hazard H16): whether the displayed position can be
    /// trusted, from the autopilot's own EKF report.
    pub fn nav_health(&self, sysid: u8) -> NavHealth {
        match self.fleet().get(sysid) {
            Some(v) => vehicle::nav::assess(v, &self.nav_thresholds(sysid), Instant::now()),
            None => NavHealth {
                trust: PositionTrust::Unknown,
                reasons: vec!["vehicle not seen"],
            },
        }
    }

    /// Answer vehicles' terrain requests from `terrain` (`None`: stop). Off by default: a
    /// vehicle with terrain enabled (`TERRAIN_ENABLE`) asks the GCS for the heights it lacks,
    /// in 4×4 sub-grids, and files what it gets; only sub-grids the terrain fully covers are
    /// sent, as whole metres above mean sea level.
    pub fn serve_terrain(&self, terrain: Option<protocols::terrain::Terrain>) {
        *self.terrain.lock().unwrap_or_else(|e| e.into_inner()) = terrain.map(Arc::new);
    }

    /// Terrain sub-grids sent so far, and requested ones the terrain did not cover.
    pub fn terrain_served(&self) -> (u64, u64) {
        (
            self.terrain_counts.0.load(Ordering::Relaxed),
            self.terrain_counts.1.load(Ordering::Relaxed),
        )
    }

    fn answer_terrain(&self, r: &mavlink_codec::messages::TerrainRequest) {
        let Some(t) = self
            .terrain
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
        else {
            return;
        };
        let (lat, lon, spacing, mask) = (r.lat, r.lon, r.grid_spacing, r.mask);
        let core = self.clone();
        tokio::spawn(async move {
            for gridbit in 0..56u8 {
                if mask & (1u64 << gridbit) == 0 {
                    continue;
                }
                match protocols::terrain::terrain_subgrid(&t, lat, lon, spacing, gridbit) {
                    Some(data) => {
                        core.send(&Message::TerrainData(
                            mavlink_codec::messages::TerrainData {
                                lat,
                                lon,
                                grid_spacing: spacing,
                                data,
                                gridbit,
                            },
                        ))
                        .await;
                        core.terrain_counts.0.fetch_add(1, Ordering::Relaxed);
                    }
                    None => {
                        core.terrain_counts.1.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        });
    }

    /// The automatic pre-flight checklist for `sysid` now (advice only; see
    /// [`vehicle::preflight`]).
    pub fn preflight(&self, sysid: u8) -> Vec<vehicle::preflight::Item> {
        let now = Instant::now();
        match self.fleet().get(sysid) {
            Some(v) => vehicle::preflight::checklist(
                v,
                &vehicle::nav::assess(v, &self.nav_thresholds(sysid), now),
                now,
            ),
            None => Vec::new(),
        }
    }

    /// Alert on traffic conflicts (closest approach within the look-ahead, see
    /// [`vehicle::Traffic::approach`]): a Low threat is a Caution, High a Warning, each time an
    /// aircraft's threat rises; "Traffic clear" when it is no longer one or is forgotten.
    fn check_traffic(&self, sysid: u8) {
        let Some(v) = self.fleet().get(sysid).cloned() else {
            return;
        };
        let Some(own) = v.position.as_ref().map(|p| p.value) else {
            return;
        };
        let now: HashMap<u32, (vehicle::Threat, String, String)> = v
            .traffic
            .values()
            .map(|t| {
                let t = &t.value;
                let a = t.approach(&own);
                let rel = t.relative_to(&own);
                let who = t
                    .callsign
                    .clone()
                    .unwrap_or_else(|| format!("{:06X}", t.icao));
                let dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
                let dir = dirs[((rel.bearing_deg + 22.5) / 45.0) as usize % 8];
                let text = format!(
                    "TRAFFIC {who}: {:.0} m {dir}{}, closest {:.0} m in {:.0} s{}",
                    rel.distance_m,
                    rel.above_m.map_or(String::new(), |h| format!(" {h:+.0} m")),
                    a.miss_xy_m,
                    a.time_s,
                    if t.simulated { " (simulated)" } else { "" }
                );
                (t.icao, (a.threat, who, text))
            })
            .collect();
        let mut alerts = Vec::new();
        {
            let mut seen = self.traffic_seen.lock().unwrap_or_else(|e| e.into_inner());
            let before = seen.entry(sysid).or_default();
            for (icao, (threat, who, text)) in &now {
                let was = before.get(icao).map(|(t, _)| *t);
                match threat {
                    vehicle::Threat::High if was < Some(vehicle::Threat::High) => {
                        alerts.push((AlertLevel::Warning, text.clone()));
                    }
                    vehicle::Threat::Low if was < Some(vehicle::Threat::Low) => {
                        alerts.push((AlertLevel::Caution, text.clone()));
                    }
                    vehicle::Threat::None if was.is_some_and(|w| w > vehicle::Threat::None) => {
                        alerts.push((AlertLevel::Advisory, format!("Traffic clear: {who}")));
                    }
                    _ => {}
                }
            }
            for (icao, (was, who)) in before.iter() {
                if *was > vehicle::Threat::None && !now.contains_key(icao) {
                    alerts.push((AlertLevel::Advisory, format!("Traffic clear: {who}")));
                }
            }
            *before = now
                .into_iter()
                .map(|(icao, (t, who, _))| (icao, (t, who)))
                .collect();
        }
        for (level, text) in alerts {
            self.alert(sysid, level, text);
        }
    }

    /// Alert on a geofence breach (Warning: a new breach, or another one counted while still
    /// outside) and when the vehicle is back inside (Advisory).
    fn check_fence(&self, sysid: u8) {
        let Some(f) = self
            .fleet()
            .get(sysid)
            .and_then(|v| v.fence.as_ref())
            .map(|f| f.value)
        else {
            return;
        };
        let before = self
            .fence_seen
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert(sysid, (f.breached, f.breach_count));
        let (level, text) = match before {
            _ if f.breached && before.is_none_or(|(was, count)| !was || f.breach_count > count) => {
                (
                    AlertLevel::Warning,
                    format!("FENCE BREACH: {}", f.breach_type.label()),
                )
            }
            Some((true, _)) if !f.breached => {
                (AlertLevel::Advisory, "Back inside the fence".to_string())
            }
            _ => return,
        };
        self.alert(sysid, level, text);
    }

    /// Tell the operator: a [`CoreEvent::Alert`], and the vehicle's annunciator updated (with
    /// a [`CoreEvent::Sound`] when it says to sound).
    fn alert(&self, sysid: u8, level: AlertLevel, text: String) {
        use annunciator::{AlertLevel as A, Effect, classify};
        let shown = match level {
            AlertLevel::Advisory => A::Advisory,
            AlertLevel::Caution => A::Caution,
            AlertLevel::Warning => A::Warning,
        };
        let now = Instant::now();
        let sound = {
            let mut all = self.annunciators.lock().unwrap_or_else(|e| e.into_inner());
            let ann = all.entry(sysid).or_default();
            match classify(&text) {
                Effect::Condition(id) => ann.raise(&id, shown, &text, now),
                Effect::Event(id) => {
                    let s = ann.raise(&id, shown, &text, now);
                    ann.clear(&id);
                    s
                }
                Effect::Clear(id) => {
                    ann.clear(&id);
                    None
                }
            }
        };
        let _ = self.events.send(CoreEvent::Alert { sysid, level, text });
        if let Some(s) = sound {
            let _ = self.events.send(CoreEvent::Sound {
                sysid,
                id: s.id,
                level: s.level,
            });
        }
    }

    /// The alerts `sysid`'s annunciator shows, highest level first, then oldest first.
    pub fn annunciator(&self, sysid: u8) -> Vec<annunciator::Alert> {
        self.annunciators
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&sysid)
            .map(|a| a.shown().into_iter().cloned().collect())
            .unwrap_or_default()
    }

    /// The master light for `sysid`: the highest unacknowledged warning or caution.
    pub fn master_alert(&self, sysid: u8) -> Option<annunciator::AlertLevel> {
        self.annunciators
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&sysid)
            .and_then(annunciator::Annunciator::master)
    }

    /// The operator acknowledges alert `id` of `sysid` (`None`: all of them).
    pub fn ack_alert(&self, sysid: u8, id: Option<&str>) {
        if let Some(a) = self
            .annunciators
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get_mut(&sysid)
        {
            match id {
                Some(id) => a.ack(id),
                None => a.ack_all(),
            }
        }
    }

    /// Alert when the navigation trust of `sysid` changes: Untrusted is a Warning, Degraded
    /// a Caution, back to Good an Advisory.
    fn check_nav(&self, sysid: u8, now: Instant) {
        let Some(health) = self
            .fleet()
            .get(sysid)
            .map(|v| vehicle::nav::assess(v, &self.nav_thresholds(sysid), now))
        else {
            return;
        };
        let before = self
            .nav_trust
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert(sysid, health.trust);
        if before == Some(health.trust) || health.trust == PositionTrust::Unknown {
            return;
        }
        let why = health.reasons.join(", ");
        let (level, text) = match health.trust {
            PositionTrust::Untrusted => (AlertLevel::Warning, format!("POSITION UNTRUSTED: {why}")),
            PositionTrust::Degraded => (AlertLevel::Caution, format!("POSITION DEGRADED: {why}")),
            PositionTrust::Good if before.is_some_and(|b| b != PositionTrust::Unknown) => {
                (AlertLevel::Advisory, "Position good again".to_string())
            }
            _ => return,
        };
        self.alert(sysid, level, text);
    }

    /// Ask the autopilot for its version banner (`MAV_CMD_DO_SEND_BANNER`), which confirms the
    /// firmware and so the meaning of its mode numbers (`vehicle::modes`). Retried every 3 s,
    /// up to 5 times, until a banner arrives.
    /// Ask for AUTOPILOT_VERSION (MAV_CMD_REQUEST_MESSAGE, read-only) until it arrives,
    /// up to 5 times.
    fn request_version(&self, sysid: u8, compid: u8) {
        let core = self.clone();
        tokio::spawn(async move {
            for _ in 0..5 {
                if core
                    .fleet()
                    .get(sysid)
                    .is_some_and(|v| v.autopilot_version.is_some())
                {
                    return;
                }
                let cmd = Message::CommandLong(CommandLong {
                    target_system: sysid,
                    target_component: compid,
                    command: MavCmd::REQUEST_MESSAGE.0,
                    param1: 148.0,
                    ..Default::default()
                });
                if !core.send(&cmd).await {
                    return;
                }
                tokio::time::sleep(Duration::from_secs(3)).await;
            }
        });
    }

    fn request_banner(&self, sysid: u8, compid: u8) {
        let core = self.clone();
        tokio::spawn(async move {
            for _ in 0..5 {
                if core
                    .fleet()
                    .get(sysid)
                    .and_then(|v| v.confirmed_firmware())
                    .is_some()
                {
                    return;
                }
                let cmd = Message::CommandLong(CommandLong {
                    target_system: sysid,
                    target_component: compid,
                    command: MavCmd::DO_SEND_BANNER.0,
                    ..Default::default()
                });
                if !core.send(&cmd).await {
                    return;
                }
                tokio::time::sleep(Duration::from_secs(3)).await;
            }
        });
    }

    /// Ask the vehicle for the configured telemetry rates (`MAV_CMD_SET_MESSAGE_INTERVAL`).
    pub async fn request_streams(&self, sysid: u8, compid: u8) {
        for &(msgid, hz) in &self.cfg.stream_rates {
            let cmd = Message::CommandLong(CommandLong {
                target_system: sysid,
                target_component: compid,
                command: MavCmd::SET_MESSAGE_INTERVAL.0,
                confirmation: 0,
                param1: msgid as f32,
                param2: 1e6 / hz,
                ..Default::default()
            });
            self.send(&cmd).await;
        }
    }

    /// Download every parameter from `target`. `progress(received, total)` is called as
    /// values arrive.
    pub async fn download_params(
        &self,
        target: (u8, u8),
        mut progress: impl FnMut(usize, usize),
    ) -> Result<BTreeMap<String, Param>, ParamError> {
        let mut rx = self.incoming.subscribe();
        let (mut dl, req) = ParamDownload::start(target, DownloadConfig::default(), Instant::now());
        self.send(&req).await;
        let mut tick = tokio::time::interval(Duration::from_millis(50));
        let mut last = (usize::MAX, 0);
        loop {
            tokio::select! {
                m = rx.recv() => match m {
                    Ok(inc) if inc.sysid == target.0 && inc.compid == target.1 => dl.on_message(&inc.msg, Instant::now()),
                    Ok(_) | Err(broadcast::error::RecvError::Lagged(_)) => {}
                    Err(broadcast::error::RecvError::Closed) => break,
                },
                _ = tick.tick() => {
                    for m in dl.poll(Instant::now()) {
                        self.send(&m).await;
                    }
                }
            }
            match dl.status() {
                DownloadStatus::Complete => {
                    let params = dl.params();
                    progress(params.len(), usize::from(dl.announced_count().unwrap_or(0)));
                    return Ok(params);
                }
                DownloadStatus::Failed => break,
                DownloadStatus::Running { received, total } => {
                    if (received, total) != last {
                        progress(received, total);
                        last = (received, total);
                    }
                }
            }
        }
        Err(ParamError {
            missing: dl.missing(),
            partial: dl.params().len(),
        })
    }

    /// Download the mission from `target`.
    pub async fn download_mission(&self, target: (u8, u8)) -> (TransferStatus, Vec<MissionItem>) {
        self.download_items(target, MissionType::Mission).await
    }

    /// Download the mission, the geofence or the rally points from `target` (read-only).
    pub async fn download_items(
        &self,
        target: (u8, u8),
        kind: MissionType,
    ) -> (TransferStatus, Vec<MissionItem>) {
        let (mut dl, first) = MissionDownload::start(
            target,
            kind.mavlink(),
            TransferConfig::default(),
            Instant::now(),
        );
        self.drive(
            target,
            &mut dl,
            first,
            MissionDownload::on_message,
            MissionDownload::poll,
            MissionDownload::status,
        )
        .await;
        (dl.status().clone(), dl.items().to_vec())
    }

    /// Upload a mission to a vehicle known to be disarmed, then download it again and compare
    /// (hazard H4). Only [`MissionReport::verified`] means the vehicle holds exactly this
    /// mission. To an armed vehicle (opt-in setting and in-flight warning), or for a fence or
    /// rally list, use [`Core::upload_items`].
    pub async fn upload_mission(
        &self,
        target: (u8, u8),
        items: Vec<MissionItem>,
    ) -> Result<MissionReport, Refusal> {
        self.upload_items(
            target,
            MissionType::Mission,
            items,
            ArmedWrite::DisarmedOnly,
        )
        .await
        .map_err(|e| Refusal::Precondition(e.to_string()))
    }

    /// The logs on the vehicle (read-only). Refused unless the vehicle is known to be disarmed:
    /// ArduPilot ignores log requests while armed.
    pub async fn list_logs(&self, target: (u8, u8)) -> Result<Vec<LogEntry>, String> {
        self.require_disarmed(target.0)?;
        let (mut l, first) = LogList::start(target, LogTransferConfig::default(), Instant::now());
        let mut rx = self.incoming.subscribe();
        self.send(&first).await;
        let mut tick = tokio::time::interval(Duration::from_millis(50));
        while *l.status() == LogTransferStatus::Running {
            let out = tokio::select! {
                m = rx.recv() => match m {
                    Ok(inc) if inc.sysid == target.0 && inc.compid == target.1 => {
                        l.on_message(&inc.msg, Instant::now());
                        Vec::new()
                    }
                    Ok(_) | Err(broadcast::error::RecvError::Lagged(_)) => Vec::new(),
                    Err(broadcast::error::RecvError::Closed) => return Err("core stopped".into()),
                },
                _ = tick.tick() => l.poll(Instant::now()),
            };
            for m in out {
                self.send(&m).await;
            }
        }
        match l.status() {
            LogTransferStatus::Failed(why) => Err(why.clone()),
            _ => Ok(l.entries()),
        }
    }

    /// Download one log (read-only; refused unless disarmed). `progress(received, size)` is
    /// called as data arrives. The whole log is held in memory.
    pub async fn download_log(
        &self,
        target: (u8, u8),
        entry: LogEntry,
        mut progress: impl FnMut(u32, u32),
    ) -> Result<Vec<u8>, String> {
        self.require_disarmed(target.0)?;
        let (mut d, first) =
            LogDownload::start(target, entry, LogTransferConfig::default(), Instant::now());
        let mut rx = self.incoming.subscribe();
        for m in first {
            self.send(&m).await;
        }
        let mut tick = tokio::time::interval(Duration::from_millis(50));
        let mut shown = (u32::MAX, 0);
        while *d.status() == LogTransferStatus::Running {
            let out = tokio::select! {
                m = rx.recv() => match m {
                    Ok(inc) if inc.sysid == target.0 && inc.compid == target.1 => {
                        d.on_message(&inc.msg, Instant::now())
                    }
                    // A lagged receiver lost LOG_DATA: the gap is re-requested later.
                    Ok(_) | Err(broadcast::error::RecvError::Lagged(_)) => Vec::new(),
                    Err(broadcast::error::RecvError::Closed) => return Err("core stopped".into()),
                },
                _ = tick.tick() => {
                    if d.progress() != shown {
                        shown = d.progress();
                        progress(shown.0, shown.1);
                    }
                    d.poll(Instant::now())
                }
            };
            for m in out {
                self.send(&m).await;
            }
        }
        match d.status() {
            LogTransferStatus::Failed(why) => Err(why.clone()),
            _ => {
                let (r, s) = d.progress();
                progress(r, s);
                Ok(d.data().unwrap_or_default().to_vec())
            }
        }
    }

    fn require_disarmed(&self, sysid: u8) -> Result<(), String> {
        match self.facts(sysid).armed {
            Some(false) => Ok(()),
            Some(true) => {
                Err("the vehicle is armed: ArduPilot only transfers logs when disarmed".into())
            }
            None => Err("no heartbeat from the vehicle yet".into()),
        }
    }

    async fn drive<M>(
        &self,
        target: (u8, u8),
        machine: &mut M,
        first: Message,
        on_message: fn(&mut M, &Message, Instant) -> Vec<Message>,
        poll: fn(&mut M, Instant) -> Vec<Message>,
        status: fn(&M) -> &TransferStatus,
    ) {
        let mut rx = self.incoming.subscribe();
        self.send(&first).await;
        let mut tick = tokio::time::interval(Duration::from_millis(50));
        while matches!(status(machine), TransferStatus::InProgress { .. }) {
            let out = tokio::select! {
                m = rx.recv() => match m {
                    Ok(inc) if inc.sysid == target.0 && inc.compid == target.1 => {
                        on_message(machine, &inc.msg, Instant::now())
                    }
                    Ok(_) | Err(broadcast::error::RecvError::Lagged(_)) => Vec::new(),
                    Err(broadcast::error::RecvError::Closed) => return,
                },
                _ = tick.tick() => poll(machine, Instant::now()),
            };
            for m in out {
                self.send(&m).await;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sequence_gaps_count_as_loss_but_reorders_do_not() {
        let t = Instant::now();
        let mut s = SeqTracker::default();
        for seq in [250u8, 251, 253, 254, 255, 0, 3] {
            s.on_frame(seq, t);
        }
        // 252 and 1, 2 missing (wrapping through 255 -> 0).
        let q = s.quality(t);
        assert_eq!((q.received, q.lost), (7, 3));
        s.on_frame(2, t); // late frame: it was counted lost, so that is taken back
        assert_eq!(s.quality(t).lost, 2);
        s.on_frame(4, t); // next in order: no loss, and no double count after the reorder
        s.on_frame(4, t); // duplicate: ignored
        let q = s.quality(t);
        assert_eq!((q.received, q.lost), (9, 2));
        assert_eq!(q.recent_loss_pct(), Some(2.0 * 100.0 / 11.0));
        // Ten seconds later the window has moved on.
        let later = t + Duration::from_secs(12);
        s.on_frame(3, later);
        assert_eq!(s.quality(later).recent_received, 1);
    }

    #[test]
    fn parses_link_specs() {
        assert_eq!(
            "tcp:127.0.0.1:5760".parse(),
            Ok(LinkSpec::Tcp("127.0.0.1:5760".parse().unwrap()))
        );
        assert_eq!(
            "udp:0.0.0.0:14550".parse(),
            Ok(LinkSpec::Udp("0.0.0.0:14550".parse().unwrap()))
        );
        assert_eq!(
            "serial:/dev/ttyACM0:115200".parse(),
            Ok(LinkSpec::Serial {
                path: "/dev/ttyACM0".into(),
                baud: 115_200
            })
        );
        assert_eq!(
            "serial:COM3:57600".parse::<LinkSpec>().unwrap().to_string(),
            "serial:COM3:57600"
        );
        assert!("bogus:1".parse::<LinkSpec>().is_err());
        assert!("tcp:nohost".parse::<LinkSpec>().is_err());
    }
}
