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
use mavlink_codec::enums::{MavAutopilot, MavCmd, MavState, MavType};
use mavlink_codec::messages::{CommandLong, Heartbeat};
use mavlink_codec::{Frame, Header, Message, Version, encode};
use protocols::audit::{self, AuditLog};
/// Command types for [`Core::request_command`] and [`Core::submit_command`].
pub use protocols::command::{
    Alert, AlertLevel, Approval, Command, CommandId, CommandState, CommandStatus, PolicyConfig,
    Refusal, Request, VehicleFacts,
};
use protocols::command::{AuditEvent, CommandPolicy, LinkInfo, LinkState, Target};
use protocols::mission::{
    self, MISSION_TYPE_MISSION, MissionDownload, MissionUpload, TransferConfig,
};
/// Mission types for [`Core::upload_mission`], [`Core::download_mission`] and
/// [`Core::download_items`].
pub use protocols::mission::{Mismatch, MissionItem, MissionType, TransferStatus};
pub use protocols::params::snapshot::ParamSnapshot;
use protocols::params::{DownloadConfig, DownloadStatus, ParamDownload};
/// Parameter values, reports and saved baselines for parameter write/rollback clients.
pub use protocols::params::{Param, SetStatus};
use tokio::sync::broadcast;

pub use vehicle::{
    Battery, Extended, Firmware, FirmwareSource, Fleet, Home, LandedState, MissionProgress,
    MissionState, RcInput, ServoOutputs, StatusText, Traffic, VehicleState, Vibration, VtolState,
    Wind,
};
pub use vehicle::{NavHealth, NavThresholds, PositionTrust};

mod parameters;
mod replay;
pub use parameters::{
    PARAM_HEARTBEAT_MAX_AGE, ParamOperationError, ParamReport, ParamRisk, ParamWriteResult,
    parameter_risk,
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
            ],
            link_stale: Duration::from_millis(1500),
            link_lost: Duration::from_secs(5),
            policy: PolicyConfig::default(),
            audit_log: None,
            nav: NavThresholds::default(),
        }
    }
}

/// Things worth telling the user about.
#[derive(Debug, Clone, PartialEq)]
pub enum CoreEvent {
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
    /// Recent STATUSTEXT per vehicle (kept outside the fleet, which is cloned per message).
    messages: Arc<std::sync::Mutex<HashMap<u8, std::collections::VecDeque<LoggedText>>>>,
    parameters: Arc<parameters::ParameterState>,
    /// Held across opening a link, so concurrent connects cannot both pass the check.
    link_mode: Arc<tokio::sync::Mutex<LinkMode>>,
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
            messages: Arc::default(),
            parameters: Arc::default(),
            link_mode: Arc::default(),
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
        let Ok(msg) = frame.decode() else { return };
        let (sysid, compid) = (frame.sysid(), frame.compid());
        let now = Instant::now();
        self.observe_parameter_safety(sysid, compid, &msg, now);
        let first_heartbeat = matches!(msg, Message::Heartbeat(ref h) if h.r#type != MavType::GCS.0)
            && self
                .fleet
                .load()
                .get(sysid)
                .is_none_or(|v| v.heartbeat.is_none());
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
        if compid == vehicle::AUTOPILOT_COMPID
            && matches!(msg, Message::EkfStatusReport(_) | Message::GpsRawInt(_))
        {
            self.check_nav(sysid, now);
        }
        if first_heartbeat && compid == vehicle::AUTOPILOT_COMPID {
            let _ = self.events.send(CoreEvent::VehicleSeen { sysid });
            self.request_streams(sysid, compid).await;
            self.request_banner(sysid, compid);
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
        let r = self
            .policy()
            .request(Self::target(sysid), command, &facts, link, now);
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
        let r = self
            .policy()
            .submit(Self::target(sysid), command, approval, &facts, link, now);
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
                let _ = self.events.send(CoreEvent::Alert {
                    sysid: s.target.sysid,
                    level: a.level,
                    text: a.text,
                });
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
        let _ = self.events.send(CoreEvent::Alert { sysid, level, text });
    }

    /// Ask the autopilot for its version banner (`MAV_CMD_DO_SEND_BANNER`), which confirms the
    /// firmware and so the meaning of its mode numbers (`vehicle::modes`). Retried every 3 s,
    /// up to 5 times, until a banner arrives.
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

    /// Upload `items` to `target`, then download the mission again and compare (hazard H4).
    /// Only [`MissionReport::verified`] means the vehicle holds exactly this mission.
    ///
    /// Refused while the vehicle is armed or its armed state is unknown: an upload in flight
    /// is a class C two-stage action (plan 08 §4.1) and that confirmation is not built for
    /// missions yet.
    pub async fn upload_mission(
        &self,
        target: (u8, u8),
        items: Vec<MissionItem>,
    ) -> Result<MissionReport, Refusal> {
        if self.facts(target.0).armed != Some(false) {
            return Err(Refusal::Precondition(
                "mission upload needs the vehicle known to be disarmed (in-flight upload needs \
                 the two-stage confirmation, not built yet)"
                    .into(),
            ));
        }
        // ArduPilot reports MAV_STATE_BOOT until init_ardupilot() has finished, which includes
        // initialising mission storage; an upload before that is refused as NO_SPACE (seen in
        // SITL under load). Say what is really wrong instead.
        let booting = self
            .fleet()
            .get(target.0)
            .and_then(|v| v.heartbeat.as_ref())
            .is_none_or(|h| h.value.system_status == MavState::BOOT.0);
        if booting {
            return Err(Refusal::Precondition(
                "the vehicle is still booting (MAV_STATE_BOOT); upload once it reports ready"
                    .into(),
            ));
        }
        let (mut up, first) = MissionUpload::start(
            target,
            items.clone(),
            MISSION_TYPE_MISSION,
            TransferConfig::default(),
            Instant::now(),
        );
        self.drive(
            target,
            &mut up,
            first,
            MissionUpload::on_message,
            MissionUpload::poll,
            MissionUpload::status,
        )
        .await;
        // Read back whatever the upload said: a lost final ACK or a partial write shows here.
        let (readback, on_vehicle) = self.download_mission(target).await;
        let mismatches = if readback == TransferStatus::Done {
            mission::verify(&items, &on_vehicle, true)
        } else {
            Vec::new()
        };
        Ok(MissionReport {
            upload: up.status().clone(),
            readback,
            on_vehicle,
            mismatches,
        })
    }

    /// Run a mission state machine until it finishes.
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
