//! `CommandPolicy`: the core-side gate and delivery engine for vehicle commands (sans-I/O).
//!
//! Implements plan 08 §4, ADR-0009 and ADR-0011:
//!
//! * **Classes A–D** ([`Class`]) with the confirmation each needs. Class C needs a single-use
//!   [`Confirmation`] token issued by [`CommandPolicy::request`], valid for
//!   [`PolicyConfig::token_ttl`], bound to the vehicle and the exact command. Class D also
//!   needs the typed challenge phrase and a reason (logged). Scripts and API clients cannot
//!   skip the request/confirm step because the check lives here, not in the UI.
//! * **Arguments are validated** (finite, in range) and **vehicle-state lockouts** apply
//!   (motor test only when known disarmed and props-off attested; parameter reset and reboot
//!   not while armed). They use the last-known state.
//! * **Link state never refuses a command** (ADR-0009). It changes delivery: the command is
//!   retransmitted until `COMMAND_ACK` or the expected state is seen, flooded over every link
//!   when the link is not CONNECTED, and never sent after its lifetime
//!   ([`PolicyConfig::lifetime_nav`] for RTL/Land, [`PolicyConfig::lifetime_mode`] for mode
//!   changes, [`PolicyConfig::lifetime`] otherwise).
//!   A newer command in the same [`Group`] supersedes it; the operator can cancel it.
//!   `COMMAND_ACK` has no request id, so after superseding a command with the same `MAV_CMD`
//!   (DISARM after ARM) ACKs are ambiguous for [`PolicyConfig::retry_max`] and are ignored;
//!   retransmission continues until an unambiguous ACK or the vehicle state settles it.
//! * **Outcome is state-based**: [`CommandState::Verified`] only when telemetry shows the
//!   expected state; otherwise `NOT CONFIRMED` until verified or expired.
//! * Every step produces an [`AuditEvent`] for the hash-chained audit log
//!   ([`crate::audit`]).
//!
//! The policy never touches the network: the caller feeds it time, link state, vehicle state
//! and `COMMAND_ACK`s, and transmits the [`Outgoing`] messages it returns.

use std::collections::{BTreeMap, HashMap};
use std::fmt;
use std::time::{Duration, Instant};

use mavlink_codec::Message;
use mavlink_codec::enums::{MavCmd, MavResult};
use mavlink_codec::messages::{CommandAck, CommandInt, CommandLong};

/// `MAV_CMD_COMPONENT_ARM_DISARM` param2 value that forces arming/disarming (common.xml).
pub const FORCE_MAGIC: f32 = 21196.0;
/// `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` (minimal.xml).
const CUSTOM_MODE_ENABLED: f32 = 1.0;
/// `MAV_FRAME_GLOBAL_RELATIVE_ALT` (common.xml).
const FRAME_GLOBAL_RELATIVE_ALT: u8 = 3;
/// `MAV_DO_REPOSITION_FLAGS_CHANGE_MODE` (common.xml).
const REPOSITION_CHANGE_MODE: f32 = 1.0;
/// `PARAM_RESET_FACTORY_DEFAULT` (`PREFLIGHT_STORAGE_PARAMETER_ACTION`, common.xml).
const PARAM_RESET_FACTORY_DEFAULT: f32 = 2.0;
/// `MOTOR_TEST_THROTTLE_PERCENT` (common.xml).
const MOTOR_TEST_THROTTLE_PERCENT: f32 = 0.0;

/// Commands the policy knows (safe command set v1, plan 07 + plan 08 §4.1).
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    /// Arm. Class B (ADR-0011).
    Arm,
    /// Arm bypassing pre-arm checks (`force`). Class D: it disables safety checks.
    ForceArm,
    /// Disarm. Class C: in flight it stops the motors (terminal). The GCS cannot yet tell
    /// landed from flying, so it is always two-stage.
    Disarm,
    /// Emergency motor termination: force-disarm even in flight. Class C (two-stage /
    /// slide, no typed text: ADR-0011).
    ForceDisarm,
    /// Change flight mode (ArduPilot `custom_mode` number). Class B: single action, the
    /// primary arrest manoeuvre (ADR-0011); lifetime [`PolicyConfig::lifetime_mode`].
    SetMode {
        /// Target `custom_mode`.
        custom_mode: u32,
    },
    /// Return to launch. Class B (ADR-0011).
    Rtl {
        /// Mode number that proves RTL engaged (from mode metadata), if known.
        expect_mode: Option<u32>,
    },
    /// Land here. Class B (ADR-0011).
    Land {
        /// Mode number that proves LAND engaged, if known.
        expect_mode: Option<u32>,
    },
    /// Take off to `alt_m` (relative). Class B (ADR-0011).
    Takeoff {
        /// Altitude above home, m (0 < alt ≤ [`MAX_TAKEOFF_ALT_M`]).
        alt_m: f32,
    },
    /// Guided go-to (`MAV_CMD_DO_REPOSITION`, switches to Guided). Class B.
    Reposition {
        /// Latitude, degrees.
        lat_deg: f64,
        /// Longitude, degrees.
        lon_deg: f64,
        /// Altitude above home, m.
        alt_m: f32,
    },
    /// Spin one motor for a bench test. Class B; only when known disarmed and with
    /// props-off attestation.
    MotorTest {
        /// Motor number, 1-based.
        motor: u8,
        /// Throttle, percent (0–[`MAX_MOTOR_TEST_PCT`]).
        throttle_pct: f32,
        /// Duration, s (0 < d ≤ [`MAX_MOTOR_TEST_S`]).
        duration_s: f32,
        /// Operator attested that the propellers are removed.
        props_removed: bool,
    },
    /// Reboot the autopilot. Class C (terminal); never while armed (ADR-0011).
    Reboot,
    /// Reset all parameters to factory defaults. Class C; not while armed.
    ParamReset,
}

/// Highest takeoff altitude accepted, m.
pub const MAX_TAKEOFF_ALT_M: f32 = 1000.0;
/// Highest motor-test throttle accepted, percent.
pub const MAX_MOTOR_TEST_PCT: f32 = 100.0;
/// Longest motor test accepted, s.
pub const MAX_MOTOR_TEST_S: f32 = 30.0;

/// Plan 08 §4.1 command classes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Class {
    /// Benign, reversible (UI-only actions; no vehicle command uses it today).
    A,
    /// Operational: one deliberate action, visible pending → ACK.
    B,
    /// Consequential: two-stage, confirmation token.
    C,
    /// Hazardous: C + typed confirmation + reason.
    D,
}

/// Commands that supersede each other on the same vehicle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Group {
    /// Arm / disarm / kill.
    Arming,
    /// Mode, RTL, Land, takeoff, go-to.
    Navigation,
    /// Motor test.
    MotorTest,
    /// Reboot.
    Reboot,
    /// Parameter storage.
    Params,
}

impl Command {
    /// Plan 08 §4.1 class.
    ///
    /// Operational profile (ADR-0011, product owner 2026-10-11): flight controls are single
    /// action (class B) because confirmation latency is itself a flight hazard; two-stage
    /// confirmation is reserved for irreversible terminal actions (in-flight disarm,
    /// autopilot reboot, emergency motor termination). Force-arm and parameter reset are
    /// ground-only actions and keep their confirmation pending owner review.
    pub fn class(&self) -> Class {
        match self {
            Command::ForceArm => Class::D,
            Command::Disarm | Command::ForceDisarm | Command::Reboot | Command::ParamReset => {
                Class::C
            }
            Command::Arm
            | Command::SetMode { .. }
            | Command::Rtl { .. }
            | Command::Land { .. }
            | Command::Takeoff { .. }
            | Command::Reposition { .. }
            | Command::MotorTest { .. } => Class::B,
        }
    }

    /// Supersede group.
    pub fn group(&self) -> Group {
        match self {
            Command::Arm | Command::ForceArm | Command::Disarm | Command::ForceDisarm => {
                Group::Arming
            }
            Command::SetMode { .. }
            | Command::Rtl { .. }
            | Command::Land { .. }
            | Command::Takeoff { .. }
            | Command::Reposition { .. } => Group::Navigation,
            Command::MotorTest { .. } => Group::MotorTest,
            Command::Reboot => Group::Reboot,
            Command::ParamReset => Group::Params,
        }
    }

    /// `MAV_CMD` number, for matching `COMMAND_ACK`.
    pub fn mav_cmd(&self) -> u16 {
        match self {
            Command::Arm | Command::ForceArm | Command::Disarm | Command::ForceDisarm => {
                MavCmd::COMPONENT_ARM_DISARM.0
            }
            Command::SetMode { .. } => MavCmd::DO_SET_MODE.0,
            Command::Rtl { .. } => MavCmd::NAV_RETURN_TO_LAUNCH.0,
            Command::Land { .. } => MavCmd::NAV_LAND.0,
            Command::Takeoff { .. } => MavCmd::NAV_TAKEOFF.0,
            Command::Reposition { .. } => MavCmd::DO_REPOSITION.0,
            Command::MotorTest { .. } => MavCmd::DO_MOTOR_TEST.0,
            Command::Reboot => MavCmd::PREFLIGHT_REBOOT_SHUTDOWN.0,
            Command::ParamReset => MavCmd::PREFLIGHT_STORAGE.0,
        }
    }

    /// Short operator-facing name.
    pub fn label(&self) -> String {
        match self {
            Command::Arm => "ARM".into(),
            Command::ForceArm => "FORCE ARM".into(),
            Command::Disarm => "DISARM".into(),
            Command::ForceDisarm => "MOTOR KILL".into(),
            Command::SetMode { custom_mode } => format!("MODE {custom_mode}"),
            Command::Rtl { .. } => "RTL".into(),
            Command::Land { .. } => "LAND".into(),
            Command::Takeoff { alt_m } => format!("TAKEOFF {alt_m} m"),
            Command::Reposition {
                lat_deg,
                lon_deg,
                alt_m,
            } => format!("GO TO {lat_deg:.7},{lon_deg:.7} {alt_m} m"),
            Command::MotorTest {
                motor,
                throttle_pct,
                duration_s,
                ..
            } => format!("MOTOR TEST #{motor} {throttle_pct}% {duration_s} s"),
            Command::Reboot => "REBOOT AUTOPILOT".into(),
            Command::ParamReset => "RESET PARAMETERS".into(),
        }
    }

    /// Lifetime under `cfg` (ADR-0009).
    pub fn lifetime(&self, cfg: &PolicyConfig) -> Duration {
        match self {
            Command::Rtl { .. } | Command::Land { .. } => cfg.lifetime_nav,
            Command::SetMode { .. } => cfg.lifetime_mode,
            _ => cfg.lifetime,
        }
    }

    fn validate(&self) -> Result<(), Refusal> {
        let bad = |s: String| Err(Refusal::InvalidArgument(s));
        match *self {
            Command::Takeoff { alt_m }
                if !(alt_m.is_finite() && alt_m > 0.0 && alt_m <= MAX_TAKEOFF_ALT_M) =>
            {
                bad(format!(
                    "takeoff altitude {alt_m} m outside (0, {MAX_TAKEOFF_ALT_M}]"
                ))
            }
            Command::Reposition {
                lat_deg,
                lon_deg,
                alt_m,
            } if !(lat_deg.is_finite()
                && lon_deg.is_finite()
                && alt_m.is_finite()
                && (-90.0..=90.0).contains(&lat_deg)
                && (-180.0..=180.0).contains(&lon_deg)) =>
            {
                bad(format!(
                    "go-to position {lat_deg},{lon_deg} {alt_m} m is not a valid position"
                ))
            }
            Command::MotorTest {
                motor,
                throttle_pct,
                duration_s,
                ..
            } if motor == 0
                || !(throttle_pct.is_finite()
                    && (0.0..=MAX_MOTOR_TEST_PCT).contains(&throttle_pct))
                || !(duration_s.is_finite()
                    && duration_s > 0.0
                    && duration_s <= MAX_MOTOR_TEST_S) =>
            {
                bad(format!(
                    "motor test #{motor} {throttle_pct}% {duration_s} s outside motor >= 1, 0-{MAX_MOTOR_TEST_PCT}%, (0, {MAX_MOTOR_TEST_S}] s"
                ))
            }
            _ => Ok(()),
        }
    }

    /// Vehicle-state lockouts (plan 08 §4.2). Link state is never consulted (ADR-0009).
    fn preconditions(&self, v: &VehicleFacts) -> Result<(), Refusal> {
        let locked = |s: &str| Err(Refusal::Precondition(s.to_string()));
        match self {
            Command::MotorTest { props_removed, .. } => {
                if v.armed != Some(false) {
                    return locked("motor test needs the vehicle known to be disarmed");
                }
                if !props_removed {
                    return locked("motor test needs the props-off attestation");
                }
                Ok(())
            }
            Command::ParamReset if v.armed == Some(true) => {
                locked("parameter reset is not allowed while armed")
            }
            Command::Reboot if v.armed == Some(true) => {
                locked("autopilot reboot is not allowed while armed")
            }
            _ => Ok(()),
        }
    }

    /// The message to send. `confirmation` counts retransmissions (`COMMAND_LONG`).
    pub fn to_message(&self, target: Target, confirmation: u8) -> Message {
        let long = |command: MavCmd, p: [f32; 7]| {
            Message::CommandLong(CommandLong {
                target_system: target.sysid,
                target_component: target.compid,
                command: command.0,
                confirmation,
                param1: p[0],
                param2: p[1],
                param3: p[2],
                param4: p[3],
                param5: p[4],
                param6: p[5],
                param7: p[6],
            })
        };
        match *self {
            Command::Arm => long(
                MavCmd::COMPONENT_ARM_DISARM,
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ),
            Command::ForceArm => long(
                MavCmd::COMPONENT_ARM_DISARM,
                [1.0, FORCE_MAGIC, 0.0, 0.0, 0.0, 0.0, 0.0],
            ),
            Command::Disarm => long(MavCmd::COMPONENT_ARM_DISARM, [0.0; 7]),
            Command::ForceDisarm => long(
                MavCmd::COMPONENT_ARM_DISARM,
                [0.0, FORCE_MAGIC, 0.0, 0.0, 0.0, 0.0, 0.0],
            ),
            Command::SetMode { custom_mode } => long(
                MavCmd::DO_SET_MODE,
                [
                    CUSTOM_MODE_ENABLED,
                    custom_mode as f32,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            ),
            Command::Rtl { .. } => long(MavCmd::NAV_RETURN_TO_LAUNCH, [0.0; 7]),
            // param4 NaN: keep the current yaw behaviour; lat/lon 0: land here.
            Command::Land { .. } => {
                long(MavCmd::NAV_LAND, [0.0, 0.0, 0.0, f32::NAN, 0.0, 0.0, 0.0])
            }
            Command::Takeoff { alt_m } => long(
                MavCmd::NAV_TAKEOFF,
                [0.0, 0.0, 0.0, f32::NAN, 0.0, 0.0, alt_m],
            ),
            // ArduPilot accepts DO_REPOSITION only as COMMAND_INT.
            Command::Reposition {
                lat_deg,
                lon_deg,
                alt_m,
            } => Message::CommandInt(CommandInt {
                target_system: target.sysid,
                target_component: target.compid,
                frame: FRAME_GLOBAL_RELATIVE_ALT,
                command: MavCmd::DO_REPOSITION.0,
                param1: -1.0,
                param2: REPOSITION_CHANGE_MODE,
                param3: 0.0,
                param4: f32::NAN,
                x: (lat_deg * 1e7).round() as i32,
                y: (lon_deg * 1e7).round() as i32,
                z: alt_m,
                ..Default::default()
            }),
            Command::MotorTest {
                motor,
                throttle_pct,
                duration_s,
                ..
            } => long(
                MavCmd::DO_MOTOR_TEST,
                [
                    f32::from(motor),
                    MOTOR_TEST_THROTTLE_PERCENT,
                    throttle_pct,
                    duration_s,
                    0.0,
                    0.0,
                    0.0,
                ],
            ),
            Command::Reboot => long(
                MavCmd::PREFLIGHT_REBOOT_SHUTDOWN,
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ),
            Command::ParamReset => long(
                MavCmd::PREFLIGHT_STORAGE,
                [PARAM_RESET_FACTORY_DEFAULT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ),
        }
    }

    /// The vehicle state that proves this command took effect, if observable.
    fn verified_by(&self, v: &VehicleFacts) -> Option<bool> {
        let mode = |m: Option<u32>| m.map(|m| v.custom_mode == Some(m));
        match *self {
            Command::Arm | Command::ForceArm => Some(v.armed == Some(true)),
            Command::Disarm | Command::ForceDisarm => Some(v.armed == Some(false)),
            Command::SetMode { custom_mode } => Some(v.custom_mode == Some(custom_mode)),
            Command::Rtl { expect_mode } | Command::Land { expect_mode } => mode(expect_mode),
            _ => None,
        }
    }
}

/// MAVLink address of the vehicle component a command goes to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Target {
    /// System id.
    pub sysid: u8,
    /// Component id (autopilot: 1).
    pub compid: u8,
}

/// What the policy knows about the vehicle (last-known values; `None` = never seen).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct VehicleFacts {
    /// Armed flag from the last heartbeat.
    pub armed: Option<bool>,
    /// `custom_mode` from the last heartbeat.
    pub custom_mode: Option<u32>,
    /// `MAV_LANDED_STATE` from the last `EXTENDED_SYS_STATE`, with its age. Recorded for
    /// ADR-0011 interpretation 2 (single-action disarm once landed is known); no rule uses it
    /// yet — that waits for the safety lead's decision.
    pub landed_state: Option<(u8, Duration)>,
}

/// Heartbeat link state (mirrors dc-core's `VehicleLink`, plan 08 §4.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkState {
    /// Heartbeats current.
    Connected,
    /// Link quality poor (loss/latency).
    Degraded,
    /// No heartbeat for a short while.
    Stale,
    /// No heartbeat for long.
    Lost,
}

impl fmt::Display for LinkState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            LinkState::Connected => "CONNECTED",
            LinkState::Degraded => "DEGRADED",
            LinkState::Stale => "STALE",
            LinkState::Lost => "LOST",
        })
    }
}

/// Link state and heartbeat age, as shown to the operator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkInfo {
    /// State.
    pub state: LinkState,
    /// Age of the last heartbeat; `None` if the vehicle was never heard.
    pub heartbeat_age: Option<Duration>,
}

impl LinkInfo {
    /// `LOST 7.2 s`, or `LOST, no heartbeat yet`.
    pub fn describe(&self) -> String {
        match self.heartbeat_age {
            Some(a) => format!("{} {:.1} s", self.state, a.as_secs_f32()),
            None => format!("{}, no heartbeat yet", self.state),
        }
    }
}

/// Tuning (defaults from ADR-0009 and plan 08 §4.1–4.2).
#[derive(Debug, Clone)]
pub struct PolicyConfig {
    /// How long a confirmation token stays valid (plan 08: 5 s auto-cancel).
    pub token_ttl: Duration,
    /// Command lifetime for RTL and Land.
    pub lifetime_nav: Duration,
    /// Command lifetime for mode changes (ADR-0011: 2–3 s). A mode switch never waits in a
    /// queue to execute when the aircraft is in a different flight state; unanswered, it
    /// expires and the operator is alerted.
    pub lifetime_mode: Duration,
    /// Command lifetime for everything else.
    pub lifetime: Duration,
    /// Retransmit interval while CONNECTED (no ACK yet).
    pub retry_connected: Duration,
    /// First retransmit interval when the link is not CONNECTED; grows ×1.5 per attempt.
    pub retry_degraded: Duration,
    /// Longest retransmit interval.
    pub retry_max: Duration,
}

impl Default for PolicyConfig {
    fn default() -> Self {
        PolicyConfig {
            token_ttl: Duration::from_secs(5),
            lifetime_nav: Duration::from_secs(30),
            lifetime_mode: Duration::from_secs(3),
            lifetime: Duration::from_secs(10),
            retry_connected: Duration::from_secs(1),
            retry_degraded: Duration::from_millis(500),
            retry_max: Duration::from_secs(2),
        }
    }
}

/// Why the policy refused a command. Never the link state (ADR-0009).
#[derive(Debug, Clone, PartialEq)]
pub enum Refusal {
    /// Argument out of range or not finite.
    InvalidArgument(String),
    /// Vehicle-state lockout.
    Precondition(String),
    /// Class C/D without a confirmation token.
    ConfirmationRequired,
    /// Token unknown, used, expired, or issued for another vehicle/command.
    BadToken(String),
    /// Class D typed phrase does not match the challenge.
    TypedMismatch,
    /// Class D without a reason.
    ReasonRequired,
}

impl fmt::Display for Refusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Refusal::InvalidArgument(s) => write!(f, "invalid argument: {s}"),
            Refusal::Precondition(s) => write!(f, "not allowed now: {s}"),
            Refusal::ConfirmationRequired => write!(f, "needs confirmation"),
            Refusal::BadToken(s) => write!(f, "confirmation not valid: {s}"),
            Refusal::TypedMismatch => write!(f, "typed confirmation does not match"),
            Refusal::ReasonRequired => write!(f, "a reason is required for this command"),
        }
    }
}

impl std::error::Error for Refusal {}

/// Single-use token issued by [`CommandPolicy::request`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Token(pub u64);

/// What the confirmation card shows and the token that confirms it.
#[derive(Debug, Clone, PartialEq)]
pub struct Confirmation {
    /// Token to pass back to [`CommandPolicy::submit`].
    pub token: Token,
    /// Class.
    pub class: Class,
    /// Operator-facing summary: vehicle, action, link state.
    pub summary: String,
    /// Class D: the exact phrase the operator must type.
    pub challenge: Option<String>,
    /// When the token stops being valid.
    pub expires: Instant,
}

/// Answer to [`CommandPolicy::request`].
#[derive(Debug, Clone, PartialEq)]
pub enum Request {
    /// Class A/B: submit with [`Approval::None`].
    Ready {
        /// Class.
        class: Class,
    },
    /// Class C/D: show the card, then submit with the token.
    Confirm(Confirmation),
}

/// What the operator gave at submission.
#[derive(Debug, Clone, PartialEq)]
pub enum Approval {
    /// Class A/B.
    None,
    /// Class C.
    Confirmed(Token),
    /// Class D.
    Typed {
        /// Token from the request.
        token: Token,
        /// What the operator typed.
        text: String,
        /// Why (logged).
        reason: String,
    },
}

/// Command handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CommandId(pub u64);

impl fmt::Display for CommandId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "cmd#{}", self.0)
    }
}

/// Lifecycle (plan 08 §4.2, ADR-0009).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandState {
    /// Sent, no answer yet; retransmitting.
    Pending,
    /// `COMMAND_ACK` `IN_PROGRESS`.
    InProgress,
    /// `ACCEPTED`, waiting for the state that proves it.
    Acknowledged,
    /// Expected state observed. Final.
    Verified,
    /// `ACCEPTED`; this command has no observable state. Final.
    Accepted,
    /// Vehicle refused (`MAV_RESULT`). Final.
    Rejected(u8),
    /// Lifetime over without verification. Final.
    Expired {
        /// Whether the vehicle had acknowledged it.
        acked: bool,
    },
    /// Cancelled by the operator. Final.
    Cancelled,
    /// Replaced by a newer command in the same group. Final.
    Superseded(CommandId),
}

impl CommandState {
    /// No more transmissions or transitions.
    pub fn is_final(&self) -> bool {
        !matches!(
            self,
            CommandState::Pending | CommandState::InProgress | CommandState::Acknowledged
        )
    }
}

/// Snapshot of one command for the UI.
#[derive(Debug, Clone, PartialEq)]
pub struct CommandStatus {
    /// Handle.
    pub id: CommandId,
    /// Target.
    pub target: Target,
    /// Command.
    pub command: Command,
    /// State.
    pub state: CommandState,
    /// Transmissions so far.
    pub attempts: u32,
    /// Time left before expiry (zero once final).
    pub time_left: Duration,
    /// Link at the last transmission.
    pub link: LinkInfo,
    /// Operator-facing name of the command ("MODE LOITER (5)" once the firmware is known).
    pub label: String,
}

impl CommandStatus {
    /// Operator-facing status line, e.g.
    /// `RTL: SENT — NOT CONFIRMED (link LOST 7.2 s), attempt 3, 22 s left`.
    pub fn summary(&self) -> String {
        let what = &self.label;
        let link = if self.link.state == LinkState::Connected {
            String::new()
        } else {
            format!(" (link {})", self.link.describe())
        };
        let left = self.time_left.as_secs();
        match self.state {
            CommandState::Pending => format!(
                "{what}: SENT — NOT CONFIRMED{link}, attempt {}, {left} s left",
                self.attempts
            ),
            CommandState::InProgress => format!("{what}: IN PROGRESS{link}, {left} s left"),
            CommandState::Acknowledged => {
                format!("{what}: ACKNOWLEDGED — NOT YET VERIFIED{link}, {left} s left")
            }
            CommandState::Verified => format!("{what}: VERIFIED"),
            CommandState::Accepted => format!("{what}: ACCEPTED"),
            CommandState::Rejected(r) => format!("{what}: REJECTED ({})", result_name(r)),
            CommandState::Expired { acked: false } => {
                format!(
                    "{what}: NOT CONFIRMED — EXPIRED after {} attempts",
                    self.attempts
                )
            }
            CommandState::Expired { acked: true } => {
                format!("{what}: ACKNOWLEDGED BUT NOT VERIFIED — EXPIRED")
            }
            CommandState::Cancelled => format!("{what}: CANCELLED"),
            CommandState::Superseded(by) => format!("{what}: SUPERSEDED by {by}"),
        }
    }
}

fn result_name(r: u8) -> &'static str {
    match MavResult(r) {
        MavResult::TEMPORARILY_REJECTED => "temporarily rejected",
        MavResult::DENIED => "denied",
        MavResult::UNSUPPORTED => "unsupported",
        MavResult::FAILED => "failed",
        MavResult::COMMAND_LONG_ONLY => "COMMAND_LONG only",
        MavResult::COMMAND_INT_ONLY => "COMMAND_INT only",
        _ => "refused",
    }
}

/// A message to transmit.
#[derive(Debug, Clone, PartialEq)]
pub struct Outgoing {
    /// Command it belongs to.
    pub id: CommandId,
    /// Message.
    pub msg: Message,
    /// Send on every link (link not CONNECTED), not just the learned route.
    pub flood: bool,
    /// 1-based transmission number.
    pub attempt: u32,
}

/// One audit-log event (written hash-chained by [`crate::audit::AuditLog`]).
#[derive(Debug, Clone, PartialEq)]
pub struct AuditEvent {
    /// When.
    pub at: Instant,
    /// Vehicle.
    pub sysid: u8,
    /// Command, if assigned.
    pub id: Option<CommandId>,
    /// What happened, e.g. `refused RTL: needs confirmation`.
    pub text: String,
}

struct Issued {
    target: Target,
    command: Command,
    expires: Instant,
    challenge: Option<String>,
}

struct InFlight {
    target: Target,
    command: Command,
    state: CommandState,
    attempts: u32,
    deadline: Instant,
    next_send: Instant,
    interval: Duration,
    link: LinkInfo,
    /// `COMMAND_ACK` has no request id: after superseding a live command with the same
    /// `MAV_CMD` (e.g. DISARM replacing ARM), an ACK until this instant may answer either one,
    /// so it is logged and ignored.
    ambiguous_until: Option<Instant>,
}

fn label_with(modes: &HashMap<u8, HashMap<u32, String>>, sysid: u8, command: &Command) -> String {
    if let Command::SetMode { custom_mode } = command
        && let Some(name) = modes.get(&sysid).and_then(|m| m.get(custom_mode))
    {
        return format!("MODE {name} ({custom_mode})");
    }
    command.label()
}

/// The policy. One per core; holds tokens, in-flight commands and pending audit events.
pub struct CommandPolicy {
    cfg: PolicyConfig,
    names: HashMap<u8, String>,
    mode_names: HashMap<u8, HashMap<u32, String>>,
    tokens: HashMap<Token, Issued>,
    inflight: BTreeMap<CommandId, InFlight>,
    next_token: u64,
    next_id: u64,
    audit: Vec<AuditEvent>,
}

impl CommandPolicy {
    /// New policy.
    pub fn new(cfg: PolicyConfig) -> Self {
        CommandPolicy {
            cfg,
            names: HashMap::new(),
            mode_names: HashMap::new(),
            tokens: HashMap::new(),
            inflight: BTreeMap::new(),
            next_token: 1,
            next_id: 1,
            audit: Vec::new(),
        }
    }

    /// Configuration.
    pub fn config(&self) -> &PolicyConfig {
        &self.cfg
    }

    /// Operator-facing vehicle name (used in summaries and the class D challenge).
    pub fn set_vehicle_name(&mut self, sysid: u8, name: impl Into<String>) {
        self.names.insert(sysid, name.into());
    }

    /// Flight-mode names of `sysid`'s firmware, from the MAVLink metadata. Set only once the
    /// firmware is confirmed (its version banner); until then mode changes read "MODE 5".
    pub fn set_mode_names(&mut self, sysid: u8, modes: impl IntoIterator<Item = (u32, String)>) {
        self.mode_names.insert(sysid, modes.into_iter().collect());
    }

    /// The command's operator-facing label for this vehicle: mode changes name the mode
    /// ("MODE LOITER (5)") when its firmware is confirmed. The number always stays visible.
    pub fn label(&self, sysid: u8, command: &Command) -> String {
        label_with(&self.mode_names, sysid, command)
    }

    fn vehicle(&self, sysid: u8) -> String {
        self.names
            .get(&sysid)
            .cloned()
            .unwrap_or_else(|| format!("VEHICLE {sysid}"))
    }

    fn log(&mut self, at: Instant, sysid: u8, id: Option<CommandId>, text: String) {
        self.audit.push(AuditEvent {
            at,
            sysid,
            id,
            text,
        });
    }

    /// Step 1: validate and, for class C/D, issue a confirmation token. Link state is shown
    /// in the summary, never used to refuse.
    pub fn request(
        &mut self,
        target: Target,
        command: Command,
        facts: &VehicleFacts,
        link: LinkInfo,
        now: Instant,
    ) -> Result<Request, Refusal> {
        let label = self.label(target.sysid, &command);
        if let Err(r) = command
            .validate()
            .and_then(|_| command.preconditions(facts))
        {
            self.log(
                now,
                target.sysid,
                None,
                format!("refused request {label}: {r}"),
            );
            return Err(r);
        }
        self.tokens.retain(|_, t| t.expires > now);
        let class = command.class();
        if class <= Class::B {
            self.log(
                now,
                target.sysid,
                None,
                format!("request {label} (class {class:?})"),
            );
            return Ok(Request::Ready { class });
        }
        let token = Token(self.next_token);
        self.next_token += 1;
        let name = self.vehicle(target.sysid);
        let challenge = (class == Class::D).then(|| name.clone());
        let armed = match facts.armed {
            Some(true) => "ARMED",
            Some(false) => "disarmed",
            None => "armed state unknown",
        };
        let link_text = match link.state {
            LinkState::Connected => "link CONNECTED".to_string(),
            _ => format!(
                "link {} — may not arrive; it will be retried for {} s",
                link.describe(),
                command.lifetime(&self.cfg).as_secs()
            ),
        };
        let summary = format!(
            "{label} → {name} (sysid {}), {armed}; {link_text}",
            target.sysid
        );
        let expires = now + self.cfg.token_ttl;
        self.tokens.insert(
            token,
            Issued {
                target,
                command,
                expires,
                challenge: challenge.clone(),
            },
        );
        self.log(
            now,
            target.sysid,
            None,
            format!("confirmation issued for {label} (class {class:?})"),
        );
        Ok(Request::Confirm(Confirmation {
            token,
            class,
            summary,
            challenge,
            expires,
        }))
    }

    /// Step 2: check the approval, supersede older commands in the same group, start
    /// delivery. Returns the first transmission.
    pub fn submit(
        &mut self,
        target: Target,
        command: Command,
        approval: Approval,
        facts: &VehicleFacts,
        link: LinkInfo,
        now: Instant,
    ) -> Result<(CommandId, Vec<Outgoing>, Vec<CommandStatus>), Refusal> {
        let label = self.label(target.sysid, &command);
        let checked = self.check_approval(target, &command, &approval, facts, now);
        if let Err(r) = checked {
            self.log(now, target.sysid, None, format!("refused {label}: {r}"));
            return Err(r);
        }
        let id = CommandId(self.next_id);
        self.next_id += 1;
        let reason = match &approval {
            Approval::Typed { reason, .. } => format!(", reason: {reason}"),
            _ => String::new(),
        };
        self.log(
            now,
            target.sysid,
            Some(id),
            format!("accepted {label} (class {:?}){reason}", command.class()),
        );

        // Supersede older live commands in the same group on the same vehicle.
        let mut changed = Vec::new();
        let group = command.group();
        let older: Vec<CommandId> = self
            .inflight
            .iter()
            .filter(|(_, f)| {
                f.target.sysid == target.sysid && f.command.group() == group && !f.state.is_final()
            })
            .map(|(k, _)| *k)
            .collect();
        let same_cmd = older
            .iter()
            .any(|o| self.inflight[o].command.mav_cmd() == command.mav_cmd());
        for old in older {
            changed.push(self.finish(old, CommandState::Superseded(id), now));
        }

        let deadline = now + command.lifetime(&self.cfg);
        self.inflight.insert(
            id,
            InFlight {
                target,
                command,
                state: CommandState::Pending,
                attempts: 0,
                deadline,
                next_send: now,
                interval: self.interval_for(link.state, None),
                link,
                ambiguous_until: same_cmd.then(|| now + self.cfg.retry_max),
            },
        );
        let out = self.transmit(id, link, now).into_iter().collect();
        changed.push(self.status_at(id, now).expect("just inserted"));
        Ok((id, out, changed))
    }

    fn check_approval(
        &mut self,
        target: Target,
        command: &Command,
        approval: &Approval,
        facts: &VehicleFacts,
        now: Instant,
    ) -> Result<(), Refusal> {
        command.validate()?;
        command.preconditions(facts)?;
        let class = command.class();
        let token = match (class, approval) {
            (Class::A | Class::B, _) => return Ok(()),
            (_, Approval::None) => return Err(Refusal::ConfirmationRequired),
            (Class::C, Approval::Confirmed(t)) => *t,
            (Class::C, Approval::Typed { token, .. }) => *token,
            (Class::D, Approval::Confirmed(_)) => return Err(Refusal::TypedMismatch),
            (Class::D, Approval::Typed { token, .. }) => *token,
        };
        // Single use: the token is consumed by any attempt to use it.
        let issued = self
            .tokens
            .remove(&token)
            .ok_or_else(|| Refusal::BadToken("unknown or already used".into()))?;
        if issued.expires <= now {
            return Err(Refusal::BadToken("expired".into()));
        }
        if issued.target != target {
            return Err(Refusal::BadToken("issued for another vehicle".into()));
        }
        if &issued.command != command {
            return Err(Refusal::BadToken("issued for a different command".into()));
        }
        if let (Class::D, Approval::Typed { text, reason, .. }) = (class, approval) {
            if Some(text.trim()) != issued.challenge.as_deref() {
                return Err(Refusal::TypedMismatch);
            }
            if reason.trim().is_empty() {
                return Err(Refusal::ReasonRequired);
            }
        }
        Ok(())
    }

    fn interval_for(&self, link: LinkState, previous: Option<Duration>) -> Duration {
        match (link, previous) {
            (LinkState::Connected, _) => self.cfg.retry_connected,
            (_, None) => self.cfg.retry_degraded,
            (_, Some(p)) => (p.mul_f32(1.5)).clamp(self.cfg.retry_degraded, self.cfg.retry_max),
        }
    }

    /// Next transmission of `id` if one is due. Never at or after the deadline.
    fn transmit(&mut self, id: CommandId, link: LinkInfo, now: Instant) -> Option<Outgoing> {
        let interval_prev = self.inflight.get(&id).map(|f| f.interval);
        let next_interval = self.interval_for(
            link.state,
            interval_prev.filter(|_| self.inflight.get(&id).is_some_and(|f| f.attempts > 0)),
        );
        let f = self.inflight.get_mut(&id)?;
        if f.state != CommandState::Pending || now >= f.deadline || now < f.next_send {
            return None;
        }
        f.attempts += 1;
        f.link = link;
        f.interval = next_interval;
        f.next_send = now + next_interval;
        let msg = f
            .command
            .to_message(f.target, (f.attempts - 1).min(255) as u8);
        let out = Outgoing {
            id,
            msg,
            flood: link.state != LinkState::Connected,
            attempt: f.attempts,
        };
        let (sysid, label) = (
            f.target.sysid,
            label_with(&self.mode_names, f.target.sysid, &f.command),
        );
        let note = if out.flood { ", all links" } else { "" };
        self.log(
            now,
            sysid,
            Some(id),
            format!(
                "sent {label} attempt {} (link {}{note})",
                out.attempt, link.state
            ),
        );
        Some(out)
    }

    /// Advance time: expire, then retransmit what is due. `link(sysid)` gives each vehicle's
    /// link. Returns messages to send and status changes.
    pub fn poll(
        &mut self,
        now: Instant,
        link: impl Fn(u8) -> LinkInfo,
    ) -> (Vec<Outgoing>, Vec<CommandStatus>) {
        let mut out = Vec::new();
        let mut changed = Vec::new();
        let ids: Vec<CommandId> = self
            .inflight
            .iter()
            .filter(|(_, f)| !f.state.is_final())
            .map(|(k, _)| *k)
            .collect();
        for id in ids {
            let (deadline, state, sysid) = {
                let f = &self.inflight[&id];
                (f.deadline, f.state, f.target.sysid)
            };
            if now >= deadline {
                let acked = state != CommandState::Pending;
                changed.push(self.finish(id, CommandState::Expired { acked }, now));
                continue;
            }
            if let Some(o) = self.transmit(id, link(sysid), now) {
                out.push(o);
                changed.push(self.status_at(id, now).expect("live"));
            }
        }
        self.tokens.retain(|_, t| t.expires > now);
        // Forget final commands after a while so the table stays bounded.
        let keep = self.cfg.lifetime_nav * 4;
        self.inflight
            .retain(|_, f| !f.state.is_final() || now.saturating_duration_since(f.deadline) < keep);
        (out, changed)
    }

    /// A `COMMAND_ACK` from `from`.
    pub fn on_ack(
        &mut self,
        from: Target,
        ack: &CommandAck,
        facts: &VehicleFacts,
        now: Instant,
    ) -> Vec<CommandStatus> {
        // The newest live command for that vehicle and MAV_CMD.
        let Some(id) = self
            .inflight
            .iter()
            .rev()
            .find(|(_, f)| {
                f.target.sysid == from.sysid
                    && f.command.mav_cmd() == ack.command
                    && !f.state.is_final()
            })
            .map(|(k, _)| *k)
        else {
            return Vec::new();
        };
        let (label, sysid, ambiguous) = {
            let f = &self.inflight[&id];
            let ambiguous = f.ambiguous_until.is_some_and(|t| now < t);
            (
                label_with(&self.mode_names, f.target.sysid, &f.command),
                f.target.sysid,
                ambiguous,
            )
        };
        if ambiguous {
            self.log(
                now,
                sysid,
                Some(id),
                format!(
                    "ack {label}: {} ignored: may answer a superseded command with the same MAV_CMD",
                    result_label(ack.result)
                ),
            );
            return Vec::new();
        }
        self.log(
            now,
            sysid,
            Some(id),
            format!("ack {label}: {}", result_label(ack.result)),
        );
        let next = match MavResult(ack.result) {
            MavResult::IN_PROGRESS => CommandState::InProgress,
            MavResult::ACCEPTED => {
                let f = &self.inflight[&id];
                match f.command.verified_by(facts) {
                    Some(true) => CommandState::Verified,
                    Some(false) => CommandState::Acknowledged,
                    None => CommandState::Accepted,
                }
            }
            r => CommandState::Rejected(r.0),
        };
        if next.is_final() {
            vec![self.finish(id, next, now)]
        } else {
            if let Some(f) = self.inflight.get_mut(&id) {
                f.state = next;
            }
            vec![self.status_at(id, now).expect("live")]
        }
    }

    /// New vehicle state (e.g. every heartbeat): verify commands whose effect is visible.
    pub fn on_state(
        &mut self,
        sysid: u8,
        facts: &VehicleFacts,
        now: Instant,
    ) -> Vec<CommandStatus> {
        let ids: Vec<CommandId> = self
            .inflight
            .iter()
            .filter(|(_, f)| {
                f.target.sysid == sysid
                    && !f.state.is_final()
                    && f.command.verified_by(facts) == Some(true)
            })
            .map(|(k, _)| *k)
            .collect();
        ids.into_iter()
            .map(|id| self.finish(id, CommandState::Verified, now))
            .collect()
    }

    /// Operator cancel. Stops retransmission; the vehicle may already have acted.
    pub fn cancel(&mut self, id: CommandId, now: Instant) -> Option<CommandStatus> {
        let live = self.inflight.get(&id).is_some_and(|f| !f.state.is_final());
        live.then(|| self.finish(id, CommandState::Cancelled, now))
    }

    fn finish(&mut self, id: CommandId, state: CommandState, now: Instant) -> CommandStatus {
        let f = self.inflight.get_mut(&id).expect("known command");
        f.state = state;
        let (sysid, label) = (
            f.target.sysid,
            label_with(&self.mode_names, f.target.sysid, &f.command),
        );
        let status = self.status_at(id, now).expect("known command");
        self.log(now, sysid, Some(id), format!("{label}: {state:?}"));
        status
    }

    fn status_at(&self, id: CommandId, now: Instant) -> Option<CommandStatus> {
        let f = self.inflight.get(&id)?;
        Some(CommandStatus {
            id,
            target: f.target,
            command: f.command.clone(),
            state: f.state,
            attempts: f.attempts,
            time_left: if f.state.is_final() {
                Duration::ZERO
            } else {
                f.deadline.saturating_duration_since(now)
            },
            link: f.link,
            label: self.label(f.target.sysid, &f.command),
        })
    }

    /// Current status of `id`.
    pub fn status(&self, id: CommandId, now: Instant) -> Option<CommandStatus> {
        self.status_at(id, now)
    }

    /// Audit events since the last call, oldest first.
    pub fn drain_audit(&mut self) -> Vec<AuditEvent> {
        std::mem::take(&mut self.audit)
    }
}

fn result_label(r: u8) -> &'static str {
    match MavResult(r) {
        MavResult::ACCEPTED => "accepted",
        MavResult::IN_PROGRESS => "in progress",
        _ => result_name(r),
    }
}
