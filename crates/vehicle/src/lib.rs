//! Vehicle telemetry state.
//!
//! [`Fleet::apply`] folds incoming messages into one [`VehicleState`] per system id.
//! Every group of fields carries the time it was last updated, so the UI can grey out or
//! invalidate stale values instead of showing old numbers as live (plan 04 §4).
//!
//! Units are SI in the public fields (radians, metres, m/s, volts, amperes) — conversion
//! from MAVLink's scaled integers happens here, once.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use mavlink_codec::Message;
use mavlink_codec::enums::MavModeFlag;

pub mod modes;
pub mod nav;
pub use modes::{Firmware, FirmwareSource};
pub use nav::{Ekf, NavHealth, NavThresholds, PositionTrust};

/// Identity and mode from `HEARTBEAT`.
#[derive(Debug, Clone, PartialEq)]
pub struct Heartbeat {
    /// Component that sent it.
    pub compid: u8,
    /// `MAV_TYPE`.
    pub mav_type: u8,
    /// `MAV_AUTOPILOT`.
    pub autopilot: u8,
    /// `MAV_MODE_FLAG` bits.
    pub base_mode: u8,
    /// Autopilot-specific mode number (ArduPilot flight mode).
    pub custom_mode: u32,
    /// `MAV_STATE`.
    pub system_status: u8,
    /// MAVLink version field.
    pub mavlink_version: u8,
}

impl VehicleState {
    /// Firmware confirmed by its version banner: the only basis for offering or naming modes
    /// to the operator.
    pub fn confirmed_firmware(&self) -> Option<Firmware> {
        match self.firmware {
            Some((f, FirmwareSource::Banner)) => Some(f),
            _ => None,
        }
    }

    /// Name of the current flight mode, if the firmware is confirmed and the mode known.
    pub fn mode_name(&self) -> Option<&'static str> {
        let hb = self.heartbeat.as_ref()?;
        self.confirmed_firmware()?.mode_name(hb.value.custom_mode)
    }
}

impl Heartbeat {
    /// Motors armed (`MAV_MODE_FLAG_SAFETY_ARMED`).
    pub fn armed(&self) -> bool {
        self.base_mode & MavModeFlag::SAFETY_ARMED.0 != 0
    }
}

/// `ATTITUDE`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Attitude {
    /// Roll, rad.
    pub roll: f32,
    /// Pitch, rad.
    pub pitch: f32,
    /// Yaw, rad.
    pub yaw: f32,
    /// Roll rate, rad/s.
    pub roll_rate: f32,
    /// Pitch rate, rad/s.
    pub pitch_rate: f32,
    /// Yaw rate, rad/s.
    pub yaw_rate: f32,
}

/// `GLOBAL_POSITION_INT`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Position {
    /// Latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees.
    pub lon_deg: f64,
    /// Altitude above MSL, m.
    pub alt_msl_m: f32,
    /// Altitude above home, m.
    pub alt_rel_m: f32,
    /// Velocity north/east/down, m/s.
    pub vel_ned: [f32; 3],
    /// Heading, degrees (None when unknown).
    pub heading_deg: Option<f32>,
}

/// `VFR_HUD`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Hud {
    /// Airspeed, m/s.
    pub airspeed: f32,
    /// Ground speed, m/s.
    pub groundspeed: f32,
    /// Altitude (MSL), m.
    pub alt_m: f32,
    /// Climb rate, m/s.
    pub climb: f32,
    /// Heading, degrees.
    pub heading_deg: i16,
    /// Throttle, percent.
    pub throttle_pct: u16,
}

/// Battery and health from `SYS_STATUS`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Power {
    /// Battery voltage, V (None if not measured).
    pub voltage: Option<f32>,
    /// Battery current, A (None if not measured).
    pub current: Option<f32>,
    /// Remaining capacity, percent (None if unknown).
    pub remaining_pct: Option<u8>,
    /// Communication drop rate seen by the vehicle, percent.
    pub drop_rate_pct: f32,
}

/// `GPS_RAW_INT`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Gps {
    /// `GPS_FIX_TYPE` (0 none … 6 RTK fixed).
    pub fix_type: u8,
    /// Satellites visible (None if unknown).
    pub satellites: Option<u8>,
    /// Horizontal dilution of precision (None if unknown).
    pub hdop: Option<f32>,
}

/// A value plus when it was last updated.
#[derive(Debug, Clone, PartialEq)]
pub struct Timed<T> {
    /// Last value.
    pub value: T,
    /// Host time of the update.
    pub at: Instant,
}

impl<T> Timed<T> {
    /// Age at `now`.
    pub fn age(&self, now: Instant) -> Duration {
        now.saturating_duration_since(self.at)
    }
}

/// Status text line (`STATUSTEXT`).
#[derive(Debug, Clone, PartialEq)]
pub struct StatusText {
    /// `MAV_SEVERITY` (0 emergency … 7 debug).
    pub severity: u8,
    /// Text without trailing NULs.
    pub text: String,
}

/// `MAV_LANDED_STATE` from `EXTENDED_SYS_STATE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LandedState {
    /// The autopilot does not know (`MAV_LANDED_STATE_UNDEFINED`, or a value we do not know).
    Undefined,
    /// On the ground.
    OnGround,
    /// Flying.
    InAir,
    /// Taking off.
    Takeoff,
    /// Landing.
    Landing,
}

impl LandedState {
    /// From the MAVLink value.
    pub fn from_mavlink(v: u8) -> Self {
        match v {
            1 => LandedState::OnGround,
            2 => LandedState::InAir,
            3 => LandedState::Takeoff,
            4 => LandedState::Landing,
            _ => LandedState::Undefined,
        }
    }
    /// The MAVLink value.
    pub fn to_mavlink(self) -> u8 {
        match self {
            LandedState::Undefined => 0,
            LandedState::OnGround => 1,
            LandedState::InAir => 2,
            LandedState::Takeoff => 3,
            LandedState::Landing => 4,
        }
    }
}

/// `MAV_VTOL_STATE` from `EXTENDED_SYS_STATE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VtolState {
    /// Not a VTOL, or unknown.
    Undefined,
    /// Transitioning to fixed-wing flight.
    TransitionToFw,
    /// Transitioning to multicopter flight.
    TransitionToMc,
    /// Multicopter flight.
    Mc,
    /// Fixed-wing flight.
    Fw,
}

impl VtolState {
    /// From the MAVLink value.
    pub fn from_mavlink(v: u8) -> Self {
        match v {
            1 => VtolState::TransitionToFw,
            2 => VtolState::TransitionToMc,
            3 => VtolState::Mc,
            4 => VtolState::Fw,
            _ => VtolState::Undefined,
        }
    }
}

/// Flight phase from `EXTENDED_SYS_STATE` (autopilot component only).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Extended {
    /// Landed state.
    pub landed: LandedState,
    /// VTOL state.
    pub vtol: VtolState,
}

/// `MISSION_STATE` from `MISSION_CURRENT`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionState {
    /// Not reported (`MISSION_STATE_UNKNOWN`, a MAVLink 1 sender, or a value we do not know).
    Unknown,
    /// No mission on the vehicle.
    NoMission,
    /// A mission is loaded and has not started.
    NotStarted,
    /// Running.
    Active,
    /// Paused (vehicle left the mission mode, or the mission was paused).
    Paused,
    /// Finished.
    Complete,
}

impl MissionState {
    /// From the MAVLink value.
    pub fn from_mavlink(v: u8) -> Self {
        match v {
            1 => MissionState::NoMission,
            2 => MissionState::NotStarted,
            3 => MissionState::Active,
            4 => MissionState::Paused,
            5 => MissionState::Complete,
            _ => MissionState::Unknown,
        }
    }
}

/// Mission progress from `MISSION_CURRENT` (autopilot component only).
///
/// `current` is the vehicle's sequence number, as uploaded: on ArduPilot item 0 is home, so the
/// first uploaded mission item is 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MissionProgress {
    /// Item being executed (or to execute next).
    pub current: u16,
    /// Items in the mission, excluding a home item; `None` if not reported (0 or a MAVLink 1
    /// sender), `Some(0)` if the vehicle has no mission (`UINT16_MAX`).
    pub total: Option<u16>,
    /// Mission state machine state.
    pub state: MissionState,
    /// In a mode that runs the mission (`Some(true)`), suspended (`Some(false)`), or not
    /// reported (`None`).
    pub in_mission_mode: Option<bool>,
}

/// Everything known about one system.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct VehicleState {
    /// MAVLink system id.
    pub sysid: u8,
    /// Last heartbeat from the autopilot component.
    pub heartbeat: Option<Timed<Heartbeat>>,
    /// Attitude.
    pub attitude: Option<Timed<Attitude>>,
    /// Position.
    pub position: Option<Timed<Position>>,
    /// HUD values.
    pub hud: Option<Timed<Hud>>,
    /// Power.
    pub power: Option<Timed<Power>>,
    /// GPS.
    pub gps: Option<Timed<Gps>>,
    /// Landed and VTOL state.
    pub extended: Option<Timed<Extended>>,
    /// EKF status (see [`nav`]).
    pub ekf: Option<Timed<Ekf>>,
    /// Mission progress.
    pub mission: Option<Timed<MissionProgress>>,
    /// Last mission item the vehicle reported reached (`MISSION_ITEM_REACHED`).
    pub mission_reached: Option<Timed<u16>>,
    /// Firmware family and how it was identified (see [`modes`]).
    pub firmware: Option<(Firmware, FirmwareSource)>,
    /// Messages applied, by message id.
    pub message_counts: BTreeMap<u32, u64>,
}

/// Which component's HEARTBEAT defines the vehicle. Autopilots are component 1; other
/// components (cameras, gimbals) also send heartbeats and must not overwrite mode/armed.
pub const AUTOPILOT_COMPID: u8 = 1;
/// `MAV_TYPE_GCS`: heartbeats from ground stations are not vehicles.
pub const MAV_TYPE_GCS: u8 = 6;

fn timed<T>(at: Instant, value: T) -> Option<Timed<T>> {
    Some(Timed { value, at })
}

/// State of every system heard.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Fleet {
    vehicles: BTreeMap<u8, VehicleState>,
}

impl Fleet {
    /// Empty fleet.
    pub fn new() -> Self {
        Self::default()
    }

    /// State of `sysid`, if heard.
    pub fn get(&self, sysid: u8) -> Option<&VehicleState> {
        self.vehicles.get(&sysid)
    }

    /// All systems heard, by id.
    pub fn vehicles(&self) -> impl Iterator<Item = &VehicleState> {
        self.vehicles.values()
    }

    /// Fold one message into the fleet. Returns the status text, if this was one, so the
    /// caller can raise it as an event. Messages from ground stations are ignored.
    pub fn apply(
        &mut self,
        sysid: u8,
        compid: u8,
        msg: &Message,
        now: Instant,
    ) -> Option<StatusText> {
        if let Message::Heartbeat(h) = msg
            && h.r#type == MAV_TYPE_GCS
        {
            return None;
        }
        let v = self.vehicles.entry(sysid).or_insert_with(|| VehicleState {
            sysid,
            ..Default::default()
        });
        *v.message_counts.entry(msg.id()).or_default() += 1;
        match msg {
            Message::Heartbeat(h) if compid == AUTOPILOT_COMPID || v.heartbeat.is_none() => {
                if compid == AUTOPILOT_COMPID
                    && !matches!(v.firmware, Some((_, FirmwareSource::Banner)))
                {
                    v.firmware = Firmware::from_mav_type(h.r#type, h.autopilot)
                        .map(|f| (f, FirmwareSource::MavType));
                }
                v.heartbeat = timed(
                    now,
                    Heartbeat {
                        compid,
                        mav_type: h.r#type,
                        autopilot: h.autopilot,
                        base_mode: h.base_mode,
                        custom_mode: h.custom_mode,
                        system_status: h.system_status,
                        mavlink_version: h.mavlink_version,
                    },
                );
            }
            Message::Attitude(a) => {
                v.attitude = timed(
                    now,
                    Attitude {
                        roll: a.roll,
                        pitch: a.pitch,
                        yaw: a.yaw,
                        roll_rate: a.rollspeed,
                        pitch_rate: a.pitchspeed,
                        yaw_rate: a.yawspeed,
                    },
                );
            }
            Message::GlobalPositionInt(p) => {
                v.position = timed(
                    now,
                    Position {
                        lat_deg: f64::from(p.lat) * 1e-7,
                        lon_deg: f64::from(p.lon) * 1e-7,
                        alt_msl_m: p.alt as f32 / 1000.0,
                        alt_rel_m: p.relative_alt as f32 / 1000.0,
                        vel_ned: [
                            f32::from(p.vx) / 100.0,
                            f32::from(p.vy) / 100.0,
                            f32::from(p.vz) / 100.0,
                        ],
                        heading_deg: (p.hdg != u16::MAX).then(|| f32::from(p.hdg) / 100.0),
                    },
                );
            }
            Message::VfrHud(h) => {
                v.hud = timed(
                    now,
                    Hud {
                        airspeed: h.airspeed,
                        groundspeed: h.groundspeed,
                        alt_m: h.alt,
                        climb: h.climb,
                        heading_deg: h.heading,
                        throttle_pct: h.throttle,
                    },
                );
            }
            Message::SysStatus(s) => {
                v.power = timed(
                    now,
                    Power {
                        voltage: (s.voltage_battery != u16::MAX)
                            .then(|| f32::from(s.voltage_battery) / 1000.0),
                        current: (s.current_battery != -1)
                            .then(|| f32::from(s.current_battery) / 100.0),
                        remaining_pct: u8::try_from(s.battery_remaining).ok(),
                        drop_rate_pct: f32::from(s.drop_rate_comm) / 100.0,
                    },
                );
            }
            Message::GpsRawInt(g) => {
                v.gps = timed(
                    now,
                    Gps {
                        fix_type: g.fix_type,
                        satellites: (g.satellites_visible != u8::MAX)
                            .then_some(g.satellites_visible),
                        hdop: (g.eph != u16::MAX).then(|| f32::from(g.eph) / 100.0),
                    },
                );
            }
            Message::EkfStatusReport(e) if compid == AUTOPILOT_COMPID => {
                v.ekf = timed(
                    now,
                    Ekf {
                        flags: e.flags,
                        velocity_var: e.velocity_variance,
                        pos_horiz_var: e.pos_horiz_variance,
                        pos_vert_var: e.pos_vert_variance,
                        compass_var: e.compass_variance,
                    },
                );
            }
            Message::ExtendedSysState(e) if compid == AUTOPILOT_COMPID => {
                v.extended = timed(
                    now,
                    Extended {
                        landed: LandedState::from_mavlink(e.landed_state),
                        vtol: VtolState::from_mavlink(e.vtol_state),
                    },
                );
            }
            Message::MissionCurrent(m) if compid == AUTOPILOT_COMPID => {
                v.mission = timed(
                    now,
                    MissionProgress {
                        current: m.seq,
                        total: match m.total {
                            0 => None,
                            u16::MAX => Some(0),
                            n => Some(n),
                        },
                        state: MissionState::from_mavlink(m.mission_state),
                        in_mission_mode: match m.mission_mode {
                            1 => Some(true),
                            2 => Some(false),
                            _ => None,
                        },
                    },
                );
            }
            Message::MissionItemReached(m) if compid == AUTOPILOT_COMPID => {
                v.mission_reached = timed(now, m.seq);
            }
            Message::Statustext(s) => {
                let end = s.text.iter().position(|&b| b == 0).unwrap_or(s.text.len());
                if compid == AUTOPILOT_COMPID
                    && let Some(f) = Firmware::from_banner(&String::from_utf8_lossy(&s.text[..end]))
                {
                    v.firmware = Some((f, FirmwareSource::Banner));
                }
                return Some(StatusText {
                    severity: s.severity,
                    text: String::from_utf8_lossy(&s.text[..end]).into_owned(),
                });
            }
            _ => {}
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::messages::{GlobalPositionInt, Heartbeat as Hb, Statustext, SysStatus};

    #[test]
    fn folds_messages_into_si_units() {
        let mut f = Fleet::new();
        let now = Instant::now();
        let hb = Hb {
            r#type: 2,
            autopilot: 3,
            base_mode: 128 | 1,
            custom_mode: 5,
            ..Default::default()
        };
        f.apply(1, 1, &Message::Heartbeat(hb), now);
        f.apply(
            1,
            1,
            &Message::GlobalPositionInt(GlobalPositionInt {
                lat: 473_977_418,
                lon: 85_455_939,
                alt: 488_000,
                relative_alt: 10_500,
                vx: 150,
                hdg: u16::MAX,
                ..Default::default()
            }),
            now,
        );
        f.apply(
            1,
            1,
            &Message::SysStatus(SysStatus {
                voltage_battery: 12_600,
                current_battery: -1,
                battery_remaining: -1,
                ..Default::default()
            }),
            now,
        );
        let v = f.get(1).unwrap();
        let hb = &v.heartbeat.as_ref().unwrap().value;
        assert!(hb.armed());
        assert_eq!(hb.custom_mode, 5);
        let p = v.position.as_ref().unwrap().value;
        assert!((p.lat_deg - 47.397_741_8).abs() < 1e-9);
        assert_eq!(
            (p.alt_rel_m, p.vel_ned[0], p.heading_deg),
            (10.5, 1.5, None)
        );
        let pw = v.power.as_ref().unwrap().value;
        assert_eq!(
            (pw.voltage, pw.current, pw.remaining_pct),
            (Some(12.6), None, None)
        );
        assert_eq!(v.message_counts[&0], 1);
    }

    #[test]
    fn ignores_gcs_and_non_autopilot_heartbeats() {
        let mut f = Fleet::new();
        let now = Instant::now();
        f.apply(
            255,
            190,
            &Message::Heartbeat(Hb {
                r#type: MAV_TYPE_GCS,
                ..Default::default()
            }),
            now,
        );
        assert!(f.get(255).is_none(), "ground stations are not vehicles");

        f.apply(
            1,
            1,
            &Message::Heartbeat(Hb {
                custom_mode: 4,
                ..Default::default()
            }),
            now,
        );
        f.apply(
            1,
            100,
            &Message::Heartbeat(Hb {
                custom_mode: 99,
                ..Default::default()
            }),
            now,
        );
        assert_eq!(
            f.get(1)
                .unwrap()
                .heartbeat
                .as_ref()
                .unwrap()
                .value
                .custom_mode,
            4,
            "camera must not overwrite mode"
        );
    }

    #[test]
    fn returns_status_text() {
        let mut f = Fleet::new();
        let mut s = Statustext {
            severity: 3,
            ..Default::default()
        };
        s.text[..14].copy_from_slice(b"PreArm: RC not");
        let t = f
            .apply(1, 1, &Message::Statustext(s), Instant::now())
            .unwrap();
        assert_eq!((t.severity, t.text.as_str()), (3, "PreArm: RC not"));
    }

    #[test]
    fn landed_state_comes_from_the_autopilot_only() {
        use mavlink_codec::messages::ExtendedSysState;
        let mut f = Fleet::new();
        let now = Instant::now();
        let ext = |landed, vtol| {
            Message::ExtendedSysState(ExtendedSysState {
                vtol_state: vtol,
                landed_state: landed,
            })
        };
        f.apply(1, 1, &ext(1, 0), now);
        let e = f.get(1).unwrap().extended.clone().unwrap().value;
        assert_eq!(
            (e.landed, e.vtol),
            (LandedState::OnGround, VtolState::Undefined)
        );
        // A camera or gimbal (another component) must not change the flight phase.
        f.apply(1, 100, &ext(2, 0), now);
        assert_eq!(
            f.get(1).unwrap().extended.clone().unwrap().value.landed,
            LandedState::OnGround
        );
        f.apply(1, 1, &ext(2, 4), now);
        let e = f.get(1).unwrap().extended.clone().unwrap().value;
        assert_eq!((e.landed, e.vtol), (LandedState::InAir, VtolState::Fw));
        // Unknown values are Undefined, never guessed.
        assert_eq!(LandedState::from_mavlink(9), LandedState::Undefined);
        assert_eq!(VtolState::from_mavlink(9), VtolState::Undefined);
    }

    #[test]
    fn mode_names_need_the_banner() {
        use mavlink_codec::messages::{Heartbeat as Hb, Statustext};
        let mut f = Fleet::new();
        let now = Instant::now();
        // A QuadPlane whose Q_MAV_TYPE says "quadrotor", in mode 5.
        let hb = Hb {
            r#type: 2,
            autopilot: 3,
            custom_mode: 5,
            ..Default::default()
        };
        f.apply(1, 1, &Message::Heartbeat(hb.clone()), now);
        let v = f.get(1).unwrap();
        assert_eq!(
            v.firmware,
            Some((Firmware::Copter, FirmwareSource::MavType))
        );
        assert_eq!(v.mode_name(), None, "a guess never names a mode");
        let mut text = [0u8; 50];
        let banner = b"ArduPlane V4.7.1 (3f2a9c1e)";
        text[..banner.len()].copy_from_slice(banner);
        let st = Statustext {
            severity: 6,
            text,
            ..Default::default()
        };
        // From another component: ignored.
        f.apply(1, 100, &Message::Statustext(st.clone()), now);
        assert_eq!(f.get(1).unwrap().confirmed_firmware(), None);
        f.apply(1, 1, &Message::Statustext(st), now);
        f.apply(1, 1, &Message::Heartbeat(hb), now);
        let v = f.get(1).unwrap();
        assert_eq!(
            v.confirmed_firmware(),
            Some(Firmware::Plane),
            "banner wins, and sticks"
        );
        assert_eq!(v.mode_name(), Some("FLY_BY_WIRE_A"), "not LOITER");
    }

    #[test]
    fn mission_progress_from_the_autopilot() {
        use mavlink_codec::messages::{MissionCurrent, MissionItemReached};
        let mut f = Fleet::new();
        let now = Instant::now();
        let cur = |seq, total, mission_state, mission_mode| {
            Message::MissionCurrent(MissionCurrent {
                seq,
                total,
                mission_state,
                mission_mode,
            })
        };
        // A camera's MISSION_CURRENT is not the vehicle's.
        f.apply(1, 100, &cur(4, 9, 3, 1), now);
        assert_eq!(f.get(1).unwrap().mission, None);
        f.apply(1, 1, &cur(2, 5, 3, 1), now);
        f.apply(
            1,
            1,
            &Message::MissionItemReached(MissionItemReached { seq: 1 }),
            now,
        );
        let v = f.get(1).unwrap();
        assert_eq!(
            v.mission.as_ref().unwrap().value,
            MissionProgress {
                current: 2,
                total: Some(5),
                state: MissionState::Active,
                in_mission_mode: Some(true),
            }
        );
        assert_eq!(v.mission_reached.as_ref().unwrap().value, 1);
        // MAVLink 1 sender: extension fields read as zero.
        f.apply(1, 1, &cur(0, 0, 0, 0), now);
        let m = f.get(1).unwrap().mission.as_ref().unwrap().value;
        assert_eq!(
            (m.total, m.state, m.in_mission_mode),
            (None, MissionState::Unknown, None)
        );
        f.apply(1, 1, &cur(0, u16::MAX, 1, 2), now);
        let m = f.get(1).unwrap().mission.as_ref().unwrap().value;
        assert_eq!(
            (m.total, m.state, m.in_mission_mode),
            (Some(0), MissionState::NoMission, Some(false))
        );
    }
}
