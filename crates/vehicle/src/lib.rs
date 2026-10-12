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

/// Home position from `HOME_POSITION` (autopilot component only): where RTL returns to and
/// what "altitude above home" is measured from.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Home {
    /// Latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees.
    pub lon_deg: f64,
    /// Altitude above MSL, m.
    pub alt_msl_m: f32,
}

/// One aircraft from `ADSB_VEHICLE`, as reported through this vehicle (its ADS-B receiver).
/// Each optional field is `None` unless the message's own `ADSB_FLAGS` mark it valid.
#[derive(Debug, Clone, PartialEq)]
pub struct Traffic {
    /// ICAO address.
    pub icao: u32,
    /// Latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees.
    pub lon_deg: f64,
    /// Altitude above MSL, m, and whether it is geometric (GNSS) rather than pressure (QNH).
    pub alt_msl_m: Option<(f32, bool)>,
    /// Course over ground, degrees.
    pub heading_deg: Option<f32>,
    /// Horizontal speed, m/s.
    pub speed_ms: Option<f32>,
    /// Vertical speed, m/s, positive up.
    pub climb_ms: Option<f32>,
    /// Callsign.
    pub callsign: Option<String>,
    /// Squawk code (as the decimal digits, e.g. 7700).
    pub squawk: Option<u16>,
    /// `ADSB_EMITTER_TYPE`.
    pub emitter_type: u8,
    /// Seconds since the receiver last heard it (`tslc`).
    pub tslc_s: u8,
    /// `ADSB_FLAGS_SIMULATED`: test traffic, not a real aircraft.
    pub simulated: bool,
}

/// Traffic not reported for this long is forgotten.
pub const TRAFFIC_FORGET: Duration = Duration::from_secs(60);

impl Traffic {
    /// From `ADSB_VEHICLE`; `None` without valid coordinates (nothing to place).
    pub fn from_mavlink(a: &mavlink_codec::messages::AdsbVehicle) -> Option<Traffic> {
        let valid = |bit: u16| a.flags & bit != 0;
        if !valid(1) {
            return None;
        }
        let end = a
            .callsign
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(a.callsign.len());
        let callsign = String::from_utf8_lossy(&a.callsign[..end])
            .trim()
            .to_string();
        Some(Traffic {
            icao: a.ICAO_address,
            lat_deg: f64::from(a.lat) * 1e-7,
            lon_deg: f64::from(a.lon) * 1e-7,
            alt_msl_m: valid(2).then(|| (a.altitude as f32 / 1000.0, a.altitude_type == 1)),
            heading_deg: valid(4).then(|| f32::from(a.heading) / 100.0),
            speed_ms: valid(8).then(|| f32::from(a.hor_velocity) / 100.0),
            climb_ms: valid(128).then(|| f32::from(a.ver_velocity) / 100.0),
            callsign: (valid(16) && !callsign.is_empty()).then_some(callsign),
            squawk: valid(32).then_some(a.squawk),
            emitter_type: a.emitter_type,
            tslc_s: a.tslc,
            simulated: valid(64),
        })
    }
}

/// One battery from `BATTERY_STATUS` (autopilot component). Unknown values are `None`.
#[derive(Debug, Clone, PartialEq)]
pub struct Battery {
    /// Battery id (instance).
    pub id: u8,
    /// `MAV_BATTERY_FUNCTION`.
    pub function: u8,
    /// `MAV_BATTERY_TYPE` (chemistry).
    pub chemistry: u8,
    /// Temperature, °C.
    pub temperature_c: Option<f32>,
    /// Pack voltage, V: the sum of the valid entries (see [`Battery::from_mavlink`]).
    pub voltage_v: Option<f32>,
    /// Cell voltages, V, when the monitor measures cells.
    pub cells_v: Vec<f32>,
    /// Current, A.
    pub current_a: Option<f32>,
    /// Charge consumed, mAh.
    pub consumed_mah: Option<i32>,
    /// Energy consumed, J.
    pub consumed_j: Option<f64>,
    /// Remaining, percent.
    pub remaining_pct: Option<u8>,
    /// Remaining time estimate, s.
    pub time_remaining_s: Option<u32>,
    /// `MAV_BATTERY_CHARGE_STATE` (0 undefined, 1 OK, 2 low, 3 critical, 4 emergency, …).
    pub charge_state: u8,
    /// `MAV_BATTERY_FAULT` bits.
    pub fault_bitmask: u32,
}

/// `UINT16_MAX` - 1: the largest value one voltage entry holds. ArduPilot (and the MAVLink
/// definition) splits a pack voltage above it across entries; no cell reads 65.534 V.
const VOLTAGE_ENTRY_MAX_MV: u16 = 0xFFFE;

impl Battery {
    /// From `BATTERY_STATUS`. Voltages follow ArduPilot's `send_battery_status` (4.7.1) and
    /// the MAVLink definition: with cell measurement the entries are cells; without it the
    /// pack voltage is in entry 0 (overflowing into entry 1 above 65.534 V). Either way the
    /// pack voltage is the sum of the valid entries. Entries are read as cells only when two
    /// or more are valid and none is the overflow value; a single entry is the pack.
    pub fn from_mavlink(b: &mavlink_codec::messages::BatteryStatus) -> Battery {
        let valid: Vec<u16> = b
            .voltages
            .iter()
            .copied()
            .filter(|&mv| mv != u16::MAX)
            .chain(
                b.voltages_ext
                    .iter()
                    .copied()
                    .filter(|&mv| mv != 0 && mv != u16::MAX),
            )
            .collect();
        let voltage_v = (!valid.is_empty())
            .then(|| valid.iter().map(|&mv| f32::from(mv)).sum::<f32>() / 1000.0);
        let cells_v = if valid.len() >= 2 && !valid.contains(&VOLTAGE_ENTRY_MAX_MV) {
            valid.iter().map(|&mv| f32::from(mv) / 1000.0).collect()
        } else {
            Vec::new()
        };
        Battery {
            id: b.id,
            function: b.battery_function,
            chemistry: b.r#type,
            temperature_c: (b.temperature != i16::MAX).then(|| f32::from(b.temperature) / 100.0),
            voltage_v,
            cells_v,
            current_a: (b.current_battery != -1).then(|| f32::from(b.current_battery) / 100.0),
            consumed_mah: (b.current_consumed != -1).then_some(b.current_consumed),
            consumed_j: (b.energy_consumed != -1).then(|| f64::from(b.energy_consumed) * 100.0),
            remaining_pct: u8::try_from(b.battery_remaining).ok(),
            time_remaining_s: u32::try_from(b.time_remaining).ok().filter(|&s| s > 0),
            charge_state: b.charge_state,
            fault_bitmask: b.fault_bitmask,
        }
    }
}

/// `VIBRATION` (autopilot component): ArduPilot's accelerometer vibration levels, the same
/// values its dataflash log records as `VIBE` (`AP_InertialSensor::get_vibration_levels`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vibration {
    /// X, Y, Z levels, m/s².
    pub levels: [f32; 3],
    /// Clipping count per accelerometer (cumulative since boot).
    pub clipping: [u32; 3],
}

/// `RC_CHANNELS` (autopilot component): what the RC receiver delivers.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct RcInput {
    /// Channel values, µs, for the channels received (at most 18 in this message).
    pub channels: Vec<u16>,
    /// Channels the receiver reports (can exceed 18).
    pub count: u8,
    /// Signal strength, 0–254 in the receiver's own scale; `None` if unknown (255).
    pub rssi: Option<u8>,
}

/// `SERVO_OUTPUT_RAW` (autopilot component): outputs 1–16 of one port, µs (0 = not driven).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ServoOutputs {
    /// Output values, µs.
    pub outputs: [u16; 16],
}

/// `WIND` (ArduPilot, autopilot component): the autopilot's wind estimate.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Wind {
    /// Direction the wind comes from, degrees.
    pub from_deg: f32,
    /// Horizontal speed, m/s.
    pub speed_ms: f32,
    /// Vertical speed, m/s.
    pub speed_z_ms: f32,
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
    /// Home position.
    pub home: Option<Timed<Home>>,
    /// Vibration levels and clipping.
    pub vibration: Option<Timed<Vibration>>,
    /// Wind estimate.
    pub wind: Option<Timed<Wind>>,
    /// RC input.
    pub rc: Option<Timed<RcInput>>,
    /// Servo/motor outputs by port (0 = MAIN).
    pub servos: BTreeMap<u8, Timed<ServoOutputs>>,
    /// Batteries from `BATTERY_STATUS`, by id.
    pub batteries: BTreeMap<u8, Timed<Battery>>,
    /// ADS-B traffic reported through this vehicle, by ICAO address (forgotten after
    /// [`TRAFFIC_FORGET`] without a report).
    pub traffic: BTreeMap<u32, Timed<Traffic>>,
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
        // Forget traffic on any message, so it ages out even when the receiver goes quiet.
        if !v.traffic.is_empty() {
            v.traffic.retain(|_, t| t.age(now) < TRAFFIC_FORGET);
        }
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
                let state = MissionState::from_mavlink(m.mission_state);
                v.mission = timed(
                    now,
                    MissionProgress {
                        current: m.seq,
                        // 0 means "not supported", except with NO_MISSION (ArduPilot 4.7.1
                        // reports an empty mission so).
                        total: match m.total {
                            0 if state == MissionState::NoMission => Some(0),
                            0 => None,
                            u16::MAX => Some(0),
                            n => Some(n),
                        },
                        state,
                        in_mission_mode: match m.mission_mode {
                            1 => Some(true),
                            2 => Some(false),
                            _ => None,
                        },
                    },
                );
            }
            // From any component: an ADS-B receiver is often its own component.
            Message::AdsbVehicle(a) => {
                if let Some(t) = Traffic::from_mavlink(a) {
                    v.traffic.insert(t.icao, Timed { value: t, at: now });
                }
            }
            Message::BatteryStatus(b) if compid == AUTOPILOT_COMPID => {
                v.batteries.insert(
                    b.id,
                    Timed {
                        value: Battery::from_mavlink(b),
                        at: now,
                    },
                );
            }
            Message::Vibration(m) if compid == AUTOPILOT_COMPID => {
                v.vibration = timed(
                    now,
                    Vibration {
                        levels: [m.vibration_x, m.vibration_y, m.vibration_z],
                        clipping: [m.clipping_0, m.clipping_1, m.clipping_2],
                    },
                );
            }
            Message::RcChannels(r) if compid == AUTOPILOT_COMPID => {
                let all = [
                    r.chan1_raw,
                    r.chan2_raw,
                    r.chan3_raw,
                    r.chan4_raw,
                    r.chan5_raw,
                    r.chan6_raw,
                    r.chan7_raw,
                    r.chan8_raw,
                    r.chan9_raw,
                    r.chan10_raw,
                    r.chan11_raw,
                    r.chan12_raw,
                    r.chan13_raw,
                    r.chan14_raw,
                    r.chan15_raw,
                    r.chan16_raw,
                    r.chan17_raw,
                    r.chan18_raw,
                ];
                v.rc = timed(
                    now,
                    RcInput {
                        channels: all[..usize::from(r.chancount).min(18)].to_vec(),
                        count: r.chancount,
                        rssi: (r.rssi != u8::MAX).then_some(r.rssi),
                    },
                );
            }
            Message::ServoOutputRaw(s) if compid == AUTOPILOT_COMPID => {
                v.servos.insert(
                    s.port,
                    Timed {
                        value: ServoOutputs {
                            outputs: [
                                s.servo1_raw,
                                s.servo2_raw,
                                s.servo3_raw,
                                s.servo4_raw,
                                s.servo5_raw,
                                s.servo6_raw,
                                s.servo7_raw,
                                s.servo8_raw,
                                s.servo9_raw,
                                s.servo10_raw,
                                s.servo11_raw,
                                s.servo12_raw,
                                s.servo13_raw,
                                s.servo14_raw,
                                s.servo15_raw,
                                s.servo16_raw,
                            ],
                        },
                        at: now,
                    },
                );
            }
            Message::Wind(w) if compid == AUTOPILOT_COMPID => {
                v.wind = timed(
                    now,
                    Wind {
                        from_deg: w.direction,
                        speed_ms: w.speed,
                        speed_z_ms: w.speed_z,
                    },
                );
            }
            Message::HomePosition(h) if compid == AUTOPILOT_COMPID => {
                v.home = timed(
                    now,
                    Home {
                        lat_deg: f64::from(h.latitude) * 1e-7,
                        lon_deg: f64::from(h.longitude) * 1e-7,
                        alt_msl_m: h.altitude as f32 / 1000.0,
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
    fn traffic_keeps_only_valid_fields_and_is_forgotten() {
        use mavlink_codec::messages::AdsbVehicle;
        let mut f = Fleet::new();
        let t0 = Instant::now();
        let mut callsign = [0u8; 9];
        callsign[..7].copy_from_slice(b"DLH4AB ");
        let a = AdsbVehicle {
            ICAO_address: 0x3C_4A_D2,
            lat: -353_600_000,
            lon: 1_491_700_000,
            altitude_type: 1,
            altitude: 1_524_000,
            heading: 27_000,
            hor_velocity: 12_000,
            ver_velocity: -500,
            callsign,
            emitter_type: 3,
            tslc: 1,
            flags: 1 | 2 | 4 | 8 | 16, // no squawk, no vertical speed
            squawk: 7700,
        };
        f.apply(1, 156, &Message::AdsbVehicle(a.clone()), t0);
        let t = &f.get(1).unwrap().traffic[&0x3C_4A_D2].value;
        assert_eq!(t.alt_msl_m, Some((1524.0, true)));
        assert_eq!((t.heading_deg, t.speed_ms), (Some(270.0), Some(120.0)));
        assert_eq!(t.callsign.as_deref(), Some("DLH4AB"));
        assert_eq!((t.squawk, t.climb_ms, t.simulated), (None, None, false));

        // Without valid coordinates there is nothing to place.
        let no_pos = AdsbVehicle {
            ICAO_address: 1,
            flags: 2,
            ..a.clone()
        };
        f.apply(1, 156, &Message::AdsbVehicle(no_pos), t0);
        assert!(!f.get(1).unwrap().traffic.contains_key(&1));

        // Another aircraft 61 s later: the first is forgotten.
        let other = AdsbVehicle {
            ICAO_address: 2,
            ..a
        };
        f.apply(
            1,
            156,
            &Message::AdsbVehicle(other),
            t0 + Duration::from_secs(61),
        );
        let keys: Vec<_> = f.get(1).unwrap().traffic.keys().copied().collect();
        assert_eq!(keys, [2]);
        // The receiver goes quiet: any later message ages the last one out too.
        f.apply(
            1,
            1,
            &Message::Heartbeat(Hb::default()),
            t0 + Duration::from_secs(122),
        );
        assert!(f.get(1).unwrap().traffic.is_empty());
    }

    #[test]
    fn battery_status_cells_or_pack() {
        use mavlink_codec::messages::BatteryStatus;
        let unknown = BatteryStatus {
            temperature: i16::MAX,
            voltages: [u16::MAX; 10],
            current_battery: -1,
            current_consumed: -1,
            energy_consumed: -1,
            battery_remaining: -1,
            ..Default::default()
        };
        // No cell measurement: the pack voltage is in entry 0 (ArduPilot).
        let mut pack = unknown.clone();
        pack.voltages[0] = 16_020;
        let b = Battery::from_mavlink(&pack);
        assert_eq!((b.voltage_v, b.cells_v.len()), (Some(16.02), 0));
        assert_eq!(
            (
                b.temperature_c,
                b.current_a,
                b.remaining_pct,
                b.time_remaining_s
            ),
            (None, None, None, None)
        );
        // A 100 V pack overflows entry 0 into entry 1: still the pack, not two cells.
        let mut big = unknown.clone();
        big.voltages[0] = 0xFFFE;
        big.voltages[1] = 34_466; // 100 000 mV - 65 534 mV
        let b = Battery::from_mavlink(&big);
        assert_eq!((b.voltage_v, b.cells_v.len()), (Some(100.0), 0));
        // 4 measured cells, plus the other fields.
        let mut cells = unknown;
        cells.voltages[..4].copy_from_slice(&[4_010, 4_020, 4_000, 4_030]);
        cells.temperature = 2_550;
        cells.current_battery = 1_234;
        cells.current_consumed = 850;
        cells.energy_consumed = 1_200;
        cells.battery_remaining = 72;
        cells.time_remaining = 600;
        cells.charge_state = 1;
        let b = Battery::from_mavlink(&cells);
        assert_eq!(b.cells_v, [4.01, 4.02, 4.0, 4.03]);
        assert!((b.voltage_v.unwrap() - 16.06).abs() < 1e-4);
        assert_eq!(
            (
                b.temperature_c,
                b.current_a,
                b.consumed_mah,
                b.consumed_j,
                b.remaining_pct,
                b.time_remaining_s
            ),
            (
                Some(25.5),
                Some(12.34),
                Some(850),
                Some(120_000.0),
                Some(72),
                Some(600)
            )
        );
        let mut f = Fleet::new();
        f.apply(1, 1, &Message::BatteryStatus(cells), Instant::now());
        assert_eq!(f.get(1).unwrap().batteries.len(), 1);
    }

    #[test]
    fn vibration_and_wind_from_the_autopilot() {
        use mavlink_codec::messages::{Vibration as Vib, Wind as W};
        let mut f = Fleet::new();
        let now = Instant::now();
        let vib = Message::Vibration(Vib {
            vibration_x: 3.5,
            vibration_y: 4.0,
            vibration_z: 12.0,
            clipping_0: 2,
            ..Default::default()
        });
        f.apply(1, 50, &vib, now);
        assert!(f.get(1).unwrap().vibration.is_none(), "autopilot only");
        f.apply(1, 1, &vib, now);
        f.apply(
            1,
            1,
            &Message::Wind(W {
                direction: 270.0,
                speed: 6.5,
                speed_z: -0.2,
            }),
            now,
        );
        let v = f.get(1).unwrap();
        let vb = v.vibration.as_ref().unwrap().value;
        assert_eq!((vb.levels, vb.clipping), ([3.5, 4.0, 12.0], [2, 0, 0]));
        assert_eq!(v.wind.as_ref().unwrap().value.from_deg, 270.0);
    }

    #[test]
    fn rc_input_and_servo_outputs() {
        use mavlink_codec::messages::{RcChannels, ServoOutputRaw};
        let mut f = Fleet::new();
        let now = Instant::now();
        f.apply(
            1,
            1,
            &Message::RcChannels(RcChannels {
                chancount: 4,
                chan1_raw: 1500,
                chan2_raw: 1500,
                chan3_raw: 1000,
                chan4_raw: 1500,
                chan5_raw: 1800,
                rssi: 255,
                ..Default::default()
            }),
            now,
        );
        f.apply(
            1,
            1,
            &Message::ServoOutputRaw(ServoOutputRaw {
                servo1_raw: 1100,
                servo4_raw: 1150,
                ..Default::default()
            }),
            now,
        );
        let v = f.get(1).unwrap();
        let rc = &v.rc.as_ref().unwrap().value;
        assert_eq!(
            rc.channels,
            [1500, 1500, 1000, 1500],
            "only chancount channels"
        );
        assert_eq!(rc.rssi, None, "255 = unknown");
        let out = v.servos[&0].value.outputs;
        assert_eq!((out[0], out[3], out[15]), (1100, 1150, 0));
    }

    #[test]
    fn home_from_the_autopilot() {
        use mavlink_codec::messages::HomePosition;
        let mut f = Fleet::new();
        let now = Instant::now();
        let h = Message::HomePosition(HomePosition {
            latitude: -353_632_610,
            longitude: 1_491_652_300,
            altitude: 584_090,
            ..Default::default()
        });
        f.apply(1, 100, &h, now);
        assert_eq!(f.get(1).unwrap().home, None, "not from another component");
        f.apply(1, 1, &h, now);
        let home = f.get(1).unwrap().home.as_ref().unwrap().value;
        assert!((home.lat_deg + 35.363_261).abs() < 1e-9);
        assert!((home.lon_deg - 149.165_23).abs() < 1e-9);
        assert_eq!(home.alt_msl_m, 584.09);
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
        f.apply(1, 1, &cur(0, 0, 1, 0), now);
        let m = f.get(1).unwrap().mission.as_ref().unwrap().value;
        assert_eq!(m.total, Some(0), "ArduPilot: total 0 with NO_MISSION");
        f.apply(1, 1, &cur(0, u16::MAX, 1, 2), now);
        let m = f.get(1).unwrap().mission.as_ref().unwrap().value;
        assert_eq!(
            (m.total, m.state, m.in_mission_mode),
            (Some(0), MissionState::NoMission, Some(false))
        );
    }
}
