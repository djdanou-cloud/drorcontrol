//! ArduPilot firmware identity and flight-mode names, from the MAVLink metadata
//! (`ardupilotmega.xml`: `COPTER_MODE`, `PLANE_MODE`, `ROVER_MODE`, `SUB_MODE`,
//! `TRACKER_MODE`) — never hand-typed (UI skill §4).
//!
//! Mode numbers mean different things per firmware (5 is LOITER on Copter, FBWA on Plane), so
//! names are only as good as the firmware identity. `HEARTBEAT.type` is a hint, not proof: a
//! QuadPlane reports whatever `Q_MAV_TYPE` says, which may be a multirotor type. The firmware
//! is **confirmed** only by its version banner (`STATUSTEXT` "ArduPlane V4.7.1 …", sent on
//! `MAV_CMD_DO_SEND_BANNER`). Mode lists offered to the operator must use a confirmed firmware.

use mavlink_codec::enums::{CopterMode, PlaneMode, RoverMode, SubMode, TrackerMode};

/// `MAV_AUTOPILOT_ARDUPILOTMEGA`.
pub const MAV_AUTOPILOT_ARDUPILOTMEGA: u8 = 3;

/// ArduPilot firmware family.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Firmware {
    /// ArduCopter (multirotors, helicopters).
    Copter,
    /// ArduPlane (fixed wing, QuadPlane / VTOL).
    Plane,
    /// ArduRover (ground vehicles and boats).
    Rover,
    /// ArduSub.
    Sub,
    /// AntennaTracker.
    Tracker,
    /// Blimp (no mode metadata in the dialect).
    Blimp,
}

/// How the firmware was identified.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FirmwareSource {
    /// Guessed from `HEARTBEAT.type`: display only, never for choosing a mode number.
    MavType,
    /// Confirmed by the firmware's own version banner.
    Banner,
}

impl Firmware {
    /// Best guess from a heartbeat (ArduPilot autopilots only).
    pub fn from_mav_type(mav_type: u8, autopilot: u8) -> Option<Firmware> {
        if autopilot != MAV_AUTOPILOT_ARDUPILOTMEGA {
            return None;
        }
        Some(match mav_type {
            // QUADROTOR, COAXIAL, HELICOPTER, HEXAROTOR, OCTOROTOR, TRICOPTER, DODECAROTOR,
            // DECAROTOR (ArduCopter's AP_Motors frame types).
            2 | 3 | 4 | 13 | 14 | 15 | 29 | 35 => Firmware::Copter,
            // FIXED_WING and the VTOL types a QuadPlane may report (Q_MAV_TYPE).
            1 | 19..=25 | 47 => Firmware::Plane,
            10 | 11 => Firmware::Rover, // GROUND_ROVER, SURFACE_BOAT
            12 => Firmware::Sub,
            5 => Firmware::Tracker,
            7 => Firmware::Blimp, // AIRSHIP
            _ => return None,
        })
    }

    /// From a version banner (`THISFIRMWARE`, e.g. "ArduCopter V4.7.1 (abcdef12)").
    pub fn from_banner(text: &str) -> Option<Firmware> {
        let (name, rest) = text.split_once(' ')?;
        if !rest.starts_with('V') {
            return None;
        }
        Some(match name {
            "ArduCopter" => Firmware::Copter,
            "ArduPlane" => Firmware::Plane,
            "ArduRover" => Firmware::Rover,
            "ArduSub" => Firmware::Sub,
            "AntennaTracker" => Firmware::Tracker,
            "Blimp" => Firmware::Blimp,
            _ => return None,
        })
    }

    /// Metadata name of mode `n` without the enum prefix ("LOITER"), if defined.
    pub fn mode_name(self, n: u32) -> Option<&'static str> {
        let n = u8::try_from(n).ok()?;
        let (full, prefix) = match self {
            Firmware::Copter => (CopterMode(n).name(), "COPTER_MODE_"),
            Firmware::Plane => (PlaneMode(n).name(), "PLANE_MODE_"),
            Firmware::Rover => (RoverMode(n).name(), "ROVER_MODE_"),
            Firmware::Sub => (SubMode(n).name(), "SUB_MODE_"),
            Firmware::Tracker => (TrackerMode(n).name(), "TRACKER_MODE_"),
            Firmware::Blimp => (None, ""),
        };
        full.map(|f| f.strip_prefix(prefix).unwrap_or(f))
    }

    /// Every mode in the metadata, by number.
    pub fn modes(self) -> Vec<(u32, &'static str)> {
        (0..=u8::MAX)
            .filter_map(|n| self.mode_name(n.into()).map(|name| (u32::from(n), name)))
            .collect()
    }

    /// Number of the mode called `name` ("LOITER"), if defined.
    pub fn mode_number(self, name: &str) -> Option<u32> {
        self.modes()
            .into_iter()
            .find(|(_, m)| m.eq_ignore_ascii_case(name))
            .map(|(n, _)| n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_come_from_the_metadata_and_differ_by_firmware() {
        assert_eq!(Firmware::Copter.mode_name(5), Some("LOITER"));
        assert_eq!(Firmware::Plane.mode_name(5), Some("FLY_BY_WIRE_A"));
        assert_eq!(Firmware::Copter.mode_name(6), Some("RTL"));
        assert_eq!(Firmware::Plane.mode_name(20), Some("QLAND"));
        assert_eq!(Firmware::Rover.mode_name(4), Some("HOLD"));
        assert_eq!(Firmware::Copter.mode_name(8), None, "gap in the enum");
        assert_eq!(Firmware::Copter.mode_name(1000), None);
        assert_eq!(Firmware::Blimp.modes(), vec![]);
        assert_eq!(Firmware::Copter.mode_number("loiter"), Some(5));
        assert_eq!(Firmware::Plane.mode_number("LOITER"), Some(12));
        assert!(Firmware::Copter.modes().len() > 20);
    }

    #[test]
    fn banner_confirms_heartbeat_only_hints() {
        assert_eq!(
            Firmware::from_banner("ArduPlane V4.7.1 (3f2a9c1e)"),
            Some(Firmware::Plane)
        );
        assert_eq!(
            Firmware::from_banner("ArduCopter V4.7.1"),
            Some(Firmware::Copter)
        );
        assert_eq!(Firmware::from_banner("ArduCopter is ready"), None);
        assert_eq!(
            Firmware::from_banner("PreArm: Compass not calibrated"),
            None
        );
        // A QuadPlane with Q_MAV_TYPE=2 looks like a quadrotor: only a hint.
        assert_eq!(Firmware::from_mav_type(2, 3), Some(Firmware::Copter));
        assert_eq!(Firmware::from_mav_type(20, 3), Some(Firmware::Plane));
        assert_eq!(Firmware::from_mav_type(11, 3), Some(Firmware::Rover));
        assert_eq!(Firmware::from_mav_type(2, 12), None, "PX4 is not ArduPilot");
    }
}
