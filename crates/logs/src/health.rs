//! Automatic health checks on a dataflash log (plan 05): vibration and clipping, battery
//! against the vehicle's own failsafe voltages, EKF variances against its own
//! `FS_EKF_THRESH`, and the autopilot's own `ERR` records.
//!
//! Every threshold is the vehicle's configuration (from the log's `PARM` records) or
//! ArduPilot's documented guidance, named in the finding; none is invented here.
//! Sources (ArduPilot 4.7.1 unless noted):
//! * `VIBE` = TimeUS, IMU, VibeX/Y/Z (m/s²), Clip (cumulative count):
//!   `AP_InertialSensor/LogStructure.h`. Guidance: below 30 m/s² normally acceptable, above
//!   30 may have problems, above 60 nearly always has problems; clipping should be 0
//!   (ArduPilot wiki `common-measuring-vibration.rst`, commit f4b21ca).
//! * `BAT` = TimeUS, Inst, Volt, …: `AP_BattMonitor/LogStructure.h`; `BATT_LOW_VOLT`,
//!   `BATT_CRT_VOLT` (0 = disabled).
//! * `XKF4` SV/SP/SH/SM are the EKF3 velocity/position/height/compass variances logged ×100
//!   (`AP_NavEKF3_Logging.cpp`), the values Copter's EKF failsafe compares with
//!   `FS_EKF_THRESH` (default 0.8; 0.5 is the GCS's caution level, as in `vehicle::nav`).
//! * `ERR` = TimeUS, Subsys, ECode: `AP_Logger/LogStructure.h`, subsystem names from
//!   `LogErrorSubsystem` in `AP_Logger.h`; ECode 0 means resolved.

use std::collections::BTreeMap;

use crate::dataflash::Log;

/// How serious a finding is (plan 04 alert levels, without the cockpit's Warning/Caution
/// semantics: this is post-flight review).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    /// Checked and fine.
    Ok,
    /// Worth a look.
    Caution,
    /// Likely a problem.
    Warning,
}

/// One result.
#[derive(Debug, Clone, PartialEq)]
pub struct Finding {
    /// Check name, e.g. `vibration`.
    pub check: &'static str,
    /// Level.
    pub level: Level,
    /// What was found and against what.
    pub text: String,
    /// Log time of the worst sample, µs, if there is one.
    pub at_us: Option<u64>,
}

/// Vibration level per ArduPilot's guidance.
pub fn vibration_level(max_ms2: f64) -> Level {
    if max_ms2 > 60.0 {
        Level::Warning
    } else if max_ms2 > 30.0 {
        Level::Caution
    } else {
        Level::Ok
    }
}

/// Lowest battery voltage against the vehicle's failsafe voltages (`None` = not set).
pub fn battery_level(v: f64, low: Option<f64>, crit: Option<f64>) -> (Level, String) {
    match (crit, low) {
        (Some(c), _) if v < c => (Level::Warning, format!("below BATT_CRT_VOLT {c} V")),
        (_, Some(l)) if v < l => (Level::Caution, format!("below BATT_LOW_VOLT {l} V")),
        (None, None) => (Level::Ok, "no failsafe voltage set to compare with".into()),
        _ => (Level::Ok, "above the failsafe voltages".into()),
    }
}

/// Highest EKF variance against `FS_EKF_THRESH` (0 = failsafe disabled; 0.5 caution).
pub fn ekf_level(v: f64, thresh: f64) -> Level {
    if thresh > 0.0 && v >= thresh {
        Level::Warning
    } else if v >= 0.5 {
        Level::Caution
    } else {
        Level::Ok
    }
}

/// `LogErrorSubsystem` name.
pub fn err_subsystem(s: u8) -> &'static str {
    match s {
        1 => "MAIN",
        2 => "RADIO",
        3 => "COMPASS",
        4 => "OPTFLOW",
        5 => "FAILSAFE_RADIO",
        6 => "FAILSAFE_BATT",
        7 => "FAILSAFE_GPS",
        8 => "FAILSAFE_GCS",
        9 => "FAILSAFE_FENCE",
        10 => "FLIGHT_MODE",
        11 => "GPS",
        12 => "CRASH_CHECK",
        13 => "FLIP",
        14 => "AUTOTUNE",
        15 => "PARACHUTES",
        16 => "EKFCHECK",
        17 => "FAILSAFE_EKFINAV",
        18 => "BARO",
        19 => "CPU",
        20 => "FAILSAFE_ADSB",
        21 => "TERRAIN",
        22 => "NAVIGATION",
        23 => "FAILSAFE_TERRAIN",
        24 => "EKF_PRIMARY",
        25 => "THRUST_LOSS_CHECK",
        26 => "FAILSAFE_SENSORS",
        27 => "FAILSAFE_LEAK",
        28 => "PILOT_INPUT",
        29 => "FAILSAFE_VIBE",
        30 => "INTERNAL_ERROR",
        31 => "FAILSAFE_DEADRECKON",
        _ => "unknown subsystem",
    }
}

/// Distinct values of an instance field (e.g. `VIBE.IMU`), in order.
fn instances(log: &Log, msg: &str, field: &str) -> Vec<f64> {
    let mut v: Vec<f64> = log
        .column(msg, field, None)
        .map(|c| c.values)
        .unwrap_or_default();
    v.sort_by(f64::total_cmp);
    v.dedup();
    v
}

/// (max value, its time) of a column.
fn max_of(log: &Log, msg: &str, field: &str, inst: Option<(&str, f64)>) -> Option<(f64, u64)> {
    let c = log.column(msg, field, inst)?;
    c.values
        .iter()
        .zip(c.time_us.iter().chain(std::iter::repeat(&0)))
        .max_by(|a, b| a.0.total_cmp(b.0))
        .map(|(v, t)| (*v, *t))
}

fn vibration(log: &Log, out: &mut Vec<Finding>) {
    for imu in instances(log, "VIBE", "IMU") {
        let inst = Some(("IMU", imu));
        let worst = ["VibeX", "VibeY", "VibeZ"]
            .iter()
            .filter_map(|axis| max_of(log, "VIBE", axis, inst).map(|(v, t)| (v, t, *axis)))
            .max_by(|a, b| a.0.total_cmp(&b.0));
        if let Some((v, t, axis)) = worst {
            out.push(Finding {
                check: "vibration",
                level: vibration_level(v),
                text: format!(
                    "IMU {imu}: highest {axis} {v:.1} m/s² (guidance: < 30 fine, > 60 nearly \
                     always a problem)"
                ),
                at_us: Some(t),
            });
        }
        if let Some(c) = log.column("VIBE", "Clip", inst)
            && let (Some(first), Some(last)) = (c.values.first(), c.values.last())
        {
            let clips = last - first;
            out.push(Finding {
                check: "clipping",
                level: if clips > 0.0 {
                    Level::Caution
                } else {
                    Level::Ok
                },
                text: format!("IMU {imu}: accelerometer clipped {clips} times in this log"),
                at_us: None,
            });
        }
    }
}

fn battery(log: &Log, params: &BTreeMap<String, f64>, out: &mut Vec<Finding>) {
    let low = params.get("BATT_LOW_VOLT").copied().filter(|v| *v > 0.0);
    let crit = params.get("BATT_CRT_VOLT").copied().filter(|v| *v > 0.0);
    for inst in instances(log, "BAT", "Inst") {
        let Some(c) = log.column("BAT", "Volt", Some(("Inst", inst))) else {
            continue;
        };
        // A monitor that reads 0 V is not measuring; ignore those samples.
        let Some((v, t)) = c
            .values
            .iter()
            .zip(c.time_us.iter())
            .filter(|(v, _)| **v > 0.1)
            .min_by(|a, b| a.0.total_cmp(b.0))
            .map(|(v, t)| (*v, *t))
        else {
            continue;
        };
        // BATT_LOW_VOLT / BATT_CRT_VOLT belong to the first monitor (BATT2_* for the second).
        let (low, crit) = if inst == 0.0 {
            (low, crit)
        } else {
            (None, None)
        };
        let (level, against) = battery_level(v, low, crit);
        out.push(Finding {
            check: "battery",
            level,
            text: format!("battery {inst}: lowest {v:.2} V, {against}"),
            at_us: Some(t),
        });
    }
}

fn ekf(log: &Log, params: &BTreeMap<String, f64>, out: &mut Vec<Finding>) {
    let thresh = params.get("FS_EKF_THRESH").copied().unwrap_or(0.8);
    for core in instances(log, "XKF4", "C") {
        let inst = Some(("C", core));
        let worst = [
            ("SV", "velocity"),
            ("SP", "position"),
            ("SH", "height"),
            ("SM", "compass"),
        ]
        .iter()
        .filter_map(|(f, name)| max_of(log, "XKF4", f, inst).map(|(v, t)| (v, t, *name)))
        .max_by(|a, b| a.0.total_cmp(&b.0));
        let Some((v, t, name)) = worst else { continue };
        let level = ekf_level(v, thresh);
        out.push(Finding {
            check: "ekf",
            level,
            text: format!(
                "EKF3 core {core}: highest {name} variance {v:.2} (FS_EKF_THRESH {thresh:.2})"
            ),
            at_us: Some(t),
        });
    }
}

fn errors(log: &Log, out: &mut Vec<Finding>) {
    let (Some(s), Some(e)) = (
        log.column("ERR", "Subsys", None),
        log.column("ERR", "ECode", None),
    ) else {
        return;
    };
    for ((sub, code), t) in s.values.iter().zip(&e.values).zip(&s.time_us) {
        let resolved = *code == 0.0;
        out.push(Finding {
            check: "error",
            level: if resolved { Level::Ok } else { Level::Warning },
            text: format!(
                "ERR {} code {code}{}",
                err_subsystem(*sub as u8),
                if resolved { " (resolved)" } else { "" }
            ),
            at_us: Some(*t),
        });
    }
}

/// Run every check. Checks whose messages are not in the log produce no finding.
pub fn assess(log: &Log) -> Vec<Finding> {
    let params = log.params();
    let mut out = Vec::new();
    vibration(log, &mut out);
    battery(log, &params, &mut out);
    ekf(log, &params, &mut out);
    errors(log, &mut out);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn levels_follow_the_guidance() {
        assert_eq!(vibration_level(12.0), Level::Ok);
        assert_eq!(vibration_level(30.0), Level::Ok);
        assert_eq!(vibration_level(30.1), Level::Caution);
        assert_eq!(vibration_level(61.0), Level::Warning);
        assert_eq!(err_subsystem(16), "EKFCHECK");
        assert_eq!(battery_level(10.4, Some(10.5), Some(9.9)).0, Level::Caution);
        assert_eq!(battery_level(9.8, Some(10.5), Some(9.9)).0, Level::Warning);
        assert_eq!(battery_level(11.0, Some(10.5), Some(9.9)).0, Level::Ok);
        assert_eq!(
            battery_level(9.0, None, None).0,
            Level::Ok,
            "nothing to compare with"
        );
        assert_eq!(ekf_level(0.3, 0.8), Level::Ok);
        assert_eq!(ekf_level(0.6, 0.8), Level::Caution);
        assert_eq!(ekf_level(0.8, 0.8), Level::Warning);
        assert_eq!(
            ekf_level(0.9, 0.0),
            Level::Caution,
            "failsafe disabled: never Warning"
        );
        assert_eq!(err_subsystem(200), "unknown subsystem");
    }
}
