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
//! * `GPS` = TimeUS, I, Status, …, HDop (`AP_GPS/LogStructure.h`); Status 3 is
//!   `GPS_OK_FIX_3D` (`AP_GPS.h`). Copter refuses to arm when HDOP × 100 is above
//!   `GPS_HDOP_GOOD` (default 140; `AP_Arming_Copter.cpp`, `ArduCopter/config.h`).
//! * `POWR` = TimeUS, Vcc, … (`AP_Logger/LogStructure.h`). Arming fails when the board
//!   voltage is outside `BRD_VBUS_MIN` (default 4.3 V, `AP_BoardConfig.cpp`) to 5.8 V
//!   (`AP_ARMING_BOARD_VOLTAGE_MAX`, `AP_Arming.cpp`).
//! * `XKF4.SS` is the EKF3 solution status, `nav_filter_status` (`AP_NavEKF/AP_Nav_Common.h`,
//!   4.7.1 and 4.6.3): bit 0 attitude, 1 horizontal velocity, 2 vertical velocity, 4 absolute
//!   horizontal position, 5 vertical position, 7 constant-position mode, 14 GPS glitching,
//!   18 dead reckoning. Judged on the primary core (`PI`) while flying.
//! * `MAG` = TimeUS, I, MagX/Y/Z (corrected field, body frame), OfsX/Y/Z (offsets), … in
//!   milligauss, I in priority order (`AP_Logger/LogFile.cpp`, 4.7.1 and 4.6.3). Arming fails
//!   with offsets above `COMPASS_OFFS_MAX` (default 1800) or compasses that disagree by more
//!   than 90° in 3-D, 60° in the horizontal plane or 200 mG in horizontal length
//!   (`Compass::consistent`, `AP_COMPASS_MAX_*`); compasses with `COMPASS_USE*` 0 are not
//!   compared. Plan 05 §2.3 adds cautions at 600 mG offsets and 30° horizontal disagreement.

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

/// GPS after its first 3D fix: (worst HDOP, fix losses) against `GPS_HDOP_GOOD` (× 100).
pub fn gps_level(max_hdop: f64, fix_losses: usize, hdop_good: f64) -> Level {
    if fix_losses > 0 {
        Level::Warning
    } else if max_hdop * 100.0 > hdop_good {
        Level::Caution
    } else {
        Level::Ok
    }
}

/// Board voltage range against the arming limits.
pub fn board_voltage_level(min: f64, max: f64, vbus_min: f64) -> Level {
    if min < vbus_min || max > BOARD_VOLTAGE_MAX {
        Level::Caution
    } else {
        Level::Ok
    }
}

/// `AP_ARMING_BOARD_VOLTAGE_MAX`.
pub const BOARD_VOLTAGE_MAX: f64 = 5.8;

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

/// Problems in a run of primary-core solution statuses `(time, SS)` taken while flying:
/// (level, what, first time).
pub fn status_problems(samples: &[(u64, u32)]) -> Vec<(Level, &'static str, u64)> {
    const CHECKS: [(u32, bool, Level, &str); 6] = [
        (0, false, Level::Warning, "attitude estimate invalid"),
        (1, false, Level::Warning, "horizontal velocity invalid"),
        (2, false, Level::Warning, "vertical velocity invalid"),
        (5, false, Level::Warning, "vertical position invalid"),
        (
            18,
            true,
            Level::Warning,
            "dead reckoning (no position or velocity aiding)",
        ),
        (
            14,
            true,
            Level::Caution,
            "GPS glitching affected navigation",
        ),
    ];
    let bit = |ss: u32, b: u32| ss & (1 << b) != 0;
    let mut out = Vec::new();
    for (b, set_is_bad, level, what) in CHECKS {
        if let Some((t, _)) = samples.iter().find(|(_, ss)| bit(*ss, b) == set_is_bad) {
            out.push((level, what, *t));
        }
    }
    // Absolute position matters only once the flight had it.
    if let Some(had) = samples.iter().position(|(_, ss)| bit(*ss, 4))
        && let Some((t, _)) = samples[had..].iter().find(|(_, ss)| !bit(*ss, 4))
    {
        out.push((Level::Warning, "absolute position lost in flight", *t));
    }
    // Constant-position mode is normal for a vehicle without a position source (indoors, no
    // GPS); it is a loss only after the flight had a relative or absolute position.
    if let Some(had) = samples
        .iter()
        .position(|(_, ss)| bit(*ss, 3) || bit(*ss, 4))
        && let Some((t, _)) = samples[had..].iter().find(|(_, ss)| bit(*ss, 7))
    {
        out.push((
            Level::Warning,
            "constant-position mode in flight (no horizontal aiding)",
            *t,
        ));
    }
    out.sort_by_key(|p| p.2);
    out
}

fn ekf_status(log: &Log, out: &mut Vec<Finding>) {
    let (flying, _) = crate::flight::flying_periods(log);
    let mut samples = Vec::new();
    for core in instances(log, "XKF4", "C") {
        let inst = Some(("C", core));
        let (Some(ss), Some(pi)) = (
            log.column("XKF4", "SS", inst),
            log.column("XKF4", "PI", inst),
        ) else {
            continue;
        };
        samples.extend(
            ss.time_us
                .iter()
                .zip(ss.values.iter().zip(&pi.values))
                .filter(|(t, (_, pi))| **pi == core && crate::flight::in_periods(&flying, **t))
                .map(|(t, (ss, _))| (*t, *ss as u32)),
        );
    }
    if samples.is_empty() {
        return;
    }
    samples.sort_unstable();
    let problems = status_problems(&samples);
    if problems.is_empty() {
        out.push(Finding {
            check: "ekf status",
            level: Level::Ok,
            text: "EKF3 solution status (primary core) healthy throughout the flight".into(),
            at_us: None,
        });
    }
    for (level, what, t) in problems {
        out.push(Finding {
            check: "ekf status",
            level,
            text: format!("EKF3 solution status: {what}"),
            at_us: Some(t),
        });
    }
}

/// Compass offsets: Caution above 600 mG, Warning above `offs_max` (arming refused).
pub fn offset_level(magnitude: f64, offs_max: f64) -> Level {
    if offs_max > 0.0 && magnitude > offs_max {
        Level::Warning
    } else if magnitude > 600.0 {
        Level::Caution
    } else {
        Level::Ok
    }
}

fn compass_offsets(log: &Log, params: &BTreeMap<String, f64>, out: &mut Vec<Finding>) {
    let offs_max = params.get("COMPASS_OFFS_MAX").copied().unwrap_or(1800.0);
    for inst in instances(log, "MAG", "I") {
        let i = Some(("I", inst));
        let last = |f: &str| {
            log.column("MAG", f, i)
                .and_then(|c| c.values.last().copied())
        };
        let (Some(x), Some(y), Some(z)) = (last("OfsX"), last("OfsY"), last("OfsZ")) else {
            continue;
        };
        let m = (x * x + y * y + z * z).sqrt();
        out.push(Finding {
            check: "compass",
            level: offset_level(m, offs_max),
            text: format!(
                "compass {inst}: offsets {m:.0} mG ({x:.0}, {y:.0}, {z:.0}); caution above 600, COMPASS_OFFS_MAX {offs_max:.0}"
            ),
            at_us: None,
        });
    }
}

/// How far compass `b` disagrees with the primary `a` (milligauss, body frame): 3-D angle
/// and horizontal angle in degrees, horizontal length difference in mG.
pub fn disagreement(a: [f64; 3], b: [f64; 3]) -> (f64, f64, f64) {
    let angle = |u: &[f64], v: &[f64]| {
        let dot: f64 = u.iter().zip(v).map(|(p, q)| p * q).sum();
        let n = |w: &[f64]| w.iter().map(|x| x * x).sum::<f64>().sqrt();
        let c = dot / (n(u) * n(v));
        if c.is_finite() {
            c.clamp(-1.0, 1.0).acos().to_degrees()
        } else {
            180.0
        }
    };
    let len = ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    (angle(&a, &b), angle(&a[..2], &b[..2]), len)
}

fn compass_consistency(log: &Log, params: &BTreeMap<String, f64>, out: &mut Vec<Finding>) {
    let armed = crate::flight::armed_periods(log);
    let field = |inst: f64| {
        let i = Some(("I", inst));
        let (x, y, z) = (
            log.column("MAG", "MagX", i)?,
            log.column("MAG", "MagY", i)?,
            log.column("MAG", "MagZ", i)?,
        );
        Some((x.time_us, x.values, y.values, z.values))
    };
    let used = |inst: f64| {
        let name = match inst as u8 {
            0 => "COMPASS_USE".to_string(),
            n => format!("COMPASS_USE{}", n + 1),
        };
        params.get(&name).is_none_or(|v| *v != 0.0)
    };
    let Some(primary) = field(0.0) else { return };
    for inst in instances(log, "MAG", "I")
        .into_iter()
        .filter(|i| *i > 0.0 && used(*i))
    {
        let Some(other) = field(inst) else { continue };
        // Instances are written together each cycle: pair them in order.
        let mut worst = (0.0, 0.0, 0.0, 0u64);
        let mut worst_level = Level::Ok;
        let mut n = 0;
        for k in 0..primary.0.len().min(other.0.len()) {
            let t = primary.0[k];
            if !crate::flight::in_periods(&armed, t) {
                continue;
            }
            n += 1;
            let a = [primary.1[k], primary.2[k], primary.3[k]];
            let b = [other.1[k], other.2[k], other.3[k]];
            let (xyz, xy, len) = disagreement(a, b);
            let level = if xyz > 90.0 || xy > 60.0 || len > 200.0 {
                Level::Warning
            } else if xy > 30.0 {
                Level::Caution
            } else {
                Level::Ok
            };
            if level > worst_level || (level == worst_level && xy > worst.1) {
                worst_level = level;
                worst = (xyz, xy, len, t);
            }
        }
        if n == 0 {
            continue;
        }
        out.push(Finding {
            check: "compass",
            level: worst_level,
            text: format!(
                "compass {inst} vs compass 0 while armed: worst {:.0}° 3-D, {:.0}° horizontal, {:.0} mG horizontal length (arming limits 90°, 60°, 200 mG)",
                worst.0, worst.1, worst.2
            ),
            at_us: Some(worst.3),
        });
    }
}

fn gps(log: &Log, params: &BTreeMap<String, f64>, out: &mut Vec<Finding>) {
    let (good, source) = match params.get("GPS_HDOP_GOOD") {
        Some(v) => (*v, "GPS_HDOP_GOOD"),
        None => (140.0, "Copter's default GPS_HDOP_GOOD"),
    };
    for inst in instances(log, "GPS", "I") {
        let i = Some(("I", inst));
        let (Some(status), Some(hdop)) =
            (log.column("GPS", "Status", i), log.column("GPS", "HDop", i))
        else {
            continue;
        };
        let Some(first_fix) = status.values.iter().position(|s| *s >= 3.0) else {
            out.push(Finding {
                check: "gps",
                level: Level::Ok,
                text: format!("GPS {inst}: no 3D fix in this log; nothing to check"),
                at_us: None,
            });
            continue;
        };
        let after = first_fix..status.values.len();
        // A loss is a step from a 3D fix to below it.
        let losses: Vec<usize> = after
            .clone()
            .skip(1)
            .filter(|&k| status.values[k] < 3.0 && status.values[k - 1] >= 3.0)
            .collect();
        let (max_hdop, t) = after
            .map(|k| (hdop.values[k], status.time_us.get(k).copied().unwrap_or(0)))
            .max_by(|a, b| a.0.total_cmp(&b.0))
            .unwrap_or((0.0, 0));
        let level = gps_level(max_hdop, losses.len(), good);
        let text = if losses.is_empty() {
            format!(
                "GPS {inst}: highest HDOP {max_hdop:.2} after the first 3D fix ({source} {:.2})",
                good / 100.0
            )
        } else {
            format!(
                "GPS {inst}: lost the 3D fix {} times after first having it; highest HDOP {max_hdop:.2}",
                losses.len()
            )
        };
        out.push(Finding {
            check: "gps",
            level,
            text,
            at_us: Some(losses.first().map_or(t, |&k| status.time_us[k])),
        });
    }
}

fn board_voltage(log: &Log, params: &BTreeMap<String, f64>, out: &mut Vec<Finding>) {
    let Some(c) = log.column("POWR", "Vcc", None) else {
        return;
    };
    // Boards without a board-voltage sensor log NaN (seen on a MatekH743, Copter 4.6.2).
    let measured = || c.values.iter().copied().filter(|v| v.is_finite());
    let (Some(min), Some(max)) = (
        measured().min_by(f64::total_cmp),
        measured().max_by(f64::total_cmp),
    ) else {
        out.push(Finding {
            check: "board",
            level: Level::Ok,
            text: "board voltage not measured by this board (logged as NaN)".into(),
            at_us: None,
        });
        return;
    };
    let vbus_min = params.get("BRD_VBUS_MIN").copied().unwrap_or(4.3);
    out.push(Finding {
        check: "board",
        level: board_voltage_level(min, max, vbus_min),
        text: format!(
            "board voltage {min:.2}–{max:.2} V (arming range BRD_VBUS_MIN {vbus_min:.2} to \
             {BOARD_VOLTAGE_MAX:.1} V)"
        ),
        at_us: None,
    });
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
    ekf_status(log, &mut out);
    compass_offsets(log, &params, &mut out);
    compass_consistency(log, &params, &mut out);
    gps(log, &params, &mut out);
    board_voltage(log, &params, &mut out);
    errors(log, &mut out);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compass_limits() {
        assert_eq!(offset_level(500.0, 1800.0), Level::Ok);
        assert_eq!(offset_level(700.0, 1800.0), Level::Caution);
        assert_eq!(offset_level(1900.0, 1800.0), Level::Warning);
        let (xyz, xy, len) = disagreement([200.0, 0.0, 400.0], [200.0, 0.0, 400.0]);
        assert!(xyz < 1e-6 && xy < 1e-6 && len < 1e-9);
        let (_, xy, len) = disagreement([200.0, 0.0, 400.0], [0.0, 200.0, 400.0]);
        assert!((xy - 90.0).abs() < 1e-9 && (len - 282.842_712).abs() < 1e-3);
    }

    #[test]
    fn solution_status_problems() {
        let good = 0b1_0010_0000_0011_0111; // att, vels, abs pos, vert pos, using GPS
        assert!(status_problems(&[(1, good), (2, good)]).is_empty());
        let p = status_problems(&[(1, good), (2, good & !(1 << 4)), (3, good | 1 << 14)]);
        assert_eq!(
            p,
            [
                (Level::Warning, "absolute position lost in flight", 2),
                (Level::Caution, "GPS glitching affected navigation", 3)
            ]
        );
        let no_gps = good & !(1 << 4); // never had absolute position (optical flow)
        assert!(status_problems(&[(1, no_gps), (2, no_gps)]).is_empty());
        assert_eq!(
            status_problems(&[(5, good & !1)])[0].1,
            "attitude estimate invalid"
        );
        let indoor = 0b10_0111 | 1 << 7; // no position source: constant-position mode
        assert!(status_problems(&[(1, indoor)]).is_empty());
        assert_eq!(
            status_problems(&[(1, good), (2, indoor)])
                .iter()
                .map(|p| p.1)
                .collect::<Vec<_>>(),
            [
                "absolute position lost in flight",
                "constant-position mode in flight (no horizontal aiding)"
            ]
        );
    }

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
        assert_eq!(gps_level(0.9, 0, 140.0), Level::Ok);
        assert_eq!(gps_level(1.5, 0, 140.0), Level::Caution);
        assert_eq!(gps_level(0.9, 1, 140.0), Level::Warning);
        assert_eq!(board_voltage_level(4.9, 5.1, 4.3), Level::Ok);
        assert_eq!(board_voltage_level(4.2, 5.1, 4.3), Level::Caution);
        assert_eq!(board_voltage_level(4.9, 5.9, 4.3), Level::Caution);
    }
}
