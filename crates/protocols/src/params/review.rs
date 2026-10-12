//! Configuration review: which safety nets a parameter set has switched off. Advisory only:
//! it names what is off so the operator sees it at a glance; it never blocks anything
//! (ADR-0012). Names, values and meanings are from ArduCopter 4.7.1 and 4.6.3:
//! * `ARMING_SKIPCHK` (4.7: bitmask of checks skipped; 0 none, -1 all) or `ARMING_CHECK`
//!   (4.6: bitmask of checks performed; 1 all, 0 none) (`AP_Arming.cpp`);
//! * `FS_THR_ENABLE` (RC failsafe), `FS_GCS_ENABLE` (GCS failsafe): 0 disabled
//!   (`ArduCopter/Parameters.cpp`);
//! * `BATT_MONITOR` (0 none), `BATT_FS_LOW_ACT` (0 no action), `BATT_LOW_VOLT` (0 disabled)
//!   (`AP_BattMonitor_Params.cpp`);
//! * `FENCE_ENABLE` (0 off), `INS_HNTCH_ENABLE` (0 off).

use std::collections::BTreeMap;

/// How much a finding matters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    /// Worth knowing.
    Note,
    /// A safety net is off.
    Off,
}

/// One finding.
#[derive(Debug, Clone, PartialEq)]
pub struct Finding {
    /// How much it matters.
    pub level: Level,
    /// The parameter.
    pub param: &'static str,
    /// What it means.
    pub text: String,
}

/// Pre-arm check names by bit (`ARMING_CHECK` / `ARMING_SKIPCHK` `@Bitmask` for Copter,
/// 4.6.3 and 4.7.1; bit 0 is "All" in `ARMING_CHECK` only, bit 9 is Plane's airspeed).
const CHECKS: [(u32, &str); 18] = [
    (1, "barometer"),
    (2, "compass"),
    (3, "GPS lock"),
    (4, "INS"),
    (5, "parameters"),
    (6, "RC channels"),
    (7, "board voltage"),
    (8, "battery level"),
    (10, "logging"),
    (11, "safety switch"),
    (12, "GPS configuration"),
    (13, "system"),
    (14, "mission"),
    (15, "rangefinder"),
    (16, "camera"),
    (17, "AuxAuth"),
    (18, "visual odometry"),
    (19, "FFT"),
];

/// Names of the checks whose bit is `set` (true) or clear (false) in `mask`.
fn checks(mask: u32, set: bool) -> String {
    CHECKS
        .iter()
        .filter(|(bit, _)| (mask & (1 << bit) != 0) == set)
        .map(|(_, n)| *n)
        .collect::<Vec<_>>()
        .join(", ")
}

/// Review a parameter set (names as the vehicle reports them).
pub fn review(p: &BTreeMap<String, f64>) -> Vec<Finding> {
    let mut out = Vec::new();
    let get = |n: &str| p.get(n).copied();
    let mut add = |level, param, text: &str| {
        out.push(Finding {
            level,
            param,
            text: text.to_string(),
        });
    };
    match (get("ARMING_SKIPCHK"), get("ARMING_CHECK")) {
        (Some(v), _) if v as i64 == -1 => add(
            Level::Off,
            "ARMING_SKIPCHK",
            "all non-mandatory pre-arm checks are skipped",
        ),
        (Some(v), _) if v != 0.0 => add(
            Level::Off,
            "ARMING_SKIPCHK",
            &format!("pre-arm checks skipped: {}", checks(v as u32, true)),
        ),
        (None, Some(0.0)) => add(
            Level::Off,
            "ARMING_CHECK",
            "all pre-arm checks are disabled",
        ),
        (None, Some(v)) if v as u32 & 1 == 0 => add(
            Level::Off,
            "ARMING_CHECK",
            &format!("pre-arm checks not run: {}", checks(v as u32, false)),
        ),
        _ => {}
    }
    if get("FS_THR_ENABLE") == Some(0.0) {
        add(
            Level::Off,
            "FS_THR_ENABLE",
            "RC (throttle) failsafe is disabled",
        );
    }
    if get("FS_GCS_ENABLE") == Some(0.0) {
        add(
            Level::Note,
            "FS_GCS_ENABLE",
            "GCS failsafe is disabled (the default)",
        );
    }
    if get("BATT_MONITOR").is_some_and(|m| m != 0.0) {
        if get("BATT_FS_LOW_ACT") == Some(0.0) {
            add(
                Level::Off,
                "BATT_FS_LOW_ACT",
                "low-battery failsafe takes no action",
            );
        }
        if get("BATT_LOW_VOLT") == Some(0.0) {
            add(
                Level::Off,
                "BATT_LOW_VOLT",
                "low-battery voltage threshold is disabled",
            );
        }
    } else if get("BATT_MONITOR") == Some(0.0) {
        add(
            Level::Off,
            "BATT_MONITOR",
            "no battery monitor: no battery failsafes",
        );
    }
    if get("FENCE_ENABLE") == Some(0.0) {
        add(Level::Note, "FENCE_ENABLE", "geofence is off");
    }
    if get("INS_HNTCH_ENABLE") == Some(0.0) {
        add(Level::Note, "INS_HNTCH_ENABLE", "no harmonic notch filter");
    }
    out.sort_by_key(|f| std::cmp::Reverse(f.level));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn set(v: &[(&str, f64)]) -> BTreeMap<String, f64> {
        v.iter().map(|(k, x)| (k.to_string(), *x)).collect()
    }

    #[test]
    fn defaults_are_quiet_and_switched_off_nets_are_named() {
        // 4.7 defaults: nothing skipped, RC failsafe on, battery failsafe set.
        let ok = set(&[
            ("ARMING_SKIPCHK", 0.0),
            ("FS_THR_ENABLE", 1.0),
            ("BATT_MONITOR", 4.0),
            ("BATT_FS_LOW_ACT", 2.0),
            ("BATT_LOW_VOLT", 10.5),
        ]);
        assert!(review(&ok).is_empty(), "{:?}", review(&ok));
        let off = set(&[
            ("ARMING_CHECK", 0.0), // 4.6 name
            ("FS_THR_ENABLE", 0.0),
            ("BATT_MONITOR", 4.0),
            ("BATT_FS_LOW_ACT", 0.0),
            ("FENCE_ENABLE", 0.0),
        ]);
        let f = review(&off);
        let params: Vec<&str> = f.iter().map(|f| f.param).collect();
        assert_eq!(
            params,
            [
                "ARMING_CHECK",
                "FS_THR_ENABLE",
                "BATT_FS_LOW_ACT",
                "FENCE_ENABLE"
            ]
        );
        assert!(f[..3].iter().all(|f| f.level == Level::Off));
        let skip_all = review(&set(&[("ARMING_SKIPCHK", -1.0)]));
        assert_eq!(
            skip_all[0].text,
            "all non-mandatory pre-arm checks are skipped"
        );
        // 4.6 bitmask 354 = bits 1, 5, 6, 8 run; the others do not.
        let some = review(&set(&[("ARMING_CHECK", 354.0)]));
        assert!(
            some[0].text.starts_with(
                "pre-arm checks not run: compass, GPS lock, INS, board voltage, logging"
            ),
            "{}",
            some[0].text
        );
        let skip = review(&set(&[("ARMING_SKIPCHK", 4.0)]));
        assert_eq!(skip[0].text, "pre-arm checks skipped: compass");
        assert!(review(&set(&[("ARMING_CHECK", 1.0)])).is_empty(), "1 = all");
    }
}
