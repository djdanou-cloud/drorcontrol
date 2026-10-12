//! Wind while flying, as the EKF estimated it.
//!
//! `XKF2` = TimeUS, C (core), AX, AY, AZ, VWN, VWE (wind velocity north and east, m/s: where
//! the air moves to), … (`AP_NavEKF3/LogStructure.h`, 4.7.1). Reported here as the direction
//! the wind comes from, like a weather report and `SIM_WIND_DIR`.
//!
//! A multicopter's EKF estimates wind only with drag fusion configured (`EK3_DRAG_BCOEF_X/Y`;
//! `NavEKF3_core::setWindMagStateLearningMode`); otherwise the wind states stay at zero, which
//! is reported as "not estimated", not as calm.

use crate::dataflash::Log;
use crate::flight::{flying_periods, in_periods};

/// Wind while flying, from EKF core 0.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WindSummary {
    /// Samples used.
    pub samples: usize,
    /// Mean wind (vector average): speed, m/s, and direction it comes from, degrees true.
    pub mean_speed_ms: f64,
    /// See `mean_speed_ms`.
    pub mean_from_deg: f64,
    /// Strongest estimate, m/s, and when, µs since boot.
    pub max_speed_ms: f64,
    /// See `max_speed_ms`.
    pub max_at_us: u64,
}

/// Direction the wind comes from, degrees true, for a wind moving north and east.
pub fn from_deg(north_ms: f64, east_ms: f64) -> f64 {
    (-east_ms).atan2(-north_ms).to_degrees().rem_euclid(360.0)
}

/// Wind while flying; `None` without `XKF2` in flight or when the EKF did not estimate wind
/// (every value exactly zero).
pub fn wind(log: &Log) -> Option<WindSummary> {
    let core0 = Some(("C", 0.0));
    let (n, e) = (
        log.column("XKF2", "VWN", core0)?,
        log.column("XKF2", "VWE", core0)?,
    );
    let (flying, _) = flying_periods(log);
    let (mut sn, mut se, mut count) = (0.0, 0.0, 0usize);
    let (mut max, mut max_at) = (0.0f64, 0u64);
    for ((t, vn), ve) in n.time_us.iter().zip(&n.values).zip(&e.values) {
        if !in_periods(&flying, *t) || !vn.is_finite() || !ve.is_finite() {
            continue;
        }
        sn += vn;
        se += ve;
        count += 1;
        let s = vn.hypot(*ve);
        if s > max {
            (max, max_at) = (s, *t);
        }
    }
    if count == 0 || max == 0.0 {
        return None;
    }
    let (mn, me) = (sn / count as f64, se / count as f64);
    Some(WindSummary {
        samples: count,
        mean_speed_ms: mn.hypot(me),
        mean_from_deg: from_deg(mn, me),
        max_speed_ms: max,
        max_at_us: max_at,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn wind_comes_from_the_opposite_way_it_blows() {
        assert!(
            (from_deg(-5.0, 0.0) - 0.0).abs() < 1e-9,
            "blowing south = from the north"
        );
        assert!(
            (from_deg(0.0, -5.0) - 90.0).abs() < 1e-9,
            "blowing west = from the east"
        );
        assert!((from_deg(5.0, 0.0) - 180.0).abs() < 1e-9);
        assert!((from_deg(0.0, 5.0) - 270.0).abs() < 1e-9);
    }

    #[test]
    fn mean_and_max_while_flying() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("XKF2", "QBcc", "TimeUS,C,VWN,VWE");
        b.rec("XKF2", &[N(0.5e6), N(0.0), N(20.0), N(0.0)]); // before arming: ignored
        b.rec("ARM", &[N(1e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        // From the west (blowing east) at 4 and 6 m/s; core 1 disagrees and is ignored.
        b.rec("XKF2", &[N(2e6), N(0.0), N(0.0), N(4.0)]);
        b.rec("XKF2", &[N(2e6), N(1.0), N(9.0), N(9.0)]);
        b.rec("XKF2", &[N(3e6), N(0.0), N(0.0), N(6.0)]);
        let w = wind(&b.open()).unwrap();
        assert_eq!(w.samples, 2);
        assert!((w.mean_speed_ms - 5.0).abs() < 1e-6 && (w.mean_from_deg - 270.0).abs() < 1e-6);
        assert!((w.max_speed_ms - 6.0).abs() < 1e-6 && w.max_at_us == 3_000_000);
    }

    #[test]
    fn zero_wind_states_mean_not_estimated() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("XKF2", "QBcc", "TimeUS,C,VWN,VWE");
        b.rec("ARM", &[N(1e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        for t in [2e6, 3e6] {
            b.rec("XKF2", &[N(t), N(0.0), N(0.0), N(0.0)]);
        }
        assert_eq!(wind(&b.open()), None);
    }
}
