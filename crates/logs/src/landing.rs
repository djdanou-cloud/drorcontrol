//! Landings: how fast the vehicle came down just before the land detector called it, and how
//! long until it disarmed. A hard touchdown or a long wait to disarm shows up here.
//!
//! Touchdown = `EV` 18 `LAND_COMPLETE` (`Copter::set_land_complete`, 4.7.1). Descent speed
//! from `CTUN.CRt` (climb rate; [`crate::flight::climb_rate`] handles 4.6's cm/s).

use crate::dataflash::Log;
use crate::flight::{armed_periods, climb_rate};

/// One landing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Landing {
    /// When the land detector reported touchdown, µs since boot.
    pub time_us: u64,
    /// Fastest and mean descent over the 3 s before touchdown, m/s (positive down).
    pub max_descent_ms: f64,
    /// See `max_descent_ms`.
    pub mean_descent_ms: f64,
    /// Time from touchdown to disarm, s (`None` if it did not disarm in this log).
    pub to_disarm_s: Option<f64>,
}

/// Every landing in the log.
pub fn landings(log: &Log) -> Vec<Landing> {
    let Some(ev) = log.column("EV", "Id", None) else {
        return Vec::new();
    };
    let crt = climb_rate(log);
    let armed = armed_periods(log);
    ev.time_us
        .iter()
        .zip(&ev.values)
        .filter(|(_, id)| **id == 18.0)
        .map(|(t, _)| {
            let before: Vec<f64> = crt
                .as_ref()
                .map(|c| {
                    c.time_us
                        .iter()
                        .zip(&c.values)
                        .filter(|(ct, v)| **ct <= *t && *t - **ct <= 3_000_000 && v.is_finite())
                        .map(|(_, v)| -*v)
                        .collect()
                })
                .unwrap_or_default();
            let disarm = armed
                .iter()
                .filter_map(|p| p.end_us)
                .filter(|e| *e >= *t)
                .min();
            Landing {
                time_us: *t,
                max_descent_ms: before.iter().copied().fold(0.0, f64::max),
                mean_descent_ms: if before.is_empty() {
                    0.0
                } else {
                    before.iter().sum::<f64>() / before.len() as f64
                },
                to_disarm_s: disarm.map(|d| (d - t) as f64 / 1e6),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn descent_before_touchdown_and_time_to_disarm() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("EV", "QB", "TimeUS,Id")
            .format("CTUN", "Qf", "TimeUS,CRt");
        b.rec("ARM", &[N(1e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        // Descending at 1.5 m/s, slowing to 0.5 m/s for the last 2 s.
        for (t, crt) in [
            (5.0, -1.5),
            (7.5, -1.5),
            (8.5, -0.5),
            (9.5, -0.5),
            (10.0, 0.0),
        ] {
            b.rec("CTUN", &[N(t * 1e6), N(crt)]);
        }
        b.rec("EV", &[N(10e6), N(18.0)]);
        b.rec("ARM", &[N(12.5e6), N(0.0), N(0.0), N(0.0), N(0.0)]);
        let l = landings(&b.open());
        assert_eq!(l.len(), 1);
        assert_eq!(l[0].max_descent_ms, 1.5, "7.5 s is within 3 s of touchdown");
        assert!((l[0].mean_descent_ms - 0.625).abs() < 1e-9, "{l:?}");
        assert_eq!(l[0].to_disarm_s, Some(2.5));
    }
}
