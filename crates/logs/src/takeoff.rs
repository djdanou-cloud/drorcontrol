//! Takeoffs: how long after arming the vehicle left the ground, how fast it climbed, and how
//! well it held attitude while doing so. A large attitude error just after liftoff is the
//! classic sign of a wrong motor order or direction, a bad centre of gravity or a loose prop.
//!
//! Liftoff = `EV` 28 `NOT_LANDED` (`Copter::set_land_complete`, 4.7.1 and 4.6.3). Attitude
//! error from `ATT` (`DesRoll`/`Roll`, `DesPitch`/`Pitch`, degrees); climb from `CTUN.CRt`
//! ([`crate::flight::climb_rate`] handles 4.6's cm/s).

use crate::dataflash::Log;
use crate::flight::{armed_periods, climb_rate};

/// How long after liftoff the climb and attitude are examined, µs.
pub const WINDOW_US: u64 = 5_000_000;

/// One takeoff.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Takeoff {
    /// When the land detector reported leaving the ground, µs since boot.
    pub time_us: u64,
    /// Time from arming to liftoff, s (`None` if the arming is not in this log).
    pub from_arm_s: Option<f64>,
    /// Fastest climb in the first [`WINDOW_US`], m/s.
    pub max_climb_ms: f64,
    /// Largest roll and pitch error (desired minus actual) in the first [`WINDOW_US`], degrees.
    pub max_roll_err_deg: f64,
    /// See `max_roll_err_deg`.
    pub max_pitch_err_deg: f64,
}

/// Largest |a − b| over samples with `t0 <= t < t1`.
fn max_err(log: &Log, des: &str, act: &str, t0: u64, t1: u64) -> f64 {
    let (Some(d), Some(a)) = (log.column("ATT", des, None), log.column("ATT", act, None)) else {
        return 0.0;
    };
    d.time_us
        .iter()
        .zip(d.values.iter().zip(&a.values))
        .filter(|(t, _)| **t >= t0 && **t < t1)
        .map(|(_, (d, a))| (d - a).abs())
        .fold(0.0, f64::max)
}

/// Every takeoff in the log.
pub fn takeoffs(log: &Log) -> Vec<Takeoff> {
    let Some(ev) = log.column("EV", "Id", None) else {
        return Vec::new();
    };
    let crt = climb_rate(log);
    let armed = armed_periods(log);
    ev.time_us
        .iter()
        .zip(&ev.values)
        .filter(|(_, id)| **id == 28.0)
        .map(|(&t, _)| {
            let end = t + WINDOW_US;
            let arm = armed
                .iter()
                .filter(|p| p.start_us <= t && p.end_us.is_none_or(|e| e >= t))
                .map(|p| p.start_us)
                .max();
            Takeoff {
                time_us: t,
                from_arm_s: arm.map(|a| (t - a) as f64 / 1e6),
                max_climb_ms: crt
                    .as_ref()
                    .map(|c| {
                        c.time_us
                            .iter()
                            .zip(&c.values)
                            .filter(|(ct, v)| **ct >= t && **ct < end && v.is_finite())
                            .map(|(_, v)| *v)
                            .fold(0.0, f64::max)
                    })
                    .unwrap_or(0.0),
                max_roll_err_deg: max_err(log, "DesRoll", "Roll", t, end),
                max_pitch_err_deg: max_err(log, "DesPitch", "Pitch", t, end),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn liftoff_delay_climb_and_attitude_error() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("EV", "QB", "TimeUS,Id")
            .format("CTUN", "Qf", "TimeUS,CRt")
            .format("ATT", "Qffff", "TimeUS,DesRoll,Roll,DesPitch,Pitch");
        b.rec("ARM", &[N(1e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        b.rec("ATT", &[N(2e6), N(0.0), N(20.0), N(0.0), N(0.0)]); // before liftoff: ignored
        b.rec("EV", &[N(3.5e6), N(28.0)]);
        for (t, crt, roll, pitch) in [
            (4.0, 1.0, 3.0, -1.0),
            (5.0, 2.5, -6.0, 2.0),
            (9.0, 4.0, 30.0, 0.0),
        ] {
            b.rec("CTUN", &[N(t * 1e6), N(crt)]);
            b.rec("ATT", &[N(t * 1e6), N(0.0), N(roll), N(1.0), N(pitch)]);
        }
        let k = takeoffs(&b.open());
        assert_eq!(k.len(), 1);
        assert_eq!(k[0].from_arm_s, Some(2.5));
        assert_eq!(k[0].max_climb_ms, 2.5, "9 s is outside the 5 s window");
        assert_eq!((k[0].max_roll_err_deg, k[0].max_pitch_err_deg), (6.0, 2.0));
    }
}
