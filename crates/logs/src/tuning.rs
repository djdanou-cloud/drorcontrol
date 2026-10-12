//! How closely the vehicle followed its attitude targets while flying: the gap between the
//! desired and the achieved roll, pitch and yaw. Large or growing errors point at tuning (or
//! at a mechanical problem); the numbers are for comparing flights and axes, not a pass/fail.
//!
//! `ATT` = TimeUS, DesRoll, Roll, DesPitch, Pitch, DesYaw, Yaw, AEKF, degrees
//! (`AP_AHRS/LogStructure.h`, 4.7.1; older versions store centidegrees, which the format
//! scale converts).

use crate::dataflash::Log;
use crate::flight::{flying_periods, in_periods};

/// Tracking error on one axis while flying.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisError {
    /// Root-mean-square error, degrees.
    pub rms_deg: f64,
    /// Largest error (absolute), degrees.
    pub max_deg: f64,
    /// When the largest error happened, µs since boot.
    pub max_at_us: u64,
}

/// Attitude tracking while flying.
#[derive(Debug, Clone, PartialEq)]
pub struct AttitudeTracking {
    /// Roll.
    pub roll: AxisError,
    /// Pitch.
    pub pitch: AxisError,
    /// Yaw (wrapped to ±180°).
    pub yaw: AxisError,
    /// `ATT` samples used.
    pub samples: usize,
    /// Whether flying was told by the land detector (else: whenever armed).
    pub land_detector: bool,
}

fn axis(time_us: &[u64], errors: impl Iterator<Item = f64>) -> AxisError {
    let mut sum_sq = 0.0;
    let mut n = 0usize;
    let (mut max_deg, mut max_at_us) = (0.0f64, 0);
    for (e, t) in errors.zip(time_us) {
        sum_sq += e * e;
        n += 1;
        if e.abs() > max_deg {
            (max_deg, max_at_us) = (e.abs(), *t);
        }
    }
    AxisError {
        rms_deg: if n > 0 {
            (sum_sq / n as f64).sqrt()
        } else {
            0.0
        },
        max_deg,
        max_at_us,
    }
}

/// Desired-minus-actual attitude while flying; `None` without `ATT` samples in flight.
pub fn attitude_tracking(log: &Log) -> Option<AttitudeTracking> {
    let (flying, land_detector) = flying_periods(log);
    let col = |f: &str| log.column("ATT", f, None);
    let (dr, r, dp, p, dy, y) = (
        col("DesRoll")?,
        col("Roll")?,
        col("DesPitch")?,
        col("Pitch")?,
        col("DesYaw")?,
        col("Yaw")?,
    );
    let keep: Vec<usize> = (0..r.values.len())
        .filter(|&i| r.time_us.get(i).is_some_and(|t| in_periods(&flying, *t)))
        .collect();
    if keep.is_empty() {
        return None;
    }
    let time: Vec<u64> = keep.iter().map(|&i| r.time_us[i]).collect();
    let diff = |d: &[f64], a: &[f64]| -> Vec<f64> { keep.iter().map(|&i| d[i] - a[i]).collect() };
    let wrap = |e: f64| (e + 180.0).rem_euclid(360.0) - 180.0;
    Some(AttitudeTracking {
        roll: axis(&time, diff(&dr.values, &r.values).into_iter()),
        pitch: axis(&time, diff(&dp.values, &p.values).into_iter()),
        yaw: axis(&time, diff(&dy.values, &y.values).into_iter().map(wrap)),
        samples: keep.len(),
        land_detector,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn errors_while_flying_with_yaw_wrapped() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format(
                "ATT",
                "QffffffB",
                "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,AEKF",
            );
        let s = |x: f64| x * 1e6;
        // Before arming: a huge error that must be ignored.
        b.rec(
            "ATT",
            &[
                N(s(0.5)),
                N(30.0),
                N(0.0),
                N(0.0),
                N(0.0),
                N(0.0),
                N(0.0),
                N(3.0),
            ],
        );
        b.rec("ARM", &[N(s(1.0)), N(1.0), N(0.0), N(0.0), N(0.0)]);
        // Roll errors 3 and -4 (rms 3.54); pitch 0; yaw 359 -> 1 is a 2° error, not 358°.
        b.rec(
            "ATT",
            &[
                N(s(2.0)),
                N(5.0),
                N(2.0),
                N(1.0),
                N(1.0),
                N(359.0),
                N(1.0),
                N(3.0),
            ],
        );
        b.rec(
            "ATT",
            &[
                N(s(3.0)),
                N(-1.0),
                N(3.0),
                N(1.0),
                N(1.0),
                N(10.0),
                N(10.0),
                N(3.0),
            ],
        );
        let a = attitude_tracking(&b.open()).unwrap();
        assert_eq!(a.samples, 2);
        assert!(!a.land_detector);
        assert!((a.roll.rms_deg - 12.5f64.sqrt()).abs() < 1e-6, "{a:?}");
        assert_eq!((a.roll.max_deg, a.roll.max_at_us), (4.0, 3_000_000));
        assert_eq!(a.pitch.max_deg, 0.0);
        assert!((a.yaw.max_deg - 2.0).abs() < 1e-6, "{a:?}");
    }
}
