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

/// Rate-controller error on one axis in the frequency domain: a single narrow peak holding a
/// large share of the error is the signature of an oscillation (usually too much P or D).
///
/// `RATE` = TimeUS, RDes, R, ROut, PDes, P, POut, YDes, Y, YOut, … in deg/s
/// (`AC_AttitudeControl/LogStructure.h`, 4.7.1 and 4.6.3).
#[derive(Debug, Clone, PartialEq)]
pub struct RateOscillation {
    /// `"roll"`, `"pitch"` or `"yaw"`.
    pub axis: &'static str,
    /// Samples analysed (the longest flying period).
    pub samples: usize,
    /// Logging rate of `RATE`, Hz.
    pub sample_rate_hz: f64,
    /// Root-mean-square rate error, deg/s.
    pub rms_degs: f64,
    /// Strongest bin at or above 1 Hz, Hz.
    pub peak_hz: f64,
    /// Share of the error power above 1 Hz within one bin of that peak, 0..1.
    pub peak_share: f64,
}

/// Rate-error spectra per axis over the longest flying period (segments of up to 256
/// samples); empty without `RATE` or with fewer than 64 samples in flight.
pub fn rate_oscillation(log: &Log) -> Vec<RateOscillation> {
    let (flying, _) = flying_periods(log);
    let end = crate::flight::log_end_us(log);
    let Some(longest) = flying
        .iter()
        .max_by_key(|p| p.end_us.unwrap_or(end) - p.start_us)
    else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for (name, des, act) in [
        ("roll", "RDes", "R"),
        ("pitch", "PDes", "P"),
        ("yaw", "YDes", "Y"),
    ] {
        let (Some(d), Some(a)) = (log.column("RATE", des, None), log.column("RATE", act, None))
        else {
            continue;
        };
        let (t, e): (Vec<u64>, Vec<f64>) = d
            .time_us
            .iter()
            .zip(d.values.iter().zip(&a.values))
            .filter(|(t, (d, a))| {
                in_periods(std::slice::from_ref(longest), **t) && d.is_finite() && a.is_finite()
            })
            .map(|(t, (d, a))| (*t, d - a))
            .unzip();
        if e.len() < 64 {
            continue;
        }
        let segment = 256.min(e.len().next_power_of_two() / 2).max(64);
        let Some(s) = crate::spectrum::spectrum(&t, &e, segment) else {
            continue;
        };
        let above: Vec<usize> = (0..s.freqs.len()).filter(|&k| s.freqs[k] >= 1.0).collect();
        let total: f64 = above.iter().map(|&k| s.power[k]).sum();
        // The strongest bin at or above 1 Hz (an axis without a distinct peak still gets
        // one, with a small share).
        let Some(k) = above
            .iter()
            .copied()
            .max_by(|&a, &b| s.power[a].total_cmp(&s.power[b]))
        else {
            continue;
        };
        let peak_hz = s.freqs[k];
        let near: f64 = (k.saturating_sub(1)..=(k + 1).min(s.power.len() - 1))
            .filter(|j| s.freqs[*j] >= 1.0)
            .map(|j| s.power[j])
            .sum();
        out.push(RateOscillation {
            axis: name,
            samples: e.len(),
            sample_rate_hz: s.sample_rate_hz,
            rms_degs: (e.iter().map(|x| x * x).sum::<f64>() / e.len() as f64).sqrt(),
            peak_hz,
            peak_share: if total > 0.0 { near / total } else { 0.0 },
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_oscillating_axis_shows_one_dominant_peak() {
        use crate::testlog::{LogBuilder, V::N};
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("RATE", "Qffffff", "TimeUS,RDes,R,PDes,P,YDes,Y");
        b.rec("ARM", &[N(0.5e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        // 4 s at 400 Hz: roll oscillates at 12 Hz (±20 deg/s); pitch has broadband noise.
        let mut seed = 1u32;
        for i in 0..1600 {
            let t = 1.0 + f64::from(i) / 400.0;
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let noise = f64::from(seed >> 8) / f64::from(1u32 << 24) - 0.5;
            let osc = 20.0 * (2.0 * std::f64::consts::PI * 12.0 * t).sin();
            b.rec(
                "RATE",
                &[
                    N(t * 1e6),
                    N(0.0),
                    N(osc),
                    N(0.0),
                    N(10.0 * noise),
                    N(0.0),
                    N(0.0),
                ],
            );
        }
        b.rec("ARM", &[N(5.5e6), N(0.0), N(0.0), N(0.0), N(0.0)]);
        let r = rate_oscillation(&b.open());
        let roll = r.iter().find(|x| x.axis == "roll").unwrap();
        let pitch = r.iter().find(|x| x.axis == "pitch").unwrap();
        assert!((roll.peak_hz - 12.0).abs() < 2.0, "{roll:?}");
        assert!(roll.peak_share > 0.8, "{roll:?}");
        assert!(pitch.peak_share < 0.2, "{pitch:?}");
        assert!((roll.rms_degs - 20.0 / 2f64.sqrt()).abs() < 1.0, "{roll:?}");
    }
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
