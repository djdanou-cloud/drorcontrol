//! Throttle-based harmonic notch settings suggested from a flight: the vibration peak in
//! steady, level flight and the throttle it was measured at. Advice to review, never written
//! to a vehicle.
//!
//! ArduCopter 4.7.1 (`AP_Vehicle::update_throttle_notch`, `HarmonicNotchFilter.cpp`): with
//! `INS_HNTCH_MODE` 1 the notch centre follows `INS_HNTCH_FREQ * sqrt(throttle /
//! INS_HNTCH_REF)`, so FREQ is the frequency at the throttle REF; `INS_HNTCH_BW` is
//! "typically set to half the base frequency".

use crate::dataflash::Log;
use crate::flight::{climb_rate, flying_periods, in_periods};
use crate::spectrum::{peaks, spectrum};

/// Suggested throttle-based notch.
#[derive(Debug, Clone, PartialEq)]
pub struct NotchAdvice {
    /// Strongest vibration peak at or above `min_hz`, Hz (`INS_HNTCH_FREQ`).
    pub freq_hz: f64,
    /// Half of it (`INS_HNTCH_BW`).
    pub bw_hz: f64,
    /// Mean throttle output while the peak was measured (`INS_HNTCH_REF`).
    pub ref_throttle: f64,
    /// Length of the steady flight analysed, s, and its samples.
    pub span_s: f64,
    /// See `span_s`.
    pub samples: usize,
    /// The vehicle's current settings from the log (`INS_HNTCH_ENABLE`, `_MODE`, `_FREQ`,
    /// `_BW`, `_REF`), where logged.
    pub current: Vec<(String, f64)>,
}

/// Longest run of `CTUN` samples while flying with |climb rate| < 0.6 m/s (Copter's own
/// "level hover" test for learning the hover throttle), as (start, end, mean ThO).
fn longest_steady_run(log: &Log) -> Option<(u64, u64, f64)> {
    let (flying, _) = flying_periods(log);
    let crt = climb_rate(log)?;
    let tho = log.column("CTUN", "ThO", None)?;
    let mut best: Option<(u64, u64, f64)> = None;
    let mut run: Option<(u64, u64, f64, usize)> = None;
    let close = |run: &mut Option<(u64, u64, f64, usize)>, best: &mut Option<(u64, u64, f64)>| {
        if let Some((a, b, sum, n)) = run.take()
            && best.is_none_or(|x| b - a > x.1 - x.0)
        {
            *best = Some((a, b, sum / n as f64));
        }
    };
    for (i, t) in crt.time_us.iter().enumerate() {
        let steady = in_periods(&flying, *t) && crt.values[i].abs() < 0.6;
        if steady {
            let o = tho.values.get(i).copied().unwrap_or(0.0);
            run = Some(match run {
                Some((a, _, sum, n)) => (a, *t, sum + o, n + 1),
                None => (*t, *t, o, 1),
            });
        } else {
            close(&mut run, &mut best);
        }
    }
    close(&mut run, &mut best);
    best
}

/// Suggest a throttle-based notch from `msg.field` (IMU `imu`) during the longest steady
/// flight; `segment` samples per FFT segment (power of two); peaks below `min_hz` ignored.
pub fn suggest(
    log: &Log,
    msg: &str,
    field: &str,
    imu: u8,
    segment: usize,
    min_hz: f64,
) -> Option<NotchAdvice> {
    let (a, b, ref_throttle) = longest_steady_run(log)?;
    let col = log.column(msg, field, Some(("I", f64::from(imu))))?;
    let (t, v): (Vec<u64>, Vec<f64>) = col
        .time_us
        .iter()
        .zip(&col.values)
        .filter(|(t, _)| (a..=b).contains(*t))
        .map(|(t, v)| (*t, *v))
        .unzip();
    let s = spectrum(&t, &v, segment)?;
    let (freq_hz, _) = *peaks(&s, min_hz, 1).first()?;
    let params = log.params();
    let current = ["ENABLE", "MODE", "FREQ", "BW", "REF"]
        .iter()
        .filter_map(|k| {
            let name = format!("INS_HNTCH_{k}");
            params.get(&name).map(|v| (name, *v))
        })
        .collect();
    Some(NotchAdvice {
        freq_hz,
        bw_hz: freq_hz / 2.0,
        ref_throttle,
        span_s: (b - a) as f64 / 1e6,
        samples: t.len(),
        current,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{
        LogBuilder,
        V::{N, T},
    };

    #[test]
    fn peak_and_throttle_from_the_longest_steady_stretch() {
        let mut b = LogBuilder::default();
        b.format("PARM", "QNff", "TimeUS,Name,Value,Default")
            .format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("CTUN", "Qff", "TimeUS,ThO,CRt")
            .format("ACC", "QBQfff", "TimeUS,I,SampleUS,AccX,AccY,AccZ");
        b.rec("PARM", &[N(0.0), T("INS_HNTCH_FREQ"), N(80.0), N(80.0)]);
        b.rec("ARM", &[N(0.0), N(1.0), N(0.0), N(0.0), N(0.0)]);
        // 0-2 s climbing (throttle 0.6), 2-10 s steady (0.35), 10-11 s descending.
        for i in 0..=110 {
            let t = f64::from(i) * 0.1;
            let (o, c) = match t {
                t if t < 2.0 => (0.6, 2.0),
                t if t <= 10.0 => (0.35, 0.1),
                _ => (0.3, -1.5),
            };
            b.rec("CTUN", &[N(t * 1e6), N(o), N(c)]);
        }
        // 1 kHz accelerometer: 120 Hz while climbing, 95 Hz when steady.
        for i in 0..11_000 {
            let t = f64::from(i) / 1000.0;
            let f = if t < 2.0 { 120.0 } else { 95.0 };
            let x = (2.0 * std::f64::consts::PI * f * t).sin();
            b.rec(
                "ACC",
                &[N(t * 1e6), N(0.0), N(t * 1e6), N(x), N(0.0), N(-9.8)],
            );
        }
        let n = suggest(&b.open(), "ACC", "AccX", 0, 512, 20.0).unwrap();
        assert!((n.freq_hz - 95.0).abs() <= 2.0, "{n:?}");
        assert!((n.bw_hz - n.freq_hz / 2.0).abs() < 1e-9);
        assert!((n.ref_throttle - 0.35).abs() < 1e-6, "{n:?}");
        assert!((n.span_s - 8.0).abs() < 0.11, "{n:?}");
        assert_eq!(n.current, [("INS_HNTCH_FREQ".to_string(), 80.0)]);
    }
}
