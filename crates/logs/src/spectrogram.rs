//! Time-frequency views of vibration (plan 05 §5): a spectrogram (one spectrum per time
//! slice) and a throttle-versus-frequency map, where motor-harmonic lines show up as ridges
//! that move with throttle. Each slice is a Welch spectrum ([`crate::spectrum::spectrum`]).

use crate::dataflash::Log;
use crate::flight::{armed_periods, in_periods};
use crate::spectrum::spectrum;

/// Spectra over time.
#[derive(Debug, Clone, PartialEq)]
pub struct Spectrogram {
    /// Bin frequencies, Hz.
    pub freqs: Vec<f64>,
    /// `(slice start µs since boot, power per bin)` in time order.
    pub slices: Vec<(u64, Vec<f64>)>,
}

/// A spectrogram of `values` in slices of `slice_us`, each a Welch spectrum of segments of
/// `segment` samples; slices with fewer samples than one segment are skipped.
pub fn spectrogram(
    time_us: &[u64],
    values: &[f64],
    slice_us: u64,
    segment: usize,
) -> Option<Spectrogram> {
    let n = time_us.len().min(values.len());
    let mut slices = Vec::new();
    let mut freqs = Vec::new();
    let mut start = 0;
    while start < n {
        let t0 = time_us[start] / slice_us * slice_us;
        let end = start + time_us[start..n].partition_point(|t| *t < t0 + slice_us);
        if let Some(s) = spectrum(&time_us[start..end], &values[start..end], segment) {
            if freqs.is_empty() {
                freqs = s.freqs.clone();
            }
            if s.power.len() == freqs.len() {
                slices.push((t0, s.power));
            }
        }
        start = end;
    }
    (!slices.is_empty()).then_some(Spectrogram { freqs, slices })
}

/// Throttle versus frequency: the mean spectrum in each throttle band.
#[derive(Debug, Clone, PartialEq)]
pub struct ThrottleMap {
    /// Bin frequencies, Hz.
    pub freqs: Vec<f64>,
    /// `(band low, band high, slices averaged, mean power per bin)` for bands with data.
    pub bands: Vec<(f64, f64, usize, Vec<f64>)>,
    /// Frequency of the strongest bin at or above `min_hz` in each band (same order).
    pub peak_hz: Vec<f64>,
}

/// Spectrogram slices of `msg.field` (instance `imu`) while armed, grouped by the mean
/// `CTUN.ThO` of each slice into `bands` equal throttle bands from 0 to 1.
pub fn throttle_map(
    log: &Log,
    msg: &str,
    field: &str,
    imu: u8,
    bands: usize,
    min_hz: f64,
) -> Option<ThrottleMap> {
    let col = log.column(msg, field, Some(("I", f64::from(imu))))?;
    let tho = log.column("CTUN", "ThO", None)?;
    let armed = armed_periods(log);
    let (t, v): (Vec<u64>, Vec<f64>) = col
        .time_us
        .iter()
        .zip(&col.values)
        .filter(|(t, _)| in_periods(&armed, **t))
        .map(|(t, v)| (*t, *v))
        .unzip();
    let slice_us = 1_000_000;
    // Segments of a quarter second (at least 32 samples) give several per slice.
    let rate = t.len() as f64 / ((t.last()? - t.first()?) as f64 / 1e6).max(1e-9);
    let segment = ((rate / 4.0) as usize).next_power_of_two().max(32);
    let sg = spectrogram(&t, &v, slice_us, segment)?;
    let bands = bands.max(1);
    let mut acc: Vec<(usize, Vec<f64>)> = vec![(0, vec![0.0; sg.freqs.len()]); bands];
    for (t0, power) in &sg.slices {
        let th: Vec<f64> = tho
            .time_us
            .iter()
            .zip(&tho.values)
            .filter(|(t, _)| **t >= *t0 && **t < t0 + slice_us)
            .map(|(_, v)| *v)
            .collect();
        if th.is_empty() {
            continue;
        }
        let mean = th.iter().sum::<f64>() / th.len() as f64;
        let b = ((mean.clamp(0.0, 1.0) * bands as f64) as usize).min(bands - 1);
        acc[b].0 += 1;
        for (a, p) in acc[b].1.iter_mut().zip(power) {
            *a += p;
        }
    }
    let mut out = ThrottleMap {
        freqs: sg.freqs.clone(),
        bands: Vec::new(),
        peak_hz: Vec::new(),
    };
    for (b, (n, sum)) in acc.into_iter().enumerate() {
        if n == 0 {
            continue;
        }
        let mean: Vec<f64> = sum.iter().map(|p| p / n as f64).collect();
        let peak = (0..mean.len())
            .filter(|&k| sg.freqs[k] >= min_hz)
            .max_by(|&a, &b| mean[a].total_cmp(&mean[b]))
            .map_or(0.0, |k| sg.freqs[k]);
        let w = 1.0 / bands as f64;
        out.bands.push((b as f64 * w, (b + 1) as f64 * w, n, mean));
        out.peak_hz.push(peak);
    }
    (!out.bands.is_empty()).then_some(out)
}

/// The spectrogram as CSV: one row per slice, `time_s` then one column per bin frequency.
pub fn spectrogram_csv(sg: &Spectrogram) -> String {
    let mut s = String::from("time_s");
    for f in &sg.freqs {
        s.push_str(&format!(",{f:.2}"));
    }
    s.push('\n');
    for (t, p) in &sg.slices {
        s.push_str(&format!("{:.3}", *t as f64 / 1e6));
        for v in p {
            s.push_str(&format!(",{v:.6e}"));
        }
        s.push('\n');
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};
    use std::f64::consts::PI;

    #[test]
    fn a_tone_that_follows_throttle_moves_between_bands() {
        // 20 s at 400 Hz. Throttle 0.2 for 10 s then 0.6; the "motor" tone is 60 Hz, then
        // 120 Hz, so the low band peaks near 60 and the high band near 120.
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("CTUN", "Qf", "TimeUS,ThO")
            .format("IMU", "QBf", "TimeUS,I,AccZ");
        b.rec("ARM", &[N(0.5e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        for i in 0..8000 {
            let t = 1.0 + f64::from(i) / 400.0;
            let (th, f) = if t < 11.0 { (0.2, 60.0) } else { (0.6, 120.0) };
            b.rec("IMU", &[N(t * 1e6), N(0.0), N((2.0 * PI * f * t).sin())]);
            if i % 40 == 0 {
                b.rec("CTUN", &[N(t * 1e6), N(th)]);
            }
        }
        b.rec("ARM", &[N(21.5e6), N(0.0), N(0.0), N(0.0), N(0.0)]);
        let log = b.open();
        let m = throttle_map(&log, "IMU", "AccZ", 0, 5, 10.0).unwrap();
        assert_eq!(m.bands.len(), 2, "{:?}", m.peak_hz);
        assert!((m.peak_hz[0] - 60.0).abs() < 5.0, "{:?}", m.peak_hz);
        assert!((m.peak_hz[1] - 120.0).abs() < 5.0, "{:?}", m.peak_hz);
        assert_eq!((m.bands[0].0 * 10.0).round(), 2.0, "band 0.2..0.4");

        let c = log.column("IMU", "AccZ", Some(("I", 0.0))).unwrap();
        let sg = spectrogram(&c.time_us, &c.values, 1_000_000, 128).unwrap();
        assert_eq!(sg.slices.len(), 20);
        let csv = spectrogram_csv(&sg);
        assert_eq!(csv.lines().count(), 21);
        assert!(csv.starts_with("time_s,0.00,"));
    }
}
