//! Frequency spectrum of IMU data (plan 05: FFT), for finding vibration peaks, e.g. to set
//! harmonic notch filters. Welch's method: Hann-windowed segments, 50 % overlap, averaged
//! power. Samples are taken as evenly spaced at the mean rate of their timestamps. The FFT is
//! `rustfft` (MIT/Apache-2.0; ADR-0012: mature crates over custom primitives).

use std::f64::consts::PI;

use rustfft::{FftPlanner, num_complex::Complex};

/// Averaged power spectrum.
#[derive(Debug, Clone, PartialEq)]
pub struct Spectrum {
    /// Mean sample rate, Hz.
    pub sample_rate_hz: f64,
    /// Frequency of each bin, Hz (0 to Nyquist).
    pub freqs: Vec<f64>,
    /// Power per bin (arbitrary units, comparable within one spectrum).
    pub power: Vec<f64>,
    /// Segments averaged.
    pub segments: usize,
}

/// Spectrum of `values` sampled at `time_us`, in segments of `segment` samples (any length
/// of at least 8; powers of two are fastest). `None` with fewer samples than one segment.
pub fn spectrum(time_us: &[u64], values: &[f64], segment: usize) -> Option<Spectrum> {
    let n = values.len().min(time_us.len());
    if segment < 8 || n < segment {
        return None;
    }
    let span = time_us[n - 1].checked_sub(time_us[0])? as f64 / 1e6;
    if span <= 0.0 {
        return None;
    }
    let rate = (n - 1) as f64 / span;
    let window: Vec<f64> = (0..segment)
        .map(|i| 0.5 - 0.5 * (2.0 * PI * i as f64 / segment as f64).cos())
        .collect();
    let bins = segment / 2 + 1;
    let fft = FftPlanner::<f64>::new().plan_fft_forward(segment);
    let mut buf = vec![Complex::new(0.0, 0.0); segment];
    let mut power = vec![0.0; bins];
    let mut segments = 0;
    let mut start = 0;
    while start + segment <= n {
        let chunk = &values[start..start + segment];
        let mean = chunk.iter().sum::<f64>() / segment as f64;
        for ((b, v), w) in buf.iter_mut().zip(chunk).zip(&window) {
            *b = Complex::new((v - mean) * w, 0.0);
        }
        fft.process(&mut buf);
        for (p, c) in power.iter_mut().zip(&buf) {
            *p += c.norm_sqr();
        }
        segments += 1;
        start += segment / 2;
    }
    for p in &mut power {
        *p /= segments as f64;
    }
    Some(Spectrum {
        sample_rate_hz: rate,
        freqs: (0..bins)
            .map(|k| k as f64 * rate / segment as f64)
            .collect(),
        power,
        segments,
    })
}

/// The strongest local maxima at or above `min_hz`, strongest first: (frequency, power).
pub fn peaks(s: &Spectrum, min_hz: f64, count: usize) -> Vec<(f64, f64)> {
    let mut p: Vec<(f64, f64)> = (1..s.power.len().saturating_sub(1))
        .filter(|&k| {
            s.freqs[k] >= min_hz && s.power[k] > s.power[k - 1] && s.power[k] >= s.power[k + 1]
        })
        .map(|k| (s.freqs[k], s.power[k]))
        .collect();
    p.sort_by(|a, b| b.1.total_cmp(&a.1));
    p.truncate(count);
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_the_frequencies_of_mixed_sines() {
        // 1 kHz for 4 s: 80 Hz strong, 210 Hz weaker, plus a constant offset.
        let rate = 1000.0;
        let t: Vec<u64> = (0..4000).map(|i| (i as f64 * 1e6 / rate) as u64).collect();
        let v: Vec<f64> = (0..4000)
            .map(|i| {
                let x = i as f64 / rate;
                9.81 + 2.0 * (2.0 * PI * 80.0 * x).sin() + 0.5 * (2.0 * PI * 210.0 * x).sin()
            })
            .collect();
        let s = spectrum(&t, &v, 512).unwrap();
        assert!((s.sample_rate_hz - 1000.0).abs() < 1.0);
        assert_eq!(s.segments, 14);
        let p = peaks(&s, 10.0, 2);
        let bin = 1000.0 / 512.0;
        assert!((p[0].0 - 80.0).abs() <= bin, "{p:?}");
        assert!((p[1].0 - 210.0).abs() <= bin, "{p:?}");
        assert!(p[0].1 > p[1].1);
    }

    #[test]
    fn rejects_too_little_data() {
        assert!(spectrum(&[0, 1000], &[1.0, 2.0], 512).is_none());
        assert!(
            spectrum(&[0; 600], &[0.0; 600], 4).is_none(),
            "segment too short"
        );
    }

    #[test]
    fn any_segment_length_works() {
        // 1 kHz, 100 Hz tone; a 1000-sample segment gives exactly 1 Hz bins.
        let t: Vec<u64> = (0..3000).map(|i| i * 1000).collect();
        let v: Vec<f64> = (0..3000)
            .map(|i| (2.0 * PI * 100.0 * i as f64 / 1000.0).sin())
            .collect();
        let s = spectrum(&t, &v, 1000).unwrap();
        assert_eq!(peaks(&s, 10.0, 1)[0].0.round(), 100.0);
    }
}
