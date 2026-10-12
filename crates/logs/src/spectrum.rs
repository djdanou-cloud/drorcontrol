//! Frequency spectrum of IMU data (plan 05: FFT), for finding vibration peaks, e.g. to set
//! harmonic notch filters. Welch's method: Hann-windowed segments, 50 % overlap, averaged
//! power. Samples are taken as evenly spaced at the mean rate of their timestamps.

use std::f64::consts::PI;

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

/// In-place radix-2 FFT of (re, im); `re.len()` must be a power of two.
fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = -2.0 * PI / len as f64;
        for start in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (s, c) = (ang * k as f64).sin_cos();
                let (a, b) = (start + k, start + k + len / 2);
                let (tr, ti) = (re[b] * c - im[b] * s, re[b] * s + im[b] * c);
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
        len <<= 1;
    }
}

/// Spectrum of `values` sampled at `time_us`, in segments of `segment` samples (a power of
/// two). `None` with fewer samples than one segment.
pub fn spectrum(time_us: &[u64], values: &[f64], segment: usize) -> Option<Spectrum> {
    let n = values.len().min(time_us.len());
    if !segment.is_power_of_two() || segment < 8 || n < segment {
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
    let mut power = vec![0.0; bins];
    let mut segments = 0;
    let mut start = 0;
    while start + segment <= n {
        let chunk = &values[start..start + segment];
        let mean = chunk.iter().sum::<f64>() / segment as f64;
        let mut re: Vec<f64> = chunk
            .iter()
            .zip(&window)
            .map(|(v, w)| (v - mean) * w)
            .collect();
        let mut im = vec![0.0; segment];
        fft(&mut re, &mut im);
        for k in 0..bins {
            power[k] += re[k] * re[k] + im[k] * im[k];
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
            spectrum(&[0; 600], &[0.0; 600], 500).is_none(),
            "not a power of two"
        );
    }
}
