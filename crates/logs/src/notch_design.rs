//! What a harmonic notch does to the gyro signal (plan 05 §5, the notch-filter designer):
//! the attenuation of a set of `INS_HNTCH_*` values at any frequency, so suggested and
//! current settings can be compared before anything is changed on a vehicle.
//!
//! The filter as ArduPilot builds it (4.7.1, `Filter/NotchFilter.cpp`,
//! `Filter/HarmonicNotchFilter.cpp`): one biquad notch per enabled harmonic (`INS_HNTCH_HMNCS`
//! bit n = harmonic n + 1), centred at (n + 1) × `FREQ`, all with the depth
//! A = 10^(−ATT/40) and the Q of the fundamental, Q = √(2^o) / (2^o − 1) with
//! o = 2·log2(FREQ / (FREQ − BW/2)); harmonics at or above the Nyquist limit are left out.
//! Single notches only (the double/triple options of `INS_HNTCH_OPTS` are not modelled).

use std::f64::consts::PI;

/// A harmonic notch configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NotchConfig {
    /// `INS_HNTCH_FREQ`, Hz.
    pub freq_hz: f64,
    /// `INS_HNTCH_BW`, Hz.
    pub bw_hz: f64,
    /// `INS_HNTCH_ATT`, dB (default 40).
    pub att_db: f64,
    /// `INS_HNTCH_HMNCS` bitmask (default 3: 1st and 2nd harmonics).
    pub harmonics: u32,
    /// Rate the filter runs at (the gyro rate), Hz.
    pub sample_hz: f64,
}

impl NotchConfig {
    fn a_and_q(&self) -> Option<(f64, f64)> {
        let half = self.freq_hz - self.bw_hz / 2.0;
        if self.freq_hz <= 0.0 || half <= 0.0 {
            return None;
        }
        let octaves = (self.freq_hz / half).log2() * 2.0;
        let q = 2f64.powf(octaves).sqrt() / (2f64.powf(octaves) - 1.0);
        Some((10f64.powf(-self.att_db / 40.0), q))
    }

    /// Centres of the notches actually built, Hz.
    pub fn centres(&self) -> Vec<f64> {
        (0..16)
            .filter(|n| self.harmonics & (1 << n) != 0)
            .map(|n| self.freq_hz * f64::from(n + 1))
            .filter(|c| *c < self.sample_hz / 2.0)
            .collect()
    }

    /// Gain of the whole filter at `f_hz`, dB (0 = untouched, negative = attenuated).
    pub fn gain_db(&self, f_hz: f64) -> f64 {
        let Some((a, q)) = self.a_and_q() else {
            return 0.0;
        };
        let w = 2.0 * PI * f_hz / self.sample_hz;
        let mut g = 0.0;
        for c in self.centres() {
            let omega = 2.0 * PI * c / self.sample_hz;
            let alpha = omega.sin() / (2.0 * q);
            let (b0, b1, b2) = (1.0 + alpha * a * a, -2.0 * omega.cos(), 1.0 - alpha * a * a);
            let (a0, a1, a2) = (1.0 + alpha, -2.0 * omega.cos(), 1.0 - alpha);
            // |H(e^jw)| for H = (b0 + b1 z^-1 + b2 z^-2) / (a0 + a1 z^-1 + a2 z^-2).
            let h = |c0: f64, c1: f64, c2: f64| {
                let re = c0 + c1 * w.cos() + c2 * (2.0 * w).cos();
                let im = -(c1 * w.sin() + c2 * (2.0 * w).sin());
                (re * re + im * im).sqrt()
            };
            g += 20.0 * (h(b0, b1, b2) / h(a0, a1, a2)).log10();
        }
        g
    }

    /// The response from 1 Hz to Nyquist in `steps` points: (Hz, dB).
    pub fn response(&self, steps: usize) -> Vec<(f64, f64)> {
        let top = self.sample_hz / 2.0;
        (0..steps)
            .map(|i| {
                let f = 1.0 + (top - 1.0) * i as f64 / (steps.max(2) - 1) as f64;
                (f, self.gain_db(f))
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> NotchConfig {
        NotchConfig {
            freq_hz: 80.0,
            bw_hz: 40.0,
            att_db: 40.0,
            harmonics: 3,
            sample_hz: 1000.0,
        }
    }

    #[test]
    fn deepest_at_the_centres_and_flat_far_away() {
        let c = cfg();
        assert_eq!(c.centres(), [80.0, 160.0]);
        // At the centre the depth is ATT (40 dB), give or take the other notch's skirt.
        assert!((c.gain_db(80.0) + 40.0).abs() < 1.5, "{}", c.gain_db(80.0));
        assert!(
            (c.gain_db(160.0) + 40.0).abs() < 1.5,
            "{}",
            c.gain_db(160.0)
        );
        assert!(
            c.gain_db(5.0).abs() < 0.5,
            "low frequencies pass: {}",
            c.gain_db(5.0)
        );
        assert!(c.gain_db(400.0).abs() < 1.0, "{}", c.gain_db(400.0));
        // Half the bandwidth off centre the notch is much shallower than at the centre.
        assert!(
            c.gain_db(60.0) > -10.0 && c.gain_db(60.0) < -1.0,
            "{}",
            c.gain_db(60.0)
        );
        // A harmonic past Nyquist is not built.
        let high = NotchConfig {
            freq_hz: 300.0,
            ..c
        };
        assert_eq!(high.centres(), [300.0]);
        assert_eq!(NotchConfig { freq_hz: 0.0, ..c }.gain_db(80.0), 0.0);
    }
}
