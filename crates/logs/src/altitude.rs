//! Do the altitude sources agree while flying? The barometer against the EKF's height, and
//! the GPS altitude against the EKF position. A growing gap shows barometer drift (heat,
//! prop wash) or a GPS height problem.
//!
//! Sources (ArduCopter 4.7.1): `CTUN.Alt` = EKF height estimate (`get_pos_estimate_U_m`),
//! `CTUN.BAlt` = barometer altitude (`Copter::Log_Write_Control_Tuning`); `GPS.Alt` and
//! `POS.Alt` are both metres above MSL (`AP_GPS/LogStructure.h`, `AP_AHRS/LogStructure.h`).

use crate::dataflash::{Column, Log};
use crate::flight::{flying_periods, in_periods};

/// The gap between two altitude sources while flying (second minus first), m.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AltGap {
    /// Samples compared.
    pub samples: usize,
    /// Mean gap.
    pub mean_m: f64,
    /// Largest gap (absolute) and when, µs since boot.
    pub max_abs_m: f64,
    /// See `max_abs_m`.
    pub max_at_us: u64,
    /// Gap at the end of flying minus at the start (drift over the flight).
    pub drift_m: f64,
}

/// Gap `b - a` at each `a` sample while flying, pairing `b`'s latest sample at or before.
fn gap(a: &Column, b: &Column, flying: &[crate::flight::ArmedPeriod]) -> Option<AltGap> {
    let mut k = 0;
    let mut gaps: Vec<(u64, f64)> = Vec::new();
    for (t, va) in a.time_us.iter().zip(&a.values) {
        if !in_periods(flying, *t) || !va.is_finite() {
            continue;
        }
        while k + 1 < b.time_us.len() && b.time_us[k + 1] <= *t {
            k += 1;
        }
        match (b.time_us.get(k), b.values.get(k)) {
            (Some(tb), Some(vb)) if *tb <= *t && vb.is_finite() => gaps.push((*t, vb - va)),
            _ => {}
        }
    }
    let (first, last) = (gaps.first()?.1, gaps.last()?.1);
    let (max_at_us, max) = gaps
        .iter()
        .copied()
        .max_by(|x, y| x.1.abs().total_cmp(&y.1.abs()))?;
    Some(AltGap {
        samples: gaps.len(),
        mean_m: gaps.iter().map(|g| g.1).sum::<f64>() / gaps.len() as f64,
        max_abs_m: max.abs(),
        max_at_us,
        drift_m: last - first,
    })
}

/// Barometer minus EKF height (`CTUN.BAlt - CTUN.Alt`) while flying.
pub fn baro_vs_ekf(log: &Log) -> Option<AltGap> {
    let (flying, _) = flying_periods(log);
    gap(
        &log.column("CTUN", "Alt", None)?,
        &log.column("CTUN", "BAlt", None)?,
        &flying,
    )
}

/// GPS altitude (first receiver) minus EKF altitude (`GPS.Alt - POS.Alt`, both MSL) while
/// flying.
pub fn gps_vs_ekf(log: &Log) -> Option<AltGap> {
    let (flying, _) = flying_periods(log);
    gap(
        &log.column("POS", "Alt", None)?,
        &log.column("GPS", "Alt", Some(("I", 0.0)))?,
        &flying,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn baro_drifting_away_from_the_ekf() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("CTUN", "Qff", "TimeUS,Alt,BAlt");
        b.rec("ARM", &[N(1e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        // EKF at 20 m; the barometer reads 0.1 m more each second, from +0.5 m.
        for i in 0..10 {
            let t = 2e6 + f64::from(i) * 1e6;
            b.rec("CTUN", &[N(t), N(20.0), N(20.5 + 0.1 * f64::from(i))]);
        }
        let g = baro_vs_ekf(&b.open()).unwrap();
        assert_eq!(g.samples, 10);
        assert!((g.drift_m - 0.9).abs() < 1e-4, "{g:?}");
        assert!(
            (g.max_abs_m - 1.4).abs() < 1e-4 && g.max_at_us == 11_000_000,
            "{g:?}"
        );
        assert!((g.mean_m - 0.95).abs() < 1e-4, "{g:?}");
    }
}
