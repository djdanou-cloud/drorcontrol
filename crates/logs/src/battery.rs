//! The battery under load while flying: current and power, how far the voltage sagged, charge
//! and energy used, and the pack's internal resistance (logged by ArduPilot and, as a
//! cross-check, fitted from voltage against current).
//!
//! `BAT` = TimeUS, Inst, Volt, VoltR (resting voltage estimate), Curr (A), CurrTot (mAh),
//! EnrgTot (Wh), Temp, Res (ohm), RemPct, H, SH (`AP_BattMonitor/LogStructure.h`, 4.7.1).

use crate::dataflash::Log;
use crate::flight::{flying_periods, in_periods};

/// One battery while flying.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BatteryUse {
    /// `BAT` samples used.
    pub samples: usize,
    /// Time flying, s.
    pub flying_s: f64,
    /// Whether flying was told by the land detector (else: whenever armed).
    pub land_detector: bool,
    /// Highest and mean current, A.
    pub max_current_a: f64,
    /// See `max_current_a`.
    pub mean_current_a: f64,
    /// Highest and mean power (volts × amps), W.
    pub max_power_w: f64,
    /// See `max_power_w`.
    pub mean_power_w: f64,
    /// Lowest voltage under load, V.
    pub min_voltage: f64,
    /// Largest gap between the resting-voltage estimate and the measured voltage, V.
    pub max_sag_v: Option<f64>,
    /// Charge used while flying, mAh.
    pub used_mah: Option<f64>,
    /// Energy used while flying, Wh.
    pub used_wh: Option<f64>,
    /// Last resistance ArduPilot estimated, ohm.
    pub logged_resistance_ohm: Option<f64>,
    /// Resistance fitted from voltage against current (least squares), ohm; needs the
    /// current to vary (standard deviation of at least 0.5 A).
    pub fitted_resistance_ohm: Option<f64>,
}

impl BatteryUse {
    /// Average consumption, mAh per minute of flight.
    pub fn mah_per_min(&self) -> Option<f64> {
        (self.flying_s > 0.0).then_some(self.used_mah? * 60.0 / self.flying_s)
    }
}

/// Battery `instance` (0 = first) while flying; `None` without current measurements in flight.
pub fn battery_use(log: &Log, instance: u8) -> Option<BatteryUse> {
    let inst = Some(("Inst", f64::from(instance)));
    let col = |f: &str| log.column("BAT", f, inst);
    let (volt, curr) = (col("Volt")?, col("Curr")?);
    let (flying, land_detector) = flying_periods(log);
    let keep: Vec<usize> = (0..volt.values.len().min(curr.values.len()))
        .filter(|&i| in_periods(&flying, volt.time_us[i]))
        .filter(|&i| volt.values[i].is_finite() && curr.values[i].is_finite())
        .collect();
    if keep.is_empty() {
        return None;
    }
    let n = keep.len() as f64;
    let v: Vec<f64> = keep.iter().map(|&i| volt.values[i]).collect();
    let a: Vec<f64> = keep.iter().map(|&i| curr.values[i]).collect();
    let p: Vec<f64> = v.iter().zip(&a).map(|(v, a)| v * a).collect();
    let max = |x: &[f64]| x.iter().copied().fold(f64::MIN, f64::max);
    let mean = |x: &[f64]| x.iter().sum::<f64>() / n;
    let delta = |field: &str| {
        let c = col(field)?;
        let inside: Vec<f64> = c
            .time_us
            .iter()
            .zip(&c.values)
            .filter(|(t, v)| in_periods(&flying, **t) && v.is_finite())
            .map(|(_, v)| *v)
            .collect();
        Some(inside.last()? - inside.first()?)
    };
    let max_sag_v = col("VoltR").map(|r| {
        keep.iter()
            .filter_map(|&i| Some(r.values.get(i)? - volt.values[i]))
            .fold(0.0, f64::max)
    });
    let (ma, mv) = (mean(&a), mean(&v));
    let var_a = a.iter().map(|x| (x - ma).powi(2)).sum::<f64>() / n;
    let cov = a
        .iter()
        .zip(&v)
        .map(|(x, y)| (x - ma) * (y - mv))
        .sum::<f64>()
        / n;
    let flying_s = flying.iter().fold(0.0, |s, p| {
        s + p.end_us.map_or(0, |e| e.saturating_sub(p.start_us)) as f64 / 1e6
    });
    Some(BatteryUse {
        samples: keep.len(),
        flying_s,
        land_detector,
        max_current_a: max(&a),
        mean_current_a: ma,
        max_power_w: max(&p),
        mean_power_w: mean(&p),
        min_voltage: v.iter().copied().fold(f64::MAX, f64::min),
        max_sag_v,
        used_mah: delta("CurrTot"),
        used_wh: delta("EnrgTot"),
        logged_resistance_ohm: col("Res").and_then(|c| {
            keep.iter()
                .rev()
                .find_map(|&i| c.values.get(i).copied().filter(|r| r.is_finite()))
        }),
        fitted_resistance_ohm: (var_a.sqrt() >= 0.5).then(|| -cov / var_a + 0.0), // no -0.0
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn load_figures_while_flying() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("EV", "QB", "TimeUS,Id")
            .format(
                "BAT",
                "QBfffffcfBBB",
                "TimeUS,Inst,Volt,VoltR,Curr,CurrTot,EnrgTot,Temp,Res,RemPct,H,SH",
            );
        let s = |x: f64| x * 1e6;
        let bat = |b: &mut LogBuilder, t: f64, inst: f64, amps: f64, mah: f64| {
            // A 16.8 V pack with 0.02 ohm internal resistance.
            let volt = 16.8 - 0.02 * amps;
            b.rec(
                "BAT",
                &[
                    N(s(t)),
                    N(inst),
                    N(volt),
                    N(16.8),
                    N(amps),
                    N(mah),
                    N(mah * 16.0 / 1000.0),
                    N(25.0),
                    N(0.021),
                    N(90.0),
                    N(1.0),
                    N(100.0),
                ],
            );
        };
        b.rec("ARM", &[N(s(1.0)), N(1.0), N(0.0), N(0.0), N(0.0)]);
        bat(&mut b, 1.5, 0.0, 1.0, 0.0); // armed on the ground: ignored
        b.rec("EV", &[N(s(2.0)), N(28.0)]);
        for (i, amps) in [10.0, 20.0, 30.0, 20.0].iter().enumerate() {
            let t = 3.0 + i as f64 * 10.0;
            bat(&mut b, t, 0.0, *amps, 100.0 + i as f64 * 50.0);
            bat(&mut b, t, 1.0, 99.0, 0.0); // a second battery: not mixed in
        }
        b.rec("EV", &[N(s(62.0)), N(18.0)]);
        let u = battery_use(&b.open(), 0).unwrap();
        assert_eq!(u.samples, 4);
        assert!((u.flying_s - 60.0).abs() < 1e-9);
        assert_eq!(u.max_current_a, 30.0);
        assert!((u.mean_current_a - 20.0).abs() < 1e-9);
        assert!((u.min_voltage - 16.2).abs() < 1e-5);
        assert!((u.max_sag_v.unwrap() - 0.6).abs() < 1e-5);
        assert!((u.used_mah.unwrap() - 150.0).abs() < 1e-3);
        assert!((u.mah_per_min().unwrap() - 150.0).abs() < 1e-3);
        assert!((u.logged_resistance_ohm.unwrap() - 0.021).abs() < 1e-6);
        assert!(
            (u.fitted_resistance_ohm.unwrap() - 0.02).abs() < 1e-4,
            "{u:?}"
        );
    }
}
