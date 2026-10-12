//! Compass interference from motor current, measured in flight: how the strength of the field a
//! compass reports changes with battery current.
//!
//! In flight the vehicle turns, so the field's direction in the body frame changes; its
//! strength does not. A strength that follows the current is interference left after any
//! compensation. Only the part along the earth field changes the strength, so this is a lower
//! bound of what Copter's on-ground `compassmot` calibration measures, not the same number.
//!
//! `MAG` = TimeUS, I, MagX, MagY, MagZ (mGauss, after offsets and motor compensation), OfsX..,
//! MOX.. (motor compensation applied), Health, S (`AP_Logger/LogStructure.h`, 4.7.1).
//! `BAT.Curr` in A (`AP_BattMonitor/LogStructure.h`).

use crate::dataflash::Log;
use crate::flight::{flying_periods, in_periods};

/// One compass against battery current, while flying.
#[derive(Debug, Clone, PartialEq)]
pub struct CompassInterference {
    /// Compass instance.
    pub instance: u8,
    /// Samples paired with a current reading.
    pub samples: usize,
    /// Mean field strength, mGauss.
    pub mean_field_mgauss: f64,
    /// Change of field strength per amp (least squares), mGauss/A.
    pub mgauss_per_amp: f64,
    /// Highest current, A.
    pub max_current_a: f64,
    /// Change at the highest current relative to no current, % of the mean field.
    pub change_at_max_pct: f64,
    /// Correlation between field strength and current (-1..1).
    pub correlation: f64,
}

/// Field strength against battery 0's current for compass `instance` while flying; `None`
/// without both in flight or without the current varying (standard deviation < 0.5 A).
pub fn compass_interference(log: &Log, instance: u8) -> Option<CompassInterference> {
    let inst = Some(("I", f64::from(instance)));
    let (mx, my, mz) = (
        log.column("MAG", "MagX", inst)?,
        log.column("MAG", "MagY", inst)?,
        log.column("MAG", "MagZ", inst)?,
    );
    let cur = log.column("BAT", "Curr", Some(("Inst", 0.0)))?;
    let (flying, _) = flying_periods(log);
    // Pair each compass sample with the latest current reading at or before it.
    let mut pairs: Vec<(f64, f64)> = Vec::new();
    let mut k = 0;
    for i in 0..mx.values.len().min(my.values.len()).min(mz.values.len()) {
        let t = mx.time_us[i];
        if !in_periods(&flying, t) {
            continue;
        }
        while k + 1 < cur.time_us.len() && cur.time_us[k + 1] <= t {
            k += 1;
        }
        let Some(&a) = cur.values.get(k) else { break };
        if cur.time_us[k] > t || !a.is_finite() {
            continue;
        }
        let b = (mx.values[i].powi(2) + my.values[i].powi(2) + mz.values[i].powi(2)).sqrt();
        pairs.push((a, b));
    }
    let n = pairs.len() as f64;
    if pairs.len() < 10 {
        return None;
    }
    let ma = pairs.iter().map(|p| p.0).sum::<f64>() / n;
    let mb = pairs.iter().map(|p| p.1).sum::<f64>() / n;
    let (mut saa, mut sbb, mut sab) = (0.0, 0.0, 0.0);
    for (a, b) in &pairs {
        saa += (a - ma).powi(2);
        sbb += (b - mb).powi(2);
        sab += (a - ma) * (b - mb);
    }
    if (saa / n).sqrt() < 0.5 || mb <= 0.0 {
        return None;
    }
    let slope = sab / saa;
    let max_current_a = pairs.iter().map(|p| p.0).fold(f64::MIN, f64::max);
    Some(CompassInterference {
        instance,
        samples: pairs.len(),
        mean_field_mgauss: mb,
        mgauss_per_amp: slope,
        max_current_a,
        change_at_max_pct: slope * max_current_a / mb * 100.0,
        correlation: if sbb > 0.0 {
            sab / (saa * sbb).sqrt()
        } else {
            0.0
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn field_strength_following_current_is_found() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("BAT", "QBf", "TimeUS,Inst,Curr")
            .format("MAG", "QBhhh", "TimeUS,I,MagX,MagY,MagZ");
        b.rec("ARM", &[N(0.0), N(1.0), N(0.0), N(0.0), N(0.0)]);
        for i in 0..40 {
            let t = 1e6 + f64::from(i) * 1e5;
            let amps = 10.0 + 10.0 * (f64::from(i) * 0.7).sin();
            b.rec("BAT", &[N(t), N(0.0), N(amps)]);
            // Field of 500 mGauss growing by 2 mGauss/A, turning with the vehicle's yaw.
            let strength = 500.0 + 2.0 * amps;
            let yaw = f64::from(i) * 0.3;
            let (x, y) = (strength * 0.6 * yaw.cos(), strength * 0.6 * yaw.sin());
            let t = t + 1e4;
            b.rec("MAG", &[N(t), N(0.0), N(x), N(y), N(strength * 0.8)]);
            b.rec("MAG", &[N(t), N(1.0), N(300.0), N(0.0), N(0.0)]); // another compass
        }
        let c = compass_interference(&b.open(), 0).unwrap();
        assert_eq!(c.samples, 40);
        assert!((c.mgauss_per_amp - 2.0).abs() < 0.05, "{c:?}"); // int16 rounding
        assert!(c.correlation > 0.99, "{c:?}");
        let want = c.mgauss_per_amp * c.max_current_a / c.mean_field_mgauss * 100.0;
        assert!((c.change_at_max_pct - want).abs() < 1e-9, "{c:?}");
        // The other compass does not change with current.
        let other = compass_interference(&b.open(), 1).unwrap();
        assert!(other.mgauss_per_amp.abs() < 1e-9, "{other:?}");
    }
}
