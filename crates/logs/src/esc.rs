//! ESC telemetry while armed: motor speed, the telemetry error rate, and (where the ESC
//! reports them) voltage, current and temperatures.
//!
//! `ESC` = TimeUS, Instance, RPM, RawRPM, Volt, Curr, Temp (°C; logged in centidegrees),
//! CTot, MotTemp, Err (error rate, %) (`AP_ESC_Telem/LogStructure.h`, 4.7.1). With
//! bidirectional DShot only RPM and the error rate are measured; the rest stay 0.

use crate::dataflash::Log;
use crate::flight::{armed_periods, in_periods};

/// One ESC while armed.
#[derive(Debug, Clone, PartialEq)]
pub struct EscStats {
    /// ESC instance (0-based; usually motor number - 1).
    pub instance: u8,
    /// Samples.
    pub samples: usize,
    /// Mean and highest RPM.
    pub mean_rpm: f64,
    /// See `mean_rpm`.
    pub max_rpm: f64,
    /// Mean and highest telemetry error rate, %.
    pub mean_err_pct: f64,
    /// See `mean_err_pct`.
    pub max_err_pct: f64,
    /// Highest ESC temperature, °C (`None` if never reported).
    pub max_temp_c: Option<f64>,
    /// Mean current, A (`None` if never reported).
    pub mean_current_a: Option<f64>,
}

/// Per-ESC statistics while armed, by instance.
pub fn esc_stats(log: &Log) -> Vec<EscStats> {
    let armed = armed_periods(log);
    let Some(inst) = log.column("ESC", "Instance", None) else {
        return Vec::new();
    };
    let mut ids: Vec<u8> = inst.values.iter().map(|v| *v as u8).collect();
    ids.sort_unstable();
    ids.dedup();
    let mut out = Vec::new();
    for id in ids {
        let sel = Some(("Instance", f64::from(id)));
        let col = |f: &str| log.column("ESC", f, sel);
        let (Some(rpm), Some(err)) = (col("RPM"), col("Err")) else {
            continue;
        };
        let (temp, curr) = (col("Temp"), col("Curr"));
        let idx: Vec<usize> = (0..rpm.values.len())
            .filter(|&i| in_periods(&armed, rpm.time_us[i]))
            .collect();
        if idx.is_empty() {
            continue;
        }
        let n = idx.len() as f64;
        let reported = |c: &Option<crate::dataflash::Column>| -> Vec<f64> {
            c.as_ref()
                .map(|c| {
                    idx.iter()
                        .filter_map(|&i| c.values.get(i).copied())
                        .filter(|v| v.is_finite() && *v != 0.0)
                        .collect()
                })
                .unwrap_or_default()
        };
        let (t, a) = (reported(&temp), reported(&curr));
        out.push(EscStats {
            instance: id,
            samples: idx.len(),
            mean_rpm: idx.iter().map(|&i| rpm.values[i]).sum::<f64>() / n,
            max_rpm: idx.iter().map(|&i| rpm.values[i]).fold(f64::MIN, f64::max),
            mean_err_pct: idx.iter().map(|&i| err.values[i]).sum::<f64>() / n,
            max_err_pct: idx.iter().map(|&i| err.values[i]).fold(f64::MIN, f64::max),
            max_temp_c: t.iter().copied().reduce(f64::max),
            mean_current_a: (!a.is_empty()).then(|| a.iter().sum::<f64>() / a.len() as f64),
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn per_esc_while_armed() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format(
                "ESC",
                "QBffffcfcf",
                "TimeUS,Instance,RPM,RawRPM,Volt,Curr,Temp,CTot,MotTemp,Err",
            );
        let esc = |b: &mut LogBuilder, t: f64, i: f64, rpm: f64, temp: f64, err: f64| {
            b.rec(
                "ESC",
                &[
                    N(t),
                    N(i),
                    N(rpm),
                    N(rpm),
                    N(0.0),
                    N(0.0),
                    N(temp),
                    N(0.0),
                    N(0.0),
                    N(err),
                ],
            );
        };
        esc(&mut b, 0.5e6, 0.0, 9999.0, 0.0, 50.0); // before arming: ignored
        b.rec("ARM", &[N(1e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        esc(&mut b, 2e6, 0.0, 3000.0, 41.5, 2.0);
        esc(&mut b, 2e6, 1.0, 3100.0, 0.0, 12.0);
        esc(&mut b, 3e6, 0.0, 5000.0, 45.0, 4.0);
        let s = esc_stats(&b.open());
        assert_eq!(s.len(), 2);
        assert_eq!(
            (s[0].samples, s[0].mean_rpm, s[0].max_rpm),
            (2, 4000.0, 5000.0)
        );
        assert!((s[0].mean_err_pct - 3.0).abs() < 1e-6 && (s[0].max_err_pct - 4.0).abs() < 1e-6);
        assert!((s[0].max_temp_c.unwrap() - 45.0).abs() < 1e-6);
        assert_eq!(
            (s[1].max_temp_c, s[1].mean_current_a),
            (None, None),
            "not reported"
        );
    }
}
