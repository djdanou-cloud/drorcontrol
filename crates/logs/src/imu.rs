//! IMU health over the log: temperatures, error counts, unhealthy samples and sample rates.
//!
//! `IMU` = TimeUS, I, GyrX..AccZ, EG (gyro error count), EA (accel error count), T
//! (temperature, °C), GH, AH (gyro/accel health, 1 = healthy), GHz, AHz (measurement rates)
//! (`AP_InertialSensor/LogStructure.h`, 4.7.1). The error counts are cumulative.
//!
//! Consistency follows the arming checks (`AP_InertialSensor::accels_consistent` and
//! `gyros_consistent`, `AP_Arming.cpp`, 4.7.1): each IMU against the first, accelerometers
//! within `ARMING_ACCTHRESH` (default 0.75 m/s², three times that from the third IMU, Z
//! difference halved), gyros within 5°/s. Those checks run at rest; in flight the vibration
//! of separately mounted IMUs differs, so the difference is taken between 5 s means
//! ([`CONSISTENCY_WINDOW_US`]; 1 s windows let heavy noise at the usual 25 Hz IMU logging
//! rate cross the caution level now and then in SITL), which
//! keeps a bias or a scale error and averages vibration out. Partial windows (fewer than half
//! the typical samples, at arming and disarming) are left out.

use crate::dataflash::Log;
use crate::flight::{armed_periods, in_periods};

/// One IMU over the log.
#[derive(Debug, Clone, PartialEq)]
pub struct ImuHealth {
    /// Instance.
    pub instance: u8,
    /// Samples.
    pub samples: usize,
    /// Lowest and highest temperature, °C.
    pub temp_c: (f64, f64),
    /// Gyro and accel errors counted during the log.
    pub gyro_errors: u64,
    /// See `gyro_errors`.
    pub accel_errors: u64,
    /// Samples reporting the gyro or accel unhealthy while armed.
    pub unhealthy_armed: usize,
    /// Gyro and accel measurement rates, Hz (latest).
    pub rates_hz: (f64, f64),
}

/// IMU health per instance.
pub fn imu_health(log: &Log) -> Vec<ImuHealth> {
    let armed = armed_periods(log);
    let Some(inst) = log.column("IMU", "I", None) else {
        return Vec::new();
    };
    let mut ids: Vec<u8> = inst.values.iter().map(|v| *v as u8).collect();
    ids.sort_unstable();
    ids.dedup();
    let mut out = Vec::new();
    for id in ids {
        let sel = Some(("I", f64::from(id)));
        let col = |f: &str| log.column("IMU", f, sel);
        let (Some(t), Some(eg), Some(ea), Some(gh), Some(ah)) =
            (col("T"), col("EG"), col("EA"), col("GH"), col("AH"))
        else {
            continue;
        };
        let n = t.values.len();
        if n == 0 {
            continue;
        }
        let growth = |c: &[f64]| {
            c.windows(2)
                .map(|w| if w[1] >= w[0] { w[1] - w[0] } else { w[1] })
                .sum::<f64>() as u64
        };
        let rate = |f: &str| col(f).and_then(|c| c.values.last().copied()).unwrap_or(0.0);
        out.push(ImuHealth {
            instance: id,
            samples: n,
            temp_c: (
                t.values.iter().copied().fold(f64::MAX, f64::min),
                t.values.iter().copied().fold(f64::MIN, f64::max),
            ),
            gyro_errors: growth(&eg.values),
            accel_errors: growth(&ea.values),
            unhealthy_armed: (0..n)
                .filter(|&i| {
                    in_periods(&armed, t.time_us[i]) && (gh.values[i] == 0.0 || ah.values[i] == 0.0)
                })
                .count(),
            rates_hz: (rate("GHz"), rate("AHz")),
        });
    }
    out
}

/// One IMU against the first while armed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuConsistency {
    /// Instance compared with IMU 0.
    pub instance: u8,
    /// Largest 5 s mean accelerometer difference, m/s² (Z halved), and the arming limit.
    pub accel_ms2: f64,
    /// See `accel_ms2`.
    pub accel_limit: f64,
    /// Largest 5 s mean gyro difference, °/s (limit 5).
    pub gyro_degs: f64,
    /// When the largest accelerometer difference was seen, µs.
    pub at_us: u64,
}

/// Length of the windows averaged before comparing IMUs, µs.
pub const CONSISTENCY_WINDOW_US: u64 = 5_000_000;

/// 5 s means of the six IMU axes of one instance while armed: (window start, means).
fn window_means(log: &Log, id: u8, armed: &[crate::flight::ArmedPeriod]) -> Vec<(u64, [f64; 6])> {
    let sel = Some(("I", f64::from(id)));
    let cols: Option<Vec<_>> = ["AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ"]
        .iter()
        .map(|f| log.column("IMU", f, sel))
        .collect();
    let Some(cols) = cols else {
        return Vec::new();
    };
    let mut out: Vec<(u64, [f64; 6], usize)> = Vec::new();
    for (k, t) in cols[0].time_us.iter().enumerate() {
        if !in_periods(armed, *t) {
            continue;
        }
        let w = t / CONSISTENCY_WINDOW_US * CONSISTENCY_WINDOW_US;
        if out.last().is_none_or(|l| l.0 != w) {
            out.push((w, [0.0; 6], 0));
        }
        let l = out.last_mut().unwrap();
        for (a, c) in l.1.iter_mut().zip(&cols) {
            *a += c.values[k];
        }
        l.2 += 1;
    }
    // Partial windows (at arming and disarming) hold a few samples, whose mean is mostly
    // noise: keep windows with at least half the typical count.
    let mut counts: Vec<usize> = out.iter().map(|w| w.2).collect();
    counts.sort_unstable();
    let typical = counts.get(counts.len() / 2).copied().unwrap_or(0);
    out.into_iter()
        .filter(|w| w.2 > 0 && w.2 * 2 >= typical)
        .map(|(t, s, n)| (t, s.map(|v| v / n as f64)))
        .collect()
}

/// Every IMU after the first, against the first, while armed.
pub fn imu_consistency(log: &Log) -> Vec<ImuConsistency> {
    let armed = armed_periods(log);
    let Some(inst) = log.column("IMU", "I", None) else {
        return Vec::new();
    };
    let mut ids: Vec<u8> = inst.values.iter().map(|v| *v as u8).collect();
    ids.sort_unstable();
    ids.dedup();
    let thresh = log
        .params()
        .get("ARMING_ACCTHRESH")
        .copied()
        .unwrap_or(0.75);
    let primary = window_means(log, 0, &armed);
    let mut out = Vec::new();
    for id in ids.into_iter().filter(|i| *i > 0) {
        let other = window_means(log, id, &armed);
        let mut worst = (0.0f64, 0.0f64, 0u64);
        for (t, b) in &other {
            let Some((_, a)) = primary.iter().find(|(pt, _)| pt == t) else {
                continue;
            };
            let d: Vec<f64> = a.iter().zip(b).map(|(x, y)| y - x).collect();
            let acc = (d[0].powi(2) + d[1].powi(2) + (0.5 * d[2]).powi(2)).sqrt();
            let gyr = (d[3].powi(2) + d[4].powi(2) + d[5].powi(2))
                .sqrt()
                .to_degrees();
            if acc > worst.0 {
                worst = (acc, worst.1, *t);
            }
            worst.1 = worst.1.max(gyr);
        }
        if other.is_empty() || primary.is_empty() {
            continue;
        }
        out.push(ImuConsistency {
            instance: id,
            accel_ms2: worst.0,
            accel_limit: if id >= 2 { thresh * 3.0 } else { thresh },
            gyro_degs: worst.1,
            at_us: worst.2,
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn temperatures_errors_and_health() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("IMU", "QBIIfBBHH", "TimeUS,I,EG,EA,T,GH,AH,GHz,AHz");
        let imu = |b: &mut LogBuilder, t: f64, i: f64, eg: f64, temp: f64, gh: f64| {
            b.rec(
                "IMU",
                &[
                    N(t),
                    N(i),
                    N(eg),
                    N(0.0),
                    N(temp),
                    N(gh),
                    N(1.0),
                    N(2000.0),
                    N(1000.0),
                ],
            );
        };
        imu(&mut b, 0.5e6, 0.0, 0.0, 40.0, 0.0); // unhealthy before arming: not counted
        b.rec("ARM", &[N(1e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        imu(&mut b, 2e6, 0.0, 3.0, 45.0, 1.0);
        imu(&mut b, 3e6, 0.0, 5.0, 47.5, 0.0);
        imu(&mut b, 3e6, 1.0, 0.0, 30.0, 1.0);
        let h = imu_health(&b.open());
        assert_eq!(h.len(), 2);
        assert_eq!((h[0].gyro_errors, h[0].unhealthy_armed), (5, 1));
        assert_eq!(h[0].temp_c, (40.0, 47.5));
        assert_eq!(h[0].rates_hz, (2000.0, 1000.0));
        assert_eq!((h[1].samples, h[1].gyro_errors), (1, 0));
    }

    #[test]
    fn consistency_uses_window_means() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("IMU", "QBffffff", "TimeUS,I,AccX,AccY,AccZ,GyrX,GyrY,GyrZ");
        b.rec("ARM", &[N(0.5e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        for k in 0..20 {
            let t = 1e6 + f64::from(k) * 0.1e6;
            // IMU 0 vibrates ±3 m/s² on X; IMU 1 has a 0.4 m/s² X bias and 2 m/s² Z bias
            // (halved: 1), and IMU 2 a 0.1 rad/s gyro bias.
            let vib = if k % 2 == 0 { 3.0 } else { -3.0 };
            b.rec(
                "IMU",
                &[
                    N(t),
                    N(0.0),
                    N(vib),
                    N(0.0),
                    N(-9.8),
                    N(0.0),
                    N(0.0),
                    N(0.0),
                ],
            );
            b.rec(
                "IMU",
                &[
                    N(t),
                    N(1.0),
                    N(0.4),
                    N(0.0),
                    N(-7.8),
                    N(0.0),
                    N(0.0),
                    N(0.0),
                ],
            );
            b.rec(
                "IMU",
                &[
                    N(t),
                    N(2.0),
                    N(0.0),
                    N(0.0),
                    N(-9.8),
                    N(0.1),
                    N(0.0),
                    N(0.0),
                ],
            );
        }
        let c = imu_consistency(&b.open());
        assert_eq!(c.len(), 2);
        let expect = (0.4f64.powi(2) + 1.0f64).sqrt();
        assert!((c[0].accel_ms2 - expect).abs() < 1e-5, "{c:?}");
        assert_eq!((c[0].accel_limit, c[1].accel_limit), (0.75, 2.25));
        assert!(c[0].gyro_degs < 1e-9);
        assert!((c[1].gyro_degs - 0.1f64.to_degrees()).abs() < 1e-5);
        assert!(c[1].accel_ms2 < 1e-9);
    }

    #[test]
    fn a_partial_window_at_disarm_is_left_out() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("IMU", "QBffffff", "TimeUS,I,AccX,AccY,AccZ,GyrX,GyrY,GyrZ");
        b.rec("ARM", &[N(0.0), N(1.0), N(0.0), N(0.0), N(0.0)]);
        // Two full 5 s windows that agree, then two noisy samples just before disarming.
        for k in 0..100 {
            let t = f64::from(k) * 0.1e6;
            for i in [0.0, 1.0] {
                b.rec(
                    "IMU",
                    &[N(t), N(i), N(0.0), N(0.0), N(-9.8), N(0.0), N(0.0), N(0.0)],
                );
            }
        }
        for t in [10.0e6, 10.1e6] {
            b.rec(
                "IMU",
                &[
                    N(t),
                    N(0.0),
                    N(0.0),
                    N(0.0),
                    N(-9.8),
                    N(0.0),
                    N(0.0),
                    N(0.0),
                ],
            );
            b.rec(
                "IMU",
                &[
                    N(t),
                    N(1.0),
                    N(3.0),
                    N(0.0),
                    N(-9.8),
                    N(0.0),
                    N(0.0),
                    N(0.0),
                ],
            );
        }
        b.rec("ARM", &[N(10.2e6), N(0.0), N(0.0), N(0.0), N(0.0)]);
        let c = imu_consistency(&b.open());
        assert!(c[0].accel_ms2 < 1e-9, "{c:?}");
    }
}
