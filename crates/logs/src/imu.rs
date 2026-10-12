//! IMU health over the log: temperatures, error counts, unhealthy samples and sample rates.
//!
//! `IMU` = TimeUS, I, GyrX..AccZ, EG (gyro error count), EA (accel error count), T
//! (temperature, °C), GH, AH (gyro/accel health, 1 = healthy), GHz, AHz (measurement rates)
//! (`AP_InertialSensor/LogStructure.h`, 4.7.1). The error counts are cumulative.

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
}
