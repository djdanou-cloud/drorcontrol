//! Optical flow and rangefinder while flying: could the EKF use them?
//!
//! Sources (ArduPilot 4.7.1):
//! * `OF` = TimeUS, Qual (0..255), flowX, flowY, bodyX, bodyY (rad/s) (`AP_Logger/LogStructure.h`).
//!   EKF3 uses a flow sample only with quality above 0 and flow and body rates under 4.2 rad/s
//!   (`NavEKF3_core::writeOptFlowMeas`).
//! * `RFND` = TimeUS, Instance, Dist (m), Stat, Orient, Quality; Stat is
//!   `RangeFinder::Status`: 0 not connected, 1 no data, 2 out of range low, 3 out of range
//!   high, 4 good (`AP_RangeFinder.h`).

use crate::dataflash::Log;
use crate::flight::{flying_periods, in_periods};

/// Optical flow while flying.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlowQuality {
    /// Samples.
    pub samples: usize,
    /// Mean and lowest quality (0..255).
    pub mean_quality: f64,
    /// See `mean_quality`.
    pub min_quality: f64,
    /// Share of samples the EKF could use (quality > 0, rates < 4.2 rad/s), 0..1.
    pub usable: f64,
    /// Share with quality 0, 0..1.
    pub zero_quality: f64,
    /// Share with a flow or body rate of 4.2 rad/s or more, 0..1.
    pub too_fast: f64,
}

/// EKF3's flow-rate limit, rad/s (`writeOptFlowMeas`).
const FLOW_RATE_LIMIT: f64 = 4.2;

/// Optical flow while flying; `None` without `OF` samples in flight.
pub fn flow(log: &Log) -> Option<FlowQuality> {
    let col = |f: &str| log.column("OF", f, None);
    let (q, fx, fy, bx, by) = (
        col("Qual")?,
        col("flowX")?,
        col("flowY")?,
        col("bodyX")?,
        col("bodyY")?,
    );
    let (flying, _) = flying_periods(log);
    let idx: Vec<usize> = (0..q.values.len())
        .filter(|&i| in_periods(&flying, q.time_us[i]))
        .collect();
    if idx.is_empty() {
        return None;
    }
    let n = idx.len() as f64;
    let fast = |i: usize| {
        fx.values[i].hypot(fy.values[i]) >= FLOW_RATE_LIMIT
            || bx.values[i].hypot(by.values[i]) >= FLOW_RATE_LIMIT
    };
    let share = |pred: &dyn Fn(usize) -> bool| idx.iter().filter(|&&i| pred(i)).count() as f64 / n;
    Some(FlowQuality {
        samples: idx.len(),
        mean_quality: idx.iter().map(|&i| q.values[i]).sum::<f64>() / n,
        min_quality: idx.iter().map(|&i| q.values[i]).fold(f64::MAX, f64::min),
        usable: share(&|i| q.values[i] > 0.0 && !fast(i)),
        zero_quality: share(&|i| q.values[i] == 0.0),
        too_fast: share(&|i| fast(i)),
    })
}

/// One rangefinder while flying.
#[derive(Debug, Clone, PartialEq)]
pub struct RangefinderUse {
    /// Instance.
    pub instance: u8,
    /// Orientation (`Rotation`, 25 = down).
    pub orientation: u8,
    /// Samples.
    pub samples: usize,
    /// Share of samples per status 0..=4 (see the module notes), 0..1.
    pub status_share: [f64; 5],
    /// Shortest and longest good reading, m.
    pub good_range_m: Option<(f64, f64)>,
}

/// Each rangefinder while flying, by instance.
pub fn rangefinders(log: &Log) -> Vec<RangefinderUse> {
    let (flying, _) = flying_periods(log);
    let Some(inst) = log.column("RFND", "Instance", None) else {
        return Vec::new();
    };
    let mut ids: Vec<u8> = inst.values.iter().map(|v| *v as u8).collect();
    ids.sort_unstable();
    ids.dedup();
    let mut out = Vec::new();
    for id in ids {
        let sel = Some(("Instance", f64::from(id)));
        let (Some(d), Some(st), Some(or)) = (
            log.column("RFND", "Dist", sel),
            log.column("RFND", "Stat", sel),
            log.column("RFND", "Orient", sel),
        ) else {
            continue;
        };
        let mut counts = [0usize; 5];
        let mut good: Option<(f64, f64)> = None;
        let mut n = 0;
        for (i, t) in d.time_us.iter().enumerate() {
            if !in_periods(&flying, *t) {
                continue;
            }
            n += 1;
            let s = st.values[i] as usize;
            if let Some(c) = counts.get_mut(s) {
                *c += 1;
            }
            if s == 4 {
                let v = d.values[i];
                good = Some(good.map_or((v, v), |(a, b)| (a.min(v), b.max(v))));
            }
        }
        if n == 0 {
            continue;
        }
        out.push(RangefinderUse {
            instance: id,
            orientation: or.values.first().copied().unwrap_or(0.0) as u8,
            samples: n,
            status_share: counts.map(|c| c as f64 / n as f64),
            good_range_m: good,
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn flow_and_rangefinder_shares() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("OF", "QBffff", "TimeUS,Qual,flowX,flowY,bodyX,bodyY")
            .format("RFND", "QBfBBb", "TimeUS,Instance,Dist,Stat,Orient,Quality");
        b.rec("ARM", &[N(1e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        // Flow: good, quality 0, too fast (5 rad/s), good.
        for (t, q, fx) in [
            (2.0, 200.0, 0.1),
            (3.0, 0.0, 0.1),
            (4.0, 150.0, 5.0),
            (5.0, 250.0, 0.0),
        ] {
            b.rec("OF", &[N(t * 1e6), N(q), N(fx), N(0.0), N(0.0), N(0.0)]);
        }
        // Rangefinder 0 (down): good 0.5 m, good 3.2 m, out of range high.
        for (t, d, s) in [(2.0, 0.5, 4.0), (3.0, 3.2, 4.0), (4.0, 7.0, 3.0)] {
            b.rec("RFND", &[N(t * 1e6), N(0.0), N(d), N(s), N(25.0), N(-1.0)]);
        }
        let log = b.open();
        let f = flow(&log).unwrap();
        assert_eq!(f.samples, 4);
        assert_eq!((f.usable, f.zero_quality, f.too_fast), (0.5, 0.25, 0.25));
        assert_eq!((f.mean_quality, f.min_quality), (150.0, 0.0));
        let r = rangefinders(&log);
        assert_eq!(r.len(), 1);
        assert_eq!((r[0].orientation, r[0].samples), (25, 3));
        assert!((r[0].status_share[4] - 2.0 / 3.0).abs() < 1e-9);
        let (lo, hi) = r[0].good_range_m.unwrap();
        assert!((lo - 0.5).abs() < 1e-6 && (hi - 3.2).abs() < 1e-6, "{r:?}");
    }
}
