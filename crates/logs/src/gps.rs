//! GPS quality over the flight (first receiver): satellites, HDOP, fix types, and the EKF's
//! GPS glitches.
//!
//! Sources (ArduPilot 4.7.1): `GPS` = TimeUS, I, Status, GMS, GWk, NSats, HDop, … with Status
//! `AP_GPS::GPS_Status` (0 no GPS, 1 no fix, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fixed,
//! 7 static, 8 PPP) (`AP_GPS/LogStructure.h`, `AP_GPS.h`); `ERR` subsystem 11 `GPS`, code 2
//! `GPS_GLITCH`, 0 resolved (`AP_Logger.h`, `ArduCopter/events.cpp`).

use crate::dataflash::Log;
use crate::flight::{armed_periods, in_periods};

/// GPS while armed.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GpsQuality {
    /// Samples while armed.
    pub samples: usize,
    /// Fewest and mean satellites.
    pub min_sats: f64,
    /// See `min_sats`.
    pub mean_sats: f64,
    /// Highest and mean HDOP.
    pub max_hdop: f64,
    /// See `max_hdop`.
    pub mean_hdop: f64,
    /// Share of samples per fix status 0..=8, 0..1.
    pub status_share: [f64; 9],
    /// Glitches over the whole log: start, and when resolved (µs since boot).
    pub glitches: Vec<(u64, Option<u64>)>,
}

impl GpsQuality {
    /// Share of samples with at least a 3D fix (status 3 or better), 0..1.
    pub fn fix_3d(&self) -> f64 {
        self.status_share[3..].iter().sum()
    }
}

/// Status names for display.
pub const STATUS_NAMES: [&str; 9] = [
    "no GPS",
    "no fix",
    "2D",
    "3D",
    "DGPS",
    "RTK float",
    "RTK fixed",
    "static",
    "PPP",
];

/// GPS quality while armed; `None` without `GPS` samples then.
pub fn quality(log: &Log) -> Option<GpsQuality> {
    let inst = Some(("I", 0.0));
    let (st, ns, hd) = (
        log.column("GPS", "Status", inst)?,
        log.column("GPS", "NSats", inst)?,
        log.column("GPS", "HDop", inst)?,
    );
    let armed = armed_periods(log);
    let idx: Vec<usize> = (0..st.values.len())
        .filter(|&i| in_periods(&armed, st.time_us[i]))
        .collect();
    if idx.is_empty() {
        return None;
    }
    let n = idx.len() as f64;
    let mut counts = [0usize; 9];
    for &i in &idx {
        if let Some(c) = counts.get_mut(st.values[i] as usize) {
            *c += 1;
        }
    }
    let mut q = GpsQuality {
        samples: idx.len(),
        min_sats: idx.iter().map(|&i| ns.values[i]).fold(f64::MAX, f64::min),
        mean_sats: idx.iter().map(|&i| ns.values[i]).sum::<f64>() / n,
        max_hdop: idx.iter().map(|&i| hd.values[i]).fold(f64::MIN, f64::max),
        mean_hdop: idx.iter().map(|&i| hd.values[i]).sum::<f64>() / n,
        status_share: counts.map(|c| c as f64 / n),
        glitches: Vec::new(),
    };
    if let (Some(s), Some(c)) = (
        log.column("ERR", "Subsys", None),
        log.column("ERR", "ECode", None),
    ) {
        for ((t, sub), code) in s.time_us.iter().zip(&s.values).zip(&c.values) {
            match (*sub as u8, *code as u8) {
                (11, 2) if q.glitches.last().is_none_or(|g| g.1.is_some()) => {
                    q.glitches.push((*t, None));
                }
                (11, 0) => {
                    if let Some(g) = q.glitches.last_mut().filter(|g| g.1.is_none()) {
                        g.1 = Some(*t);
                    }
                }
                _ => {}
            }
        }
    }
    Some(q)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn satellites_hdop_fix_and_glitches() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("GPS", "QBBBc", "TimeUS,I,Status,NSats,HDop")
            .format("ERR", "QBB", "TimeUS,Subsys,ECode");
        b.rec("GPS", &[N(0.5e6), N(0.0), N(1.0), N(3.0), N(9.9)]); // before arming
        b.rec("ARM", &[N(1e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        for (t, status, sats, hdop) in [
            (2.0, 3.0, 14.0, 0.8),
            (3.0, 6.0, 18.0, 0.6),
            (4.0, 2.0, 7.0, 2.5),
        ] {
            b.rec("GPS", &[N(t * 1e6), N(0.0), N(status), N(sats), N(hdop)]);
        }
        b.rec("GPS", &[N(4e6), N(1.0), N(0.0), N(0.0), N(0.0)]); // second receiver: ignored
        b.rec("ERR", &[N(3.5e6), N(11.0), N(2.0)]);
        b.rec("ERR", &[N(3.9e6), N(11.0), N(0.0)]);
        let q = quality(&b.open()).unwrap();
        assert_eq!(q.samples, 3);
        assert_eq!((q.min_sats, q.mean_sats), (7.0, 13.0));
        assert!(
            (q.max_hdop - 2.5).abs() < 1e-9 && (q.mean_hdop - 1.3).abs() < 1e-9,
            "{q:?}"
        );
        assert!((q.fix_3d() - 2.0 / 3.0).abs() < 1e-9);
        assert_eq!(q.glitches, [(3_500_000, Some(3_900_000))]);
    }
}
