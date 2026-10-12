//! RC link: receiver signal while armed, radio failsafes and late frames.
//!
//! Sources (ArduPilot 4.7.1):
//! * `RSSI` = TimeUS, RXRSSI (0..1), RXLQ (link quality, %; -1 when the receiver does not
//!   report it: `AP_RSSI::read_receiver_link_quality`) (`AP_Logger/LogStructure.h`).
//! * `ERR` subsystem 5 `FAILSAFE_RADIO`: code 1 occurred, 0 resolved; subsystem 2 `RADIO`:
//!   code 2 `RADIO_LATE_FRAME` (`AP_Logger.h`).

use crate::dataflash::Log;
use crate::flight::{armed_periods, in_periods};

/// The RC link over a log.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct RcLink {
    /// `RSSI` samples while armed with a value.
    pub rssi_samples: usize,
    /// Mean and lowest receiver RSSI while armed, % (`None` if not reported).
    pub rssi_pct: Option<(f64, f64)>,
    /// Mean and lowest link quality while armed, % (`None` if not reported).
    pub lq_pct: Option<(f64, f64)>,
    /// Radio failsafes: start and end, µs since boot (`None`: still in failsafe at the end).
    pub failsafes: Vec<(u64, Option<u64>)>,
    /// Late-frame errors logged.
    pub late_frames: usize,
}

fn mean_min(v: &[f64]) -> Option<(f64, f64)> {
    (!v.is_empty()).then(|| {
        (
            v.iter().sum::<f64>() / v.len() as f64,
            v.iter().copied().fold(f64::MAX, f64::min),
        )
    })
}

/// The RC link: signal while armed, failsafes and late frames over the whole log.
pub fn rc_link(log: &Log) -> RcLink {
    let armed = armed_periods(log);
    let mut out = RcLink::default();
    if let (Some(r), Some(q)) = (
        log.column("RSSI", "RXRSSI", None),
        log.column("RSSI", "RXLQ", None),
    ) {
        let (mut rs, mut lq) = (Vec::new(), Vec::new());
        for (i, t) in r.time_us.iter().enumerate() {
            if !in_periods(&armed, *t) {
                continue;
            }
            if r.values[i] >= 0.0 {
                rs.push(r.values[i] * 100.0);
            }
            if q.values.get(i).is_some_and(|v| *v >= 0.0) {
                lq.push(q.values[i]);
            }
        }
        out.rssi_samples = rs.len().max(lq.len());
        out.rssi_pct = mean_min(&rs);
        out.lq_pct = mean_min(&lq);
    }
    if let (Some(s), Some(c)) = (
        log.column("ERR", "Subsys", None),
        log.column("ERR", "ECode", None),
    ) {
        for ((t, sub), code) in s.time_us.iter().zip(&s.values).zip(&c.values) {
            match (*sub as u8, *code as u8) {
                (5, 1) => {
                    if out.failsafes.last().is_none_or(|f| f.1.is_some()) {
                        out.failsafes.push((*t, None));
                    }
                }
                (5, 0) => {
                    if let Some(f) = out.failsafes.last_mut().filter(|f| f.1.is_none()) {
                        f.1 = Some(*t);
                    }
                }
                (2, 2) => out.late_frames += 1,
                _ => {}
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn signal_while_armed_and_failsafes() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("RSSI", "Qff", "TimeUS,RXRSSI,RXLQ")
            .format("ERR", "QBB", "TimeUS,Subsys,ECode");
        b.rec("RSSI", &[N(0.5e6), N(0.1), N(10.0)]); // before arming: ignored
        b.rec("ARM", &[N(1e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        b.rec("RSSI", &[N(2e6), N(0.9), N(100.0)]);
        b.rec("RSSI", &[N(3e6), N(0.5), N(60.0)]);
        b.rec("ERR", &[N(4e6), N(5.0), N(1.0)]); // failsafe
        b.rec("ERR", &[N(4.5e6), N(2.0), N(2.0)]); // late frame
        b.rec("ERR", &[N(6e6), N(5.0), N(0.0)]); // resolved
        b.rec("ERR", &[N(8e6), N(5.0), N(1.0)]); // again, never resolved
        let l = rc_link(&b.open());
        assert_eq!(l.rssi_samples, 2);
        let (m, lo) = l.rssi_pct.unwrap();
        assert!((m - 70.0).abs() < 1e-4 && (lo - 50.0).abs() < 1e-4, "{l:?}");
        assert_eq!(l.lq_pct, Some((80.0, 60.0)));
        assert_eq!(
            l.failsafes,
            [(4_000_000, Some(6_000_000)), (8_000_000, None)]
        );
        assert_eq!(l.late_frames, 1);
    }

    #[test]
    fn link_quality_not_reported() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("RSSI", "Qff", "TimeUS,RXRSSI,RXLQ");
        b.rec("ARM", &[N(1e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        b.rec("RSSI", &[N(2e6), N(0.8), N(-1.0)]);
        let l = rc_link(&b.open());
        assert!(l.rssi_pct.is_some() && l.lq_pct.is_none());
    }
}
