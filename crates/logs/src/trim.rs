//! A smaller copy of a log: only the records inside a time window, plus what every reader
//! needs to make sense of them (format, unit and parameter records, the boot messages, and
//! the flight mode in force when the window opens). Records keep their order and bytes, so
//! the copy is an ordinary dataflash log.

use crate::dataflash::{FMT_LEN, FMT_TYPE, Log};
use crate::flight::armed_periods;

/// Message types copied whatever their time: the formats, units and multipliers that
/// describe every record, the parameters, the firmware version, and the text messages
/// (which carry the boot banner).
const ALWAYS: [&str; 7] = ["FMTU", "UNIT", "MULT", "PARM", "VER", "MSG", "FILE"];

/// What trimming did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrimReport {
    /// Records in the input and the copy.
    pub records_in: u64,
    /// See `records_in`.
    pub records_out: u64,
    /// Bytes in the copy.
    pub bytes_out: u64,
}

/// The window from `margin_us` before the first arming to `margin_us` after the last
/// disarm (the end of the log if still armed); `None` if the log never armed.
pub fn armed_window(log: &Log, margin_us: u64) -> Option<(u64, u64)> {
    let armed = armed_periods(log);
    let start = armed.first()?.start_us.saturating_sub(margin_us);
    let end = match armed.last()?.end_us {
        Some(e) => e.saturating_add(margin_us),
        None => u64::MAX,
    };
    Some((start, end))
}

/// The records with `TimeUS` in `[t0, t1]`, plus [`ALWAYS`] types, records without a
/// `TimeUS`, and the last `MODE` before `t0`.
pub fn trim(log: &Log, t0: u64, t1: u64) -> (Vec<u8>, TrimReport) {
    let data = log.bytes();
    let mut keep: Vec<(usize, usize)> = Vec::new();
    let mut records_in = 0;
    for (t, ti) in &log.index {
        let (len, fmt) = if *t == FMT_TYPE {
            (FMT_LEN, None)
        } else {
            match log.formats.get(t) {
                Some(f) => (f.len, Some(f)),
                None => continue,
            }
        };
        let time = fmt.and_then(|f| f.field("TimeUS").map(|i| (f, i)));
        let always = fmt.is_none_or(|f| ALWAYS.contains(&f.name.as_str()));
        let is_mode = fmt.is_some_and(|f| f.name == "MODE");
        let mut last_before: Option<(u64, usize)> = None;
        for (base, rel) in ti.block_slices() {
            for &r in rel {
                records_in += 1;
                let at = base + r as usize;
                let Some(rec) = data.get(at..at + len) else {
                    continue;
                };
                let ts = time.and_then(|(f, i)| f.u64_value(rec, i));
                match ts {
                    _ if always => keep.push((at, len)),
                    None => keep.push((at, len)),
                    Some(ts) if ts >= t0 && ts <= t1 => keep.push((at, len)),
                    Some(ts) if is_mode && ts < t0 => {
                        if last_before.is_none_or(|(p, _)| ts >= p) {
                            last_before = Some((ts, at));
                        }
                    }
                    Some(_) => {}
                }
            }
        }
        if let Some((_, at)) = last_before {
            keep.push((at, len));
        }
    }
    keep.sort_unstable();
    let mut out = Vec::with_capacity(keep.iter().map(|k| k.1).sum());
    for (at, len) in &keep {
        out.extend_from_slice(&data[*at..*at + *len]);
    }
    let report = TrimReport {
        records_in,
        records_out: keep.len() as u64,
        bytes_out: out.len() as u64,
    };
    (out, report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn keeps_the_window_and_what_explains_it() {
        let mut b = LogBuilder::default();
        b.format("PARM", "Qf", "TimeUS,Value")
            .format("MODE", "QB", "TimeUS,Mode")
            .format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("ATT", "Qf", "TimeUS,Roll");
        b.rec("PARM", &[N(0.1e6), N(1.0)]);
        b.rec("MODE", &[N(0.5e6), N(0.0)]);
        b.rec("MODE", &[N(1e6), N(5.0)]); // in force when the window opens
        for t in 1..=20 {
            b.rec("ATT", &[N(f64::from(t) * 1e6), N(f64::from(t))]);
        }
        b.rec("ARM", &[N(8e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        b.rec("ARM", &[N(12e6), N(0.0), N(0.0), N(0.0), N(0.0)]);
        let log = b.open();
        let (t0, t1) = armed_window(&log, 2_000_000).unwrap();
        assert_eq!((t0, t1), (6_000_000, 14_000_000));
        let (bytes, r) = trim(&log, t0, t1);
        let path = std::env::temp_dir().join(format!("dc-trim-{}.bin", std::process::id()));
        std::fs::write(&path, &bytes).unwrap();
        let s = Log::open(&path).unwrap();
        let att = s.column("ATT", "Roll", None).unwrap().values;
        assert_eq!(att, (6..=14).map(f64::from).collect::<Vec<_>>());
        assert_eq!(s.column("MODE", "Mode", None).unwrap().values, [5.0]);
        assert_eq!(s.column("PARM", "Value", None).unwrap().values, [1.0]);
        assert_eq!(s.column("ARM", "ArmState", None).unwrap().values.len(), 2);
        assert!(r.records_out < r.records_in);
        assert_eq!(r.bytes_out, bytes.len() as u64);
        drop(s);
        let _ = std::fs::remove_file(path);
    }
}
