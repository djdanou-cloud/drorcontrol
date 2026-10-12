//! Autopilot performance: main-loop rate and overruns, processor load, free memory and
//! internal errors.
//!
//! `PM` = TimeUS, LR (loop rate, Hz), NLon (long loops), NL (loops measured), MaxT (longest
//! loop, µs), Mem (free bytes), Load (‰: logged with multiplier 'A' = 0.1, so 508 is 50.8 %),
//! ErrL, InE (internal-error mask), ErC (internal-error count), … (`AP_Logger/LogStructure.h`,
//! 4.7.1). A loop is long when it takes over 1.2 × the loop period
//! (`AP::PerfInfo::set_loop_rate`). The first record after boot includes start-up loops.

use crate::dataflash::Log;

/// Performance over the log.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Perf {
    /// `PM` records.
    pub records: usize,
    /// Configured main-loop rate, Hz (latest).
    pub loop_rate_hz: f64,
    /// Long loops over all loops measured, % (the first record, at boot, left out when
    /// there are others).
    pub long_loop_pct: f64,
    /// Longest loop, ms, and when (µs since boot), boot record left out likewise.
    pub max_loop_ms: f64,
    /// See `max_loop_ms`.
    pub max_loop_at_us: u64,
    /// Highest processor load, %.
    pub max_load_pct: f64,
    /// Least free memory, bytes.
    pub min_free_mem: f64,
    /// Internal errors counted by the autopilot (latest `ErC`) and their mask (`InE`).
    pub internal_errors: u64,
    /// See `internal_errors`.
    pub internal_error_mask: u64,
}

/// Performance from `PM`; `None` without it.
pub fn perf(log: &Log) -> Option<Perf> {
    let col = |f: &str| log.column("PM", f, None);
    let (lr, nlon, nl, maxt) = (col("LR")?, col("NLon")?, col("NL")?, col("MaxT")?);
    let n = nl.values.len();
    if n == 0 {
        return None;
    }
    // Skip the boot record when there is more than one.
    let from = usize::from(n > 1);
    let (mut long, mut loops) = (0.0, 0.0);
    let (mut max_t, mut max_at) = (0.0f64, 0u64);
    for i in from..n {
        long += nlon.values[i];
        loops += nl.values[i];
        if maxt.values[i] > max_t {
            (max_t, max_at) = (maxt.values[i], maxt.time_us[i]);
        }
    }
    let last = |c: Option<crate::dataflash::Column>| {
        c.and_then(|c| c.values.last().copied()).unwrap_or(0.0)
    };
    Some(Perf {
        records: n,
        loop_rate_hz: lr.values[n - 1],
        long_loop_pct: if loops > 0.0 {
            long * 100.0 / loops
        } else {
            0.0
        },
        max_loop_ms: max_t / 1000.0,
        max_loop_at_us: max_at,
        max_load_pct: col("Load")
            .map(|c| c.values.iter().copied().fold(0.0, f64::max) / 10.0)
            .unwrap_or(0.0),
        min_free_mem: col("Mem")
            .map(|c| c.values.iter().copied().fold(f64::MAX, f64::min))
            .unwrap_or(0.0),
        internal_errors: last(col("ErC")) as u64,
        internal_error_mask: last(col("InE")) as u64,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn long_loops_load_and_errors() {
        let mut b = LogBuilder::default();
        b.format(
            "PM",
            "QHHHIIHHIIIIIIQ",
            "TimeUS,LR,NLon,NL,MaxT,Mem,Load,ErrL,InE,ErC,SPIC,I2CC,I2CI,Ex,R",
        );
        let pm = |b: &mut LogBuilder, t: f64, nlon: f64, maxt: f64, mem: f64, load: f64| {
            let mut v = vec![N(t), N(400.0), N(nlon), N(4000.0), N(maxt), N(mem), N(load)];
            v.extend([
                N(0.0),
                N(0.0),
                N(0.0),
                N(0.0),
                N(0.0),
                N(0.0),
                N(0.0),
                N(0.0),
            ]);
            b.rec("PM", &v);
        };
        pm(&mut b, 1e6, 300.0, 90_000.0, 60_000.0, 900.0); // boot: left out of loop stats
        pm(&mut b, 11e6, 40.0, 3_500.0, 52_000.0, 455.0);
        pm(&mut b, 21e6, 0.0, 2_600.0, 51_000.0, 470.0);
        let p = perf(&b.open()).unwrap();
        assert_eq!((p.records, p.loop_rate_hz), (3, 400.0));
        assert!((p.long_loop_pct - 0.5).abs() < 1e-9, "{p:?}"); // 40 of 8000
        assert_eq!((p.max_loop_ms, p.max_loop_at_us), (3.5, 11_000_000));
        assert_eq!((p.max_load_pct, p.min_free_mem), (90.0, 51_000.0));
    }
}
