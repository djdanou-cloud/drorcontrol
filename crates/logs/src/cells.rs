//! Battery cell balance while flying, for packs whose monitor reports cell voltages.
//!
//! `BCL` = TimeUS, Instance, Volt, V1..V12: cell voltages in mV plus 1 (0 = no cell;
//! `AP_BattMonitor_Backend::Log_Write_BCL`, `AP_BattMonitor/LogStructure.h`, 4.7.1).

use crate::dataflash::Log;
use crate::flight::{flying_periods, in_periods};

/// One pack's cells while flying.
#[derive(Debug, Clone, PartialEq)]
pub struct CellBalance {
    /// Battery instance.
    pub instance: u8,
    /// Cells reported.
    pub cells: usize,
    /// Samples.
    pub samples: usize,
    /// Largest gap between the highest and lowest cell, V, and when (µs since boot).
    pub max_spread_v: f64,
    /// See `max_spread_v`.
    pub max_spread_at_us: u64,
    /// Mean gap, V.
    pub mean_spread_v: f64,
    /// Lowest cell voltage seen, V, and which cell (1-based).
    pub lowest_v: f64,
    /// See `lowest_v`.
    pub lowest_cell: usize,
}

/// Cell balance per battery while flying.
pub fn cell_balance(log: &Log) -> Vec<CellBalance> {
    let (flying, _) = flying_periods(log);
    let Some(inst) = log.column("BCL", "Instance", None) else {
        return Vec::new();
    };
    let mut ids: Vec<u8> = inst.values.iter().map(|v| *v as u8).collect();
    ids.sort_unstable();
    ids.dedup();
    let mut out = Vec::new();
    for id in ids {
        let sel = Some(("Instance", f64::from(id)));
        let cols: Vec<crate::dataflash::Column> = (1..=12)
            .filter_map(|k| log.column("BCL", &format!("V{k}"), sel))
            .collect();
        let Some(first) = cols.first() else { continue };
        let mut b = CellBalance {
            instance: id,
            cells: 0,
            samples: 0,
            max_spread_v: 0.0,
            max_spread_at_us: 0,
            mean_spread_v: 0.0,
            lowest_v: f64::MAX,
            lowest_cell: 0,
        };
        let mut spread_sum = 0.0;
        for (i, t) in first.time_us.iter().enumerate() {
            if !in_periods(&flying, *t) {
                continue;
            }
            // Logged in mV + 1 (0 = no cell); the format's multiplier is not applied by the
            // reader, so convert here.
            let cells: Vec<(usize, f64)> = cols
                .iter()
                .enumerate()
                .filter_map(|(k, c)| {
                    let raw = *c.values.get(i)?;
                    (raw > 0.0).then(|| (k + 1, (raw - 1.0) / 1000.0))
                })
                .collect();
            if cells.len() < 2 {
                continue;
            }
            b.cells = b.cells.max(cells.len());
            b.samples += 1;
            let lo = cells
                .iter()
                .copied()
                .min_by(|x, y| x.1.total_cmp(&y.1))
                .unwrap();
            let hi = cells.iter().map(|c| c.1).fold(f64::MIN, f64::max);
            let spread = hi - lo.1;
            spread_sum += spread;
            if spread > b.max_spread_v {
                (b.max_spread_v, b.max_spread_at_us) = (spread, *t);
            }
            if lo.1 < b.lowest_v {
                (b.lowest_v, b.lowest_cell) = (lo.1, lo.0);
            }
        }
        if b.samples > 0 {
            b.mean_spread_v = spread_sum / b.samples as f64;
            out.push(b);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn spread_and_weakest_cell_while_flying() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("BCL", "QBfHHHH", "TimeUS,Instance,Volt,V1,V2,V3,V4");
        b.rec("ARM", &[N(1e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        // A 3S pack (V4 = 0: no cell). mV + 1 as logged.
        for (t, c) in [
            (2.0, [4101.0, 4091.0, 4081.0]),
            (3.0, [3801.0, 3751.0, 3701.0]),
        ] {
            b.rec(
                "BCL",
                &[
                    N(t * 1e6),
                    N(0.0),
                    N(0.0),
                    N(c[0]),
                    N(c[1]),
                    N(c[2]),
                    N(0.0),
                ],
            );
        }
        let r = cell_balance(&b.open());
        assert_eq!(r.len(), 1);
        let p = &r[0];
        assert_eq!((p.cells, p.samples, p.lowest_cell), (3, 2, 3));
        assert!(
            (p.max_spread_v - 0.1).abs() < 1e-9 && p.max_spread_at_us == 3_000_000,
            "{p:?}"
        );
        assert!((p.mean_spread_v - 0.06).abs() < 1e-9 && (p.lowest_v - 3.7).abs() < 1e-9);
    }
}
