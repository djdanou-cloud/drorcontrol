//! Export from a dataflash log: one message type as CSV, and the recorded parameters as a
//! Mission Planner-compatible `.param` file.

use std::fmt::Write as _;

use crate::dataflash::{Kind, Log};

/// Every record of `msg` as CSV: a header of field names, then one row per record. Text
/// fields are quoted; array fields (no single value) are left out. `None` if the log has
/// no such message.
pub fn csv(log: &Log, msg: &str) -> Option<String> {
    let fmt = log.format_by_name(msg)?;
    let cols: Vec<usize> = (0..fmt.fields.len())
        .filter(|&i| fmt.fields[i].kind != Kind::Array)
        .collect();
    let mut s = cols
        .iter()
        .map(|&i| fmt.fields[i].name.as_str())
        .collect::<Vec<_>>()
        .join(",");
    s.push('\n');
    let n = log.counts().get(msg).copied().unwrap_or(0);
    for r in 0..n {
        let Some(rec) = log.record(msg, r) else {
            continue;
        };
        for (k, &i) in cols.iter().enumerate() {
            if k > 0 {
                s.push(',');
            }
            let f = &fmt.fields[i];
            if f.kind == Kind::Text {
                let raw = &rec[f.offset..f.offset + f.size];
                let end = raw.iter().position(|&c| c == 0).unwrap_or(raw.len());
                let text = String::from_utf8_lossy(&raw[..end]).replace('"', "\"\"");
                let _ = write!(s, "\"{text}\"");
            } else if let Some(v) = fmt.value(rec, i) {
                let _ = write!(s, "{v}");
            }
        }
        s.push('\n');
    }
    Some(s)
}

/// The parameters recorded in the log (`PARM`, last value per name) as `NAME,value` lines,
/// sorted by name. Values are logged as 32-bit floats and written in their shortest exact
/// form. Empty if the log has no `PARM` records.
pub fn param_file(log: &Log) -> String {
    log.params()
        .iter()
        .map(|(k, v)| format!("{k},{}\n", *v as f32))
        .collect()
}

#[cfg(test)]
mod tests {
    #[test]
    fn floats_are_written_in_shortest_form() {
        assert_eq!(format!("{}", 0.8_f64 as f32), "0.8");
        assert_eq!(format!("{}", 3.0_f64), "3");
    }
}
