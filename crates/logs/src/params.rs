//! Parameters recorded in a log (`PARM` records, written at boot and on every change).

use std::collections::BTreeMap;

use crate::dataflash::{Kind, Log};

impl Log {
    /// Parameter values from `PARM` records; a parameter logged more than once keeps its
    /// last value. Empty if the log has no `PARM` records.
    pub fn params(&self) -> BTreeMap<String, f64> {
        let mut out = BTreeMap::new();
        let Some(fmt) = self.format_by_name("PARM") else {
            return out;
        };
        let (Some(name), Some(value)) = (fmt.field("Name"), fmt.field("Value")) else {
            return out;
        };
        let nf = &fmt.fields[name];
        if nf.kind != Kind::Text {
            return out;
        }
        let n = self.counts().get("PARM").copied().unwrap_or(0);
        for i in 0..n {
            let Some(rec) = self.record("PARM", i) else {
                continue;
            };
            let raw = &rec[nf.offset..nf.offset + nf.size];
            let end = raw.iter().position(|&c| c == 0).unwrap_or(raw.len());
            if let (Ok(k), Some(v)) = (std::str::from_utf8(&raw[..end]), fmt.value(rec, value)) {
                out.insert(k.to_string(), v);
            }
        }
        out
    }
}
