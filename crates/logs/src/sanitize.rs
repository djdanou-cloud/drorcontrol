//! Remove real-world positions from a dataflash log before it is shared or committed
//! (owner rule, ADR-0012 item 16: no field logs with real positions in the repository).
//!
//! Every latitude/longitude field (format code `L`: int32 × 1e-7 degrees) is moved by the
//! same offset so that the log's reference position (the first `POS`, else `GPS`, else any
//! non-zero position) lands on a stand-in location. Distances and shapes are kept (east-west
//! distances scale by the ratio of the cosines of the two latitudes, under 1 % for moves of a
//! few degrees), so every analysis still works. Zero positions (no fix yet) stay zero. A log
//! with no fix at all has nothing to move relative to, so its positions are zeroed instead.
//! Altitudes, times and everything else are untouched: MSL altitude and GPS time remain, which
//! the caller should know before sharing.

use crate::dataflash::{Kind, Log};

/// What sanitising did.
#[derive(Debug, Clone, PartialEq)]
pub struct SanitizeReport {
    /// The log's reference position before, (lat, lon) degrees; `None` when the log has no
    /// fix, in which case every position was zeroed instead of moved.
    pub reference: Option<(f64, f64)>,
    /// Where it was moved to.
    pub stand_in: (f64, f64),
    /// Position values rewritten.
    pub values: u64,
    /// Message.field names that were rewritten.
    pub fields: Vec<String>,
    /// `L` fields whose name says neither latitude nor longitude: zeroed, never left as they
    /// were (every `L` field is a position).
    pub zeroed: Vec<String>,
}

fn is_lat(name: &str) -> bool {
    name.starts_with("Lat") || name.ends_with("Lat") || name.ends_with("Lt") || name == "lat"
}

fn is_lon(name: &str) -> bool {
    name.starts_with("Lng")
        || name.starts_with("Lon")
        || name.ends_with("Lng")
        || name.ends_with("Lg")
        || name == "lon"
}

/// The log's reference position: first non-zero `POS`, else `GPS` (receiver 0).
fn reference(log: &Log) -> Option<(f64, f64)> {
    for (msg, inst) in [("POS", None), ("GPS", Some(("I", 0.0)))] {
        if let (Some(la), Some(lo)) = (log.column(msg, "Lat", inst), log.column(msg, "Lng", inst))
            && let Some(p) = la
                .values
                .iter()
                .zip(&lo.values)
                .find(|(a, b)| **a != 0.0 || **b != 0.0)
        {
            return Some((*p.0, *p.1));
        }
    }
    None
}

/// A copy of the log's bytes with every position moved to `stand_in` (zeroed when the log has
/// no `POS`/`GPS` fix to take as reference).
pub fn sanitize(log: &Log, stand_in: (f64, f64)) -> (Vec<u8>, SanitizeReport) {
    let reference = reference(log);
    let offset = reference.map(|r| (stand_in.0 - r.0, stand_in.1 - r.1));
    let mut out = log.bytes().to_vec();
    let base = log.bytes().as_ptr() as usize;
    let mut report = SanitizeReport {
        reference,
        stand_in,
        values: 0,
        fields: Vec::new(),
        zeroed: Vec::new(),
    };
    let counts = log.counts();
    for fmt in log.formats.values() {
        let n = counts.get(&fmt.name).copied().unwrap_or(0);
        for f in fmt
            .fields
            .iter()
            .filter(|f| f.kind == Kind::I32 && f.mul == Some(1e-7))
        {
            let shift = match offset {
                Some((dlat, _)) if is_lat(&f.name) => Some(dlat),
                Some((_, dlon)) if is_lon(&f.name) => Some(dlon),
                _ => None,
            };
            match shift {
                Some(_) => report.fields.push(format!("{}.{}", fmt.name, f.name)),
                None => report.zeroed.push(format!("{}.{}", fmt.name, f.name)),
            }
            for i in 0..n {
                let Some(rec) = log.record(&fmt.name, i) else {
                    continue;
                };
                let at = rec.as_ptr() as usize - base + f.offset;
                let raw = i32::from_le_bytes(out[at..at + 4].try_into().unwrap());
                if raw == 0 {
                    continue;
                }
                let Some(shift) = shift else {
                    out[at..at + 4].copy_from_slice(&0i32.to_le_bytes());
                    report.values += 1;
                    continue;
                };
                let deg = f64::from(raw) * 1e-7 + shift;
                let limit = if is_lat(&f.name) { 90.0 } else { 180.0 };
                let deg = if is_lat(&f.name) {
                    deg.clamp(-limit, limit)
                } else {
                    (deg + 180.0).rem_euclid(360.0) - 180.0
                };
                let new = (deg * 1e7).round() as i32;
                out[at..at + 4].copy_from_slice(&new.to_le_bytes());
                report.values += 1;
            }
        }
    }
    sanitize_params(log, &mut out, base, offset, &mut report);
    (out, report)
}

/// Parameter name that holds a position in degrees (e.g. `SIM_OPOS_LAT`).
fn param_axis(name: &str) -> Option<bool> {
    if name.ends_with("_LAT") {
        Some(true)
    } else if name.ends_with("_LNG") || name.ends_with("_LON") {
        Some(false)
    } else {
        None
    }
}

/// `PARM` rows whose name says latitude/longitude: `Value` and `Default` (float degrees) move
/// by the same offset as every other position, or are zeroed without one.
fn sanitize_params(
    log: &Log,
    out: &mut [u8],
    base: usize,
    offset: Option<(f64, f64)>,
    report: &mut SanitizeReport,
) {
    let Some(fmt) = log.format_by_name("PARM") else {
        return;
    };
    let Some(name) = fmt
        .fields
        .iter()
        .find(|f| f.name == "Name" && f.kind == Kind::Text)
    else {
        return;
    };
    let values: Vec<_> = fmt
        .fields
        .iter()
        .filter(|f| (f.name == "Value" || f.name == "Default") && f.kind == Kind::F32)
        .collect();
    let n = log.counts().get("PARM").copied().unwrap_or(0);
    let mut seen = Vec::new();
    for i in 0..n {
        let Some(rec) = log.record("PARM", i) else {
            continue;
        };
        let text = &rec[name.offset..name.offset + name.size];
        let text = String::from_utf8_lossy(text.split(|b| *b == 0).next().unwrap_or(&[]));
        let Some(is_lat) = param_axis(&text) else {
            continue;
        };
        let at0 = rec.as_ptr() as usize - base;
        for f in &values {
            let at = at0 + f.offset;
            let v = f32::from_le_bytes(out[at..at + 4].try_into().unwrap());
            if v == 0.0 || !v.is_finite() {
                continue;
            }
            let Some((dlat, dlon)) = offset else {
                out[at..at + 4].copy_from_slice(&0f32.to_le_bytes());
                report.values += 1;
                continue;
            };
            let deg = f64::from(v) + if is_lat { dlat } else { dlon };
            let deg = if is_lat {
                deg.clamp(-90.0, 90.0)
            } else {
                (deg + 180.0).rem_euclid(360.0) - 180.0
            };
            out[at..at + 4].copy_from_slice(&(deg as f32).to_le_bytes());
            report.values += 1;
        }
        let field = format!("PARM.{text}");
        if !seen.contains(&field) {
            seen.push(field);
        }
    }
    if offset.is_some() {
        report.fields.extend(seen);
    } else {
        report.zeroed.extend(seen);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N, V::T};

    #[test]
    fn positions_move_together_and_zeros_stay() {
        let mut b = LogBuilder::default();
        b.format("POS", "QLLf", "TimeUS,Lat,Lng,Alt")
            .format("CMD", "QHLL", "TimeUS,CNum,Lat,Lng")
            .format("TST", "QL", "TimeUS,Mystery")
            .format("PARM", "QNff", "TimeUS,Name,Value,Default");
        b.rec("POS", &[N(1e6), N(0.0), N(0.0), N(600.0)]); // no fix yet
        b.rec("POS", &[N(2e6), N(31.5), N(34.8), N(600.0)]);
        b.rec("POS", &[N(3e6), N(31.501), N(34.8), N(605.0)]);
        b.rec("CMD", &[N(3e6), N(1.0), N(31.502), N(34.801)]);
        b.rec("TST", &[N(3e6), N(12.0)]);
        b.rec("PARM", &[N(4e6), T("SIM_OPOS_LAT"), N(31.5), N(0.0)]);
        b.rec("PARM", &[N(4e6), T("SIM_OPOS_LNG"), N(34.8), N(0.0)]);
        b.rec("PARM", &[N(4e6), T("FENCE_RADIUS"), N(31.5), N(0.0)]);
        let log = b.open();
        let (bytes, r) = sanitize(&log, (-35.0, 149.0));
        assert_eq!(r.reference, Some((31.5, 34.8)));
        assert_eq!(
            r.values, 9,
            "2 POS rows and 1 CMD row, lat and lon each; 1 zeroed; 2 parameters"
        );
        assert_eq!(r.zeroed, ["TST.Mystery"]);
        let path = std::env::temp_dir().join(format!("dc-sanitized-{}.bin", std::process::id()));
        std::fs::write(&path, &bytes).unwrap();
        let s = Log::open(&path).unwrap();
        let lat = s.column("POS", "Lat", None).unwrap().values;
        assert_eq!(lat[0], 0.0, "no-fix zero stays zero");
        assert!(
            (lat[1] + 35.0).abs() < 1e-7 && (lat[2] + 34.999).abs() < 1e-7,
            "{lat:?}"
        );
        let cmd = (
            s.column("CMD", "Lat", None).unwrap().values[0],
            s.column("CMD", "Lng", None).unwrap().values[0],
        );
        assert!(
            (cmd.0 + 34.998).abs() < 1e-7 && (cmd.1 - 149.001).abs() < 1e-7,
            "{cmd:?}"
        );
        assert_eq!(s.column("POS", "Alt", None).unwrap().values[2], 605.0);
        assert_eq!(
            s.column("TST", "Mystery", None).unwrap().values[0],
            0.0,
            "zeroed"
        );
        let p = s.column("PARM", "Value", None).unwrap().values;
        assert!(
            (p[0] + 35.0).abs() < 1e-4 && (p[1] - 149.0).abs() < 1e-4,
            "{p:?}"
        );
        assert_eq!(p[2], 31.5, "not a position parameter");
        drop(s);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn without_a_fix_every_position_is_zeroed() {
        let mut b = LogBuilder::default();
        b.format("CMD", "QHLL", "TimeUS,CNum,Lat,Lng").format(
            "PARM",
            "QNff",
            "TimeUS,Name,Value,Default",
        );
        b.rec("CMD", &[N(1e6), N(1.0), N(31.502), N(34.801)]);
        b.rec("PARM", &[N(1e6), T("SIM_OPOS_LAT"), N(31.5), N(0.0)]);
        let log = b.open();
        let (bytes, r) = sanitize(&log, (-35.0, 149.0));
        assert_eq!(r.reference, None);
        assert_eq!(r.values, 3);
        assert_eq!(r.zeroed, ["CMD.Lat", "CMD.Lng", "PARM.SIM_OPOS_LAT"]);
        let path = std::env::temp_dir().join(format!("dc-sanitized0-{}.bin", std::process::id()));
        std::fs::write(&path, &bytes).unwrap();
        let s = Log::open(&path).unwrap();
        assert_eq!(s.column("CMD", "Lat", None).unwrap().values[0], 0.0);
        assert_eq!(s.column("PARM", "Value", None).unwrap().values[0], 0.0);
        drop(s);
        let _ = std::fs::remove_file(path);
    }
}
