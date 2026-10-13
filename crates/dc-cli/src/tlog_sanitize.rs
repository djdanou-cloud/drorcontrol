//! Remove real-world positions from a `.tlog` before it is shared or committed (owner rule,
//! ADR-0012: no field logs with real positions in the repository). The `.bin` counterpart is
//! `logs::sanitize`.
//!
//! Every position moves by one offset so the recording's first fix (`GLOBAL_POSITION_INT`,
//! else `GPS_RAW_INT`) lands on a stand-in location; shapes and distances are kept. Fields
//! are found from the dialect's layout, so every message is covered:
//! - `int32_t` fields with units `degE7` (named lat/lon in some form; any other is zeroed);
//! - `float` fields `lat`/`lon`/`lng` with units `deg` (`SIM_STATE`, `FENCE_POINT`);
//! - `x`/`y` of `MISSION_ITEM_INT`, `COMMAND_INT` (degE7) and `MISSION_ITEM` (deg) in a
//!   global frame;
//! - `param5`/`param6` of `COMMAND_LONG` for commands whose params 5/6 are latitude and
//!   longitude (listed in [`POSITION_COMMANDS`]);
//! - `PARAM_VALUE`/`PARAM_SET` of parameters named `*_LAT`/`*_LNG`/`*_LON`.
//!
//! Zero values (no fix) stay zero. A recording without a fix has its positions zeroed.
//! Changed frames get a fresh checksum; a signed frame that changes loses its signature
//! (it cannot be re-signed without the key), so the output reports how many. Altitudes and
//! times are not changed.

use std::collections::BTreeSet;
use std::io::{self, Read};

use mavlink_codec::crc::frame_crc;
use mavlink_codec::{Frame, message_fields, message_info};
use recorder::TlogReader;

/// `MAV_FRAME`s whose x/y are latitude/longitude (common.xml: GLOBAL, GLOBAL_RELATIVE_ALT,
/// GLOBAL_INT, GLOBAL_RELATIVE_ALT_INT, GLOBAL_TERRAIN_ALT, GLOBAL_TERRAIN_ALT_INT).
const GLOBAL_FRAMES: [u8; 6] = [0, 3, 5, 6, 10, 11];

/// `MAV_CMD`s whose param5/param6 are latitude/longitude (common.xml): NAV_WAYPOINT,
/// NAV_LOITER_UNLIM, NAV_LOITER_TURNS, NAV_LOITER_TIME, NAV_LAND, NAV_TAKEOFF,
/// NAV_LOITER_TO_ALT, NAV_SPLINE_WAYPOINT, NAV_VTOL_TAKEOFF, NAV_VTOL_LAND, DO_SET_HOME,
/// DO_REPOSITION, DO_SET_ROI_LOCATION, DO_SET_ROI, NAV_PAYLOAD_PLACE.
pub const POSITION_COMMANDS: [u16; 15] = [
    16, 17, 18, 19, 21, 22, 31, 82, 84, 85, 179, 192, 195, 201, 94,
];

const GLOBAL_POSITION_INT: u32 = 33;
const GPS_RAW_INT: u32 = 24;
const PARAM_VALUE: u32 = 22;
const PARAM_SET: u32 = 23;
const MISSION_ITEM: u32 = 39;
const MISSION_ITEM_INT: u32 = 73;
const COMMAND_INT: u32 = 75;
const COMMAND_LONG: u32 = 76;

/// What sanitising did.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TlogSanitizeReport {
    /// The first fix, (lat, lon) degrees; `None` if there was none (positions zeroed).
    pub reference: Option<(f64, f64)>,
    /// Frames read and frames changed.
    pub frames: u64,
    /// See `frames`.
    pub changed: u64,
    /// Position values rewritten.
    pub values: u64,
    /// Changed frames that were signed and are now unsigned.
    pub unsigned: u64,
    /// `MESSAGE.field` names rewritten.
    pub fields: BTreeSet<String>,
    /// Frames of message IDs outside the dialect, copied unchanged (they cannot be read).
    pub unknown: u64,
}

#[derive(Clone, Copy, PartialEq)]
enum Axis {
    Lat,
    Lon,
    /// A position field that does not say which.
    Unknown,
}

fn axis(name: &str) -> Axis {
    let n = name.to_ascii_lowercase();
    if n.contains("lat") {
        Axis::Lat
    } else if n.contains("lon") || n.contains("lng") {
        Axis::Lon
    } else {
        Axis::Unknown
    }
}

#[derive(Clone, Copy)]
enum Ty {
    E7,
    Deg32,
}

fn offset_of(id: u32, name: &str) -> Option<usize> {
    message_fields(id)
        .iter()
        .find(|f| f.name == name)
        .map(|f| f.offset)
}

fn get<const N: usize>(p: &[u8], at: usize) -> [u8; N] {
    let mut b = [0u8; N];
    for (i, v) in b.iter_mut().enumerate() {
        *v = p.get(at + i).copied().unwrap_or(0);
    }
    b
}

/// The position fields of one payload: (offset, type, axis, name).
fn targets(id: u32, p: &[u8]) -> Vec<(usize, Ty, Axis, String)> {
    let mut out = Vec::new();
    for f in message_fields(id) {
        if f.array_len != 0 {
            continue;
        }
        let ty = match (f.ty, f.units) {
            ("int32_t", Some("degE7")) => Ty::E7,
            ("float", Some("deg")) if matches!(f.name, "lat" | "lon" | "lng") => Ty::Deg32,
            _ => continue,
        };
        out.push((f.offset, ty, axis(f.name), f.name.to_string()));
    }
    let frame_is_global = |name: &str| {
        offset_of(id, name).is_some_and(|o| GLOBAL_FRAMES.contains(&get::<1>(p, o)[0]))
    };
    let xy = |ty: Ty, out: &mut Vec<_>| {
        for (n, a) in [("x", Axis::Lat), ("y", Axis::Lon)] {
            if let Some(o) = offset_of(id, n) {
                out.push((o, ty, a, n.to_string()));
            }
        }
    };
    match id {
        MISSION_ITEM_INT | COMMAND_INT if frame_is_global("frame") => xy(Ty::E7, &mut out),
        MISSION_ITEM if frame_is_global("frame") => xy(Ty::Deg32, &mut out),
        COMMAND_LONG => {
            let cmd = offset_of(id, "command").map(|o| u16::from_le_bytes(get(p, o)));
            if cmd.is_some_and(|c| POSITION_COMMANDS.contains(&c)) {
                for (n, a) in [("param5", Axis::Lat), ("param6", Axis::Lon)] {
                    if let Some(o) = offset_of(id, n) {
                        out.push((o, Ty::Deg32, a, n.to_string()));
                    }
                }
            }
        }
        PARAM_VALUE | PARAM_SET => {
            if let (Some(n), Some(v)) = (offset_of(id, "param_id"), offset_of(id, "param_value")) {
                let raw: [u8; 16] = get(p, n);
                let end = raw.iter().position(|b| *b == 0).unwrap_or(16);
                let name = String::from_utf8_lossy(&raw[..end]).to_string();
                let a = if name.ends_with("_LAT") {
                    Some(Axis::Lat)
                } else if name.ends_with("_LNG") || name.ends_with("_LON") {
                    Some(Axis::Lon)
                } else {
                    None
                };
                if let Some(a) = a {
                    out.push((v, Ty::Deg32, a, name));
                }
            }
        }
        _ => {}
    }
    out
}

/// The first non-zero fix in the recording.
fn reference(frames: &[(u64, Frame)]) -> Option<(f64, f64)> {
    for id in [GLOBAL_POSITION_INT, GPS_RAW_INT] {
        let (lat, lon) = (offset_of(id, "lat")?, offset_of(id, "lon")?);
        for (_, f) in frames.iter().filter(|(_, f)| f.msgid() == id) {
            let p = f.payload();
            let (a, b) = (
                i32::from_le_bytes(get(p, lat)),
                i32::from_le_bytes(get(p, lon)),
            );
            if a != 0 || b != 0 {
                return Some((f64::from(a) * 1e-7, f64::from(b) * 1e-7));
            }
        }
    }
    None
}

fn wrap(axis: Axis, deg: f64) -> f64 {
    match axis {
        Axis::Lat => deg.clamp(-90.0, 90.0),
        _ => (deg + 180.0).rem_euclid(360.0) - 180.0,
    }
}

/// Rewrite one frame; `None` if nothing in it is a non-zero position.
fn rewrite(f: &Frame, offset: Option<(f64, f64)>, r: &mut TlogSanitizeReport) -> Option<Vec<u8>> {
    let id = f.msgid();
    let info = message_info(id)?;
    let bytes = f.as_bytes();
    let h = bytes.len() - f.payload().len() - 2 - f.signature_block().map_or(0, <[u8]>::len);
    let mut payload = f.payload().to_vec();
    let mut touched = false;
    for (at, ty, axis, name) in targets(id, f.payload()) {
        let size = 4;
        if at + size > payload.len() {
            continue; // truncated v2 payload: the value is zero
        }
        let shift = match (offset, axis) {
            (Some((dlat, _)), Axis::Lat) => Some(dlat),
            (Some((_, dlon)), Axis::Lon) => Some(dlon),
            _ => None,
        };
        let slot = &mut payload[at..at + size];
        match ty {
            Ty::E7 => {
                let v = i32::from_le_bytes(get(slot, 0));
                if v == 0 {
                    continue;
                }
                let new = shift.map_or(0, |s| {
                    (wrap(axis, f64::from(v) * 1e-7 + s) * 1e7).round() as i32
                });
                slot.copy_from_slice(&new.to_le_bytes());
            }
            Ty::Deg32 => {
                let v = f32::from_le_bytes(get(slot, 0));
                if v == 0.0 || !v.is_finite() {
                    continue;
                }
                let new = shift.map_or(0.0, |s| wrap(axis, f64::from(v) + s) as f32);
                slot.copy_from_slice(&new.to_le_bytes());
            }
        }
        r.values += 1;
        touched = true;
        r.fields.insert(format!("{}.{name}", info.name));
    }
    if !touched {
        return None;
    }
    let mut out = bytes[..h].to_vec();
    out.extend_from_slice(&payload);
    if f.is_signed() {
        out[2] &= !0x01; // incompat flag: signed
        r.unsigned += 1;
    }
    let crc = frame_crc(&out[1..], info.crc_extra);
    out.extend_from_slice(&crc.to_le_bytes());
    Some(out)
}

/// A sanitised copy of a `.tlog`. A bad record ends the read; what was read before it is
/// written (the error is returned with the report so the caller can say so).
pub fn sanitize(
    input: impl Read,
    stand_in: (f64, f64),
) -> (Vec<u8>, TlogSanitizeReport, Option<io::Error>) {
    let mut reader = TlogReader::new(input);
    let mut frames = Vec::new();
    let error = loop {
        match reader.next_record() {
            Ok(Some(rec)) => frames.push((rec.unix_us, rec.frame)),
            Ok(None) => break None,
            Err(e) => break Some(e),
        }
    };
    let mut r = TlogSanitizeReport {
        reference: reference(&frames),
        ..Default::default()
    };
    let offset = r
        .reference
        .map(|(lat, lon)| (stand_in.0 - lat, stand_in.1 - lon));
    let mut out = Vec::new();
    for (t, f) in &frames {
        r.frames += 1;
        if message_info(f.msgid()).is_none() {
            r.unknown += 1;
        }
        out.extend_from_slice(&t.to_be_bytes());
        match rewrite(f, offset, &mut r) {
            Some(b) => {
                r.changed += 1;
                out.extend_from_slice(&b);
            }
            None => out.extend_from_slice(f.as_bytes()),
        }
    }
    (out, r, error)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::messages::{CommandLong, GlobalPositionInt, MissionItemInt, ParamValue};
    use mavlink_codec::{Header, Message, Version, encode};
    use recorder::{TlogWriter, from_unix_micros};

    fn tlog(msgs: &[Message]) -> Vec<u8> {
        let mut w = TlogWriter::new(Vec::new());
        for (i, m) in msgs.iter().enumerate() {
            let f = encode(
                Version::V2,
                Header {
                    seq: i as u8,
                    sysid: 1,
                    compid: 1,
                },
                m,
            )
            .unwrap();
            w.write(from_unix_micros(1_000_000 * (i as u64 + 1)), &f)
                .unwrap();
        }
        w.into_inner().unwrap()
    }

    fn read(b: &[u8]) -> Vec<Message> {
        let mut r = TlogReader::new(b);
        let mut out = Vec::new();
        while let Some(rec) = r.next_record().unwrap() {
            assert!(rec.frame.crc_checked());
            out.push(rec.frame.decode().unwrap());
        }
        out
    }

    fn param(name: &str, v: f32) -> Message {
        let mut id = [0u8; 16];
        id[..name.len()].copy_from_slice(name.as_bytes());
        Message::ParamValue(ParamValue {
            param_id: id,
            param_value: v,
            ..Default::default()
        })
    }

    #[test]
    fn every_kind_of_position_moves_by_one_offset() {
        let input = tlog(&[
            Message::GlobalPositionInt(GlobalPositionInt::default()), // no fix yet
            Message::GlobalPositionInt(GlobalPositionInt {
                lat: 315_000_000,
                lon: 348_000_000,
                alt: 600_000,
                ..Default::default()
            }),
            Message::MissionItemInt(MissionItemInt {
                frame: 3,
                x: 315_010_000,
                y: 348_010_000,
                z: 20.0,
                ..Default::default()
            }),
            Message::MissionItemInt(MissionItemInt {
                frame: 2, // MAV_FRAME_MISSION: x/y are not a position
                x: 7,
                y: 8,
                ..Default::default()
            }),
            Message::CommandLong(CommandLong {
                command: 179, // DO_SET_HOME
                param5: 31.5,
                param6: 34.8,
                ..Default::default()
            }),
            param("SIM_OPOS_LAT", 31.5),
            param("FENCE_RADIUS", 31.5),
        ]);
        let (out, r, err) = sanitize(&input[..], (-35.0, 149.0));
        assert!(err.is_none());
        assert_eq!(r.reference, Some((31.5, 34.8)));
        assert_eq!((r.frames, r.changed, r.values), (7, 4, 7));
        let m = read(&out);
        let Message::GlobalPositionInt(p) = &m[0] else {
            panic!()
        };
        assert_eq!((p.lat, p.lon), (0, 0), "no-fix zero stays zero");
        let Message::GlobalPositionInt(p) = &m[1] else {
            panic!()
        };
        assert_eq!(
            (p.lat, p.lon, p.alt),
            (-350_000_000, 1_490_000_000, 600_000)
        );
        let Message::MissionItemInt(w) = &m[2] else {
            panic!()
        };
        assert_eq!((w.x, w.y, w.z), (-349_990_000, 1_490_010_000, 20.0));
        let Message::MissionItemInt(w) = &m[3] else {
            panic!()
        };
        assert_eq!((w.x, w.y), (7, 8));
        let Message::CommandLong(c) = &m[4] else {
            panic!()
        };
        assert!((c.param5 + 35.0).abs() < 1e-4 && (c.param6 - 149.0).abs() < 1e-4);
        let (Message::ParamValue(a), Message::ParamValue(b)) = (&m[5], &m[6]) else {
            panic!()
        };
        assert!((a.param_value + 35.0).abs() < 1e-4);
        assert_eq!(b.param_value, 31.5);
    }

    #[test]
    fn without_a_fix_positions_are_zeroed() {
        let input = tlog(&[Message::MissionItemInt(MissionItemInt {
            frame: 6,
            x: 315_010_000,
            y: 348_010_000,
            ..Default::default()
        })]);
        let (out, r, _) = sanitize(&input[..], (-35.0, 149.0));
        assert_eq!((r.reference, r.values), (None, 2));
        let Message::MissionItemInt(w) = &read(&out)[0] else {
            panic!()
        };
        assert_eq!((w.x, w.y), (0, 0));
    }
}
