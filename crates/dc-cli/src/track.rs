//! Flight track export: GPX 1.1 and KML 2.2 (Google Earth), as Mission Planner offers,
//! from a `.tlog` (here) or a dataflash log ([`logs::track`]).
//!
//! Points come from `GLOBAL_POSITION_INT` of the autopilot component (the fused position the
//! GCS displays), with the record's receive time. Samples at exactly 0, 0 (no position yet)
//! are skipped.

use std::fmt::Write as _;
use std::io::{self, Read};

use mavlink_codec::Message;
use recorder::TlogReader;

/// One track point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    /// UTC, µs since the Unix epoch (`None`: a log without a GPS time reference).
    pub unix_us: Option<u64>,
    /// Latitude, degrees.
    pub lat: f64,
    /// Longitude, degrees.
    pub lon: f64,
    /// Altitude above MSL, m.
    pub alt_msl: f64,
}

/// The track of `sysid` (default: the first system with a position). Returns the system
/// id and its points; a bad record ends the track there.
pub fn from_tlog(input: impl Read, sysid: Option<u8>) -> io::Result<Option<(u8, Vec<Point>)>> {
    let mut r = TlogReader::new(input);
    let mut chosen = sysid;
    let mut points = Vec::new();
    while let Ok(Some(rec)) = r.next_record() {
        let f = &rec.frame;
        if f.compid() != 1 {
            continue;
        }
        let Ok(Message::GlobalPositionInt(p)) = f.decode() else {
            continue;
        };
        if p.lat == 0 && p.lon == 0 {
            continue;
        }
        if *chosen.get_or_insert(f.sysid()) != f.sysid() {
            continue;
        }
        points.push(Point {
            unix_us: Some(rec.unix_us),
            lat: f64::from(p.lat) * 1e-7,
            lon: f64::from(p.lon) * 1e-7,
            alt_msl: f64::from(p.alt) / 1000.0,
        });
    }
    Ok(chosen.map(|s| (s, points)))
}

fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

/// ISO 8601 UTC, millisecond precision.
fn iso(unix_us: u64) -> String {
    crate::tlog::utc(unix_us).replacen(' ', "T", 1) + "Z"
}

/// From a dataflash log's track.
pub fn from_bin(log: &logs::Log) -> Vec<Point> {
    logs::track::track(log)
        .into_iter()
        .map(|p| Point {
            unix_us: p.unix_us,
            lat: p.lat,
            lon: p.lon,
            alt_msl: p.alt_msl,
        })
        .collect()
}

/// GPX 1.1 track.
pub fn gpx(name: &str, points: &[Point]) -> String {
    let mut s = String::from(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
         <gpx version=\"1.1\" creator=\"DrorControl\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n",
    );
    let _ = writeln!(
        s,
        "  <trk>\n    <name>{}</name>\n    <trkseg>",
        xml_escape(name)
    );
    for p in points {
        let _ = writeln!(
            s,
            "      <trkpt lat=\"{:.7}\" lon=\"{:.7}\"><ele>{:.2}</ele>{}</trkpt>",
            p.lat,
            p.lon,
            p.alt_msl,
            p.unix_us
                .map_or_else(String::new, |t| format!("<time>{}</time>", iso(t)))
        );
    }
    s.push_str("    </trkseg>\n  </trk>\n</gpx>\n");
    s
}

/// KML 2.2 line at absolute altitude (MSL), for Google Earth.
pub fn kml(name: &str, points: &[Point]) -> String {
    let mut s = String::from(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
         <kml xmlns=\"http://www.opengis.net/kml/2.2\">\n  <Document>\n",
    );
    let _ = writeln!(
        s,
        "    <name>{0}</name>\n    <Placemark>\n      <name>{0}</name>\n      <LineString>\n        \
         <altitudeMode>absolute</altitudeMode>\n        <coordinates>",
        xml_escape(name)
    );
    for p in points {
        // KML order is longitude,latitude,altitude.
        let _ = writeln!(s, "          {:.7},{:.7},{:.2}", p.lon, p.lat, p.alt_msl);
    }
    s.push_str(
        "        </coordinates>\n      </LineString>\n    </Placemark>\n  </Document>\n</kml>\n",
    );
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::messages::GlobalPositionInt;
    use mavlink_codec::{Header, Version, encode};
    use recorder::TlogWriter;

    fn tlog(points: &[(u64, u8, u8, i32, i32, i32)]) -> Vec<u8> {
        let mut w = TlogWriter::new(Vec::new());
        for &(us, sysid, compid, lat, lon, alt) in points {
            let m = Message::GlobalPositionInt(GlobalPositionInt {
                lat,
                lon,
                alt,
                ..Default::default()
            });
            let h = Header {
                seq: 0,
                sysid,
                compid,
            };
            w.write_raw(us, encode(Version::V2, h, &m).unwrap().as_bytes())
                .unwrap();
        }
        w.into_inner().unwrap()
    }

    #[test]
    fn extracts_one_vehicles_track() {
        let t0 = 1_760_000_000_000_000;
        let bytes = tlog(&[
            (t0, 1, 1, 0, 0, 0), // no position yet
            (t0 + 1, 1, 1, -353_632_610, 1_491_652_300, 584_000),
            (t0 + 2, 1, 154, -1, -1, 0), // gimbal: not the vehicle
            (t0 + 3, 2, 1, 10, 20, 30),  // another vehicle
            (t0 + 1_000_000, 1, 1, -353_622_610, 1_491_652_300, 604_500),
        ]);
        let (sys, pts) = from_tlog(&bytes[..], None).unwrap().unwrap();
        assert_eq!(sys, 1);
        assert_eq!(pts.len(), 2);
        assert_eq!(pts[1].alt_msl, 604.5);
        let (sys, pts) = from_tlog(&bytes[..], Some(2)).unwrap().unwrap();
        assert_eq!((sys, pts.len()), (2, 1));
        assert!(from_tlog(&tlog(&[])[..], None).unwrap().is_none());
    }

    #[test]
    fn writes_gpx_and_kml() {
        let pts = [Point {
            unix_us: Some(1_760_000_000_250_000),
            lat: -35.363261,
            lon: 149.16523,
            alt_msl: 584.0,
        }];
        let g = gpx("a & b", &pts);
        assert!(g.contains("<name>a &amp; b</name>"));
        assert!(g.contains(
            "<trkpt lat=\"-35.3632610\" lon=\"149.1652300\"><ele>584.00</ele>\
             <time>2025-10-09T08:53:20.250Z</time></trkpt>"
        ));
        let k = kml("flight", &pts);
        assert!(k.contains("<altitudeMode>absolute</altitudeMode>"));
        assert!(
            k.contains("149.1652300,-35.3632610,584.00"),
            "lon,lat,alt order"
        );
    }
}
