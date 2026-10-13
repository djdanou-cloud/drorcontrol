//! Flight track export: GPX 1.1 and KML 2.2 (Google Earth), as Mission Planner offers,
//! from a `.tlog` (here) or a dataflash log ([`logs::track`]).
//!
//! Points come from `GLOBAL_POSITION_INT` of the autopilot component (the fused position the
//! GCS displays), with the record's receive time. Samples at exactly 0, 0 (no position yet)
//! are skipped.

use std::collections::BTreeMap;
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

/// A `.tlog` track with the flight mode at each point.
pub struct TlogTrack {
    /// The vehicle.
    pub sysid: u8,
    /// `(HEARTBEAT.custom_mode, point)`; `u32::MAX` before the first heartbeat.
    pub points: Vec<(u32, Point)>,
    /// Firmware from its version banner, else guessed from the heartbeat (display only).
    pub firmware: Option<dc_core::Firmware>,
}

/// The track of `sysid` (default: the first system with a position) with the flight mode
/// at each point; a bad record ends the track there.
pub fn from_tlog_modes(input: impl Read, sysid: Option<u8>) -> io::Result<Option<TlogTrack>> {
    let mut r = TlogReader::new(input);
    let mut chosen = sysid;
    let mut points = Vec::new();
    let mut mode = BTreeMap::<u8, u32>::new();
    let (mut banner, mut guess) = (None, None);
    while let Ok(Some(rec)) = r.next_record() {
        let f = &rec.frame;
        if f.compid() != 1 {
            continue;
        }
        let p = match f.decode() {
            Ok(Message::GlobalPositionInt(p)) => p,
            Ok(Message::Heartbeat(h)) => {
                mode.insert(f.sysid(), h.custom_mode);
                if chosen.is_none_or(|c| c == f.sysid()) {
                    guess = guess.or(dc_core::Firmware::from_mav_type(h.r#type, h.autopilot));
                }
                continue;
            }
            Ok(Message::Statustext(s)) => {
                let end = s.text.iter().position(|c| *c == 0).unwrap_or(s.text.len());
                if chosen.is_none_or(|c| c == f.sysid()) {
                    banner = banner.or(dc_core::Firmware::from_banner(&String::from_utf8_lossy(
                        &s.text[..end],
                    )));
                }
                continue;
            }
            _ => continue,
        };
        if p.lat == 0 && p.lon == 0 {
            continue;
        }
        if *chosen.get_or_insert(f.sysid()) != f.sysid() {
            continue;
        }
        points.push((
            mode.get(&f.sysid()).copied().unwrap_or(u32::MAX),
            Point {
                unix_us: Some(rec.unix_us),
                lat: f64::from(p.lat) * 1e-7,
                lon: f64::from(p.lon) * 1e-7,
                alt_msl: f64::from(p.alt) / 1000.0,
            },
        ));
    }
    Ok(chosen.map(|sysid| TlogTrack {
        sysid,
        points,
        firmware: banner.or(guess),
    }))
}

/// A mission waypoint for the KML. Missions are usually flown relative to home, so the
/// altitude is drawn relative to the ground.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    /// Mission sequence number.
    pub seq: u16,
    /// Latitude, degrees.
    pub lat: f64,
    /// Longitude, degrees.
    pub lon: f64,
    /// Altitude as planned, m.
    pub alt_m: f64,
}

/// Mission waypoints with a position (`x`/`y` in a global frame).
pub fn waypoints(items: &[dc_core::MissionItem]) -> Vec<Waypoint> {
    items
        .iter()
        .enumerate()
        .filter(|(_, i)| [0, 3, 5, 6, 10, 11].contains(&i.frame) && (i.x != 0 || i.y != 0))
        .map(|(n, i)| Waypoint {
            seq: n as u16,
            lat: f64::from(i.x) * 1e-7,
            lon: f64::from(i.y) * 1e-7,
            alt_m: f64::from(i.z),
        })
        .collect()
}

/// Consecutive runs of the same mode, labelled; each run also takes the next run's first
/// point so the line stays continuous.
pub fn split_by_mode(
    points: &[(u32, Point)],
    label: impl Fn(u32) -> String,
) -> Vec<(String, Vec<Point>)> {
    let mut out: Vec<(u32, Vec<Point>)> = Vec::new();
    for (m, p) in points {
        match out.last_mut() {
            Some((lm, v)) if lm == m => v.push(*p),
            Some((_, v)) => {
                v.push(*p);
                out.push((*m, vec![*p]));
            }
            None => out.push((*m, vec![*p])),
        }
    }
    out.into_iter().map(|(m, v)| (label(m), v)).collect()
}

/// Colours for track segments: the DrorAero brand colours from the design tokens
/// (`ui/tokens/tokens.json`), as KML `aabbggrr`.
pub fn segment_colours() -> Vec<String> {
    let Ok(t) = design_tokens::Tokens::load() else {
        return vec!["ffffffff".into()];
    };
    ["azure", "pizazz", "vapor", "silver", "smoke"]
        .iter()
        .filter_map(|n| t.json()["brand"][n].as_str())
        .filter_map(|hex| {
            let h = hex.trim_start_matches('#');
            (h.len() >= 6).then(|| format!("ff{}{}{}", &h[4..6], &h[2..4], &h[0..2]))
        })
        .collect()
}

/// KML 2.2 with the track split by flight mode (one coloured, extruded line per mode, same
/// mode same colour), a time-stamped `gx:Track` for Google Earth's time slider when the
/// points have UTC, and the mission's waypoints.
pub fn kml_rich(
    name: &str,
    segments: &[(String, Vec<Point>)],
    waypoints: &[Waypoint],
    altitude_mode: &str,
) -> String {
    let colours = segment_colours();
    let mut names: Vec<&str> = Vec::new();
    for (n, _) in segments {
        if !names.contains(&n.as_str()) {
            names.push(n);
        }
    }
    let mut s = String::from(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
         <kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\">\n  <Document>\n",
    );
    let _ = writeln!(s, "    <name>{}</name>", xml_escape(name));
    for (i, _) in names.iter().enumerate() {
        let _ = writeln!(
            s,
            "    <Style id=\"m{i}\"><LineStyle><color>{}</color><width>3</width></LineStyle><PolyStyle><color>40{}</color></PolyStyle></Style>",
            colours[i % colours.len()],
            &colours[i % colours.len()][2..]
        );
    }
    s.push_str("    <Folder>\n      <name>Track by flight mode</name>\n");
    for (n, pts) in segments {
        let style = names.iter().position(|x| x == n).unwrap_or(0);
        let _ = writeln!(
            s,
            "      <Placemark>\n        <name>{}</name>\n        <styleUrl>#m{style}</styleUrl>\n        <LineString>\n          <extrude>1</extrude>\n          <altitudeMode>{altitude_mode}</altitudeMode>\n          <coordinates>",
            xml_escape(n)
        );
        for p in pts {
            let _ = writeln!(s, "            {:.7},{:.7},{:.2}", p.lon, p.lat, p.alt_msl);
        }
        s.push_str("          </coordinates>\n        </LineString>\n      </Placemark>\n");
    }
    s.push_str("    </Folder>\n");
    let all: Vec<&Point> = segments.iter().flat_map(|(_, v)| v).collect();
    if !all.is_empty() && all.iter().all(|p| p.unix_us.is_some()) {
        let _ = writeln!(
            s,
            "    <Placemark>\n      <name>Timed track</name>\n      <gx:Track>\n        <altitudeMode>{altitude_mode}</altitudeMode>"
        );
        let mut last = None;
        let mut coords = String::new();
        for p in all {
            // Segment joins repeat a point; gx:Track wants one entry per time.
            if last == p.unix_us {
                continue;
            }
            last = p.unix_us;
            let _ = writeln!(s, "        <when>{}</when>", iso(p.unix_us.unwrap_or(0)));
            let _ = writeln!(
                coords,
                "        <gx:coord>{:.7} {:.7} {:.2}</gx:coord>",
                p.lon, p.lat, p.alt_msl
            );
        }
        s.push_str(&coords);
        s.push_str("      </gx:Track>\n    </Placemark>\n");
    }
    if !waypoints.is_empty() {
        s.push_str("    <Folder>\n      <name>Mission</name>\n");
        for w in waypoints {
            let _ = writeln!(
                s,
                "      <Placemark>\n        <name>WP {}</name>\n        <Point>\n          <altitudeMode>relativeToGround</altitudeMode>\n          <coordinates>{:.7},{:.7},{:.2}</coordinates>\n        </Point>\n      </Placemark>",
                w.seq, w.lon, w.lat, w.alt_m
            );
        }
        s.push_str("    </Folder>\n");
    }
    s.push_str("  </Document>\n</kml>\n");
    s
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

/// KML 2.2 line; `altitude_mode` is `absolute` (MSL) or `relativeToGround` (`alt_msl`
/// then holds the height above the ground).
pub fn kml_with(name: &str, points: &[Point], altitude_mode: &str) -> String {
    let mut s = String::from(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
         <kml xmlns=\"http://www.opengis.net/kml/2.2\">\n  <Document>\n",
    );
    let _ = writeln!(
        s,
        "    <name>{0}</name>\n    <Placemark>\n      <name>{0}</name>\n      <LineString>\n        \
         <altitudeMode>{1}</altitudeMode>\n        <coordinates>",
        xml_escape(name),
        altitude_mode
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

/// `kml` (from [`kml_with`]) with named polygons (lat, lon; closed in the file) and points
/// added, e.g. a mission's geofence and rally points, so they can be edited in Google Earth
/// and read back by `mission fence-from-kml` / `rally-from-kml`.
pub fn kml_add_shapes(
    kml: &str,
    polygons: &[(String, Vec<(f64, f64)>)],
    points: &[(String, (f64, f64))],
) -> String {
    let mut add = String::new();
    for (name, poly) in polygons {
        let _ = writeln!(
            add,
            "    <Placemark>\n      <name>{}</name>\n      <Polygon><outerBoundaryIs><LinearRing><coordinates>",
            xml_escape(name)
        );
        for (lat, lon) in poly.iter().chain(poly.first()) {
            let _ = writeln!(add, "          {lon:.7},{lat:.7},0");
        }
        add.push_str(
            "      </coordinates></LinearRing></outerBoundaryIs></Polygon>\n    </Placemark>\n",
        );
    }
    for (name, (lat, lon)) in points {
        let _ = writeln!(
            add,
            "    <Placemark>\n      <name>{}</name>\n      <Point><coordinates>{lon:.7},{lat:.7},0</coordinates></Point>\n    </Placemark>",
            xml_escape(name)
        );
    }
    match kml.rfind("  </Document>") {
        Some(at) => format!("{}{add}{}", &kml[..at], &kml[at..]),
        None => kml.to_string(),
    }
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
        let t = from_tlog_modes(&bytes[..], None).unwrap().unwrap();
        assert_eq!(t.sysid, 1);
        assert_eq!(t.points.len(), 2);
        assert_eq!(t.points[1].1.alt_msl, 604.5);
        assert_eq!(t.points[0].0, u32::MAX, "no heartbeat yet");
        let t = from_tlog_modes(&bytes[..], Some(2)).unwrap().unwrap();
        assert_eq!((t.sysid, t.points.len()), (2, 1));
        assert!(from_tlog_modes(&tlog(&[])[..], None).unwrap().is_none());
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
        let k = kml_with("flight", &pts, "absolute");
        assert!(k.contains("<altitudeMode>absolute</altitudeMode>"));
        assert!(
            k.contains("149.1652300,-35.3632610,584.00"),
            "lon,lat,alt order"
        );
    }

    #[test]
    fn rich_kml_splits_by_mode_and_carries_time_and_waypoints() {
        let p = |t: u64, lat: f64| Point {
            unix_us: Some(1_760_000_000_000_000 + t * 1_000_000),
            lat,
            lon: 149.0,
            alt_msl: 600.0,
        };
        let tagged = [
            (0, p(0, -35.0)),
            (0, p(1, -35.1)),
            (3, p(2, -35.2)),
            (0, p(3, -35.3)),
        ];
        let seg = split_by_mode(&tagged, |m| format!("m{m}"));
        let names: Vec<_> = seg.iter().map(|(n, v)| (n.as_str(), v.len())).collect();
        assert_eq!(
            names,
            [("m0", 3), ("m3", 2), ("m0", 1)],
            "joins share a point"
        );
        let wp = [Waypoint {
            seq: 1,
            lat: -35.1,
            lon: 149.1,
            alt_m: 20.0,
        }];
        let k = kml_rich("f", &seg, &wp, "absolute");
        assert_eq!(
            k.matches("<styleUrl>#m0</styleUrl>").count(),
            2,
            "same mode, same style"
        );
        assert_eq!(k.matches("<Style id=").count(), 2);
        assert_eq!(
            k.matches("<when>").count(),
            4,
            "one per time, joins not repeated"
        );
        assert!(k.contains("<when>2025-10-09T08:53:21.000Z</when>"));
        assert!(k.contains("<name>WP 1</name>"));
        assert!(k.contains("<extrude>1</extrude>"));
        assert!(k.contains("149.1000000,-35.1000000,20.00"));
        let colours = segment_colours();
        assert!(
            colours.len() >= 2 && colours.iter().all(|c| c.len() == 8),
            "{colours:?}"
        );
    }

    #[test]
    fn shapes_read_back_as_placemarks() {
        let base = kml_with("m", &[], "relativeToGround");
        let k = kml_add_shapes(
            &base,
            &[(
                "No-fly 1 & co".into(),
                vec![(-35.0, 149.0), (-35.0, 149.1), (-35.1, 149.1)],
            )],
            &[("Rally 1".into(), (-35.05, 149.05))],
        );
        let p = protocols::kml::placemarks(&k).unwrap();
        let shapes: Vec<_> = p
            .iter()
            .map(|p| (p.name.as_deref(), p.kind, p.points.len()))
            .collect();
        assert_eq!(
            shapes,
            [
                (Some("m"), "LineString", 0),
                (Some("No-fly 1 & co"), "Polygon", 3),
                (Some("Rally 1"), "Point", 1)
            ]
        );
        assert!(k.trim_end().ends_with("</kml>"));
    }
}
