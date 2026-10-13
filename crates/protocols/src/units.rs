//! The one conversion layer for what the operator reads (parity XC-002, plan 01 §5.2):
//! distances, altitudes and speeds in metric, imperial or nautical units, and positions as
//! decimal degrees, degrees-minutes-seconds, degrees and decimal minutes, UTM or MGRS.
//! Values are kept in SI internally; only display goes through here.
//!
//! UTM uses the WGS-84 ellipsoid and Krüger's series to the 4th order (Karney 2011, as
//! on Wikipedia "Universal Transverse Mercator coordinate system"), sub-millimetre inside a
//! zone; zones include the Norway (32V) and Svalbard (31X–37X) exceptions. MGRS is the
//! WGS-84 (AA) lettering. Both cover 80°S–84°N; the polar UPS regions are not converted.
//! Checked against PROJ (pyproj) and the `mgrs` reference library.

/// A family of display units.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnitSystem {
    /// m, km, m/s, km/h.
    Metric,
    /// ft, mi, mph.
    Imperial,
    /// ft altitudes, nautical miles, knots (aviation).
    Nautical,
}

/// Feet in metres (exact).
pub const FT: f64 = 0.3048;
/// Statute mile in metres (exact).
pub const MI: f64 = 1609.344;
/// Nautical mile in metres (exact).
pub const NM: f64 = 1852.0;

/// An altitude or height, e.g. "120 m" or "394 ft".
pub fn altitude(m: f64, sys: UnitSystem) -> String {
    match sys {
        UnitSystem::Metric => format!("{m:.0} m"),
        UnitSystem::Imperial | UnitSystem::Nautical => format!("{:.0} ft", m / FT),
    }
}

/// A distance, switching to the larger unit for long ones: "850 m", "1.25 km",
/// "2789 ft", "1.20 mi", "0.46 NM".
pub fn distance(m: f64, sys: UnitSystem) -> String {
    match sys {
        UnitSystem::Metric if m.abs() < 1000.0 => format!("{m:.0} m"),
        UnitSystem::Metric => format!("{:.2} km", m / 1000.0),
        UnitSystem::Imperial if m.abs() < 0.25 * MI => format!("{:.0} ft", m / FT),
        UnitSystem::Imperial => format!("{:.2} mi", m / MI),
        UnitSystem::Nautical => format!("{:.2} NM", m / NM),
    }
}

/// A speed: "12.0 m/s", "26.8 mph", "23.3 kn".
pub fn speed(mps: f64, sys: UnitSystem) -> String {
    match sys {
        UnitSystem::Metric => format!("{mps:.1} m/s"),
        UnitSystem::Imperial => format!("{:.1} mph", mps * 3600.0 / MI),
        UnitSystem::Nautical => format!("{:.1} kn", mps * 3600.0 / NM),
    }
}

/// A position format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoordFormat {
    /// −35.3632000, 149.1652000
    Decimal,
    /// 35°21'47.5"S 149°09'54.7"E
    Dms,
    /// 35°21.7920'S 149°09.9120'E
    Ddm,
    /// 55H 696716 6084526
    Utm,
    /// 55H FA 96716 84526 (1 m)
    Mgrs,
}

fn hemi(v: f64, pos: char, neg: char) -> char {
    if v < 0.0 { neg } else { pos }
}

/// (degrees, minutes, seconds) of |v| with the seconds rounded to `dec` places, carried.
fn dms_parts(v: f64, dec: i32) -> (u32, u32, f64) {
    let scale = 10f64.powi(dec);
    let total = (v.abs() * 3600.0 * scale).round() / scale;
    let d = (total / 3600.0).floor();
    let m = ((total - d * 3600.0) / 60.0).floor();
    let s = total - d * 3600.0 - m * 60.0;
    (d as u32, m as u32, s.max(0.0))
}

/// A position in `fmt`; `None` where the format does not reach (UTM/MGRS at the poles).
pub fn position(lat: f64, lon: f64, fmt: CoordFormat) -> Option<String> {
    Some(match fmt {
        CoordFormat::Decimal => format!("{lat:.7}, {lon:.7}"),
        CoordFormat::Dms => {
            let (a, b, c) = dms_parts(lat, 1);
            let (d, e, f) = dms_parts(lon, 1);
            format!(
                "{a}°{b:02}'{c:04.1}\"{} {d}°{e:02}'{f:04.1}\"{}",
                hemi(lat, 'N', 'S'),
                hemi(lon, 'E', 'W')
            )
        }
        CoordFormat::Ddm => {
            let ddm = |v: f64| {
                let total = (v.abs() * 60.0 * 1e4).round() / 1e4;
                let d = (total / 60.0).floor();
                (d as u32, total - d * 60.0)
            };
            let ((a, b), (c, d)) = (ddm(lat), ddm(lon));
            format!(
                "{a}°{b:07.4}'{} {c}°{d:07.4}'{}",
                hemi(lat, 'N', 'S'),
                hemi(lon, 'E', 'W')
            )
        }
        CoordFormat::Utm => {
            let u = utm(lat, lon)?;
            format!(
                "{}{} {:.0} {:.0}",
                u.zone,
                u.band,
                u.easting.floor(),
                u.northing.floor()
            )
        }
        CoordFormat::Mgrs => {
            let m = mgrs(lat, lon, 5)?;
            format!("{} {} {} {}", &m[..3], &m[3..5], &m[5..10], &m[10..])
        }
    })
}

/// A UTM position.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Utm {
    /// Zone 1–60.
    pub zone: u8,
    /// Latitude band letter (C–X).
    pub band: char,
    /// Easting, m (500 000 at the central meridian).
    pub easting: f64,
    /// Northing, m (10 000 000 added south of the equator).
    pub northing: f64,
}

const BANDS: &[u8] = b"CDEFGHJKLMNPQRSTUVWX";

/// UTM zone, band and coordinates of (lat, lon); `None` outside 80°S–84°N.
pub fn utm(lat: f64, lon: f64) -> Option<Utm> {
    if !(-80.0..=84.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return None;
    }
    let lon = if lon >= 180.0 { lon - 360.0 } else { lon };
    let mut zone = (((lon + 180.0) / 6.0).floor() as i32 + 1).clamp(1, 60);
    if (56.0..64.0).contains(&lat) && (3.0..12.0).contains(&lon) {
        zone = 32;
    }
    if (72.0..=84.0).contains(&lat) {
        zone = match lon {
            l if (0.0..9.0).contains(&l) => 31,
            l if (9.0..21.0).contains(&l) => 33,
            l if (21.0..33.0).contains(&l) => 35,
            l if (33.0..42.0).contains(&l) => 37,
            _ => zone,
        };
    }
    let band = BANDS[(((lat + 80.0) / 8.0).floor() as usize).min(BANDS.len() - 1)] as char;

    // Krüger series (WGS-84).
    let a: f64 = 6_378_137.0;
    let f: f64 = 1.0 / 298.257_223_563;
    let k0 = 0.9996;
    let n = f / (2.0 - f);
    let (n2, n3, n4) = (n * n, n * n * n, n * n * n * n);
    let big_a = a / (1.0 + n) * (1.0 + n2 / 4.0 + n4 / 64.0);
    let alpha = [
        n / 2.0 - 2.0 / 3.0 * n2 + 5.0 / 16.0 * n3 + 41.0 / 180.0 * n4,
        13.0 / 48.0 * n2 - 3.0 / 5.0 * n3 + 557.0 / 1440.0 * n4,
        61.0 / 240.0 * n3 - 103.0 / 140.0 * n4,
        49561.0 / 161_280.0 * n4,
    ];
    let phi = lat.to_radians();
    let lambda = (lon - (f64::from(zone) * 6.0 - 183.0)).to_radians();
    let c = 2.0 * n.sqrt() / (1.0 + n);
    let t = (phi.sin().atanh() - c * (c * phi.sin()).atanh()).sinh();
    let xi_p = t.atan2(lambda.cos());
    let eta_p = (lambda.sin() / (1.0 + t * t).sqrt()).atanh();
    let (mut xi, mut eta) = (xi_p, eta_p);
    for (j, al) in alpha.iter().enumerate() {
        let k = 2.0 * (j + 1) as f64;
        xi += al * (k * xi_p).sin() * (k * eta_p).cosh();
        eta += al * (k * xi_p).cos() * (k * eta_p).sinh();
    }
    Some(Utm {
        zone: zone as u8,
        band,
        easting: 500_000.0 + k0 * big_a * eta,
        northing: if lat < 0.0 { 10_000_000.0 } else { 0.0 } + k0 * big_a * xi,
    })
}

/// MGRS reference with `digits` (1–5) per coordinate, compact ("55HFA9671684526").
pub fn mgrs(lat: f64, lon: f64, digits: u32) -> Option<String> {
    let digits = digits.clamp(1, 5);
    let u = utm(lat, lon)?;
    const COLS: [&[u8]; 3] = [b"ABCDEFGH", b"JKLMNPQR", b"STUVWXYZ"];
    const ROWS: &[u8] = b"ABCDEFGHJKLMNPQRSTUV";
    let set = COLS[usize::from(u.zone - 1) % 3];
    let e100k = (u.easting / 100_000.0).floor() as usize;
    let col = *set.get(e100k.checked_sub(1)?)? as char;
    let shift = if u.zone % 2 == 0 { 5 } else { 0 };
    let row = ROWS[((u.northing / 100_000.0).floor() as usize + shift) % 20] as char;
    let div = 10f64.powi(5 - digits as i32);
    let e = ((u.easting % 100_000.0) / div).floor() as u64;
    let n = ((u.northing % 100_000.0) / div).floor() as u64;
    let w = digits as usize;
    Some(format!("{:02}{}{col}{row}{e:0w$}{n:0w$}", u.zone, u.band))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// (lat, lon, zone, band, easting, northing, MGRS) from PROJ 9 (pyproj) and the `mgrs`
    /// library: both hemispheres, the antimeridian, the Norway and Svalbard zones, the
    /// equator and Greenwich.
    const REF: &[(f64, f64, u8, char, f64, f64, &str)] = &[
        (
            -35.3632,
            149.1652,
            55,
            'H',
            696716.667,
            6084526.413,
            "55HFA9671684526",
        ),
        (
            47.3985099,
            8.5451002,
            32,
            'T',
            465673.995,
            5249550.921,
            "32TMT6567349550",
        ),
        (
            38.8895,
            -77.0353,
            18,
            'S',
            323478.063,
            4306483.242,
            "18SUJ2347806483",
        ),
        (
            0.0001,
            0.0001,
            31,
            'N',
            166032.586,
            11.068,
            "31NAA6603200011",
        ),
        (
            -0.5,
            -179.9,
            1,
            'M',
            177176.037,
            9944663.622,
            "01MAV7717644663",
        ),
        (
            71.0,
            25.0,
            35,
            'W',
            427338.667,
            7878596.002,
            "35WMU2733878596",
        ),
        (
            60.5,
            5.0,
            32,
            'V',
            280356.453,
            6713774.225,
            "32VKN8035613774",
        ),
        (
            78.0,
            15.0,
            33,
            'X',
            500000.000,
            8658369.586,
            "33XWG0000058369",
        ),
        (
            -79.9,
            10.0,
            32,
            'C',
            519576.611,
            1129407.483,
            "32CNS1957629407",
        ),
        (
            51.4778,
            -0.0015,
            30,
            'U',
            708213.951,
            5707224.543,
            "30UYC0821307224",
        ),
    ];

    #[test]
    fn utm_and_mgrs_match_proj() {
        for &(lat, lon, zone, band, e, n, m) in REF {
            let u = utm(lat, lon).unwrap();
            assert_eq!((u.zone, u.band), (zone, band), "{lat},{lon}");
            assert!(
                (u.easting - e).abs() < 0.002 && (u.northing - n).abs() < 0.002,
                "{lat},{lon}: {u:?} vs {e} {n}"
            );
            assert_eq!(mgrs(lat, lon, 5).unwrap(), m, "{lat},{lon}");
        }
        assert_eq!(mgrs(-35.3632, 149.1652, 3).unwrap(), "55HFA967845");
        assert_eq!(utm(85.0, 0.0), None, "UPS region");
        assert_eq!(utm(-80.5, 0.0), None);
    }

    #[test]
    fn display_formats() {
        let (lat, lon) = (-35.3632, 149.1652);
        let p = |f| position(lat, lon, f).unwrap();
        assert_eq!(p(CoordFormat::Decimal), "-35.3632000, 149.1652000");
        assert_eq!(p(CoordFormat::Dms), "35°21'47.5\"S 149°09'54.7\"E");
        assert_eq!(p(CoordFormat::Ddm), "35°21.7920'S 149°09.9120'E");
        assert_eq!(p(CoordFormat::Utm), "55H 696716 6084526");
        assert_eq!(p(CoordFormat::Mgrs), "55H FA 96716 84526");
        // Seconds that round up to 60 carry into the minutes.
        assert_eq!(
            position(10.0 + 59.0 / 60.0 + 59.99 / 3600.0, 0.0, CoordFormat::Dms).unwrap(),
            "11°00'00.0\"N 0°00'00.0\"E"
        );
        assert_eq!(position(89.0, 0.0, CoordFormat::Mgrs), None);
    }

    #[test]
    fn units() {
        assert_eq!(altitude(120.0, UnitSystem::Metric), "120 m");
        assert_eq!(altitude(120.0, UnitSystem::Imperial), "394 ft");
        assert_eq!(distance(850.0, UnitSystem::Metric), "850 m");
        assert_eq!(distance(1250.0, UnitSystem::Metric), "1.25 km");
        assert_eq!(distance(300.0, UnitSystem::Imperial), "984 ft");
        assert_eq!(distance(1931.2128, UnitSystem::Imperial), "1.20 mi");
        assert_eq!(distance(1852.0, UnitSystem::Nautical), "1.00 NM");
        assert_eq!(speed(12.0, UnitSystem::Metric), "12.0 m/s");
        assert_eq!(speed(12.0, UnitSystem::Imperial), "26.8 mph");
        assert_eq!(speed(12.0, UnitSystem::Nautical), "23.3 kn");
    }
}
