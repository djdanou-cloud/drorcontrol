//! Survey grid ("lawnmower") over a polygon: parallel lines at a fixed spacing and angle,
//! flown alternately in opposite directions. Pure geometry; the result is an ordinary
//! mission to check, save or upload through the usual verified path.
//!
//! The polygon is projected onto a local plane (equirectangular about its centre), which
//! is accurate to well under a metre over survey-sized areas (a few km).

use crate::mission::MissionItem;

const EARTH_RADIUS_M: f64 = 6_371_008.8;

/// What to fly.
#[derive(Debug, Clone, PartialEq)]
pub struct SurveyConfig {
    /// Polygon vertices (latitude, longitude), degrees, in order; at least 3.
    pub polygon: Vec<(f64, f64)>,
    /// Distance between lines, m.
    pub spacing_m: f64,
    /// Line direction, degrees clockwise from north.
    pub angle_deg: f64,
    /// Altitude above home, m.
    pub alt_m: f32,
}

/// Why a survey cannot be made.
#[derive(Debug, Clone, PartialEq)]
pub enum SurveyError {
    /// Fewer than 3 vertices.
    TooFewVertices,
    /// Spacing must be positive and leave at least one line.
    BadSpacing,
    /// Altitude must be above home.
    BadAltitude,
    /// No line crosses the polygon.
    Empty,
}

impl std::fmt::Display for SurveyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            SurveyError::TooFewVertices => "the polygon needs at least 3 vertices",
            SurveyError::BadSpacing => "line spacing must be positive",
            SurveyError::BadAltitude => "altitude must be above home (> 0 m)",
            SurveyError::Empty => "no survey line crosses the polygon",
        })
    }
}

/// Local plane about `origin`: x east, y north, metres.
struct Plane {
    lat0: f64,
    lon0: f64,
    cos_lat0: f64,
}

impl Plane {
    fn new(origin: (f64, f64)) -> Self {
        Plane {
            lat0: origin.0,
            lon0: origin.1,
            cos_lat0: origin.0.to_radians().cos(),
        }
    }
    fn to_xy(&self, (lat, lon): (f64, f64)) -> (f64, f64) {
        (
            (lon - self.lon0).to_radians() * self.cos_lat0 * EARTH_RADIUS_M,
            (lat - self.lat0).to_radians() * EARTH_RADIUS_M,
        )
    }
    fn to_latlon(&self, (x, y): (f64, f64)) -> (f64, f64) {
        (
            self.lat0 + (y / EARTH_RADIUS_M).to_degrees(),
            self.lon0 + (x / (EARTH_RADIUS_M * self.cos_lat0)).to_degrees(),
        )
    }
}

/// The survey's waypoints (latitude, longitude), in flying order. Each line enters and
/// leaves the polygon at its edges; in a concave polygon a line crossing it more than once
/// is flown as one segment per crossing, in order along the line.
pub fn grid(cfg: &SurveyConfig) -> Result<Vec<(f64, f64)>, SurveyError> {
    if cfg.polygon.len() < 3 {
        return Err(SurveyError::TooFewVertices);
    }
    if !(cfg.spacing_m.is_finite() && cfg.spacing_m > 0.0) {
        return Err(SurveyError::BadSpacing);
    }
    let n = cfg.polygon.len() as f64;
    let centre = (
        cfg.polygon.iter().map(|p| p.0).sum::<f64>() / n,
        cfg.polygon.iter().map(|p| p.1).sum::<f64>() / n,
    );
    let plane = Plane::new(centre);
    // Rotate so that the survey lines are horizontal: the line direction (`angle`, clockwise
    // from north) becomes the rotated x axis, and lines are stepped along the rotated y.
    let a = (90.0 - cfg.angle_deg).to_radians();
    let (sin, cos) = a.sin_cos();
    let rot = |(x, y): (f64, f64)| (x * cos + y * sin, -x * sin + y * cos);
    let unrot = |(x, y): (f64, f64)| (x * cos - y * sin, x * sin + y * cos);
    let poly: Vec<(f64, f64)> = cfg.polygon.iter().map(|&p| rot(plane.to_xy(p))).collect();
    let (ymin, ymax) = poly.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| {
        (lo.min(p.1), hi.max(p.1))
    });

    let mut out = Vec::new();
    let mut y = ymin + cfg.spacing_m / 2.0;
    let mut forward = true;
    while y < ymax {
        // Crossings of the line with each edge (half-open rule avoids double vertices).
        let mut xs: Vec<f64> = poly
            .iter()
            .zip(poly.iter().cycle().skip(1))
            .filter(|(a, b)| (a.1 <= y) != (b.1 <= y))
            .map(|(a, b)| a.0 + (y - a.1) * (b.0 - a.0) / (b.1 - a.1))
            .collect();
        xs.sort_by(f64::total_cmp);
        let mut segments: Vec<(f64, f64)> = xs.chunks_exact(2).map(|c| (c[0], c[1])).collect();
        if !forward {
            segments.reverse();
        }
        for (x0, x1) in segments {
            let (a, b) = if forward { (x0, x1) } else { (x1, x0) };
            out.push(plane.to_latlon(unrot((a, y))));
            out.push(plane.to_latlon(unrot((b, y))));
        }
        forward = !forward;
        y += cfg.spacing_m;
    }
    if out.is_empty() {
        return Err(SurveyError::Empty);
    }
    Ok(out)
}

/// A complete mission for the survey: home placeholder (item 0, replaced by the vehicle),
/// TAKEOFF to the survey altitude, the grid waypoints, RTL.
pub fn mission(cfg: &SurveyConfig) -> Result<Vec<MissionItem>, SurveyError> {
    if cfg.alt_m.is_nan() || cfg.alt_m <= 0.0 {
        return Err(SurveyError::BadAltitude);
    }
    let points = grid(cfg)?;
    let item = |command: u16, frame: u8, (lat, lon): (f64, f64), z: f32| MissionItem {
        frame,
        command,
        params: [0.0; 4],
        x: (lat * 1e7).round() as i32,
        y: (lon * 1e7).round() as i32,
        z,
        autocontinue: true,
    };
    let mut m = vec![
        item(16, 0, cfg.polygon[0], 0.0),
        item(22, 3, (0.0, 0.0), cfg.alt_m),
    ];
    m.extend(points.into_iter().map(|p| item(16, 3, p, cfg.alt_m)));
    m.push(item(20, 2, (0.0, 0.0), 0.0));
    Ok(m)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A square of `side` metres around (lat, lon), as (lat, lon) vertices.
    fn square(lat: f64, lon: f64, side: f64) -> Vec<(f64, f64)> {
        let p = Plane::new((lat, lon));
        let h = side / 2.0;
        [(-h, -h), (h, -h), (h, h), (-h, h)]
            .iter()
            .map(|&xy| p.to_latlon(xy))
            .collect()
    }

    #[test]
    fn north_south_lines_over_a_square() {
        let cfg = SurveyConfig {
            polygon: square(-35.36, 149.16, 100.0),
            spacing_m: 25.0,
            angle_deg: 0.0,
            alt_m: 50.0,
        };
        let pts = grid(&cfg).unwrap();
        assert_eq!(pts.len(), 8, "4 lines, 2 points each");
        let p = Plane::new((-35.36, 149.16));
        let xy: Vec<(f64, f64)> = pts.iter().map(|&q| p.to_xy(q)).collect();
        // Lines run north-south, 25 m apart at x = ±12.5 and ±37.5, edge to edge, flown in
        // alternating directions.
        let mut xs: Vec<f64> = xy.chunks(2).map(|l| l[0].0).collect();
        for (k, line) in xy.chunks(2).enumerate() {
            assert!(
                (line[0].0 - line[1].0).abs() < 0.01,
                "north-south: {line:?}"
            );
            assert!(
                (line[0].1.abs() - 50.0).abs() < 0.01,
                "ends on the edge: {line:?}"
            );
            if k > 0 {
                assert!(((xs[k] - xs[k - 1]).abs() - 25.0).abs() < 0.01, "spacing");
                let prev = xy[2 * k - 1].1 > xy[2 * k - 2].1;
                assert_ne!(line[1].1 > line[0].1, prev, "line {k} reverses direction");
            }
        }
        xs.sort_by(f64::total_cmp);
        for (x, want) in xs.iter().zip([-37.5, -12.5, 12.5, 37.5]) {
            assert!((x - want).abs() < 0.01, "{xs:?}");
        }
    }

    #[test]
    fn east_west_lines_and_the_mission_around_them() {
        let cfg = SurveyConfig {
            polygon: square(-35.36, 149.16, 100.0),
            spacing_m: 50.0,
            angle_deg: 90.0,
            alt_m: 40.0,
        };
        let m = mission(&cfg).unwrap();
        // home, takeoff, 2 lines x 2 points, RTL
        assert_eq!(
            m.iter().map(|i| i.command).collect::<Vec<_>>(),
            [16, 22, 16, 16, 16, 16, 20]
        );
        let p = Plane::new((-35.36, 149.16));
        let a = p.to_xy((f64::from(m[2].x) * 1e-7, f64::from(m[2].y) * 1e-7));
        let b = p.to_xy((f64::from(m[3].x) * 1e-7, f64::from(m[3].y) * 1e-7));
        assert!((a.1 - b.1).abs() < 0.05, "an east-west line: {a:?} {b:?}");
        assert!(
            crate::mission::lint(&m).is_empty(),
            "{:?}",
            crate::mission::lint(&m)
        );
    }

    #[test]
    fn a_concave_polygon_gives_a_segment_per_crossing() {
        // A "U": lines across the gap cross the polygon twice.
        let p = Plane::new((0.0, 0.0));
        let poly: Vec<(f64, f64)> = [
            (-60.0, -60.0),
            (60.0, -60.0),
            (60.0, 60.0),
            (20.0, 60.0),
            (20.0, -20.0),
            (-20.0, -20.0),
            (-20.0, 60.0),
            (-60.0, 60.0),
        ]
        .iter()
        .map(|&xy| p.to_latlon(xy))
        .collect();
        let pts = grid(&SurveyConfig {
            polygon: poly,
            spacing_m: 40.0,
            angle_deg: 90.0,
            alt_m: 30.0,
        })
        .unwrap();
        // y = -40: one segment; y = 0 and 40: two each.
        assert_eq!(pts.len(), 2 + 4 + 4);
    }

    #[test]
    fn rejects_what_it_cannot_fly() {
        let mut cfg = SurveyConfig {
            polygon: square(0.0, 0.0, 100.0),
            spacing_m: 0.0,
            angle_deg: 0.0,
            alt_m: 50.0,
        };
        assert_eq!(grid(&cfg), Err(SurveyError::BadSpacing));
        cfg.spacing_m = 500.0;
        assert_eq!(grid(&cfg), Err(SurveyError::Empty));
        cfg.spacing_m = 20.0;
        cfg.alt_m = 0.0;
        assert_eq!(mission(&cfg), Err(SurveyError::BadAltitude));
        cfg.polygon.truncate(2);
        assert_eq!(grid(&cfg), Err(SurveyError::TooFewVertices));
    }
}
