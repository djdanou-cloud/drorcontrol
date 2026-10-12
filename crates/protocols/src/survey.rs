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
    /// Photo distance along the lines, m: adds `DO_SET_CAM_TRIGG_DIST` (206) at the first
    /// survey waypoint and switches it off (0) before RTL. `None`: no camera commands.
    pub trigger_m: Option<f64>,
    /// Fly a second set of lines at 90° to the first (crosshatch), after the first set.
    pub crosshatch: bool,
    /// Extend each line this far beyond the polygon at both ends, m, so the vehicle is
    /// straight and level (and the camera steady) before it reaches the area.
    pub overshoot_m: f64,
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
        let o = cfg.overshoot_m.max(0.0);
        for (x0, x1) in segments {
            let (x0, x1) = (x0 - o, x1 + o);
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

/// A camera, for working out line spacing and photo distance from the overlap wanted.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    /// Sensor width and height, mm.
    pub sensor_w_mm: f64,
    /// See `sensor_w_mm`.
    pub sensor_h_mm: f64,
    /// Focal length, mm.
    pub focal_mm: f64,
    /// Image width and height, pixels.
    pub image_w_px: u32,
    /// See `image_w_px`.
    pub image_h_px: u32,
}

/// What one photo covers on flat ground straight below, with the image width across the
/// survey lines (pinhole camera, nadir).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Footprint {
    /// Ground sample distance (ground size of one pixel), cm.
    pub gsd_cm: f64,
    /// Ground covered across the lines (image width), m.
    pub across_m: f64,
    /// Ground covered along the lines (image height), m.
    pub along_m: f64,
    /// Distance between lines for the side overlap, m.
    pub spacing_m: f64,
    /// Distance between photos along a line for the front overlap, m.
    pub trigger_m: f64,
}

/// Footprint at `alt_m` above the ground, with side and front overlap as fractions (0..1).
pub fn footprint(cam: &Camera, alt_m: f64, side: f64, front: f64) -> Option<Footprint> {
    let ok = |v: f64| v.is_finite() && v > 0.0;
    if !(ok(cam.sensor_w_mm) && ok(cam.sensor_h_mm) && ok(cam.focal_mm) && ok(alt_m))
        || cam.image_w_px == 0
        || cam.image_h_px == 0
        || !(0.0..1.0).contains(&side)
        || !(0.0..1.0).contains(&front)
    {
        return None;
    }
    let across_m = cam.sensor_w_mm * alt_m / cam.focal_mm;
    let along_m = cam.sensor_h_mm * alt_m / cam.focal_mm;
    Some(Footprint {
        gsd_cm: across_m * 100.0 / f64::from(cam.image_w_px),
        across_m,
        along_m,
        spacing_m: across_m * (1.0 - side),
        trigger_m: along_m * (1.0 - front),
    })
}

/// Length flown along the survey lines (not the turns between them), m: pairs of points.
pub fn line_length_m(points: &[(f64, f64)]) -> f64 {
    points
        .chunks_exact(2)
        .map(|l| {
            let plane = Plane::new(l[0]);
            let (x, y) = plane.to_xy(l[1]);
            x.hypot(y)
        })
        .sum()
}

/// A complete mission for the survey: home placeholder (item 0, replaced by the vehicle),
/// TAKEOFF to the survey altitude, the grid waypoints, RTL. With `trigger_m`, the camera is
/// triggered by distance from the first survey waypoint (ArduPilot 4.7.1 `AP_Camera`:
/// param1 = metres, param3 = 1 takes one photo at once) and stopped (0 m) before RTL.
pub fn mission(cfg: &SurveyConfig) -> Result<Vec<MissionItem>, SurveyError> {
    if cfg.alt_m.is_nan() || cfg.alt_m <= 0.0 {
        return Err(SurveyError::BadAltitude);
    }
    let mut points = grid(cfg)?;
    if cfg.crosshatch {
        points.extend(grid(&SurveyConfig {
            angle_deg: cfg.angle_deg + 90.0,
            ..cfg.clone()
        })?);
    }
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
    let trigg = |metres: f64, now: f32| MissionItem {
        params: [metres as f32, 0.0, now, 0.0],
        ..item(206, 2, (0.0, 0.0), 0.0)
    };
    for (k, p) in points.into_iter().enumerate() {
        m.push(item(16, 3, p, cfg.alt_m));
        if k == 0
            && let Some(d) = cfg.trigger_m
        {
            m.push(trigg(d, 1.0));
        }
    }
    if cfg.trigger_m.is_some() {
        m.push(trigg(0.0, 0.0));
    }
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
            trigger_m: None,
            crosshatch: false,
            overshoot_m: 0.0,
        };
        let pts = grid(&cfg).unwrap();
        assert_eq!(pts.len(), 8, "4 lines, 2 points each");
        // With a 10 m overshoot the lines run 10 m past both edges.
        let longer = grid(&SurveyConfig {
            overshoot_m: 10.0,
            ..cfg.clone()
        })
        .unwrap();
        let pl = Plane::new((-35.36, 149.16));
        let (a, b) = (pl.to_xy(longer[0]), pl.to_xy(longer[1]));
        assert!(
            ((a.1 - b.1).abs() - 120.0).abs() < 0.05,
            "100 m + 2 x 10 m: {a:?} {b:?}"
        );
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
            trigger_m: None,
            crosshatch: false,
            overshoot_m: 0.0,
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
        // With a photo distance: trigger on at the first survey waypoint, off before RTL.
        let m = mission(&SurveyConfig {
            trigger_m: Some(12.5),
            ..cfg.clone()
        })
        .unwrap();
        assert_eq!(
            m.iter().map(|i| i.command).collect::<Vec<_>>(),
            [16, 22, 16, 206, 16, 16, 16, 206, 20]
        );
        assert_eq!((m[3].params[0], m[3].params[2]), (12.5, 1.0));
        assert_eq!((m[7].params[0], m[7].params[2]), (0.0, 0.0));
        let text = crate::wpl::to_string(&m);
        assert_eq!(crate::wpl::parse(&text).unwrap(), m);
        // Crosshatch: the east-west lines, then north-south lines over the same square.
        let x = mission(&SurveyConfig {
            crosshatch: true,
            trigger_m: None,
            ..cfg
        })
        .unwrap();
        assert_eq!(x.iter().filter(|i| i.command == 16).count(), 1 + 4 + 4);
        let p = Plane::new((-35.36, 149.16));
        let a = p.to_xy((f64::from(x[6].x) * 1e-7, f64::from(x[6].y) * 1e-7));
        let b = p.to_xy((f64::from(x[7].x) * 1e-7, f64::from(x[7].y) * 1e-7));
        assert!(
            (a.0 - b.0).abs() < 0.05,
            "second set runs north-south: {a:?} {b:?}"
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
            trigger_m: None,
            crosshatch: false,
            overshoot_m: 0.0,
        })
        .unwrap();
        // y = -40: one segment; y = 0 and 40: two each.
        assert_eq!(pts.len(), 2 + 4 + 4);
    }

    #[test]
    fn camera_footprint_and_overlaps() {
        // A 1-inch 20 MP camera (13.2 x 8.8 mm, 8.8 mm lens, 5472 x 3648) at 100 m.
        let cam = Camera {
            sensor_w_mm: 13.2,
            sensor_h_mm: 8.8,
            focal_mm: 8.8,
            image_w_px: 5472,
            image_h_px: 3648,
        };
        let f = footprint(&cam, 100.0, 0.7, 0.8).unwrap();
        assert!((f.across_m - 150.0).abs() < 1e-9 && (f.along_m - 100.0).abs() < 1e-9);
        assert!((f.gsd_cm - 2.741).abs() < 0.001, "{f:?}");
        assert!((f.spacing_m - 45.0).abs() < 1e-9 && (f.trigger_m - 20.0).abs() < 1e-9);
        assert_eq!(
            footprint(&cam, 100.0, 1.0, 0.8),
            None,
            "100 % overlap never advances"
        );
        assert_eq!(footprint(&cam, 0.0, 0.7, 0.8), None);
        // Line length of the 100 m square's 4 lines.
        let cfg = SurveyConfig {
            polygon: square(-35.36, 149.16, 100.0),
            spacing_m: 25.0,
            angle_deg: 0.0,
            alt_m: 50.0,
            trigger_m: None,
            crosshatch: false,
            overshoot_m: 0.0,
        };
        assert!((line_length_m(&grid(&cfg).unwrap()) - 400.0).abs() < 0.5);
    }

    #[test]
    fn rejects_what_it_cannot_fly() {
        let mut cfg = SurveyConfig {
            polygon: square(0.0, 0.0, 100.0),
            spacing_m: 0.0,
            angle_deg: 0.0,
            alt_m: 50.0,
            trigger_m: None,
            crosshatch: false,
            overshoot_m: 0.0,
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
