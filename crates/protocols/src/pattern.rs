//! Flight patterns other than the area survey (plan 03, mission planning): a corridor scan
//! along a path, and the IAMSAR search patterns. Pure geometry on the same local plane as
//! the survey; the result is a list of waypoints for `mission::route`, to check, save or
//! upload through the usual verified path.

use crate::survey::Plane;

/// Why a pattern cannot be made.
#[derive(Debug, Clone, PartialEq)]
pub enum PatternError {
    /// The corridor's path needs at least 2 points.
    TooFewPoints,
    /// A size (width, spacing, radius) must be positive and finite.
    BadSize(&'static str),
}

impl std::fmt::Display for PatternError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PatternError::TooFewPoints => f.write_str("the path needs at least 2 points"),
            PatternError::BadSize(what) => write!(f, "{what} must be above 0 m"),
        }
    }
}

fn positive(v: f64, what: &'static str) -> Result<(), PatternError> {
    if v.is_finite() && v > 0.0 {
        Ok(())
    } else {
        Err(PatternError::BadSize(what))
    }
}

/// Unit vector of a → b on the plane.
fn dir(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let l = dx.hypot(dy).max(1e-9);
    (dx / l, dy / l)
}

/// Corridor scan: lines parallel to `path` (latitude, longitude), `spacing_m` apart, enough
/// of them to cover `width_m` centred on the path (at least one), flown alternately in
/// opposite directions. Corners are mitred (capped at 3× the offset on very sharp turns);
/// each line runs `overshoot_m` past both ends of the path.
pub fn corridor(
    path: &[(f64, f64)],
    width_m: f64,
    spacing_m: f64,
    overshoot_m: f64,
) -> Result<Vec<(f64, f64)>, PatternError> {
    if path.len() < 2 {
        return Err(PatternError::TooFewPoints);
    }
    positive(width_m, "corridor width")?;
    positive(spacing_m, "line spacing")?;
    let plane = Plane::new(path[0]);
    let mut xy: Vec<(f64, f64)> = path.iter().map(|&p| plane.to_xy(p)).collect();
    xy.dedup_by(|a, b| (a.0 - b.0).hypot(a.1 - b.1) < 0.01);
    if xy.len() < 2 {
        return Err(PatternError::TooFewPoints);
    }
    let o = overshoot_m.max(0.0);
    let n = xy.len();
    let (d0, d1) = (dir(xy[0], xy[1]), dir(xy[n - 2], xy[n - 1]));
    xy[0] = (xy[0].0 - d0.0 * o, xy[0].1 - d0.1 * o);
    xy[n - 1] = (xy[n - 1].0 + d1.0 * o, xy[n - 1].1 + d1.1 * o);
    // Left normal of each segment, and the mitred offset direction at each vertex.
    let normals: Vec<(f64, f64)> = xy
        .windows(2)
        .map(|s| {
            let d = dir(s[0], s[1]);
            (-d.1, d.0)
        })
        .collect();
    let miter: Vec<(f64, f64)> = (0..n)
        .map(|k| {
            let a = normals[k.saturating_sub(1).min(n - 2)];
            let b = normals[k.min(n - 2)];
            let (mx, my) = (a.0 + b.0, a.1 + b.1);
            let l = mx.hypot(my);
            if l < 1e-6 {
                return a; // a full reversal: no sensible mitre
            }
            let (mx, my) = (mx / l, my / l);
            let scale = (1.0 / (mx * a.0 + my * a.1)).min(3.0);
            (mx * scale, my * scale)
        })
        .collect();
    let lines = (width_m / spacing_m).ceil().max(1.0) as usize;
    let mut out = Vec::new();
    for i in 0..lines {
        let off = (i as f64 - (lines - 1) as f64 / 2.0) * spacing_m;
        let mut line: Vec<(f64, f64)> = xy
            .iter()
            .zip(&miter)
            .map(|(p, m)| plane.to_latlon((p.0 + m.0 * off, p.1 + m.1 * off)))
            .collect();
        if i % 2 == 1 {
            line.reverse();
        }
        out.extend(line);
    }
    Ok(out)
}

/// A point `dist_m` from `o` (plane x east, y north) on `bearing_deg` (clockwise from north).
fn step(o: (f64, f64), bearing_deg: f64, dist_m: f64) -> (f64, f64) {
    let (s, c) = bearing_deg.to_radians().sin_cos();
    (o.0 + s * dist_m, o.1 + c * dist_m)
}

/// IAMSAR expanding square search from `datum`: legs of 1, 1, 2, 2, 3, 3 … × `spacing_m`,
/// the first on `track_deg` (clockwise from north), each turn 90° to the right, until the
/// next leg would be longer than `extent_m` (the side of the square searched). Starts at
/// the datum.
pub fn expanding_square(
    datum: (f64, f64),
    spacing_m: f64,
    extent_m: f64,
    track_deg: f64,
) -> Result<Vec<(f64, f64)>, PatternError> {
    positive(spacing_m, "track spacing")?;
    positive(extent_m, "search extent")?;
    let plane = Plane::new(datum);
    let mut at = (0.0, 0.0);
    let mut out = vec![datum];
    for leg in 0.. {
        let len = f64::from(leg / 2 + 1) * spacing_m;
        if len > extent_m + 1e-9 || leg > 10_000 {
            break;
        }
        at = step(at, track_deg + 90.0 * f64::from(leg), len);
        out.push(plane.to_latlon(at));
    }
    Ok(out)
}

/// IAMSAR sector search (single pattern) from `datum`: three equilateral triangles of side
/// `radius_m`, the first leg out on `track_deg`, every turn 120° to the right, each return
/// leg through the datum; the out legs end 60° apart all round. Starts and ends at the datum.
pub fn sector(
    datum: (f64, f64),
    radius_m: f64,
    track_deg: f64,
) -> Result<Vec<(f64, f64)>, PatternError> {
    positive(radius_m, "search radius")?;
    let plane = Plane::new(datum);
    let mut out = vec![datum];
    // Corner bearings from the datum, in flying order (derived from the 120° right turns).
    for b in [0.0, 60.0, 240.0, 300.0, 120.0, 180.0] {
        out.push(plane.to_latlon(step((0.0, 0.0), track_deg + b, radius_m)));
    }
    out.push(datum);
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corridor_lines_follow_the_path_and_alternate() {
        let p = Plane::new((-35.36, 149.16));
        // An L: 200 m east, then 100 m north.
        let path: Vec<_> = [(0.0, 0.0), (200.0, 0.0), (200.0, 100.0)]
            .iter()
            .map(|&q| p.to_latlon(q))
            .collect();
        let pts = corridor(&path, 60.0, 20.0, 0.0).unwrap();
        assert_eq!(pts.len(), 9, "3 lines of 3 points");
        let xy: Vec<_> = pts.iter().map(|&q| p.to_xy(q)).collect();
        let near =
            |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).abs() < 0.05 && (a.1 - b.1).abs() < 0.05;
        // Line 0 is 20 m right of the path (south of the east leg, east of the north leg),
        // with the corner mitred to (220, -20).
        assert!(near(xy[0], (0.0, -20.0)), "{:?}", xy[0]);
        assert!(near(xy[1], (220.0, -20.0)), "{:?}", xy[1]);
        assert!(near(xy[2], (220.0, 100.0)), "{:?}", xy[2]);
        // Line 1 is the path itself, flown back.
        assert!(near(xy[3], (200.0, 100.0)) && near(xy[5], (0.0, 0.0)));
        // Line 2 is 20 m left, forward again.
        assert!(near(xy[6], (0.0, 20.0)) && near(xy[7], (180.0, 20.0)));
        // Overshoot lengthens the ends only.
        let o = corridor(&path, 10.0, 20.0, 15.0).unwrap();
        assert_eq!(o.len(), 3);
        assert!(near(p.to_xy(o[0]), (-15.0, 0.0)) && near(p.to_xy(o[2]), (200.0, 115.0)));
        assert_eq!(
            corridor(&path[..1], 10.0, 5.0, 0.0),
            Err(PatternError::TooFewPoints)
        );
        assert_eq!(
            corridor(&path, 10.0, 0.0, 0.0),
            Err(PatternError::BadSize("line spacing"))
        );
    }

    #[test]
    fn expanding_square_legs_grow_every_second_turn() {
        let d = (-35.36, 149.16);
        let p = Plane::new(d);
        let pts = expanding_square(d, 10.0, 30.0, 0.0).unwrap();
        let xy: Vec<_> = pts.iter().map(|&q| p.to_xy(q)).collect();
        // N 10, E 10, S 20, W 20, N 30, E 30; the next leg (40 m) is past the extent.
        let want = [
            (0.0, 0.0),
            (0.0, 10.0),
            (10.0, 10.0),
            (10.0, -10.0),
            (-10.0, -10.0),
            (-10.0, 20.0),
            (20.0, 20.0),
        ];
        assert_eq!(xy.len(), want.len());
        for (a, b) in xy.iter().zip(want) {
            assert!(
                (a.0 - b.0).abs() < 0.01 && (a.1 - b.1).abs() < 0.01,
                "{a:?} {b:?}"
            );
        }
        // A track of 90° starts east and turns south.
        let e = expanding_square(d, 10.0, 10.0, 90.0).unwrap();
        let exy: Vec<_> = e.iter().map(|&q| p.to_xy(q)).collect();
        assert!(
            (exy[1].0 - 10.0).abs() < 0.01 && (exy[2].1 + 10.0).abs() < 0.01,
            "{exy:?}"
        );
        assert!(expanding_square(d, 0.0, 10.0, 0.0).is_err());
    }

    #[test]
    fn sector_legs_are_equal_with_right_turns_of_120() {
        let d = (-35.36, 149.16);
        let p = Plane::new(d);
        let pts = sector(d, 100.0, 30.0).unwrap();
        assert_eq!(pts.len(), 8);
        let xy: Vec<_> = pts.iter().map(|&q| p.to_xy(q)).collect();
        let bearing = |a: (f64, f64), b: (f64, f64)| (b.0 - a.0).atan2(b.1 - a.1).to_degrees();
        let mut prev: Option<f64> = None;
        for w in xy.windows(2) {
            let len = (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1);
            // Legs through the datum are two radii long; all others one.
            assert!(
                (len - 100.0).abs() < 0.01 || (len - 200.0).abs() < 0.01,
                "{len}"
            );
            let b = bearing(w[0], w[1]);
            if let Some(pb) = prev {
                let turn = (b - pb).rem_euclid(360.0);
                assert!((turn - 120.0).abs() < 0.01, "right turn of 120: {turn}");
            }
            prev = Some(b);
        }
        assert!(
            (bearing(xy[0], xy[1]) - 30.0).abs() < 0.01,
            "first leg on the track"
        );
        assert!(
            xy[7].0.abs() < 1e-6 && xy[7].1.abs() < 1e-6,
            "ends at the datum"
        );
    }
}
