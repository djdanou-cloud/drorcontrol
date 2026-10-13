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
}
