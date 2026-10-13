//! Does a mission stay inside the geofence? Checked before upload, along the whole path (not
//! only at waypoints), the way ArduPilot decides a breach. Read-only: nothing is sent.
//!
//! ArduPilot 4.7.1 (`AC_PolyFence_loader::breached`, `AC_Fence.cpp`/`.h`):
//! * Fence items: polygon vertices `NAV_FENCE_POLYGON_VERTEX_INCLUSION` 5001 / `EXCLUSION` 5002
//!   (param1 = the polygon's vertex count, consecutive items), circles `NAV_FENCE_CIRCLE_
//!   INCLUSION` 5003 / `EXCLUSION` 5004 (param1 = radius, m), return point 5000.
//! * Inside any exclusion zone is a breach. With several inclusion zones a position must be
//!   inside every one of them, unless `FENCE_OPTIONS` bit 1 (`INCLUSION_UNION`) asks for any.
//! * `FENCE_TYPE` bit 1 (`AC_FENCE_TYPE_ALT_MAX`): `FENCE_ALT_MAX` (default 100 m above home);
//!   bit 2 (`AC_FENCE_TYPE_CIRCLE`): `FENCE_RADIUS` (default 300 m) around home.

use std::collections::BTreeMap;

use crate::mission::{Finding, LintLevel, MissionItem, PathPoint};

const EARTH_RADIUS_M: f64 = 6_371_008.8;

/// A geofence as zones (latitude, longitude in degrees).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Fence {
    /// Inclusion polygons.
    pub inclusion_polygons: Vec<Vec<(f64, f64)>>,
    /// Exclusion polygons.
    pub exclusion_polygons: Vec<Vec<(f64, f64)>>,
    /// Inclusion circles: centre and radius, m.
    pub inclusion_circles: Vec<((f64, f64), f64)>,
    /// Exclusion circles.
    pub exclusion_circles: Vec<((f64, f64), f64)>,
}

impl Fence {
    /// From fence items as downloaded. Fails on a polygon whose vertex count does not match.
    pub fn from_items(items: &[MissionItem]) -> Result<Fence, String> {
        let mut f = Fence::default();
        let deg = |i: &MissionItem| (f64::from(i.x) * 1e-7, f64::from(i.y) * 1e-7);
        let mut k = 0;
        while k < items.len() {
            let i = &items[k];
            match i.command {
                5001 | 5002 => {
                    let n = i.params[0] as usize;
                    if n < 3 || k + n > items.len() {
                        return Err(format!("fence item {k}: polygon of {n} vertices"));
                    }
                    let poly: Vec<(f64, f64)> = items[k..k + n]
                        .iter()
                        .map(|v| {
                            if v.command == i.command && v.params[0] as usize == n {
                                Ok(deg(v))
                            } else {
                                Err(format!("fence item {k}: polygon vertices do not match"))
                            }
                        })
                        .collect::<Result<_, _>>()?;
                    if i.command == 5001 {
                        f.inclusion_polygons.push(poly);
                    } else {
                        f.exclusion_polygons.push(poly);
                    }
                    k += n;
                    continue;
                }
                5003 => f.inclusion_circles.push((deg(i), f64::from(i.params[0]))),
                5004 => f.exclusion_circles.push((deg(i), f64::from(i.params[0]))),
                _ => {} // return point and anything else: not a zone
            }
            k += 1;
        }
        Ok(f)
    }

    /// As fence items (inverse of `from_items`): polygons as consecutive 5001/5002 vertices
    /// with the vertex count in param1, circles as 5003/5004 with the radius, frame 0
    /// (global, the altitude is unused).
    pub fn to_items(&self) -> Vec<MissionItem> {
        let e7 = |d: f64| (d * 1e7).round() as i32;
        let one = |command: u16, p1: f64, (lat, lon): (f64, f64)| MissionItem {
            frame: 0,
            command,
            params: [p1 as f32, 0.0, 0.0, 0.0],
            x: e7(lat),
            y: e7(lon),
            z: 0.0,
            autocontinue: true,
        };
        let mut out = Vec::new();
        for (command, polys) in [
            (5001, &self.inclusion_polygons),
            (5002, &self.exclusion_polygons),
        ] {
            for poly in polys {
                out.extend(poly.iter().map(|&v| one(command, poly.len() as f64, v)));
            }
        }
        for (command, circles) in [
            (5003, &self.inclusion_circles),
            (5004, &self.exclusion_circles),
        ] {
            out.extend(circles.iter().map(|&(c, r)| one(command, r, c)));
        }
        out
    }

    /// No zones at all.
    pub fn is_empty(&self) -> bool {
        self.inclusion_polygons.is_empty()
            && self.exclusion_polygons.is_empty()
            && self.inclusion_circles.is_empty()
            && self.exclusion_circles.is_empty()
    }
}

/// The parameter-defined fences (`FENCE_TYPE`, `FENCE_RADIUS`, `FENCE_ALT_MAX`,
/// `FENCE_OPTIONS`); each absent when not configured.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FenceParams {
    /// Circle around home, m.
    pub circle_radius_m: Option<f64>,
    /// Highest altitude above home, m.
    pub alt_max_m: Option<f64>,
    /// Inclusion zones combine as a union (`FENCE_OPTIONS` bit 1).
    pub union: bool,
}

impl FenceParams {
    /// From a vehicle's parameters (4.7.1 names and defaults).
    pub fn from_params(p: &BTreeMap<String, f64>) -> FenceParams {
        let ty = p.get("FENCE_TYPE").copied().unwrap_or(0.0) as u32;
        FenceParams {
            circle_radius_m: (ty & 2 != 0).then(|| p.get("FENCE_RADIUS").copied().unwrap_or(300.0)),
            alt_max_m: (ty & 1 != 0).then(|| p.get("FENCE_ALT_MAX").copied().unwrap_or(100.0)),
            union: p.get("FENCE_OPTIONS").copied().unwrap_or(0.0) as u32 & 2 != 0,
        }
    }
}

/// Where the path leaves the fence.
#[derive(Debug, Clone, PartialEq)]
pub struct Breach {
    /// Item whose leg (or position) breaches.
    pub seq: u16,
    /// What is breached.
    pub what: String,
    /// The first breaching position on that leg.
    pub at: (f64, f64),
}

/// Local plane about `o`: (east, north) metres.
fn xy(o: (f64, f64), p: (f64, f64)) -> (f64, f64) {
    (
        (p.1 - o.1).to_radians() * o.0.to_radians().cos() * EARTH_RADIUS_M,
        (p.0 - o.0).to_radians() * EARTH_RADIUS_M,
    )
}

fn inside_polygon(o: (f64, f64), poly: &[(f64, f64)], p: (f64, f64)) -> bool {
    let (px, py) = xy(o, p);
    let v: Vec<(f64, f64)> = poly.iter().map(|&q| xy(o, q)).collect();
    let mut inside = false;
    for (a, b) in v.iter().zip(v.iter().cycle().skip(1)) {
        if (a.1 > py) != (b.1 > py) && px < a.0 + (py - a.1) * (b.0 - a.0) / (b.1 - a.1) {
            inside = !inside;
        }
    }
    inside
}

fn dist(o: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (p, q) = (xy(o, a), xy(o, b));
    (p.0 - q.0).hypot(p.1 - q.1)
}

/// What `p` (at `alt_rel` above home) breaches, if anything.
fn breach_at(
    home: (f64, f64),
    fence: &Fence,
    params: &FenceParams,
    p: (f64, f64),
    alt_rel: f64,
) -> Option<String> {
    for (k, poly) in fence.exclusion_polygons.iter().enumerate() {
        if inside_polygon(home, poly, p) {
            return Some(format!("inside exclusion polygon {}", k + 1));
        }
    }
    for (k, (c, r)) in fence.exclusion_circles.iter().enumerate() {
        if dist(home, *c, p) < *r {
            return Some(format!("inside exclusion circle {}", k + 1));
        }
    }
    let mut zones: Vec<(String, bool)> = Vec::new();
    for (k, poly) in fence.inclusion_polygons.iter().enumerate() {
        zones.push((
            format!("inclusion polygon {}", k + 1),
            inside_polygon(home, poly, p),
        ));
    }
    for (k, (c, r)) in fence.inclusion_circles.iter().enumerate() {
        zones.push((
            format!("inclusion circle {}", k + 1),
            dist(home, *c, p) <= *r,
        ));
    }
    let outside: Vec<&String> = zones.iter().filter(|z| !z.1).map(|z| &z.0).collect();
    let breached = if params.union {
        !zones.is_empty() && outside.len() == zones.len()
    } else {
        !outside.is_empty()
    };
    if breached {
        return Some(format!("outside {}", outside[0]));
    }
    if let Some(r) = params.circle_radius_m
        && dist(home, home, p) > r
    {
        return Some(format!("beyond FENCE_RADIUS {r:.0} m from home"));
    }
    if let Some(a) = params.alt_max_m
        && alt_rel > a
    {
        return Some(format!("above FENCE_ALT_MAX {a:.0} m"));
    }
    None
}

/// Breaches along `path` (from [`crate::mission::path`]; its first point is home), sampled
/// every `step_m` metres along each leg; at most one per leg.
pub fn check_path(
    path: &[PathPoint],
    fence: &Fence,
    params: &FenceParams,
    step_m: f64,
) -> Vec<Breach> {
    let Some(home) = path.first().map(|p| (p.lat, p.lon)) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for leg in path.windows(2) {
        let (a, b) = (leg[0], leg[1]);
        let len = dist(home, (a.lat, a.lon), (b.lat, b.lon));
        let n = (len / step_m).ceil().max(1.0) as usize;
        for k in 0..=n {
            let f = k as f64 / n as f64;
            let p = (a.lat + (b.lat - a.lat) * f, a.lon + (b.lon - a.lon) * f);
            let alt = a.alt_rel_m + (b.alt_rel_m - a.alt_rel_m) * f;
            if let Some(what) = breach_at(home, fence, params, p, alt) {
                out.push(Breach {
                    seq: b.seq,
                    what,
                    at: p,
                });
                break;
            }
        }
    }
    out
}

/// Do segments p1-p2 and q1-q2 cross (touching ends do not count)?
fn segments_cross(p1: (f64, f64), p2: (f64, f64), q1: (f64, f64), q2: (f64, f64)) -> bool {
    let side = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| {
        (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
    };
    let (d1, d2) = (side(q1, q2, p1), side(q1, q2, p2));
    let (d3, d4) = (side(p1, p2, q1), side(p1, p2, q2));
    d1 * d2 < 0.0 && d3 * d4 < 0.0
}

/// Does the polygon's outline cross itself (a bow-tie)? Inside and outside are then not
/// what the drawing suggests.
pub fn self_intersects(poly: &[(f64, f64)]) -> bool {
    let Some(&o) = poly.first() else {
        return false;
    };
    let v: Vec<(f64, f64)> = poly.iter().map(|&p| xy(o, p)).collect();
    let n = v.len();
    (0..n).any(|i| {
        (i + 2..n)
            .filter(|&j| !(i == 0 && j == n - 1))
            .any(|j| segments_cross(v[i], v[(i + 1) % n], v[j], v[(j + 1) % n]))
    })
}

/// Check fence items before upload: what ArduPilot refuses (4.7.1
/// `AC_PolyFence_loader::validate_fence`: polygon vertex counts, circle radius, more than
/// one return point, positions) and what it does not check: polygons crossing themselves,
/// the return point outside the fence (its own TODO), home outside (arming then fails with
/// "Vehicle breaching …" while fences are enabled) and rally points outside. `home` is
/// (lat, lon); `rally` are rally items (5100, z above home). Item numbers are fence items.
pub fn lint(
    items: &[MissionItem],
    home: Option<(f64, f64)>,
    rally: &[MissionItem],
    params: &FenceParams,
) -> Vec<Finding> {
    let mut out = Vec::new();
    let mut add = |seq: Option<usize>, level, text: String| {
        out.push(Finding {
            seq: seq.map(|s| s as u16),
            level,
            text,
        });
    };
    let deg = |i: &MissionItem| (f64::from(i.x) * 1e-7, f64::from(i.y) * 1e-7);
    let mut returns = 0;
    for (k, i) in items.iter().enumerate() {
        let (lat, lon) = deg(i);
        if matches!(i.command, 5000..=5004)
            && (!(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon))
        {
            add(
                Some(k),
                LintLevel::Warning,
                format!("position {lat}, {lon} is invalid"),
            );
        }
        match i.command {
            5003 | 5004 if i.params[0].is_nan() || i.params[0] <= 0.0 => add(
                Some(k),
                LintLevel::Warning,
                format!(
                    "circle radius {} m: ArduPilot refuses the fence",
                    i.params[0]
                ),
            ),
            5000 => {
                returns += 1;
                if returns == 2 {
                    add(
                        Some(k),
                        LintLevel::Warning,
                        "more than one return point: ArduPilot refuses the fence".into(),
                    );
                }
            }
            _ => {}
        }
    }
    let fence = match Fence::from_items(items) {
        Ok(f) => f,
        Err(e) => {
            add(
                None,
                LintLevel::Warning,
                format!("{e}: ArduPilot refuses the fence"),
            );
            return out;
        }
    };
    let mut first = 0usize;
    let mut polys = Vec::new();
    while first < items.len() {
        let i = &items[first];
        if matches!(i.command, 5001 | 5002) {
            let n = (i.params[0] as usize).max(1);
            polys.push((first, &items[first..(first + n).min(items.len())]));
            first += n;
        } else {
            first += 1;
        }
    }
    for (k, vs) in polys {
        let poly: Vec<(f64, f64)> = vs.iter().map(deg).collect();
        if self_intersects(&poly) {
            add(
                Some(k),
                LintLevel::Warning,
                "the polygon's outline crosses itself: inside and outside are not what the drawing suggests"
                    .into(),
            );
        }
    }
    let check = |p: (f64, f64), alt: f64| breach_at(p, &fence, params, p, alt);
    // The circle fence is around home; without home only the zones are checked.
    let zones_only = FenceParams {
        circle_radius_m: None,
        ..*params
    };
    if let Some((k, r)) = items.iter().enumerate().find(|(_, i)| i.command == 5000)
        && let Some(what) = breach_at(deg(r), &fence, &zones_only, deg(r), 0.0)
    {
        add(
            Some(k),
            LintLevel::Warning,
            format!("the return point is {what}"),
        );
    }
    if let Some(h) = home
        && let Some(what) = check(h, 0.0)
    {
        add(
            None,
            LintLevel::Warning,
            format!(
                "home is {what}: arming fails (\"Vehicle breaching\") while the fence is enabled"
            ),
        );
    }
    for (k, r) in rally.iter().enumerate() {
        let p = deg(r);
        let what = match home {
            Some(h) => breach_at(h, &fence, params, p, f64::from(r.z)),
            None => breach_at(p, &fence, &zones_only, p, f64::from(r.z)),
        };
        if let Some(what) = what {
            add(
                None,
                LintLevel::Warning,
                format!("rally point {} is {what}", k + 1),
            );
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn it(command: u16, p1: f32, lat: f64, lon: f64) -> MissionItem {
        MissionItem {
            frame: 0,
            command,
            params: [p1, 0.0, 0.0, 0.0],
            x: (lat * 1e7).round() as i32,
            y: (lon * 1e7).round() as i32,
            z: 0.0,
            autocontinue: true,
        }
    }

    fn pt(seq: u16, lat: f64, lon: f64, alt: f64) -> PathPoint {
        PathPoint {
            seq,
            lat,
            lon,
            alt_rel_m: alt,
        }
    }

    #[test]
    fn a_leg_crossing_out_of_the_polygon_is_found_between_waypoints() {
        // Square inclusion polygon about 222 m on a side around home; an exclusion circle of
        // 20 m inside it.
        let items = [
            it(5001, 4.0, -35.001, 148.999),
            it(5001, 4.0, -35.001, 149.001),
            it(5001, 4.0, -34.999, 149.001),
            it(5001, 4.0, -34.999, 148.999),
            it(5004, 20.0, -35.0, 149.0005),
            it(5000, 0.0, -35.0, 149.0),
        ];
        let fence = Fence::from_items(&items).unwrap();
        assert_eq!(fence.inclusion_polygons.len(), 1);
        assert_eq!(fence.exclusion_circles.len(), 1);
        let p = FenceParams::default();
        // Out to a corner inside, then a leg that clips the outside, then home.
        let ok_path = [pt(0, -35.0, 149.0, 0.0), pt(1, -34.9995, 148.9995, 20.0)];
        assert!(check_path(&ok_path, &fence, &p, 5.0).is_empty());
        let path = [
            pt(0, -35.0, 149.0, 0.0),
            pt(1, -34.9995, 148.9995, 20.0),
            // Point 2 is past the north edge (about 56 m outside).
            pt(2, -34.9985, 149.0, 20.0),
            pt(3, -35.0, 149.0, 0.0),
        ];
        let b = check_path(&path, &fence, &p, 5.0);
        assert_eq!(b.len(), 2, "{b:?}"); // leg to 2 leaves; leg back to home starts outside
        assert_eq!(
            (b[0].seq, b[0].what.as_str()),
            (2, "outside inclusion polygon 1")
        );
        // A leg east through the exclusion circle.
        let east = [pt(0, -35.0, 149.0, 0.0), pt(1, -35.0, 149.0009, 10.0)];
        let b = check_path(&east, &fence, &p, 5.0);
        assert_eq!(b[0].what, "inside exclusion circle 1");
    }

    #[test]
    fn parameter_fences_and_union() {
        let mut params = BTreeMap::new();
        params.insert("FENCE_TYPE".to_string(), 3.0); // alt max + circle
        params.insert("FENCE_RADIUS".to_string(), 100.0);
        let p = FenceParams::from_params(&params);
        assert_eq!(
            (p.circle_radius_m, p.alt_max_m, p.union),
            (Some(100.0), Some(100.0), false)
        );
        let empty = Fence::default();
        let high = [pt(0, -35.0, 149.0, 0.0), pt(1, -35.0, 149.0, 120.0)];
        assert_eq!(
            check_path(&high, &empty, &p, 5.0)[0].what,
            "above FENCE_ALT_MAX 100 m"
        );
        let far = [pt(0, -35.0, 149.0, 0.0), pt(1, -35.002, 149.0, 20.0)];
        assert_eq!(
            check_path(&far, &empty, &p, 5.0)[0].what,
            "beyond FENCE_RADIUS 100 m from home"
        );
        // Two inclusion circles side by side: inside one only is a breach unless union.
        let two = Fence {
            inclusion_circles: vec![((-35.0, 149.0), 50.0), ((-35.0, 149.001), 50.0)],
            ..Default::default()
        };
        let inside_first = [pt(0, -35.0, 149.0, 0.0), pt(1, -35.0, 149.0001, 10.0)];
        let inter = FenceParams::default();
        assert!(!check_path(&inside_first, &two, &inter, 5.0).is_empty());
        let union = FenceParams {
            union: true,
            ..Default::default()
        };
        assert!(check_path(&inside_first, &two, &union, 5.0).is_empty());
        assert!(Fence::from_items(&[it(5001, 4.0, 0.0, 0.0)]).is_err());
    }

    #[test]
    fn items_round_trip() {
        let f = Fence {
            inclusion_polygons: vec![vec![(47.0, 8.0), (47.001, 8.0), (47.001, 8.001)]],
            exclusion_polygons: vec![vec![
                (47.0002, 8.0002),
                (47.0004, 8.0002),
                (47.0004, 8.0004),
                (47.0002, 8.0004),
            ]],
            inclusion_circles: vec![],
            exclusion_circles: vec![((47.0005, 8.0005), 20.0)],
        };
        let items = f.to_items();
        let cmds: Vec<_> = items.iter().map(|i| (i.command, i.params[0])).collect();
        assert_eq!(
            cmds,
            [
                (5001, 3.0),
                (5001, 3.0),
                (5001, 3.0),
                (5002, 4.0),
                (5002, 4.0),
                (5002, 4.0),
                (5002, 4.0),
                (5004, 20.0)
            ]
        );
        let back = Fence::from_items(&items).unwrap();
        assert_eq!(back.exclusion_polygons[0].len(), 4);
        assert!((back.inclusion_polygons[0][1].0 - 47.001).abs() < 1e-7);
        let ((lat, lon), r) = back.exclusion_circles[0];
        assert!((lat - 47.0005).abs() < 1e-7 && (lon - 8.0005).abs() < 1e-7 && r == 20.0);
    }

    #[test]
    fn lint_finds_what_arducopter_refuses_and_what_it_misses() {
        let sq = vec![(47.0, 8.0), (47.002, 8.0), (47.002, 8.003), (47.0, 8.003)];
        let f = Fence {
            inclusion_polygons: vec![sq.clone()],
            exclusion_polygons: vec![vec![
                (47.0005, 8.0005),
                (47.0010, 8.0005),
                (47.0010, 8.0010),
                (47.0005, 8.0010),
            ]],
            ..Default::default()
        };
        let items = f.to_items();
        let p = FenceParams::default();
        let inside = (47.001, 8.002);
        assert_eq!(lint(&items, Some(inside), &[], &p), []);
        // Home in the no-fly square; a rally point outside the field.
        let rally = |lat: f64, lon: f64| MissionItem {
            frame: 3,
            command: 5100,
            params: [0.0; 4],
            x: (lat * 1e7) as i32,
            y: (lon * 1e7) as i32,
            z: 20.0,
            autocontinue: true,
        };
        let l = lint(&items, Some((47.0007, 8.0007)), &[rally(47.01, 8.0)], &p);
        assert_eq!(l.len(), 2, "{l:#?}");
        assert!(l[0].text.starts_with("home is inside exclusion polygon 1"));
        assert!(
            l[1].text
                .starts_with("rally point 1 is outside inclusion polygon 1")
        );
        // A bow-tie, a zero radius and two return points.
        let bow = Fence {
            inclusion_polygons: vec![vec![sq[0], sq[2], sq[1], sq[3]]],
            exclusion_circles: vec![((47.001, 8.001), 0.0)],
            ..Default::default()
        };
        let mut items = bow.to_items();
        let ret = MissionItem {
            command: 5000,
            ..items[0]
        };
        items.extend([ret, ret]);
        let l = lint(&items, None, &[], &p);
        let seqs: Vec<_> = l.iter().map(|f| f.seq).collect();
        assert_eq!(seqs, [Some(4), Some(6), Some(0)], "{l:#?}");
        assert!(l[2].text.contains("crosses itself"));
        assert!(self_intersects(&[sq[0], sq[2], sq[1], sq[3]]) && !self_intersects(&sq));
        // A broken polygon is reported once, as ArduPilot would refuse it.
        let l = lint(&f.to_items()[1..], None, &[], &p);
        assert!(l.iter().any(|f| f.text.contains("refuses")), "{l:#?}");
    }
}
