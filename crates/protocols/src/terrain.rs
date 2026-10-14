//! Ground elevation from SRTM `.hgt` tiles (plan 01 §5.3), offline: the files are read from
//! disk by the caller, nothing is downloaded.
//!
//! The format (NASA SRTM v3 / the void-filled 1″ and 3″ releases): one tile per 1°×1° cell,
//! named after its south-west corner (`S36E149.hgt` covers 36°S–35°S, 149°E–150°E); a square
//! grid of big-endian `i16` metres above the EGM96 geoid, 3601×3601 (1″) or 1201×1201 (3″),
//! rows from north to south, the edge rows and columns shared with the neighbouring tiles;
//! −32768 marks a void. Elevations are sampled bilinearly; a void corner is left out and the
//! others re-weighted (a point on a void takes the mean of the valid corners), and a point
//! with all four corners void has no elevation.

use std::collections::HashMap;

use crate::mission::{self, MissionItem};
use crate::survey::Plane;

/// Why a tile cannot be read.
#[derive(Debug, Clone, PartialEq)]
pub enum TerrainError {
    /// The file name is not like `N35W120.hgt`.
    BadName(String),
    /// The size is not a square grid of 2-byte samples.
    BadSize(usize),
}

impl std::fmt::Display for TerrainError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TerrainError::BadName(n) => write!(f, "{n}: not an SRTM tile name like N35W120.hgt"),
            TerrainError::BadSize(s) => write!(f, "{s} bytes is not a square SRTM grid"),
        }
    }
}

const VOID: i16 = i16::MIN;

/// One 1°×1° tile.
#[derive(Debug, Clone, PartialEq)]
pub struct HgtTile {
    /// South-west corner, whole degrees.
    pub lat0: i32,
    /// See `lat0`.
    pub lon0: i32,
    /// Samples per side (3601 or 1201).
    pub size: usize,
    data: Vec<i16>,
}

/// The south-west corner named by a tile file name (`N35W120.hgt`, any case, any directory).
pub fn tile_origin(file_name: &str) -> Option<(i32, i32)> {
    let base = file_name.rsplit(['/', '\\']).next()?;
    let stem = base.get(..7)?.to_ascii_uppercase();
    if !base.get(7..)?.eq_ignore_ascii_case(".hgt") {
        return None;
    }
    let b = stem.as_bytes();
    let sign = |c: u8, pos: u8, neg: u8| match c {
        c if c == pos => Some(1),
        c if c == neg => Some(-1),
        _ => None,
    };
    let lat: i32 = stem.get(1..3)?.parse().ok()?;
    let lon: i32 = stem.get(4..7)?.parse().ok()?;
    let (la, lo) = (sign(b[0], b'N', b'S')? * lat, sign(b[3], b'E', b'W')? * lon);
    ((-90..90).contains(&la) && (-180..180).contains(&lo)).then_some((la, lo))
}

/// The tile file name for the cell holding (lat, lon).
pub fn tile_name(lat: f64, lon: f64) -> String {
    let (la, lo) = (lat.floor() as i32, lon.floor() as i32);
    format!(
        "{}{:02}{}{:03}.hgt",
        if la < 0 { 'S' } else { 'N' },
        la.unsigned_abs(),
        if lo < 0 { 'W' } else { 'E' },
        lo.unsigned_abs()
    )
}

impl HgtTile {
    /// A tile from its file name and contents.
    pub fn from_bytes(file_name: &str, bytes: &[u8]) -> Result<HgtTile, TerrainError> {
        let (lat0, lon0) =
            tile_origin(file_name).ok_or_else(|| TerrainError::BadName(file_name.into()))?;
        let samples = bytes.len() / 2;
        let size = (samples as f64).sqrt().round() as usize;
        if !bytes.len().is_multiple_of(2) || size < 2 || size * size != samples {
            return Err(TerrainError::BadSize(bytes.len()));
        }
        let data = bytes
            .chunks_exact(2)
            .map(|b| i16::from_be_bytes([b[0], b[1]]))
            .collect();
        Ok(HgtTile {
            lat0,
            lon0,
            size,
            data,
        })
    }

    fn at(&self, row: usize, col: usize) -> Option<f64> {
        let v = self.data[row * self.size + col];
        (v != VOID).then_some(f64::from(v))
    }

    /// Elevation at (lat, lon), m above the geoid; `None` outside the tile or over a void.
    pub fn elevation(&self, lat: f64, lon: f64) -> Option<f64> {
        let last = (self.size - 1) as f64;
        let r = (f64::from(self.lat0) + 1.0 - lat) * last;
        let c = (lon - f64::from(self.lon0)) * last;
        if !(0.0..=last).contains(&r) || !(0.0..=last).contains(&c) {
            return None;
        }
        let (r0, c0) = (r.floor().min(last - 1.0), c.floor().min(last - 1.0));
        let (fr, fc) = (r - r0, c - c0);
        let (r0, c0) = (r0 as usize, c0 as usize);
        let (mut sum, mut weight, mut plain, mut valid) = (0.0, 0.0, 0.0, 0);
        for (dr, dc, w) in [
            (0, 0, (1.0 - fr) * (1.0 - fc)),
            (0, 1, (1.0 - fr) * fc),
            (1, 0, fr * (1.0 - fc)),
            (1, 1, fr * fc),
        ] {
            if let Some(v) = self.at(r0 + dr, c0 + dc) {
                sum += v * w;
                weight += w;
                plain += v;
                valid += 1;
            }
        }
        if weight > 1e-6 {
            Some(sum / weight)
        } else {
            // On (or next to) a void sample: the other corners alike.
            (valid > 0).then(|| plain / f64::from(valid))
        }
    }
}

/// A georeferenced elevation grid of any extent (from a GeoTIFF, e.g. Copernicus GLO-30
/// or an SRTM GeoTIFF): samples on a regular latitude/longitude grid (WGS-84), rows from
/// north to south. Sampled like the `.hgt` tiles (bilinear, nodata corners left out).
#[derive(Debug, Clone, PartialEq)]
pub struct GeoGrid {
    /// Latitude of the first row's sample centres, degrees.
    pub north: f64,
    /// Longitude of the first column's sample centres, degrees.
    pub west: f64,
    /// Spacing between rows (positive, going south) and columns, degrees.
    pub step: (f64, f64),
    /// Columns and rows.
    pub size: (usize, usize),
    data: Vec<f32>,
    nodata: Option<f32>,
}

impl GeoGrid {
    /// A grid from its samples (row-major, north first). `None` if `data` does not match
    /// `size` or a step is not positive.
    pub fn new(
        north: f64,
        west: f64,
        step: (f64, f64),
        size: (usize, usize),
        data: Vec<f32>,
        nodata: Option<f32>,
    ) -> Option<GeoGrid> {
        (data.len() == size.0 * size.1
            && step.0 > 0.0
            && step.1 > 0.0
            && size.0 >= 2
            && size.1 >= 2)
            .then_some(GeoGrid {
                north,
                west,
                step,
                size,
                data,
                nodata,
            })
    }

    /// Elevation at (lat, lon); `None` outside the grid or over nodata.
    pub fn elevation(&self, lat: f64, lon: f64) -> Option<f64> {
        let (cols, rows) = self.size;
        // A point on the edge can land a rounding error outside it: allow 1e-6 of a step.
        const EDGE: f64 = 1e-6;
        let fr = (self.north - lat) / self.step.0;
        let fc = (lon - self.west) / self.step.1;
        let (rmax, cmax) = ((rows - 1) as f64, (cols - 1) as f64);
        if !(-EDGE..=rmax + EDGE).contains(&fr) || !(-EDGE..=cmax + EDGE).contains(&fc) {
            return None;
        }
        let (fr, fc) = (fr.clamp(0.0, rmax), fc.clamp(0.0, cmax));
        let (r0, c0) = (
            (fr.floor() as usize).min(rows - 2),
            (fc.floor() as usize).min(cols - 2),
        );
        let (dr, dc) = (fr - r0 as f64, fc - c0 as f64);
        let mut sum = 0.0;
        let mut weight = 0.0;
        for (r, c, w) in [
            (r0, c0, (1.0 - dr) * (1.0 - dc)),
            (r0, c0 + 1, (1.0 - dr) * dc),
            (r0 + 1, c0, dr * (1.0 - dc)),
            (r0 + 1, c0 + 1, dr * dc),
        ] {
            let v = self.data[r * cols + c];
            if v.is_nan() || self.nodata.is_some_and(|n| v == n) {
                continue;
            }
            sum += f64::from(v) * w;
            weight += w;
        }
        // A point exactly on valid corners has weight; on a void it takes the valid mean.
        if weight > 1e-12 {
            Some(sum / weight)
        } else {
            let valid: Vec<f64> = [(r0, c0), (r0, c0 + 1), (r0 + 1, c0), (r0 + 1, c0 + 1)]
                .iter()
                .map(|&(r, c)| self.data[r * cols + c])
                .filter(|v| !v.is_nan() && self.nodata.is_none_or(|n| *v != n))
                .map(f64::from)
                .collect();
            (!valid.is_empty()).then(|| valid.iter().sum::<f64>() / valid.len() as f64)
        }
    }
}

/// A set of tiles.
#[derive(Debug, Clone, Default)]
pub struct Terrain {
    tiles: HashMap<(i32, i32), HgtTile>,
    grids: Vec<GeoGrid>,
}

impl Terrain {
    /// Add a tile (replacing one for the same cell).
    pub fn add(&mut self, t: HgtTile) {
        self.tiles.insert((t.lat0, t.lon0), t);
    }

    /// Add a georeferenced grid (searched after the `.hgt` tiles, in the order added).
    pub fn add_grid(&mut self, g: GeoGrid) {
        self.grids.push(g);
    }

    /// Number of tiles and grids.
    pub fn len(&self) -> usize {
        self.tiles.len() + self.grids.len()
    }

    /// No tiles or grids.
    pub fn is_empty(&self) -> bool {
        self.tiles.is_empty() && self.grids.is_empty()
    }

    /// Elevation at (lat, lon); `None` without a tile or grid there or over a void.
    pub fn elevation(&self, lat: f64, lon: f64) -> Option<f64> {
        self.tiles
            .get(&(lat.floor() as i32, lon.floor() as i32))
            .and_then(|t| t.elevation(lat, lon))
            .or_else(|| self.grids.iter().find_map(|g| g.elevation(lat, lon)))
    }
}

/// Lowest clearance over terrain on one leg of a mission.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LegClearance {
    /// Item the leg flies to.
    pub seq: u16,
    /// Lowest height above the ground on the leg, m.
    pub min_m: f64,
    /// Where (latitude, longitude).
    pub at: (f64, f64),
    /// Ground elevation there, m.
    pub ground_m: f64,
}

/// Terrain clearance along a mission.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ClearanceReport {
    /// Ground elevation at home, m (`None`: no data there, nothing checked).
    pub home_ground_m: Option<f64>,
    /// Every horizontal leg with terrain data.
    pub legs: Vec<LegClearance>,
    /// Samples without terrain data (missing tile or void).
    pub no_data: usize,
}

fn is_terrain_frame(frame: u8) -> bool {
    matches!(frame, 10 | 11)
}

/// Points every `step_m` along the mission's path (for loading the tiles it crosses).
pub fn sample_points(items: &[MissionItem], step_m: f64) -> Vec<(f64, f64)> {
    let path = mission::path(items);
    let mut out: Vec<(f64, f64)> = path.iter().map(|p| (p.lat, p.lon)).collect();
    for w in path.windows(2) {
        out.extend(leg_samples((w[0].lat, w[0].lon), (w[1].lat, w[1].lon), step_m).map(|s| s.1));
    }
    out
}

/// (fraction, point) every `step_m` from a to b, both ends included; nothing for a leg
/// under 1 m long (straight up or down).
fn leg_samples(
    a: (f64, f64),
    b: (f64, f64),
    step_m: f64,
) -> impl Iterator<Item = (f64, (f64, f64))> {
    let plane = Plane::new(a);
    let d = {
        let (x, y) = plane.to_xy(b);
        x.hypot(y)
    };
    let n = if d < 1.0 {
        0
    } else {
        (d / step_m.max(1.0)).ceil().max(1.0) as usize
    };
    let count = if n == 0 { 0 } else { n + 1 };
    (0..count).map(move |k| {
        let f = k as f64 / n as f64;
        (f, (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f))
    })
}

/// The mission's height above the ground along every horizontal leg, sampled every
/// `step_m`. Heights above home are taken from the ground at home (the vehicle starts
/// there); a leg to an item in a terrain frame (10, 11) is flown at its height above the
/// ground, blended from the leg's start, as ArduPilot does with terrain data. Climbs and
/// descents straight up or down (takeoff, landing) are not legs.
pub fn clearance(items: &[MissionItem], t: &Terrain, step_m: f64) -> ClearanceReport {
    let path = mission::path(items);
    let Some(home_ground) = path.first().and_then(|h| t.elevation(h.lat, h.lon)) else {
        return ClearanceReport::default();
    };
    let mut r = ClearanceReport {
        home_ground_m: Some(home_ground),
        ..Default::default()
    };
    let frame = |seq: u16| items.get(usize::from(seq)).map_or(0, |i| i.frame);
    // Height above the ground at a path point (terrain frames as they are).
    let agl = |p: &mission::PathPoint, ground: f64| {
        if p.seq != 0 && is_terrain_frame(frame(p.seq)) {
            p.alt_rel_m
        } else {
            home_ground + p.alt_rel_m - ground
        }
    };
    for w in path.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        let (Some(ga), Some(gb)) = (t.elevation(a.lat, a.lon), t.elevation(b.lat, b.lon)) else {
            r.no_data += 1;
            continue;
        };
        let (agl_a, agl_b) = (agl(a, ga), agl(b, gb));
        let (msl_a, msl_b) = (ga + agl_a, gb + agl_b);
        let follows = is_terrain_frame(frame(b.seq));
        let mut low: Option<LegClearance> = None;
        for (f, (lat, lon)) in leg_samples((a.lat, a.lon), (b.lat, b.lon), step_m) {
            let Some(g) = t.elevation(lat, lon) else {
                r.no_data += 1;
                continue;
            };
            let c = if follows {
                agl_a + (agl_b - agl_a) * f
            } else {
                msl_a + (msl_b - msl_a) * f - g
            };
            if low.is_none_or(|l| c < l.min_m) {
                low = Some(LegClearance {
                    seq: b.seq,
                    min_m: c,
                    at: (lat, lon),
                    ground_m: g,
                });
            }
        }
        r.legs.extend(low);
    }
    r
}

/// One sample of a mission's terrain profile.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProfilePoint {
    /// Item the leg flies to (0: home).
    pub seq: u16,
    /// Horizontal distance flown so far, m.
    pub dist_m: f64,
    /// Position.
    pub lat: f64,
    /// See `lat`.
    pub lon: f64,
    /// Ground elevation, m (`None`: no data).
    pub ground_m: Option<f64>,
    /// The vehicle's height above home, m.
    pub alt_rel_m: f64,
}

/// The mission's altitude against the ground, sampled every `step_m` along each horizontal
/// leg (plus every path corner), for a profile view: heights as `clearance` reckons them
/// (terrain-frame legs blended above the ground). Without terrain at home the heights are
/// still given, the ground is not.
pub fn profile(items: &[MissionItem], t: &Terrain, step_m: f64) -> Vec<ProfilePoint> {
    let path = mission::path(items);
    let Some(h) = path.first() else {
        return Vec::new();
    };
    let home_ground = t.elevation(h.lat, h.lon);
    let frame = |seq: u16| items.get(usize::from(seq)).map_or(0, |i| i.frame);
    let mut out = vec![ProfilePoint {
        seq: 0,
        dist_m: 0.0,
        lat: h.lat,
        lon: h.lon,
        ground_m: home_ground,
        alt_rel_m: 0.0,
    }];
    let mut dist = 0.0;
    for w in path.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        let follows = is_terrain_frame(frame(b.seq));
        let ga = t.elevation(a.lat, a.lon);
        let gb = t.elevation(b.lat, b.lon);
        // Above home at each end; a terrain-frame end is converted with the ground there.
        let rel = |p: &mission::PathPoint, g: Option<f64>| match (home_ground, g) {
            (Some(hg), Some(g)) if p.seq != 0 && is_terrain_frame(frame(p.seq)) => {
                g + p.alt_rel_m - hg
            }
            _ => p.alt_rel_m,
        };
        let (ra, rb) = (rel(a, ga), rel(b, gb));
        let samples: Vec<_> = leg_samples((a.lat, a.lon), (b.lat, b.lon), step_m).collect();
        if samples.is_empty() {
            // Straight up or down: one point at the new height.
            out.push(ProfilePoint {
                seq: b.seq,
                dist_m: dist,
                lat: b.lat,
                lon: b.lon,
                ground_m: gb,
                alt_rel_m: rb,
            });
            continue;
        }
        let plane = Plane::new((a.lat, a.lon));
        let (x, y) = plane.to_xy((b.lat, b.lon));
        let len = x.hypot(y);
        for &(f, (lat, lon)) in samples.iter().skip(1) {
            let g = t.elevation(lat, lon);
            let alt = match (follows, home_ground, g, ga) {
                (true, Some(hg), Some(g), Some(ga)) => {
                    let (aa, ab) = (ra + hg - ga, b.alt_rel_m);
                    g + aa + (ab - aa) * f - hg
                }
                _ => ra + (rb - ra) * f,
            };
            out.push(ProfilePoint {
                seq: b.seq,
                dist_m: dist + len * f,
                lat,
                lon,
                ground_m: g,
                alt_rel_m: alt,
            });
        }
        dist += len;
    }
    out
}

/// Why a mission cannot be made to follow the terrain.
#[derive(Debug, Clone, PartialEq)]
pub enum FollowError {
    /// No elevation at home.
    NoHomeData,
    /// No elevation at this item (or at a point added before it).
    NoData(u16),
    /// Adding waypoints would move the targets of `DO_JUMP` (177).
    HasJump,
}

impl std::fmt::Display for FollowError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FollowError::NoHomeData => f.write_str("no terrain data at home"),
            FollowError::NoData(s) => write!(f, "no terrain data on the way to item {s}"),
            FollowError::HasJump => {
                f.write_str("the mission has DO_JUMP; adding waypoints would move its targets")
            }
        }
    }
}

/// Terrain following without terrain data on the vehicle: every `NAV_WAYPOINT` above home
/// (frame 3) is set to `agl_m` above the ground under it (ground − ground at home + agl).
/// With `step_m` > 0, legs between such waypoints get extra waypoints every `step_m` so the
/// height follows the ground in between. Other items are kept as they are.
pub fn follow(
    items: &[MissionItem],
    t: &Terrain,
    agl_m: f32,
    step_m: f64,
) -> Result<Vec<MissionItem>, FollowError> {
    let Some(home) = items.first() else {
        return Ok(Vec::new());
    };
    let deg = |i: &MissionItem| (f64::from(i.x) * 1e-7, f64::from(i.y) * 1e-7);
    let (hlat, hlon) = deg(home);
    let home_ground = t.elevation(hlat, hlon).ok_or(FollowError::NoHomeData)?;
    if step_m > 0.0 && items.iter().any(|i| i.command == 177) {
        return Err(FollowError::HasJump);
    }
    let height = |seq: u16, (lat, lon): (f64, f64)| {
        t.elevation(lat, lon)
            .map(|g| (g - home_ground + f64::from(agl_m)) as f32)
            .ok_or(FollowError::NoData(seq))
    };
    let mut out = vec![*home];
    let mut prev: Option<(f64, f64)> = None;
    for (seq, i) in items.iter().enumerate().skip(1) {
        let seq = seq as u16;
        let located = i.x != 0 || i.y != 0;
        if i.command == 16 && i.frame == 3 && located {
            let at = deg(i);
            if let Some(p) = prev.filter(|_| step_m > 0.0) {
                let samples: Vec<_> = leg_samples(p, at, step_m).collect();
                for &(_, q) in samples.iter().skip(1).take(samples.len().saturating_sub(2)) {
                    out.push(MissionItem {
                        x: (q.0 * 1e7).round() as i32,
                        y: (q.1 * 1e7).round() as i32,
                        z: height(seq, q)?,
                        params: [0.0; 4],
                        ..*i
                    });
                }
            }
            out.push(MissionItem {
                z: height(seq, at)?,
                ..*i
            });
            prev = Some(at);
        } else {
            if located && mission::has_location(i.command) {
                prev = None;
            }
            out.push(*i);
        }
    }
    Ok(out)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A synthetic 3″ tile: elevation rises 1 m per sample eastward from `base`, with a void
    /// at row 10, column 10.
    pub(crate) fn ramp(name: &str, base: i16) -> HgtTile {
        let n = 1201;
        let mut bytes = Vec::with_capacity(n * n * 2);
        for r in 0..n {
            for c in 0..n {
                let v = if (r, c) == (10, 10) {
                    VOID
                } else {
                    base + (c as i16 / 4)
                };
                bytes.extend_from_slice(&v.to_be_bytes());
            }
        }
        HgtTile::from_bytes(name, &bytes).unwrap()
    }

    #[test]
    fn names_and_corners() {
        assert_eq!(tile_origin("S36E149.hgt"), Some((-36, 149)));
        assert_eq!(tile_origin("dem/n35w120.HGT"), Some((35, -120)));
        assert_eq!(tile_origin("X35W120.hgt"), None);
        assert_eq!(tile_origin("N35W120.tif"), None);
        assert_eq!(tile_name(-35.36, 149.16), "S36E149.hgt");
        assert_eq!(tile_name(35.5, -119.2), "N35W120.hgt");
        assert_eq!(tile_name(0.5, 0.5), "N00E000.hgt");
        assert_eq!(
            HgtTile::from_bytes("N00E000.hgt", &[0; 6]),
            Err(TerrainError::BadSize(6))
        );
    }

    #[test]
    fn bilinear_with_voids_left_out() {
        let t = ramp("S36E149.hgt", 500);
        let step = 1.0 / 1200.0;
        // Column 400 → 600 m (base + 400/4), at any row.
        assert_eq!(t.elevation(-35.5, 149.0 + 400.0 * step), Some(600.0));
        // Half-way between columns 2 and 3 (500 and 500), and between 3 and 4 (500, 501).
        let e = t.elevation(-35.5, 149.0 + 3.5 * step).unwrap();
        assert!((e - 500.5).abs() < 1e-6, "{e}");
        // Next to the void the remaining corners are used; on it exactly, its neighbours.
        let v = t
            .elevation(-35.0 - 10.0 * step, 149.0 + 10.0 * step)
            .unwrap();
        assert!((v - 502.0).abs() < 1e-6, "{v}");
        // The corners and edges belong to the tile; outside does not.
        assert_eq!(t.elevation(-36.0, 150.0), Some(800.0));
        assert_eq!(t.elevation(-35.0, 149.0), Some(500.0));
        assert_eq!(t.elevation(-36.0001, 149.5), None);
        let mut set = Terrain::default();
        set.add(t);
        assert_eq!(set.elevation(-35.5, 149.0 + 400.0 * step), Some(600.0));
        assert_eq!(set.elevation(-34.5, 149.5), None, "no tile");
    }

    #[test]
    fn clearance_over_rising_ground() {
        let mut t = Terrain::default();
        t.add(ramp("S36E149.hgt", 500));
        let step = 1.0 / 1200.0;
        let lat: f64 = -35.5;
        let (lon_home, lon_far) = (149.0 + 120.0 * step, 149.0 + 240.0 * step);
        // Ground: 530 m at home, rising east to 560 m.
        let wp = |frame: u8, command: u16, lon: f64, z: f32| MissionItem {
            frame,
            command,
            params: [0.0; 4],
            x: (lat * 1e7).round() as i32,
            y: (lon * 1e7).round() as i32,
            z,
            autocontinue: true,
        };
        let m = vec![
            wp(0, 16, lon_home, 0.0),
            wp(3, 22, 0.0, 40.0),
            wp(3, 16, lon_far, 40.0),
            wp(2, 20, 0.0, 0.0),
        ];
        let r = clearance(&m, &t, 10.0);
        assert!((r.home_ground_m.unwrap() - 530.0).abs() < 0.01);
        assert_eq!(r.no_data, 0);
        // Out at 570 m MSL over ground rising to 560: 10 m at the far end; RTL back the same.
        let out = r.legs.iter().find(|l| l.seq == 2).unwrap();
        assert!((out.min_m - 10.0).abs() < 0.2, "{out:?}");
        assert!((out.ground_m - 560.0).abs() < 0.2);
        assert_eq!(
            r.legs.len(),
            2,
            "out and back; takeoff and landing are vertical"
        );
        // The same waypoint in the terrain frame keeps 40 m over the ground.
        let mut tf = m.clone();
        tf[2].frame = 10;
        let r = clearance(&tf, &t, 10.0);
        let out = r.legs.iter().find(|l| l.seq == 2).unwrap();
        assert!((out.min_m - 40.0).abs() < 0.2, "{out:?}");
        // No tile at home: nothing checked.
        let mut away = m.clone();
        away[0].x = -340_000_000;
        assert_eq!(clearance(&away, &t, 10.0), ClearanceReport::default());
        assert!(sample_points(&m, 10.0).len() > 20);
    }

    #[test]
    fn following_sets_heights_from_the_ground() {
        let mut t = Terrain::default();
        t.add(ramp("S36E149.hgt", 500));
        let step = 1.0 / 1200.0;
        let lat: f64 = -35.5;
        let wp = |frame: u8, command: u16, lon: f64, z: f32| MissionItem {
            frame,
            command,
            params: [0.0; 4],
            x: (lat * 1e7).round() as i32,
            y: if lon == 0.0 {
                0
            } else {
                (lon * 1e7).round() as i32
            },
            z,
            autocontinue: true,
        };
        // Home on 530 m ground, waypoints over 545 m and 560 m.
        let m = vec![
            wp(0, 16, 149.0 + 120.0 * step, 0.0),
            wp(3, 22, 0.0, 40.0),
            wp(3, 16, 149.0 + 180.0 * step, 99.0),
            wp(3, 16, 149.0 + 240.0 * step, 99.0),
            wp(2, 20, 0.0, 0.0),
        ];
        let f = follow(&m, &t, 40.0, 0.0).unwrap();
        assert_eq!(f.len(), m.len());
        assert!(
            (f[2].z - 55.0).abs() < 0.1 && (f[3].z - 70.0).abs() < 0.1,
            "{f:?}"
        );
        assert_eq!((f[1], f[4]), (m[1], m[4]), "takeoff and RTL kept");
        // The two waypoints are 60 samples (about 4.5 km) apart: with a point every 500 m
        // in between, the whole leg stays 40 m up.
        let d = follow(&m, &t, 40.0, 500.0).unwrap();
        assert!(d.len() > m.len(), "{}", d.len());
        let r = clearance(&d, &t, 10.0);
        let low = r.legs.iter().map(|l| l.min_m).fold(f64::MAX, f64::min);
        assert!(low > 39.0, "{low}");
        let mut jump = m.clone();
        jump.insert(4, wp(2, 177, 0.0, 0.0));
        assert_eq!(follow(&jump, &t, 40.0, 50.0), Err(FollowError::HasJump));
        assert!(follow(&jump, &t, 40.0, 0.0).is_ok());
    }

    #[test]
    fn profile_runs_along_the_path() {
        let mut t = Terrain::default();
        t.add(ramp("S36E149.hgt", 500));
        let step = 1.0 / 1200.0;
        let lat: f64 = -35.5;
        let wp = |frame: u8, command: u16, lon: f64, z: f32| MissionItem {
            frame,
            command,
            params: [0.0; 4],
            x: (lat * 1e7).round() as i32,
            y: if lon == 0.0 {
                0
            } else {
                (lon * 1e7).round() as i32
            },
            z,
            autocontinue: true,
        };
        let m = vec![
            wp(0, 16, 149.0 + 120.0 * step, 0.0),
            wp(3, 22, 0.0, 40.0),
            wp(10, 16, 149.0 + 240.0 * step, 40.0),
            wp(2, 20, 0.0, 0.0),
        ];
        let p = profile(&m, &t, 100.0);
        // Home, takeoff (vertical), the leg out, RTL back, landing.
        assert_eq!(p[0].alt_rel_m, 0.0);
        assert_eq!((p[1].seq, p[1].alt_rel_m), (1, 40.0));
        let out: Vec<_> = p.iter().filter(|q| q.seq == 2).collect();
        assert!(out.len() > 3);
        for q in &out {
            // Terrain frame: 40 m over the ground all the way (ground − home ground + 40).
            let above = q.alt_rel_m + 530.0 - q.ground_m.unwrap();
            assert!((above - 40.0).abs() < 0.3, "{q:?}");
        }
        let last = out.last().unwrap();
        assert!(
            (last.dist_m - 9071.0).abs() < 100.0,
            "about 0.1° of longitude: {last:?}"
        );
        assert!(p.windows(2).all(|w| w[1].dist_m >= w[0].dist_m));
        assert_eq!(p.last().unwrap().alt_rel_m, 0.0, "landed");
    }

    #[test]
    fn geogrid_bilinear_and_nodata() {
        // 3×3 samples, 0.01° apart, NW corner sample at (-35.0, 149.0).
        let g = GeoGrid::new(
            -35.0,
            149.0,
            (0.01, 0.01),
            (3, 3),
            vec![
                100.0, 110.0, 120.0, 200.0, 210.0, -9999.0, 300.0, 310.0, 320.0,
            ],
            Some(-9999.0),
        )
        .unwrap();
        let near = |a: Option<f64>, b: f64| a.is_some_and(|a| (a - b).abs() < 1e-6);
        assert!(near(g.elevation(-35.0, 149.0), 100.0));
        assert!(near(g.elevation(-35.01, 149.01), 210.0));
        let mid = g.elevation(-35.005, 149.005).unwrap();
        assert!((mid - 155.0).abs() < 1e-9, "{mid}");
        // Next to the nodata sample: the valid corners re-weighted.
        let near_void = g.elevation(-35.01, 149.015).unwrap();
        assert!((near_void - 210.0).abs() < 1e-9, "{near_void}");
        assert_eq!(g.elevation(-34.99, 149.0), None, "north of the grid");
        let mut t = Terrain::default();
        t.add_grid(g);
        assert_eq!(t.len(), 1);
        assert!(near(t.elevation(-35.02, 149.02), 320.0));
    }
}
