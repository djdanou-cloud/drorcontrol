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

/// A set of tiles.
#[derive(Debug, Clone, Default)]
pub struct Terrain {
    tiles: HashMap<(i32, i32), HgtTile>,
}

impl Terrain {
    /// Add a tile (replacing one for the same cell).
    pub fn add(&mut self, t: HgtTile) {
        self.tiles.insert((t.lat0, t.lon0), t);
    }

    /// Number of tiles.
    pub fn len(&self) -> usize {
        self.tiles.len()
    }

    /// No tiles.
    pub fn is_empty(&self) -> bool {
        self.tiles.is_empty()
    }

    /// Elevation at (lat, lon); `None` without a tile there or over a void.
    pub fn elevation(&self, lat: f64, lon: f64) -> Option<f64> {
        self.tiles
            .get(&(lat.floor() as i32, lon.floor() as i32))?
            .elevation(lat, lon)
    }
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
}
