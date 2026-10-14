//! Elevation GeoTIFFs (e.g. Copernicus GLO-30, SRTM GeoTIFF) as terrain grids: one band of
//! heights on a WGS-84 latitude/longitude grid (EPSG:4326), any sample type, Deflate or LZW,
//! strips or tiles. The georeference comes from `ModelPixelScale` and `ModelTiepoint`; the
//! raster type (GeoKey 1025) says whether the tie point is a pixel's corner (area, GDAL's
//! default) or its centre (point, Copernicus). `GDAL_NODATA` marks voids.

use std::io::{Read, Seek};

use protocols::terrain::GeoGrid;
use tiff::decoder::{Decoder, DecodingResult};
use tiff::tags::Tag;

/// `GTModelTypeGeoKey`, `GTRasterTypeGeoKey`, `GeographicTypeGeoKey`.
const KEY_MODEL: u16 = 1024;
const KEY_RASTER: u16 = 1025;
const KEY_GEOGRAPHIC: u16 = 2048;
/// `ModelTypeGeographic`, `RasterPixelIsPoint`, WGS 84.
const MODEL_GEOGRAPHIC: u16 = 2;
const RASTER_POINT: u16 = 2;
const EPSG_WGS84: u16 = 4326;

/// Read an elevation GeoTIFF.
pub fn read<R: Read + Seek>(r: R) -> Result<GeoGrid, String> {
    let mut d = Decoder::new(r).map_err(|e| format!("not a TIFF: {e}"))?;
    let (w, h) = d.dimensions().map_err(|e| e.to_string())?;
    let scale = d
        .get_tag(Tag::ModelPixelScaleTag)
        .and_then(|v| v.into_f64_vec())
        .map_err(|_| "no ModelPixelScale: not a GeoTIFF".to_string())?;
    let tie = d
        .get_tag(Tag::ModelTiepointTag)
        .and_then(|v| v.into_f64_vec())
        .map_err(|_| "no ModelTiepoint: not a GeoTIFF".to_string())?;
    if scale.len() < 2 || tie.len() < 6 {
        return Err("malformed ModelPixelScale/ModelTiepoint".into());
    }
    let keys: Vec<u16> = d
        .get_tag(Tag::GeoKeyDirectoryTag)
        .and_then(|v| v.into_u32_vec())
        .map(|v| v.into_iter().map(|k| k as u16).collect())
        .map_err(|_| "no GeoKeyDirectory: not a GeoTIFF".to_string())?;
    let key = |id: u16| {
        keys.get(4..)
            .unwrap_or_default()
            .chunks_exact(4)
            .find(|k| k[0] == id && k[1] == 0)
            .map(|k| k[3])
    };
    if key(KEY_MODEL) != Some(MODEL_GEOGRAPHIC) {
        return Err(
            "not on a latitude/longitude grid (GTModelType is not geographic): \
             reproject to EPSG:4326 first"
                .into(),
        );
    }
    if let Some(g) = key(KEY_GEOGRAPHIC)
        && g != EPSG_WGS84
    {
        return Err(format!("geographic CRS EPSG:{g}, not WGS 84 (EPSG:4326)"));
    }
    let nodata = d
        .get_tag_ascii_string(Tag::GdalNodata)
        .ok()
        .and_then(|s| s.trim_matches(char::from(0)).trim().parse::<f32>().ok());
    let data: Vec<f32> = match d.read_image().map_err(|e| e.to_string())? {
        DecodingResult::F32(v) => v,
        DecodingResult::F64(v) => v.into_iter().map(|x| x as f32).collect(),
        DecodingResult::I16(v) => v.into_iter().map(f32::from).collect(),
        DecodingResult::U16(v) => v.into_iter().map(f32::from).collect(),
        DecodingResult::I32(v) => v.into_iter().map(|x| x as f32).collect(),
        DecodingResult::U8(v) => v.into_iter().map(f32::from).collect(),
        _ => return Err("unsupported sample type".into()),
    };
    if data.len() != w as usize * h as usize {
        return Err(format!(
            "{} samples for {w}×{h}: one band of heights expected",
            data.len()
        ));
    }
    // Tie point (raster i, j) ↔ (lon, lat); for PixelIsArea it is a pixel corner.
    let (sx, sy) = (scale[0], scale[1]);
    let half = if key(KEY_RASTER) == Some(RASTER_POINT) {
        0.0
    } else {
        0.5
    };
    let west = tie[3] - tie[0] * sx + half * sx;
    let north = tie[4] + tie[1] * sy - half * sy;
    GeoGrid::new(
        north,
        west,
        (sy, sx),
        (w as usize, h as usize),
        data,
        nodata,
    )
    .ok_or_else(|| "empty or one-sample-wide grid".into())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(name: &str) -> GeoGrid {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/data/dem")
            .join(name);
        read(std::fs::File::open(p).unwrap()).unwrap()
    }

    /// Fixtures made with GDAL 3.8.4 from one 20×15 grid (height 500 + 3·column + 7·row,
    /// 0.001° cells, north-west corner 149.16°E 35.355°S, one void at row 7 column 9):
    /// Copernicus-like Float32/Deflate/float predictor/16×16 tiles/PixelIsPoint, and
    /// Int16/LZW/strips/PixelIsArea. Values checked against `gdallocationinfo -wgs84`.
    #[test]
    fn both_layouts_place_the_samples_as_gdal_does() {
        for name in ["cop-like-f32-deflate-point.tif", "int16-lzw-area.tif"] {
            let g = fixture(name);
            let at = |lat: f64, lon: f64| g.elevation(lat, lon).unwrap();
            // Sample centres (gdallocationinfo: 545, 645, 555).
            for (lat, lon, want) in [
                (-35.3615, 149.1615, 545.0),
                (-35.3685, 149.1785, 645.0),
                (-35.3625, 149.1625, 555.0),
            ] {
                assert!((at(lat, lon) - want).abs() < 1e-6, "{name} {lat},{lon}");
            }
            // Between four samples: the bilinear mean (545, 548, 552, 555 → 550).
            assert!((at(-35.362, 149.162) - 550.0).abs() < 1e-6, "{name}");
            // Next to the void (row 7, column 9: 149.1695, -35.3625): re-weighted.
            assert!(g.elevation(-35.3625, 149.1695).is_some(), "{name}");
            assert_eq!(g.elevation(-35.30, 149.17), None, "{name}: outside");
        }
    }
}
