//! The first polygon in a KML file (e.g. an area drawn in Google Earth), for survey planning.
//! A small reader for `<Polygon>…<outerBoundaryIs>…<coordinates>lon,lat[,alt] …</coordinates>`
//! (KML 2.2); holes, other placemarks and styles are ignored.

/// Why no polygon could be read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum KmlError {
    /// No `<Polygon>` with an outer boundary.
    NoPolygon,
    /// A coordinate tuple is not `lon,lat[,alt]`.
    BadCoordinate(String),
    /// Fewer than 3 distinct vertices.
    TooFewVertices,
}

impl std::fmt::Display for KmlError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            KmlError::NoPolygon => f.write_str("no <Polygon> with an outer boundary"),
            KmlError::BadCoordinate(c) => {
                write!(f, "bad coordinate {c:?} (expected lon,lat[,alt])")
            }
            KmlError::TooFewVertices => f.write_str("the polygon has fewer than 3 vertices"),
        }
    }
}

/// The first polygon's outer boundary as (latitude, longitude) vertices; the closing vertex
/// that repeats the first is dropped.
pub fn first_polygon(kml: &str) -> Result<Vec<(f64, f64)>, KmlError> {
    let poly = kml.find("<Polygon").ok_or(KmlError::NoPolygon)?;
    let rest = &kml[poly..];
    let outer = rest.find("<outerBoundaryIs").ok_or(KmlError::NoPolygon)?;
    let rest = &rest[outer..];
    let start = rest.find("<coordinates>").ok_or(KmlError::NoPolygon)? + "<coordinates>".len();
    let end = rest[start..]
        .find("</coordinates>")
        .ok_or(KmlError::NoPolygon)?
        + start;
    let mut pts = Vec::new();
    for tuple in rest[start..end].split_whitespace() {
        let mut it = tuple.split(',');
        let (Some(lon), Some(lat)) = (it.next(), it.next()) else {
            return Err(KmlError::BadCoordinate(tuple.into()));
        };
        let num = |s: &str| {
            s.parse::<f64>()
                .map_err(|_| KmlError::BadCoordinate(tuple.into()))
        };
        let (lat, lon) = (num(lat)?, num(lon)?);
        if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
            return Err(KmlError::BadCoordinate(tuple.into()));
        }
        pts.push((lat, lon));
    }
    if pts.len() > 1 && pts.first() == pts.last() {
        pts.pop();
    }
    if pts.len() < 3 {
        return Err(KmlError::TooFewVertices);
    }
    Ok(pts)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_a_google_earth_polygon() {
        let kml = r#"<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"><Document><Placemark><name>Field</name>
<Polygon><outerBoundaryIs><LinearRing><coordinates>
  149.1652,-35.3632,0 149.1672,-35.3632,0
  149.1672,-35.3650,0 149.1652,-35.3650,0 149.1652,-35.3632,0
</coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark></Document></kml>"#;
        let p = first_polygon(kml).unwrap();
        assert_eq!(p.len(), 4, "closing vertex dropped");
        assert_eq!(p[0], (-35.3632, 149.1652));
        assert_eq!(p[2], (-35.3650, 149.1672));
        assert_eq!(first_polygon("<kml/>"), Err(KmlError::NoPolygon));
        let bad = kml.replace("149.1672,-35.3632,0", "149.1672;-35.3632");
        assert!(matches!(
            first_polygon(&bad),
            Err(KmlError::BadCoordinate(_))
        ));
    }
}
