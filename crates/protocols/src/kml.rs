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

/// `lon,lat[,alt]` tuples of a `<coordinates>` body as (latitude, longitude).
fn tuples(body: &str) -> Result<Vec<(f64, f64)>, KmlError> {
    let mut pts = Vec::new();
    for tuple in body.split_whitespace() {
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
    Ok(pts)
}

/// Text between `open` (a tag start, attributes allowed) and `close` in `s`, from `from`:
/// (body, end index).
fn section<'a>(s: &'a str, from: usize, open: &str, close: &str) -> Option<(&'a str, usize)> {
    let a = s[from..].find(open)? + from;
    let body_start = s[a..].find('>')? + a + 1;
    let b = s[body_start..].find(close)? + body_start;
    Some((&s[body_start..b], b + close.len()))
}

/// The placemark's `<name>`, with CDATA and the XML entities decoded.
fn name_of(placemark: &str) -> Option<String> {
    let (n, _) = section(placemark, 0, "<name", "</name>")?;
    let n = n.trim();
    if let Some(c) = n
        .strip_prefix("<![CDATA[")
        .and_then(|c| c.strip_suffix("]]>"))
    {
        return Some(c.to_string());
    }
    Some(
        n.replace("&lt;", "<")
            .replace("&gt;", ">")
            .replace("&quot;", "\"")
            .replace("&apos;", "'")
            .replace("&amp;", "&"),
    )
}

/// The first `<LineString>` (e.g. a path drawn in Google Earth) as (latitude, longitude)
/// points, in drawing order; at least 2.
pub fn first_line(kml: &str) -> Result<Vec<(f64, f64)>, KmlError> {
    let (line, _) = section(kml, 0, "<LineString", "</LineString>").ok_or(KmlError::NoPolygon)?;
    let (body, _) =
        section(line, 0, "<coordinates", "</coordinates>").ok_or(KmlError::NoPolygon)?;
    let pts = tuples(body)?;
    if pts.len() < 2 {
        return Err(KmlError::TooFewVertices);
    }
    Ok(pts)
}

/// A placemark's geometry.
#[derive(Debug, Clone, PartialEq)]
pub struct Placemark {
    /// `<name>`, if any.
    pub name: Option<String>,
    /// "Point", "LineString" or "Polygon".
    pub kind: &'static str,
    /// (latitude, longitude); a polygon's outer boundary without its closing vertex.
    pub points: Vec<(f64, f64)>,
}

/// Every placemark with a geometry, in file order.
pub fn placemarks(kml: &str) -> Result<Vec<Placemark>, KmlError> {
    let mut out = Vec::new();
    let mut at = 0;
    while let Some((pm, end)) = section(kml, at, "<Placemark", "</Placemark>") {
        at = end;
        let name = name_of(pm);
        for kind in ["Polygon", "LineString", "Point"] {
            if pm.contains(&format!("<{kind}")) {
                let geom = section(pm, 0, &format!("<{kind}"), &format!("</{kind}>"))
                    .map_or("", |(g, _)| g);
                let Some((body, _)) = section(geom, 0, "<coordinates", "</coordinates>") else {
                    continue;
                };
                let mut pts = tuples(body)?;
                if kind == "Polygon" && pts.len() > 1 && pts.first() == pts.last() {
                    pts.pop();
                }
                out.push(Placemark {
                    name: name.clone(),
                    kind,
                    points: pts,
                });
                break;
            }
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DRAWN: &str = r#"<kml xmlns="http://www.opengis.net/kml/2.2"><Document>
<Placemark><name>Route</name><LineString><tessellate>1</tessellate><coordinates>
 149.1652,-35.3632,0 149.1662,-35.3632,0 149.1662,-35.3642,0</coordinates></LineString></Placemark>
<Placemark><name>Field</name><Polygon><outerBoundaryIs><LinearRing><coordinates>
 149.16,-35.36,0 149.17,-35.36,0 149.17,-35.37,0 149.16,-35.36,0</coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark>
<Placemark><name>Rally A</name><Point><coordinates>149.1655,-35.3635,0</coordinates></Point></Placemark>
<Placemark><Point><coordinates>149.1665,-35.3645</coordinates></Point></Placemark>
</Document></kml>"#;

    #[test]
    fn lines_and_placemarks_in_file_order() {
        let l = first_line(DRAWN).unwrap();
        assert_eq!(
            l,
            [
                (-35.3632, 149.1652),
                (-35.3632, 149.1662),
                (-35.3642, 149.1662)
            ]
        );
        let p = placemarks(DRAWN).unwrap();
        let kinds: Vec<_> = p
            .iter()
            .map(|p| (p.name.as_deref(), p.kind, p.points.len()))
            .collect();
        assert_eq!(
            kinds,
            [
                (Some("Route"), "LineString", 3),
                (Some("Field"), "Polygon", 3),
                (Some("Rally A"), "Point", 1),
                (None, "Point", 1)
            ]
        );
        assert!(first_line("<kml/>").is_err());
    }

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
