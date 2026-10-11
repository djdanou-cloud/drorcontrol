//! DrorControl design tokens (plan 04 §1–2).
//!
//! `ui/tokens/tokens.json` is the single source; `scripts/gen_tokens.py` turns it into the
//! QML singleton and CSS variables. This crate gives Rust code (GPU-drawn instruments) the same
//! values and checks the contrast rules declared in the file ([`Tokens::contrast`]).

use serde_json::Value;

/// The token file, embedded at build time.
pub const SOURCE: &str = include_str!("../../../ui/tokens/tokens.json");

/// An sRGB colour with straight alpha.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rgba {
    /// Red, 0–255.
    pub r: u8,
    /// Green, 0–255.
    pub g: u8,
    /// Blue, 0–255.
    pub b: u8,
    /// Opacity, 0–1.
    pub a: f64,
}

impl Rgba {
    /// Parse `#RRGGBB`.
    pub fn from_hex(s: &str) -> Option<Rgba> {
        let h = s.strip_prefix('#')?;
        if h.len() != 6 || !h.bytes().all(|b| b.is_ascii_hexdigit()) {
            return None;
        }
        let c = |i: usize| u8::from_str_radix(&h[i..i + 2], 16).ok();
        Some(Rgba {
            r: c(0)?,
            g: c(2)?,
            b: c(4)?,
            a: 1.0,
        })
    }

    /// `#RRGGBB` (alpha dropped).
    pub fn hex(&self) -> String {
        format!("#{:02X}{:02X}{:02X}", self.r, self.g, self.b)
    }

    /// This colour composited over an opaque backdrop (sRGB blending, as compositors do).
    pub fn over(&self, backdrop: Rgba) -> Rgba {
        let mix =
            |f: u8, b: u8| (self.a * f64::from(f) + (1.0 - self.a) * f64::from(b)).round() as u8;
        Rgba {
            r: mix(self.r, backdrop.r),
            g: mix(self.g, backdrop.g),
            b: mix(self.b, backdrop.b),
            a: 1.0,
        }
    }

    /// WCAG 2.x relative luminance (alpha ignored).
    pub fn luminance(&self) -> f64 {
        let lin = |c: u8| {
            let c = f64::from(c) / 255.0;
            if c <= 0.04045 {
                c / 12.92
            } else {
                ((c + 0.055) / 1.055).powf(2.4)
            }
        };
        0.2126 * lin(self.r) + 0.7152 * lin(self.g) + 0.0722 * lin(self.b)
    }
}

/// WCAG 2.x contrast ratio between two opaque colours (1–21).
pub fn contrast_ratio(a: Rgba, b: Rgba) -> f64 {
    let (la, lb) = (a.luminance(), b.luminance());
    (la.max(lb) + 0.05) / (la.min(lb) + 0.05)
}

/// One evaluated contrast pair.
#[derive(Debug, Clone, PartialEq)]
pub struct ContrastCheck {
    /// Theme key, e.g. `standard`.
    pub theme: String,
    /// Rule id from the token file.
    pub rule: String,
    /// Foreground token path, e.g. `textTelemetry`.
    pub fg: String,
    /// Background token path (with `@backdrop` when translucent).
    pub bg: String,
    /// Measured ratio.
    pub ratio: f64,
    /// Required ratio (the exemption's, if one applies).
    pub min: f64,
    /// Exemption reason, if the rule's minimum was lowered for this pair.
    pub exemption: Option<String>,
}

impl ContrastCheck {
    /// True when the measured ratio meets the required one.
    pub fn passes(&self) -> bool {
        self.ratio + 1e-9 >= self.min
    }
}

/// Parsed token file.
pub struct Tokens {
    json: Value,
}

impl Tokens {
    /// Load the embedded token file.
    pub fn load() -> Result<Tokens, String> {
        Tokens::parse(SOURCE)
    }

    /// Parse a token file.
    pub fn parse(text: &str) -> Result<Tokens, String> {
        let json: Value = serde_json::from_str(text).map_err(|e| e.to_string())?;
        if !json["themes"].is_object() {
            return Err("no themes".into());
        }
        Ok(Tokens { json })
    }

    /// Raw JSON.
    pub fn json(&self) -> &Value {
        &self.json
    }

    /// Theme keys (sorted).
    pub fn themes(&self) -> Vec<String> {
        self.json["themes"]
            .as_object()
            .map(|m| m.keys().cloned().collect())
            .unwrap_or_default()
    }

    /// Colour token `path` (e.g. `state.warning`) in `theme`, references resolved.
    pub fn color(&self, theme: &str, path: &str) -> Result<Rgba, String> {
        let mut v = &self.json["themes"][theme]["color"];
        for part in path.split('.') {
            v = &v[part];
        }
        self.resolve(v)
            .ok_or_else(|| format!("{theme}: bad or missing colour token {path}"))
    }

    fn resolve(&self, v: &Value) -> Option<Rgba> {
        match v {
            Value::String(s) => match s.strip_prefix("{brand.").and_then(|r| r.strip_suffix('}')) {
                Some(name) => Rgba::from_hex(self.json["brand"][name].as_str()?),
                None => Rgba::from_hex(s),
            },
            Value::Object(o) => {
                let mut c = self.resolve(o.get("hex")?)?;
                c.a = o
                    .get("alpha")?
                    .as_f64()
                    .filter(|a| (0.0..=1.0).contains(a))?;
                Some(c)
            }
            _ => None,
        }
    }

    /// Evaluate every contrast rule in every theme, applying exemptions.
    pub fn contrast(&self) -> Result<Vec<ContrastCheck>, String> {
        let rules = self.json["contrast"]["rules"]
            .as_array()
            .ok_or("no contrast rules")?;
        let exemptions = self.json["contrast"]["exemptions"]
            .as_array()
            .cloned()
            .unwrap_or_default();
        let mut out = Vec::new();
        for theme in self.themes() {
            for rule in rules {
                let id = rule["id"].as_str().ok_or("rule without id")?;
                let min = rule["min"].as_f64().ok_or("rule without min")?;
                let strs = |v: &Value| -> Vec<String> {
                    v.as_array()
                        .map(|a| {
                            a.iter()
                                .filter_map(|x| x.as_str().map(String::from))
                                .collect()
                        })
                        .unwrap_or_default()
                };
                let pairs: Vec<(String, String)> = match rule["pairs"].as_array() {
                    Some(p) => p
                        .iter()
                        .map(|x| {
                            let s = strs(x);
                            (s[0].clone(), s[1].clone())
                        })
                        .collect(),
                    None => {
                        let bgs = strs(&rule["bg"]);
                        strs(&rule["fg"])
                            .into_iter()
                            .flat_map(|f| bgs.iter().map(move |b| (f.clone(), b.clone())))
                            .collect()
                    }
                };
                let backdrops = strs(&rule["backdrops"]);
                for (fg, bg) in pairs {
                    let f = self.color(&theme, &fg)?;
                    let b = self.color(&theme, &bg)?;
                    let mut grounds = vec![(bg.clone(), b)];
                    if b.a < 1.0 {
                        grounds.clear();
                        for d in &backdrops {
                            let dc = if d == "canvas" {
                                self.color(&theme, "canvas")?
                            } else {
                                Rgba::from_hex(d).ok_or_else(|| format!("bad backdrop {d}"))?
                            };
                            grounds.push((format!("{bg}@{d}"), b.over(dc)));
                        }
                    }
                    for (name, ground) in grounds {
                        let ex = exemptions.iter().find(|e| {
                            e["theme"] == theme.as_str()
                                && e["rule"] == id
                                && e.get("fg").is_none_or(|v| v == fg.as_str())
                                && e.get("bg").is_none_or(|v| v == bg.as_str())
                        });
                        out.push(ContrastCheck {
                            theme: theme.clone(),
                            rule: id.to_string(),
                            fg: fg.clone(),
                            bg: name,
                            ratio: contrast_ratio(f.over(ground), ground),
                            min: ex.and_then(|e| e["min"].as_f64()).unwrap_or(min),
                            exemption: ex.and_then(|e| e["reason"].as_str()).map(String::from),
                        });
                    }
                }
            }
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(s: &str) -> Rgba {
        Rgba::from_hex(s).unwrap()
    }

    #[test]
    fn wcag_reference_values() {
        assert!((contrast_ratio(c("#000000"), c("#FFFFFF")) - 21.0).abs() < 1e-9);
        assert!((contrast_ratio(c("#777777"), c("#FFFFFF")) - 4.48).abs() < 0.01);
        assert!((contrast_ratio(c("#FAFAFA"), c("#212121")) - 15.43).abs() < 0.01);
        assert_eq!(contrast_ratio(c("#123456"), c("#123456")), 1.0);
    }

    #[test]
    fn compositing() {
        let glass = Rgba {
            a: 0.5,
            ..c("#000000")
        };
        assert_eq!(glass.over(c("#FFFFFF")).hex(), "#808080");
        assert!(Rgba::from_hex("#12345").is_none());
        assert!(Rgba::from_hex("123456").is_none());
    }
}
