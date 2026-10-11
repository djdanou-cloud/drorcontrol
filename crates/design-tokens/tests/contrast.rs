//! Token checks (T0-12 acceptance): every contrast rule in `ui/tokens/tokens.json` holds in
//! every theme, exemptions are explicit and still needed, and themes are complete.

use design_tokens::Tokens;

#[test]
fn contrast_rules_hold_in_every_theme() {
    let t = Tokens::load().unwrap();
    let checks = t.contrast().unwrap();
    let mut table = String::new();
    for c in &checks {
        table += &format!(
            "{:<12} {:<22} {:<16} {:<28} {:>6.2} >= {:<4} {}{}\n",
            c.theme,
            c.rule,
            c.fg,
            c.bg,
            c.ratio,
            c.min,
            if c.passes() { "ok" } else { "FAIL" },
            if c.exemption.is_some() {
                " (exempt)"
            } else {
                ""
            }
        );
    }
    eprintln!("{table}");
    let failures: Vec<_> = checks.iter().filter(|c| !c.passes()).collect();
    assert!(failures.is_empty(), "contrast failures:\n{failures:#?}");
    // Every theme is checked and telemetry text is covered.
    for theme in t.themes() {
        assert!(
            checks
                .iter()
                .any(|c| c.theme == theme && c.rule == "telemetry-text"),
            "{theme}"
        );
    }
}

#[test]
fn telemetry_text_is_aaa_except_documented_exemptions() {
    let t = Tokens::load().unwrap();
    for c in t.contrast().unwrap() {
        if c.rule == "telemetry-text" && c.exemption.is_none() {
            assert!(c.ratio >= 7.0, "{c:?}");
        }
    }
}

#[test]
fn every_exemption_is_needed_and_explained() {
    let t = Tokens::load().unwrap();
    let checks = t.contrast().unwrap();
    for e in t.json()["contrast"]["exemptions"].as_array().unwrap() {
        let reason = e["reason"].as_str().unwrap_or("");
        assert!(reason.len() > 40, "exemption needs a reason: {e}");
        assert!(e["issue"].is_string(), "exemption needs an issue: {e}");
        let rule = e["rule"].as_str().unwrap();
        let rule_min = t.json()["contrast"]["rules"]
            .as_array()
            .unwrap()
            .iter()
            .find(|r| r["id"] == rule)
            .and_then(|r| r["min"].as_f64())
            .unwrap_or_else(|| panic!("unknown rule {rule}"));
        // Stale exemptions must be removed: some covered pair must actually miss the rule.
        let needed = checks
            .iter()
            .any(|c| c.exemption.as_deref() == Some(reason) && c.ratio < rule_min);
        assert!(needed, "exemption no longer needed, remove it: {e}");
    }
}

#[test]
fn themes_define_the_same_tokens() {
    let t = Tokens::load().unwrap();
    fn keys(v: &serde_json::Value, prefix: &str, out: &mut Vec<String>) {
        if let Some(o) = v.as_object() {
            if o.contains_key("hex") {
                out.push(prefix.to_string());
                return;
            }
            for (k, v) in o {
                keys(
                    v,
                    &format!("{prefix}{}{k}", if prefix.is_empty() { "" } else { "." }),
                    out,
                );
            }
        } else {
            out.push(prefix.to_string());
        }
    }
    let themes = t.themes();
    let mut sorted = themes.clone();
    sorted.sort();
    assert_eq!(sorted, ["daylight", "highContrast", "nightOps", "standard"]);
    let mut first = Vec::new();
    keys(&t.json()["themes"][&themes[0]], "", &mut first);
    for th in &themes {
        let mut k = Vec::new();
        keys(&t.json()["themes"][th], "", &mut k);
        assert_eq!(k, first, "{th} defines different tokens");
        for path in k.iter().filter_map(|p| p.strip_prefix("color.")) {
            t.color(th, path).unwrap();
        }
    }
}

#[test]
fn night_ops_has_no_blue_green_or_white_emitters() {
    // Plan 04 §1.3: red monochrome; the plan's brightest red is #FF4D4D.
    let t = Tokens::load().unwrap();
    let mut paths = Vec::new();
    fn walk(v: &serde_json::Value, p: &str, out: &mut Vec<String>) {
        match v.as_object() {
            Some(o) if !o.contains_key("hex") => {
                for (k, v) in o {
                    walk(
                        v,
                        &format!("{p}{}{k}", if p.is_empty() { "" } else { "." }),
                        out,
                    );
                }
            }
            _ => out.push(p.to_string()),
        }
    }
    walk(&t.json()["themes"]["nightOps"]["color"], "", &mut paths);
    for p in paths {
        let c = t.color("nightOps", &p).unwrap();
        assert!(
            c.g <= 0x4D && c.b <= 0x4D && c.g <= c.r && c.b <= c.r,
            "nightOps {p} = {} is not red-only",
            c.hex()
        );
    }
}
