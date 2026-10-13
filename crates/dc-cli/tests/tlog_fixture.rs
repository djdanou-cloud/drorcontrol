//! The `.tlog` tools on a real recording: a SITL ArduCopter 4.7.1 AUTO flight (takeoff in
//! GUIDED, a 4-item mission, RTL and landing) recorded by the core (`DC_SITL_TLOG`).
//! Numbers are pinned from the first verified run (docs/corpus/README.md).

use std::path::PathBuf;
use std::process::Command;

fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/sitl-copter-4.7.1-auto.tlog")
}

fn dc(args: &[&str]) -> (bool, String) {
    let out = Command::new(env!("CARGO_BIN_EXE_dc-cli"))
        .args(args)
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&out.stdout).into_owned();
    // On failure the reason is on stderr: carry it in the text the asserts print.
    let text = if out.status.success() {
        stdout
    } else {
        format!(
            "{stdout}\n[exit {:?}] {}",
            out.status.code(),
            String::from_utf8_lossy(&out.stderr)
        )
    };
    (out.status.success(), text)
}

fn tmp(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("dc-fixture-{}-{name}", std::process::id()))
}

#[test]
fn info_and_modes() {
    let f = fixture();
    let (ok, out) = dc(&["tlog", "info", f.to_str().unwrap()]);
    assert!(ok, "{out}");
    assert!(out.contains("5651 records"), "{out}");
    assert!(out.contains("GUIDED 7.1 s, AUTO 80.9 s"), "{out}");
    assert!(out.contains("0 lost in transit"), "{out}");
}

#[test]
fn health_is_ok_with_every_check() {
    let f = fixture();
    let (ok, out) = dc(&[
        "tlog",
        "health",
        f.to_str().unwrap(),
        "--json",
        "--fail-on",
        "caution",
    ]);
    assert!(ok, "{out}");
    for check in [
        "\"vibration\"",
        "\"clipping\"",
        "\"battery\"",
        "\"ekf\"",
        "\"gps\"",
    ] {
        assert!(out.contains(check), "{check} missing: {out}");
    }
    assert!(out.contains("\"overall\": \"ok\""), "{out}");
}

#[test]
fn index_events_and_seek() {
    let f = fixture();
    let (ok, out) = dc(&["tlog", "index", f.to_str().unwrap(), "--at", "60"]);
    assert!(ok, "{out}");
    assert!(
        out.contains("5651 records, 135.9 s, 1312 checkpoints"),
        "{out}"
    );
    assert!(
        out.contains("Armed(true)") && out.contains("ItemReached(4)"),
        "{out}"
    );
    assert!(out.contains("35 streams restored"), "{out}");
}

#[test]
fn link_has_no_loss() {
    let f = fixture();
    let (ok, out) = dc(&["tlog", "link", f.to_str().unwrap(), "--window", "1000"]);
    assert!(ok, "{out}");
    assert!(out.contains("5463     0    0.0 %"), "{out}");
}

#[test]
fn sanitized_copy_keeps_the_flight_but_not_the_place() {
    let f = fixture();
    let out_path = tmp("san.tlog");
    let (ok, out) = dc(&[
        "tlog",
        "sanitize",
        f.to_str().unwrap(),
        "--out",
        out_path.to_str().unwrap(),
        "--to",
        "10,20",
    ]);
    assert!(ok, "{out}");
    let (ok, info) = dc(&["tlog", "info", out_path.to_str().unwrap()]);
    assert!(ok && info.contains("GUIDED 7.1 s, AUTO 80.9 s"), "{info}");
    let (_, mission) = dc(&["tlog", "mission", out_path.to_str().unwrap()]);
    assert!(
        !mission.contains("-35.36"),
        "original latitude left: {mission}"
    );
    std::fs::remove_file(out_path).unwrap();
}

#[test]
fn kml_by_flight_mode() {
    let f = fixture();
    let kml = tmp("f.kml");
    let out = Command::new(env!("CARGO_BIN_EXE_dc-cli"))
        .args([
            "tlog",
            "export",
            f.to_str().unwrap(),
            "--kml",
            kml.to_str().unwrap(),
        ])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let k = std::fs::read_to_string(&kml).unwrap();
    for name in [
        "<name>GUIDED</name>",
        "<name>AUTO</name>",
        "<name>WP 2</name>",
        "<gx:Track>",
    ] {
        assert!(k.contains(name), "{name} missing");
    }
    std::fs::remove_file(kml).unwrap();
}
