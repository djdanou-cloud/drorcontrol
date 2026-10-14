//! Mission-file tools on `docs/dev/bench/bench-test.plan` (home at the SITL site, a fence and
//! two rally points).

use std::path::PathBuf;
use std::process::Command;

fn plan() -> String {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../docs/dev/bench/bench-test.plan")
        .to_string_lossy()
        .into_owned()
}

fn dc(args: &[&str]) -> String {
    let out = Command::new(env!("CARGO_BIN_EXE_dc-cli"))
        .args(args)
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8_lossy(&out.stdout).into_owned()
}

#[test]
fn rtl_target_follows_copters_rally_rules() {
    let p = plan();
    let near_rally_2 = dc(&["mission", "rtl-target", &p, "--from=-35.3646,149.1666"]);
    assert!(
        near_rally_2.contains("RTL goes to rally point 2 at -35.3645000,149.1665000"),
        "{near_rally_2}"
    );
    let near_home = dc(&["mission", "rtl-target", &p, "--from=-35.3625,149.1653"]);
    assert!(
        near_home.contains("RTL goes home, 78 m away: home is nearer"),
        "{near_home}"
    );
    let far = dc(&["mission", "rtl-target", &p, "--from=-35.3700,149.1652"]);
    assert!(far.contains("beyond RALLY_LIMIT_KM"), "{far}");
}

#[test]
fn edit_applies_operations_in_order_and_renumbers_jumps() {
    let dir = std::env::temp_dir().join(format!("dc-edit-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    // Home, takeoff, three waypoints, DO_JUMP to item 3 twice, RTL.
    let src = dir.join("jump.waypoints");
    std::fs::write(
        &src,
        "QGC WPL 110\n\
         0\t1\t0\t16\t0\t0\t0\t0\t-35.3632\t149.1652\t584\t1\n\
         1\t0\t3\t22\t0\t0\t0\t0\t0\t0\t20\t1\n\
         2\t0\t3\t16\t0\t0\t0\t0\t-35.3622\t149.1652\t20\t1\n\
         3\t0\t3\t16\t0\t0\t0\t0\t-35.3622\t149.1662\t20\t1\n\
         4\t0\t3\t16\t0\t0\t0\t0\t-35.3632\t149.1662\t20\t1\n\
         5\t0\t0\t177\t3\t2\t0\t0\t0\t0\t0\t1\n\
         6\t0\t0\t20\t0\t0\t0\t0\t0\t0\t0\t1\n",
    )
    .unwrap();
    let out = dir.join("out.waypoints");
    let run = Command::new(env!("CARGO_BIN_EXE_dc-cli"))
        .args([
            "mission",
            "edit",
            src.to_str().unwrap(),
            "--out",
            out.to_str().unwrap(),
        ])
        .args(["insert:2:-35.3627,149.1650,25", "alt:3:30"])
        .output()
        .unwrap();
    let err = String::from_utf8_lossy(&run.stderr);
    assert!(run.status.success(), "{err}");
    let text = std::fs::read_to_string(&out).unwrap();
    let rows: Vec<Vec<&str>> = text
        .lines()
        .skip(1)
        .map(|l| l.split('\t').collect())
        .collect();
    assert_eq!(rows.len(), 8);
    assert_eq!((rows[2][3], rows[2][10]), ("16", "25"), "inserted at 2");
    assert_eq!(rows[3][10], "30", "the old item 2, now 3, at 30 m");
    // The DO_JUMP moved to 6 and still targets the same waypoint, now item 4.
    assert_eq!((rows[6][3], rows[6][4]), ("177", "4"), "{text}");
    // A bad operation stops before writing anything.
    let bad = Command::new(env!("CARGO_BIN_EXE_dc-cli"))
        .args([
            "mission",
            "edit",
            src.to_str().unwrap(),
            "--out",
            dir.join("x.waypoints").to_str().unwrap(),
            "delete:0",
        ])
        .output()
        .unwrap();
    assert!(!bad.status.success());
    assert!(String::from_utf8_lossy(&bad.stderr).contains("item 0 is home"));
    assert!(!dir.join("x.waypoints").exists());
    let _ = std::fs::remove_dir_all(dir);
}
