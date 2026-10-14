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
