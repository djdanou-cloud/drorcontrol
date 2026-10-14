//! Log tools on the real ArduCopter 4.6.2 corpus log (`crates/logs/tests/data`), which, like
//! the 4.6.3 bench log of 2026-10-13, has IMU but no ACC/GYR messages (raw IMU logging off).

use std::path::PathBuf;
use std::process::Command;

fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../logs/tests/data/copter-4.6.2-matekh743-head.bin")
}

fn dc(args: &[&str]) -> (bool, String, String) {
    let out = Command::new(env!("CARGO_BIN_EXE_dc-cli"))
        .args(args)
        .output()
        .unwrap();
    (
        out.status.success(),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

#[test]
fn fft_on_acc_falls_back_to_imu_without_raw_imu_logging() {
    let log = fixture();
    let (ok, out, err) = dc(&[
        "log",
        "fft",
        log.to_str().unwrap(),
        "--field",
        "ACC.AccZ",
        "--segment",
        "64",
    ]);
    assert!(ok, "{out}{err}");
    assert!(err.contains("no ACC messages"), "{err}");
    assert!(out.starts_with("IMU.AccZ IMU 0:"), "{out}");
    // Not a field that IMU has either: still an error.
    let (ok, _, err) = dc(&["log", "fft", log.to_str().unwrap(), "--field", "ACC.Nope"]);
    assert!(!ok && err.contains("no ACC.Nope or IMU.Nope"), "{err}");
}
