//! Health checks on the two fixture logs (both start on the ground, before flight).

use logs::Log;
use logs::health::{Level, assess};

fn findings(name: &str) -> Vec<(Level, &'static str, String)> {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name);
    let log = Log::open(&path).unwrap();
    assess(&log)
        .into_iter()
        .map(|f| (f.level, f.check, f.text))
        .collect()
}

#[test]
fn sitl_log() {
    let f = findings("sitl-copter-4.7.1-head.bin");
    let checks: Vec<_> = f.iter().map(|(_, c, _)| *c).collect();
    assert_eq!(
        checks,
        [
            "vibration",
            "clipping",
            "vibration",
            "clipping",
            "battery",
            "ekf",
            "ekf",
            "error"
        ]
    );
    assert!(f.iter().all(|(l, _, _)| *l == Level::Ok), "{f:#?}");
    assert!(
        f[4].2.starts_with("battery 0: lowest 12.60 V"),
        "{}",
        f[4].2
    );
    // SITL logs a compass error at boot and resolves it (ECode 0).
    assert_eq!(f[7].2, "ERR COMPASS code 0 (resolved)");
}

#[test]
fn real_board_log() {
    let f = findings("copter-4.6.2-matekh743-head.bin");
    let battery = f.iter().find(|(_, c, _)| *c == "battery").unwrap();
    assert!(
        battery.2.starts_with("battery 0: lowest 16.02 V"),
        "{}",
        battery.2
    );
    // This vehicle sets FS_EKF_THRESH = 1; the check uses the vehicle's own value.
    let ekf = f.iter().find(|(_, c, _)| *c == "ekf").unwrap();
    assert!(ekf.2.ends_with("(FS_EKF_THRESH 1.00)"), "{}", ekf.2);
    assert!(f.iter().all(|(_, c, _)| *c != "error"), "no ERR records");
}
