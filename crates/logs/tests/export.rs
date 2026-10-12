//! CSV and .param export, checked against pymavlink's values for the same log.

use logs::Log;
use logs::export::{csv, param_file};

fn log(name: &str) -> Log {
    Log::open(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/data")
            .join(name),
    )
    .unwrap()
}

#[test]
fn csv_matches_pymavlink() {
    let log = log("sitl-copter-4.7.1-head.bin");
    let text = csv(&log, "ATT").unwrap();
    let mut lines = text.lines();
    assert_eq!(
        lines.next().unwrap(),
        "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,AEKF"
    );
    // pymavlink, record 0: TimeUS 2539817, Roll 0.010051097720861435, AEKF 0.
    let row: Vec<f64> = lines
        .next()
        .unwrap()
        .split(',')
        .map(|v| v.parse().unwrap())
        .collect();
    assert_eq!(row[0], 2_539_817.0);
    assert!((row[2] - 0.010_051_097_720_861_435).abs() < 1e-9);
    assert_eq!(row[7], 0.0);
    assert_eq!(text.lines().count(), 1 + log.counts()["ATT"]);
    assert!(csv(&log, "NOPE").is_none());
    // Text fields are quoted.
    let msg = csv(&log, "MSG").unwrap();
    assert!(msg.lines().nth(1).unwrap().contains(",\""), "{msg}");
}

#[test]
fn param_file_has_every_recorded_parameter() {
    let log = log("copter-4.6.2-matekh743-head.bin");
    let text = param_file(&log);
    assert_eq!(text.lines().count(), log.params().len());
    assert!(
        text.lines().any(|l| l == "FS_EKF_THRESH,1"),
        "the vehicle's own value"
    );
}
