//! Parameters read from a real flight log (`PARM`) compared with a real `.param` file from
//! the same airframe saved at another time (see docs/corpus/README.md). Expected numbers were
//! computed independently with pymavlink `DFReader` and a plain-Python parse of the file.

use std::path::Path;

use logs::Log;
use protocols::params::{diff_params, parse_param_file};

#[test]
fn log_params_vs_param_file() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let log = Log::open(&root.join("tests/data/copter-4.6.2-matekh743-head.bin")).unwrap();
    let from_log = log.params();
    let text = std::fs::read_to_string(root.join("../protocols/tests/data/u3_whoop_260725.param"))
        .unwrap();
    let from_file = parse_param_file(&text).unwrap();

    assert_eq!(from_file.len(), 1551);
    assert_eq!(from_log.len(), 1548);
    assert_eq!(from_log["FRAME_TYPE"], 18.0);
    assert_eq!(from_file["ATC_RAT_RLL_P"], 0.143237);

    let d = diff_params(&from_file, &from_log);
    assert_eq!(d.only_left.len(), 17, "only in file: {:?}", d.only_left);
    assert!(
        d.only_left
            .iter()
            .all(|k| k.starts_with("QUIK_") || k.starts_with("RELAY1_"))
    );
    assert_eq!(d.only_right.len(), 14, "only in log: {:?}", d.only_right);
    assert!(
        d.only_right
            .iter()
            .all(|k| k.starts_with("INS_HNTC2_") || k.starts_with("SID_"))
    );
    assert_eq!(d.changed.len(), 150);
    let changed = |k: &str| d.changed.iter().find(|c| c.0 == k).map(|c| (c.1, c.2));
    assert_eq!(changed("AHRS_ORIENTATION"), Some((0.0, 8.0)));
    assert_eq!(changed("BARO2_DEVID"), Some((751369.0, 358145.0)));
    assert_eq!(changed("BATT_CAPACITY"), Some((4000.0, 4800.0)));
    assert_eq!(changed("SCHED_LOOP_RATE"), Some((400.0, 800.0)));
    for same in ["FRAME_CLASS", "FRAME_TYPE", "RTL_ALT", "FS_THR_ENABLE"] {
        assert_eq!(changed(same), None, "{same}");
    }
}
