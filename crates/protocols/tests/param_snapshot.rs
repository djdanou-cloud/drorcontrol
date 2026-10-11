//! Offline parameter edit/rollback previews; these tests never send vehicle commands.

use std::collections::BTreeMap;

use mavlink_codec::enums::MavParamType;
use protocols::params::snapshot::{ParamSnapshot, SnapshotError};
use protocols::params::{Param, parse_param_file};

fn baseline() -> BTreeMap<String, Param> {
    BTreeMap::from([
        (
            "FLOAT".into(),
            Param {
                value: 0.1,
                ty: MavParamType::REAL32.0,
                index: 0,
            },
        ),
        (
            "INTEGER".into(),
            Param {
                value: 7.0,
                ty: MavParamType::INT8.0,
                index: 1,
            },
        ),
        (
            "UNTOUCHED".into(),
            Param {
                value: 30.0,
                ty: MavParamType::UINT32.0,
                index: 2,
            },
        ),
    ])
}

#[test]
fn edit_then_rollback_restores_values_and_types_without_touching_other_parameters() {
    let original = baseline();
    let snapshot = ParamSnapshot::capture((4, 1), 3, original.clone()).unwrap();
    let edits = parse_param_file("INTEGER,8\nFLOAT,0.2\n").unwrap();
    let plan = snapshot.plan(&edits).unwrap();
    assert_eq!(plan.target(), (4, 1));
    assert_eq!(
        plan.changes()
            .iter()
            .map(|c| c.name.as_str())
            .collect::<Vec<_>>(),
        ["FLOAT", "INTEGER"]
    );
    assert_eq!(
        snapshot.params(),
        &original,
        "preview does not modify the baseline"
    );
    assert_eq!(snapshot.target(), (4, 1));

    let mut simulated_store = original.clone();
    for change in plan.changes() {
        assert_eq!(change.after.ty, change.before.ty);
        assert_eq!(change.after.index, change.before.index);
        simulated_store.insert(change.name.clone(), change.after);
    }
    // A concurrent change to a parameter outside the batch must survive restoration.
    simulated_store.get_mut("UNTOUCHED").unwrap().value = 31.0;
    let rollback = plan.rollback();
    assert_eq!(rollback.target(), (4, 1));
    assert_eq!(rollback.rollback(), plan);
    for change in rollback.changes() {
        simulated_store.insert(change.name.clone(), change.after);
    }
    assert_eq!(simulated_store["FLOAT"], original["FLOAT"]);
    assert_eq!(simulated_store["INTEGER"], original["INTEGER"]);
    assert_eq!(simulated_store["UNTOUCHED"].value, 31.0);
}

#[test]
fn rejects_partial_download_duplicate_indices_and_broadcast_targets() {
    assert_eq!(
        ParamSnapshot::capture((0, 1), 3, baseline()),
        Err(SnapshotError::BroadcastTarget)
    );
    assert_eq!(
        ParamSnapshot::capture((1, 0), 3, baseline()),
        Err(SnapshotError::BroadcastTarget)
    );
    for count in [0, 2, 4, u16::MAX] {
        assert_eq!(
            ParamSnapshot::capture((1, 1), count, baseline()),
            Err(SnapshotError::IncompleteDownload)
        );
    }
    let mut missing = baseline();
    missing.remove("UNTOUCHED");
    assert_eq!(
        ParamSnapshot::capture((1, 1), 3, missing),
        Err(SnapshotError::IncompleteDownload)
    );
    for bad_index in [0, 3, u16::MAX] {
        let mut bad = baseline();
        bad.get_mut("UNTOUCHED").unwrap().index = bad_index;
        assert_eq!(
            ParamSnapshot::capture((1, 1), 3, bad),
            Err(SnapshotError::IncompleteDownload)
        );
    }
}

#[test]
fn rejects_invalid_baselines_before_any_plan_can_be_created() {
    for name in ["", "NAME_TOO_LONG_1234", "BAD,NAME", "BAD\0NAME", "é"] {
        let mut bad = baseline();
        let param = bad.remove("FLOAT").unwrap();
        bad.insert(name.into(), param);
        assert_eq!(
            ParamSnapshot::capture((1, 1), 3, bad),
            Err(SnapshotError::InvalidName(name.into()))
        );
    }
    let mut bad = baseline();
    bad.get_mut("FLOAT").unwrap().value = f32::NAN;
    assert_eq!(
        ParamSnapshot::capture((1, 1), 3, bad),
        Err(SnapshotError::InvalidValue("FLOAT".into()))
    );
    let mut bad = baseline();
    bad.get_mut("FLOAT").unwrap().ty = 255;
    assert_eq!(
        ParamSnapshot::capture((1, 1), 3, bad),
        Err(SnapshotError::InvalidType("FLOAT".into()))
    );
}

#[test]
fn no_op_edits_are_omitted_after_wire_precision_comparison() {
    let snapshot = ParamSnapshot::capture((1, 1), 3, baseline()).unwrap();
    let plan = snapshot
        .plan(&parse_param_file("FLOAT,0.1\nINTEGER,7\n").unwrap())
        .unwrap();
    assert!(plan.changes().is_empty());
    assert!(plan.rollback().changes().is_empty());
    assert!(
        snapshot
            .plan(&BTreeMap::new())
            .unwrap()
            .changes()
            .is_empty()
    );
}

#[test]
fn a_batch_with_an_unknown_or_invalid_value_fails_as_a_whole() {
    let original = baseline();
    let snapshot = ParamSnapshot::capture((1, 1), 3, original.clone()).unwrap();
    assert_eq!(
        snapshot.plan(&parse_param_file("FLOAT,0.2\nUNKNOWN,1").unwrap()),
        Err(SnapshotError::UnknownParameter("UNKNOWN".into()))
    );
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, f64::MAX, 1e-300] {
        assert_eq!(
            snapshot.plan(&BTreeMap::from([("FLOAT".into(), value)])),
            Err(SnapshotError::InvalidValue("FLOAT".into()))
        );
    }
    for value in [7.5, -129.0, 128.0] {
        assert_eq!(
            snapshot.plan(&BTreeMap::from([("INTEGER".into(), value)])),
            Err(SnapshotError::InvalidValue("INTEGER".into()))
        );
    }
    assert_eq!(snapshot.params(), &original);
}

#[test]
fn integer_limits_and_c_cast_precision_are_checked_for_every_type() {
    let cases = [
        (MavParamType::UINT8, 0.0, 255.0, -1.0, 256.0),
        (MavParamType::INT8, -128.0, 127.0, -129.0, 128.0),
        (MavParamType::UINT16, 0.0, 65535.0, -1.0, 65536.0),
        (MavParamType::INT16, -32768.0, 32767.0, -32769.0, 32768.0),
        (MavParamType::UINT32, 0.0, 4294967040.0, -1.0, 4294967296.0),
        (
            MavParamType::INT32,
            -2147483648.0,
            2147483520.0,
            -2147483904.0,
            2147483648.0,
        ),
        (
            MavParamType::UINT64,
            0.0,
            18446742974197923840.0,
            -1.0,
            18446744073709551616.0,
        ),
        (
            MavParamType::INT64,
            -9223372036854775808.0,
            9223371487098961920.0,
            -9223373136366403584.0,
            9223372036854775808.0,
        ),
    ];
    for (ty, min, max_wire, below, above) in cases {
        let snapshot = ParamSnapshot::capture(
            (1, 1),
            1,
            BTreeMap::from([(
                "P".into(),
                Param {
                    value: 0.0,
                    ty: ty.0,
                    index: 0,
                },
            )]),
        )
        .unwrap();
        for value in [min, max_wire] {
            assert!(
                snapshot
                    .plan(&BTreeMap::from([("P".into(), value)]))
                    .is_ok(),
                "type {} value {value}",
                ty.0
            );
        }
        for value in [below, above, 0.5] {
            assert!(
                snapshot
                    .plan(&BTreeMap::from([("P".into(), value)]))
                    .is_err(),
                "type {} value {value}",
                ty.0
            );
        }
    }
    let snapshot = ParamSnapshot::capture((1, 1), 3, baseline()).unwrap();
    assert_eq!(
        snapshot.plan(&BTreeMap::from([("UNTOUCHED".into(), 16777217.0)])),
        Err(SnapshotError::InvalidValue("UNTOUCHED".into())),
        "integer cannot silently become 16777216 on the wire"
    );
}

#[test]
fn stored_snapshot_preserves_exact_bits_types_indices_and_target() {
    let mut params = baseline();
    params.get_mut("FLOAT").unwrap().value = -0.0;
    let snapshot = ParamSnapshot::capture((42, 7), 3, params).unwrap();
    let text = snapshot.to_text();
    let loaded = ParamSnapshot::from_text(&text).unwrap();
    assert_eq!(loaded.target(), (42, 7));
    for (name, original) in snapshot.params() {
        let restored = loaded.params()[name];
        assert_eq!(restored.value.to_bits(), original.value.to_bits());
        assert_eq!(restored.ty, original.ty);
        assert_eq!(restored.index, original.index);
    }
    assert_eq!(loaded.to_text(), text);
    let plan = loaded
        .plan(&parse_param_file("FLOAT,0.2\nINTEGER,8").unwrap())
        .unwrap();
    let rollback = plan.rollback();
    assert_eq!(
        rollback.changes()[0].after.value.to_bits(),
        (-0.0f32).to_bits()
    );
}

#[test]
fn corrupted_truncated_or_edited_storage_is_not_a_rollback_baseline() {
    let snapshot = ParamSnapshot::capture((1, 1), 3, baseline()).unwrap();
    let text = snapshot.to_text();
    for edited in [
        text.replace("V1,1,1,3", "V1,2,1,3"),
        text.replace("FLOAT,3dcccccd", "FLOAT,3e4ccccd"),
        text.replace("INTEGER,40e00000,2,1\n", ""),
    ] {
        assert_eq!(
            ParamSnapshot::from_text(&edited),
            Err(SnapshotError::ChecksumMismatch)
        );
    }
    assert_eq!(
        ParamSnapshot::from_text(text.split("SHA256,").next().unwrap()),
        Err(SnapshotError::InvalidFormat)
    );
    assert_eq!(
        ParamSnapshot::from_text(&(text.clone() + "extra\n")),
        Err(SnapshotError::InvalidFormat)
    );
    assert_eq!(
        ParamSnapshot::from_text("not a snapshot"),
        Err(SnapshotError::InvalidFormat)
    );
}

#[test]
fn valid_checksum_does_not_bypass_format_and_baseline_validation() {
    use sha2::{Digest, Sha256};
    for (body, want) in [
        (
            "DC_PARAM_SNAPSHOT_V2,1,1,1\nP,00000000,9,0\n",
            SnapshotError::InvalidFormat,
        ),
        (
            "DC_PARAM_SNAPSHOT_V1,0,1,1\nP,00000000,9,0\n",
            SnapshotError::BroadcastTarget,
        ),
        (
            "DC_PARAM_SNAPSHOT_V1,1,1,2\nP,00000000,9,0\n",
            SnapshotError::IncompleteDownload,
        ),
        (
            "DC_PARAM_SNAPSHOT_V1,1,1,2\nP,00000000,9,0\nP,00000000,9,1\n",
            SnapshotError::IncompleteDownload,
        ),
        (
            "DC_PARAM_SNAPSHOT_V1,1,1,1\nP,7fc00000,9,0\n",
            SnapshotError::InvalidValue("P".into()),
        ),
    ] {
        let digest: String = Sha256::digest(body.as_bytes())
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect();
        assert_eq!(
            ParamSnapshot::from_text(&format!("{body}SHA256,{digest}\n")),
            Err(want)
        );
    }
}

#[test]
fn comparison_reports_clamping_missing_values_types_and_wrong_targets() {
    let snapshot = ParamSnapshot::capture((1, 1), 3, baseline()).unwrap();
    let plan = snapshot
        .plan(&parse_param_file("FLOAT,0.2\nINTEGER,8").unwrap())
        .unwrap();
    let mut downloaded = baseline();
    downloaded.get_mut("FLOAT").unwrap().value = 0.2;
    downloaded.get_mut("INTEGER").unwrap().value = 8.0;
    downloaded.get_mut("UNTOUCHED").unwrap().value = 31.0;
    assert!(
        plan.compare(&ParamSnapshot::capture((1, 1), 3, downloaded.clone()).unwrap())
            .unwrap()
            .is_empty()
    );
    assert_eq!(
        plan.compare(&ParamSnapshot::capture((2, 1), 3, downloaded.clone()).unwrap()),
        Err(SnapshotError::TargetMismatch)
    );
    assert_eq!(
        plan.compare(&ParamSnapshot::capture((1, 2), 3, downloaded.clone()).unwrap()),
        Err(SnapshotError::TargetMismatch)
    );
    downloaded.get_mut("FLOAT").unwrap().value = 0.15;
    downloaded.get_mut("INTEGER").unwrap().ty = MavParamType::REAL32.0;
    let observed = ParamSnapshot::capture((1, 1), 3, downloaded.clone()).unwrap();
    let differences = plan.compare(&observed).unwrap();
    assert_eq!(differences.len(), 2);
    assert_eq!(differences[0].name, "FLOAT");
    assert_eq!(differences[0].expected.value, 0.2);
    assert_eq!(differences[0].actual.unwrap().value, 0.15);
    assert_eq!(differences[1].name, "INTEGER");
    assert_eq!(differences[1].actual.unwrap().ty, MavParamType::REAL32.0);
    let missing = downloaded.remove("INTEGER").unwrap();
    downloaded.insert("RENAMED".into(), missing);
    let observed = ParamSnapshot::capture((1, 1), 3, downloaded).unwrap();
    assert_eq!(plan.compare(&observed).unwrap()[1].actual, None);
    let restored = ParamSnapshot::capture((1, 1), 3, baseline()).unwrap();
    assert!(plan.rollback().compare(&restored).unwrap().is_empty());
}
