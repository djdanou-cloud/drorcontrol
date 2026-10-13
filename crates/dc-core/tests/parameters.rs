//! Parameter transactions against a scripted vehicle: snapshot ordering, explicit read-back,
//! refusal/interruption, source identity, and verified rollback.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use dc_core::{
    ARMED_WRITE_WARNING, ArmedWrite, Core, CoreConfig, LinkSpec, PARAM_HEARTBEAT_MAX_AGE,
    ParamOperationError, ParamRisk, parameter_risk,
};
use mavlink_codec::enums::{MavModeFlag, MavParamType};
use mavlink_codec::messages::{Heartbeat, ParamValue};
use mavlink_codec::{FrameParser, Header, Message, Version, encode};
use protocols::params::snapshot::ParamSnapshot;
use protocols::params::{Param, SetStatus, decode_id, encode_id};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;

struct Vehicle {
    params: BTreeMap<String, Param>,
    armed: bool,
    silent: bool,
    clamp: bool,
    arm_after_set: bool,
    omit_echo: bool,
    omit_read: bool,
    wrong_sender: bool,
    silence_after_set: bool,
    arm_during_download: bool,
    snapshot: PathBuf,
    snapshots_valid: Vec<bool>,
    sets: Vec<String>,
    reads: usize,
}

fn value(name: &str, param: Param, count: usize) -> Message {
    Message::ParamValue(ParamValue {
        param_id: encode_id(name),
        param_value: param.value,
        param_type: param.ty,
        param_index: param.index,
        param_count: count as u16,
    })
}

fn heartbeat(armed: bool) -> Message {
    Message::Heartbeat(Heartbeat {
        r#type: 2,
        autopilot: 3,
        base_mode: if armed {
            MavModeFlag::SAFETY_ARMED.0
        } else {
            0
        },
        mavlink_version: 3,
        ..Default::default()
    })
}

async fn vehicle(listener: TcpListener, state: Arc<Mutex<Vehicle>>) {
    let (mut socket, _) = listener.accept().await.unwrap();
    let mut parser = FrameParser::new();
    let mut buffer = [0; 8192];
    let mut seq = 0u8;
    let mut tick = tokio::time::interval(Duration::from_millis(50));
    loop {
        let mut out = Vec::new();
        tokio::select! {
            _ = tick.tick() => {
                let state = state.lock().unwrap();
                if !state.silent { out.push((1, 1, heartbeat(state.armed))); }
            },
            n = socket.read(&mut buffer) => {
                let Ok(n) = n else { return };
                if n == 0 { return; }
                parser.push(&buffer[..n]);
                while let Some(frame) = parser.next_frame() {
                    let mut state = state.lock().unwrap();
                    match frame.decode().unwrap() {
                        Message::ParamRequestList(_) => {
                            if state.arm_during_download {
                                state.armed = true;
                                out.push((1, 1, heartbeat(true)));
                            }
                            for (name, param) in &state.params { out.push((1, 1, value(name, *param, state.params.len()))); }
                        },
                        Message::ParamSet(set) => {
                            let name = decode_id(&set.param_id);
                            let valid = std::fs::read_to_string(&state.snapshot).ok().and_then(|s| ParamSnapshot::from_text(&s).ok()).is_some_and(|s| s.target() == (1, 1) && s.params().len() == state.params.len());
                            state.snapshots_valid.push(valid);
                            state.sets.push(name.clone());
                            if state.silence_after_set { state.silent = true; }
                            let clamp = state.clamp;
                            let count = state.params.len();
                            let Some(param) = state.params.get_mut(&name) else { continue };
                            param.value = if clamp { set.param_value - 1.0 } else { set.param_value };
                            let reply = value(&name, *param, count);
                            if state.arm_after_set {
                                // Both arrive before the echo: the armed transition must latch
                                // even though the latest fleet heartbeat is disarmed again.
                                out.push((1, 1, heartbeat(true)));
                                out.push((1, 1, heartbeat(false)));
                            }
                            if !state.omit_echo { out.push((1, 1, reply)); }
                        },
                        Message::ParamRequestRead(read) => {
                            state.reads += 1;
                            let name = decode_id(&read.param_id);
                            if let Some(param) = state.params.get(&name) {
                                let reply = value(&name, *param, state.params.len());
                                if state.wrong_sender {
                                    out.push((2, 1, reply.clone()));
                                    out.push((1, 100, reply));
                                } else if !state.omit_read { out.push((1, 1, reply)); }
                            }
                        },
                        _ => {},
                    }
                }
            }
        }
        for (sysid, compid, msg) in out {
            let frame = encode(Version::V2, Header { seq, sysid, compid }, &msg).unwrap();
            seq = seq.wrapping_add(1);
            if socket.write_all(frame.as_bytes()).await.is_err() {
                return;
            }
        }
    }
}

static NEXT: AtomicUsize = AtomicUsize::new(0);
fn path(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!(
        "dc-params-{name}-{}-{}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ))
}

async fn setup() -> (Core, Arc<Mutex<Vehicle>>, tokio::task::JoinHandle<()>) {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let state = Arc::new(Mutex::new(Vehicle {
        params: BTreeMap::from([
            (
                "A".into(),
                Param {
                    value: 7.0,
                    ty: MavParamType::INT16.0,
                    index: 0,
                },
            ),
            (
                "B".into(),
                Param {
                    value: 8.0,
                    ty: MavParamType::REAL32.0,
                    index: 1,
                },
            ),
            (
                "FS_EKF_THRESH".into(),
                Param {
                    value: 0.9,
                    ty: MavParamType::REAL32.0,
                    index: 2,
                },
            ),
            (
                "UNRELATED".into(),
                Param {
                    value: 10.0,
                    ty: MavParamType::REAL32.0,
                    index: 3,
                },
            ),
        ]),
        armed: false,
        silent: false,
        clamp: false,
        arm_after_set: false,
        omit_echo: false,
        omit_read: false,
        wrong_sender: false,
        silence_after_set: false,
        arm_during_download: false,
        snapshot: path("baseline"),
        snapshots_valid: Vec::new(),
        sets: Vec::new(),
        reads: 0,
    }));
    let task = tokio::spawn(vehicle(listener, state.clone()));
    let core = Core::start(CoreConfig::default()).await.unwrap();
    core.connect(&LinkSpec::Tcp(addr)).await.unwrap();
    assert_eq!(core.wait_for_vehicle(Duration::from_secs(5)).await, Some(1));
    (core, state, task)
}

fn edits() -> BTreeMap<String, f64> {
    BTreeMap::from([("A".into(), 9.0), ("B".into(), 12.0)])
}

#[tokio::test(flavor = "multi_thread")]
async fn persists_before_sets_explicitly_reads_back_and_restores_only_selected_parameters() {
    let (core, state, task) = setup().await;
    let baseline = state.lock().unwrap().snapshot.clone();
    let report = core.write_params((1, 1), edits(), &baseline).await.unwrap();
    assert!(report.verified(), "{report:?}");
    assert_eq!(report.results.len(), 2);
    assert_eq!(
        core.nav_thresholds(1).ekf_fail,
        0.9,
        "vehicle parameter overrides default"
    );
    assert_eq!(
        core.nav_thresholds(2).ekf_fail,
        0.8,
        "thresholds are per vehicle"
    );
    let original = ParamSnapshot::from_text(&std::fs::read_to_string(&baseline).unwrap()).unwrap();
    assert_eq!(original.params()["A"].value, 7.0);
    {
        let mut state = state.lock().unwrap();
        assert!(state.snapshots_valid.iter().all(|valid| *valid));
        assert_eq!(state.reads, 2, "matching echoes alone do not verify writes");
        state.params.get_mut("UNRELATED").unwrap().value = 99.0;
        state.snapshot = path("recovery");
    }
    let recovery = state.lock().unwrap().snapshot.clone();
    let names: Vec<_> = report.results.iter().map(|r| r.name.clone()).collect();
    let restored = core
        .rollback_params(&baseline, &names, &recovery)
        .await
        .unwrap();
    assert!(restored.verified(), "{restored:?}");
    {
        let state = state.lock().unwrap();
        assert_eq!(state.params["A"].value, 7.0);
        assert_eq!(state.params["B"].value, 8.0);
        assert_eq!(state.params["UNRELATED"].value, 99.0);
        assert!(state.snapshots_valid.iter().all(|valid| *valid));
    }
    task.abort();
    std::fs::remove_file(baseline).unwrap();
    std::fs::remove_file(recovery).unwrap();
}

#[tokio::test(flavor = "multi_thread")]
async fn mismatch_is_reported_and_stops_the_rest_of_the_batch() {
    let (core, state, task) = setup().await;
    state.lock().unwrap().clamp = true;
    let path = state.lock().unwrap().snapshot.clone();
    let report = core.write_params((1, 1), edits(), &path).await.unwrap();
    assert!(!report.verified());
    assert_eq!(report.planned, 2);
    assert_eq!(report.results.len(), 1);
    assert!(matches!(report.results[0].status, SetStatus::Failed(Some(p)) if p.value == 8.0));
    assert_eq!(state.lock().unwrap().sets, ["A"]);
    assert_eq!(state.lock().unwrap().params["B"].value, 8.0);
    assert!(path.exists(), "retain recovery baseline on failure");
    task.abort();
    std::fs::remove_file(path).unwrap();
}

#[tokio::test(flavor = "multi_thread")]
async fn unknown_armed_stale_and_non_autopilot_targets_send_no_sets() {
    let unknown = Core::start(CoreConfig::default()).await.unwrap();
    assert!(matches!(
        unknown.write_params((1, 1), edits(), path("unknown")).await,
        Err(ParamOperationError::Refused(_))
    ));
    let (core, state, task) = setup().await;
    let path = state.lock().unwrap().snapshot.clone();
    for target in [(0, 1), (1, 100)] {
        assert!(matches!(
            core.write_params(target, edits(), &path).await,
            Err(ParamOperationError::Refused(_))
        ));
    }
    state.lock().unwrap().armed = true;
    tokio::time::timeout(Duration::from_secs(3), async {
        while core.facts(1).armed != Some(true) {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
    // Armed: nothing is sent; the core hands back the warning for the operator.
    assert!(matches!(
        core.write_params((1, 1), edits(), &path).await,
        Err(ParamOperationError::ArmedWarningRequired(w)) if w == ARMED_WRITE_WARNING
    ));
    state.lock().unwrap().armed = false;
    tokio::time::timeout(Duration::from_secs(3), async {
        while core.facts(1).armed != Some(false) {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
    state.lock().unwrap().silent = true;
    tokio::time::sleep(PARAM_HEARTBEAT_MAX_AGE + Duration::from_millis(150)).await;
    assert!(matches!(
        core.write_params((1, 1), edits(), &path).await,
        Err(ParamOperationError::Refused(_))
    ));
    assert!(state.lock().unwrap().sets.is_empty());
    assert!(!path.exists());
    task.abort();
}

#[tokio::test(flavor = "multi_thread")]
async fn armed_transition_is_latched_even_if_disarmed_again_before_the_echo() {
    let (core, state, task) = setup().await;
    state.lock().unwrap().arm_after_set = true;
    let path = state.lock().unwrap().snapshot.clone();
    let report = core.write_params((1, 1), edits(), &path).await.unwrap();
    assert!(!report.verified(), "{report:?}");
    assert!(report.interruption.is_some());
    tokio::time::sleep(Duration::from_millis(1100)).await;
    assert_eq!(
        state.lock().unwrap().sets,
        ["A"],
        "no next write or retry after arming"
    );
    assert_eq!(state.lock().unwrap().params["B"].value, 8.0);
    task.abort();
    std::fs::remove_file(path).unwrap();
}

#[tokio::test(flavor = "multi_thread")]
async fn existing_or_unwritable_snapshot_path_refuses_all_sets() {
    let (core, state, task) = setup().await;
    let path = state.lock().unwrap().snapshot.clone();
    std::fs::write(&path, "user data").unwrap();
    assert!(matches!(
        core.write_params((1, 1), edits(), &path).await,
        Err(ParamOperationError::Storage(_))
    ));
    assert_eq!(std::fs::read_to_string(&path).unwrap(), "user data");
    let absent_parent = path.join("no-such-directory/snapshot");
    assert!(matches!(
        core.write_params((1, 1), edits(), absent_parent).await,
        Err(ParamOperationError::Storage(_))
    ));
    assert!(state.lock().unwrap().sets.is_empty());
    task.abort();
    std::fs::remove_file(path).unwrap();
}

#[tokio::test(flavor = "multi_thread")]
async fn matching_echo_without_readback_is_unconfirmed_and_concurrent_intents_are_refused() {
    let (core, state, task) = setup().await;
    state.lock().unwrap().omit_read = true;
    state.lock().unwrap().wrong_sender = true;
    let path = state.lock().unwrap().snapshot.clone();
    let writer = core.clone();
    let saved_path = path.clone();
    let operation =
        tokio::spawn(async move { writer.write_params((1, 1), edits(), saved_path).await });
    tokio::time::timeout(Duration::from_secs(5), async {
        while state.lock().unwrap().reads == 0 {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
    assert!(matches!(
        core.write_params((1, 1), edits(), path.join("second"))
            .await,
        Err(ParamOperationError::Busy)
    ));
    let report = operation.await.unwrap().unwrap();
    assert!(!report.verified());
    assert_eq!(report.results.len(), 1);
    assert_eq!(report.results[0].status, SetStatus::Failed(None));
    assert_eq!(
        state.lock().unwrap().sets,
        ["A"],
        "read retry must not repeat PARAM_SET"
    );
    assert!(state.lock().unwrap().reads >= 2);
    task.abort();
    std::fs::remove_file(path).unwrap();
}

#[tokio::test(flavor = "multi_thread")]
async fn rollback_refuses_changed_parameter_types_without_any_restoration_sets() {
    let (core, state, task) = setup().await;
    let baseline = state.lock().unwrap().snapshot.clone();
    assert!(
        core.write_params((1, 1), edits(), &baseline)
            .await
            .unwrap()
            .verified()
    );
    let sets_before = state.lock().unwrap().sets.len();
    state.lock().unwrap().params.get_mut("A").unwrap().ty = MavParamType::REAL32.0;
    let result = core
        .rollback_params(&baseline, &["A".into()], path("new-type"))
        .await;
    assert!(matches!(result, Err(ParamOperationError::Refused(_))));
    assert_eq!(state.lock().unwrap().sets.len(), sets_before);
    task.abort();
    std::fs::remove_file(baseline).unwrap();
}

#[tokio::test(flavor = "multi_thread")]
async fn dangerous_parameters_are_labeled_and_written_with_read_back() {
    for name in [
        "FS_EKF_THRESH",
        "FS_THR_ENABLE",
        "FENCE_ENABLE",
        "BATT_FS_LOW_ACT",
        "BATT2_FS_CRT_ACT",
        "RC_FS_TIMEOUT",
        "ARMING_CHECK",
        "BRD_SAFETY_DEFLT",
    ] {
        assert_eq!(parameter_risk(name), ParamRisk::Dangerous, "{name}");
    }
    assert_eq!(parameter_risk("WPNAV_SPEED"), ParamRisk::Ordinary);
    // ADR-0012 item 1: written like any other parameter (the label is for the confirmation),
    // with the same baseline, disarmed and read-back rules.
    let (core, state, task) = setup().await;
    let path = state.lock().unwrap().snapshot.clone();
    let report = core
        .write_params(
            (1, 1),
            BTreeMap::from([("FS_EKF_THRESH".into(), 0.0)]),
            &path,
        )
        .await
        .unwrap();
    assert!(report.verified(), "{report:?}");
    assert_eq!(state.lock().unwrap().sets, ["FS_EKF_THRESH"]);
    assert_eq!(state.lock().unwrap().params["FS_EKF_THRESH"].value, 0.0);
    assert!(path.exists(), "baseline kept for rollback");
    task.abort();
    std::fs::remove_file(path).unwrap();
}

#[tokio::test(flavor = "multi_thread")]
async fn heartbeat_staleness_stops_retries_and_the_next_parameter() {
    let (core, state, task) = setup().await;
    {
        let mut state = state.lock().unwrap();
        state.omit_echo = true;
        state.silence_after_set = true;
    }
    let path = state.lock().unwrap().snapshot.clone();
    let report = core.write_params((1, 1), edits(), &path).await.unwrap();
    assert!(!report.verified());
    assert!(report.interruption.is_some());
    let sent = state.lock().unwrap().sets.len();
    assert!(
        (1..=2).contains(&sent),
        "retry only while last disarmed report is fresh"
    );
    assert!(state.lock().unwrap().sets.iter().all(|name| name == "A"));
    state.lock().unwrap().silent = false;
    tokio::time::sleep(Duration::from_millis(1100)).await;
    assert_eq!(
        state.lock().unwrap().sets.len(),
        sent,
        "fresh reports do not revive a stopped transaction"
    );
    task.abort();
    std::fs::remove_file(path).unwrap();
}

#[tokio::test(flavor = "multi_thread")]
async fn armed_transition_during_baseline_download_refuses_before_any_set() {
    let (core, state, task) = setup().await;
    state.lock().unwrap().arm_during_download = true;
    let path = state.lock().unwrap().snapshot.clone();
    assert!(matches!(
        core.write_params((1, 1), edits(), &path).await,
        Err(ParamOperationError::Refused(_))
    ));
    assert!(state.lock().unwrap().sets.is_empty());
    assert!(!path.exists());
    task.abort();
}

async fn wait_armed(core: &Core, armed: bool) {
    tokio::time::timeout(Duration::from_secs(3), async {
        while core.facts(1).armed != Some(armed) {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
}

#[tokio::test(flavor = "multi_thread")]
async fn armed_write_needs_the_acknowledged_warning_then_is_verified() {
    // ADR-0012 (owner, 2026-10-13): parameter changes while armed and flying, after a
    // warning the operator acknowledges.
    let (core, state, task) = setup().await;
    state.lock().unwrap().armed = true;
    wait_armed(&core, true).await;
    let path = state.lock().unwrap().snapshot.clone();
    assert!(matches!(
        core.write_params_with((1, 1), edits(), &path, ArmedWrite::DisarmedOnly)
            .await,
        Err(ParamOperationError::ArmedWarningRequired(_))
    ));
    assert!(state.lock().unwrap().sets.is_empty() && !path.exists());
    let report = core
        .write_params_with((1, 1), edits(), &path, ArmedWrite::Acknowledged)
        .await
        .unwrap();
    assert!(report.verified(), "{report:?}");
    assert_eq!(state.lock().unwrap().sets, ["A", "B"]);
    assert!(
        state.lock().unwrap().reads >= 2,
        "each write read back explicitly"
    );
    assert_eq!(state.lock().unwrap().params["B"].value, 12.0);
    assert!(path.exists(), "baseline kept, as for any write");
    task.abort();
    std::fs::remove_file(path).unwrap();
}

#[tokio::test(flavor = "multi_thread")]
async fn acknowledged_write_continues_when_the_vehicle_arms_mid_batch() {
    let (core, state, task) = setup().await;
    state.lock().unwrap().arm_after_set = true;
    let path = state.lock().unwrap().snapshot.clone();
    let report = core
        .write_params_with((1, 1), edits(), &path, ArmedWrite::Acknowledged)
        .await
        .unwrap();
    assert!(report.verified(), "{report:?}");
    assert_eq!(state.lock().unwrap().sets, ["A", "B"]);
    task.abort();
    std::fs::remove_file(path).unwrap();
}

#[tokio::test(flavor = "multi_thread")]
async fn acknowledged_write_still_stops_on_a_heartbeat_gap() {
    let (core, state, task) = setup().await;
    state.lock().unwrap().armed = true;
    wait_armed(&core, true).await;
    {
        let mut state = state.lock().unwrap();
        state.omit_echo = true;
        state.silence_after_set = true;
    }
    let path = state.lock().unwrap().snapshot.clone();
    let report = core
        .write_params_with((1, 1), edits(), &path, ArmedWrite::Acknowledged)
        .await
        .unwrap();
    assert!(!report.verified());
    assert!(report.interruption.is_some(), "{report:?}");
    assert!(state.lock().unwrap().sets.iter().all(|name| name == "A"));
    task.abort();
    std::fs::remove_file(path).unwrap();
}
