//! SITL scenarios (plan 08 §2, T0-14): dc-core against a real ArduCopter SITL, directly and
//! through the link-chaos proxy. Skips without a SITL binary unless `DC_SITL_REQUIRED=1`.

use std::time::{Duration, Instant};

use dc_core::{Core, CoreConfig, CoreEvent, LinkSpec, VehicleLink};
use sitl_harness::chaos::{ChaosConfig, ChaosProxy};
use sitl_harness::{Sitl, SitlConfig};

async fn core_on(addr: std::net::SocketAddr) -> Core {
    let core = Core::start(CoreConfig::default()).await.unwrap();
    core.connect(&LinkSpec::Tcp(addr)).await.unwrap();
    core
}

async fn wait_link(core: &Core, want: VehicleLink, within: Duration) -> Duration {
    let t0 = Instant::now();
    tokio::time::timeout(within, async {
        while core.vehicle_link(1) != Some(want) {
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    })
    .await
    .unwrap_or_else(|_| panic!("vehicle link never {want}"));
    t0.elapsed()
}

#[tokio::test(flavor = "multi_thread")]
async fn connect_detects_the_vehicle_and_receives_telemetry() {
    let Some(cfg) = SitlConfig::copter_or_skip() else {
        return;
    };
    let sitl = Sitl::start(&cfg).unwrap();
    let core = core_on(sitl.addr()).await;
    assert_eq!(
        core.wait_for_vehicle(Duration::from_secs(60)).await,
        Some(1)
    );
    let hb = core.fleet().get(1).unwrap().heartbeat.clone().unwrap();
    assert_eq!(
        (hb.value.mav_type, hb.value.autopilot),
        (2, 3),
        "quadrotor, ArduPilot"
    );
    // The core asked for streams; attitude arrives.
    tokio::time::timeout(Duration::from_secs(20), async {
        while core
            .fleet()
            .get(1)
            .and_then(|v| v.attitude.clone())
            .is_none()
        {
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("ATTITUDE within 20 s");
    assert_eq!(core.vehicle_link(1), Some(VehicleLink::Connected));
    // EXTENDED_SYS_STATE is requested too: a copter on the ground says so (ADR-0011 I2).
    tokio::time::timeout(Duration::from_secs(20), async {
        while core.facts(1).landed_state.is_none() {
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("EXTENDED_SYS_STATE within 20 s");
    let ext = core.fleet().get(1).unwrap().extended.clone().unwrap().value;
    assert_eq!(ext.landed, dc_core::LandedState::OnGround);
    assert_eq!(core.facts(1).landed_state.map(|(s, _)| s), Some(1));
}

#[tokio::test(flavor = "multi_thread")]
async fn parameter_download_is_complete() {
    let Some(cfg) = SitlConfig::copter_or_skip() else {
        return;
    };
    let sitl = Sitl::start(&cfg).unwrap();
    let core = core_on(sitl.addr()).await;
    assert_eq!(
        core.wait_for_vehicle(Duration::from_secs(60)).await,
        Some(1)
    );
    let mut total = 0;
    let t0 = Instant::now();
    let params = core
        .download_params((1, 1), |_, t| total = t)
        .await
        .expect("complete download");
    eprintln!(
        "{} parameters in {:.1} s",
        params.len(),
        t0.elapsed().as_secs_f32()
    );
    assert!(
        params.len() > 1000,
        "a copter has >1000 parameters: {}",
        params.len()
    );
    assert_eq!(params.len(), total, "every index the vehicle announced");
    // ArduPilot 4.7 renamed SYSID_THISMAV to MAV_SYSID.
    assert_eq!(params["MAV_SYSID"].value, 1.0);
    assert_eq!(params["FRAME_CLASS"].value, 1.0, "quad, from copter.parm");
}

/// T0-18 / H5: persist baseline, set a non-dangerous parameter, explicitly read it back,
/// and restore it through the same verified path with real 30 % bidirectional frame loss.
#[tokio::test(flavor = "multi_thread")]
async fn parameter_write_and_rollback_are_verified_through_30_percent_loss() {
    use dc_core::ParamSnapshot;
    use std::collections::BTreeMap;
    use std::sync::atomic::Ordering;

    let Some(cfg) = SitlConfig::copter_or_skip() else {
        return;
    };
    let sitl = Sitl::start(&cfg).unwrap();
    let chaos = ChaosProxy::start(
        sitl.addr(),
        ChaosConfig {
            drop: 0.30,
            seed: 0xD0C_2018,
            ..Default::default()
        },
    )
    .await
    .unwrap();
    let mut config = CoreConfig::default();
    // The disarmed heartbeat must stay fresh during writes even on a lossy link. Ask the
    // simulator for 5 Hz heartbeats (supported by pinned GCS_Common.cpp's message map),
    // rather than relaxing the production 1.5 s freshness rule to make this test pass.
    config.stream_rates.push((0, 5.0));
    let core = Core::start(config).await.unwrap();
    core.connect(&LinkSpec::Tcp(chaos.addr())).await.unwrap();
    assert_eq!(
        core.wait_for_vehicle(Duration::from_secs(60)).await,
        Some(1)
    );
    for _ in 0..3 {
        core.request_streams(1, 1).await;
        tokio::time::sleep(Duration::from_millis(250)).await;
    }
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!(
        "dc-sitl-param-write-{}-{stamp}",
        std::process::id()
    ));
    std::fs::create_dir(&dir).unwrap();
    let baseline_path = dir.join("original.snapshot");
    let recovery_path = dir.join("rollback.snapshot");
    // Copter 4.7.1 renamed WPNAV_SPEED (cm/s) to WP_SPD (m/s); see AC_WPNav.cpp and
    // the WP_ group in ArduCopter/Parameters.cpp. Change only this simulator's setting.
    let report = core
        .write_params(
            (1, 1),
            BTreeMap::from([("WP_SPD".into(), 6.5)]),
            &baseline_path,
        )
        .await
        .unwrap();
    assert!(report.verified(), "{report:?}");
    assert_eq!(
        report.results.len(),
        1,
        "the test must actually change a parameter"
    );
    let original =
        ParamSnapshot::from_text(&std::fs::read_to_string(&baseline_path).unwrap()).unwrap();
    assert_ne!(original.params()["WP_SPD"].value, 6.5);
    assert_eq!(report.results[0].name, "WP_SPD");
    let restored = core
        .rollback_params(&baseline_path, &["WP_SPD".into()], &recovery_path)
        .await
        .unwrap();
    assert!(restored.verified(), "{restored:?}");
    assert!(
        matches!(restored.results[0].status, dc_core::SetStatus::Verified(p) if p.value == original.params()["WP_SPD"].value)
    );
    let dropped = chaos.stats().dropped.load(Ordering::Relaxed);
    assert!(dropped > 100, "the link really dropped frames: {dropped}");
    eprintln!("WP_SPD set and rollback verified through 30 % loss; {dropped} frames dropped");
    std::fs::remove_dir_all(dir).unwrap();
}

/// Plan 08 §2: "full download with 30 % loss (must complete)".
#[tokio::test(flavor = "multi_thread")]
async fn parameter_download_completes_through_30_percent_loss() {
    let Some(cfg) = SitlConfig::copter_or_skip() else {
        return;
    };
    let sitl = Sitl::start(&cfg).unwrap();
    let seed = 0xD0C_2026;
    let chaos = ChaosProxy::start(
        sitl.addr(),
        ChaosConfig {
            drop: 0.30,
            jitter: Duration::from_millis(20),
            seed,
            ..Default::default()
        },
    )
    .await
    .unwrap();
    eprintln!("chaos seed {seed:#x}");
    let core = core_on(chaos.addr()).await;
    assert_eq!(
        core.wait_for_vehicle(Duration::from_secs(60)).await,
        Some(1)
    );
    let mut total = 0;
    let t0 = Instant::now();
    let params = core
        .download_params((1, 1), |_, t| total = t)
        .await
        .expect("complete despite 30 % loss");
    let dropped = chaos
        .stats()
        .dropped
        .load(std::sync::atomic::Ordering::Relaxed);
    eprintln!(
        "{} parameters in {:.1} s through 30 % loss ({dropped} frames dropped)",
        params.len(),
        t0.elapsed().as_secs_f32()
    );
    assert_eq!(params.len(), total);
    assert!(dropped > 100, "the link really was lossy: {dropped}");
}

/// A radio fade: heartbeats stop, the GCS shows STALE then LOST (never stale data as live),
/// and CONNECTED again when the link returns.
#[tokio::test(flavor = "multi_thread")]
async fn radio_fade_goes_stale_then_lost_then_recovers() {
    let Some(cfg) = SitlConfig::copter_or_skip() else {
        return;
    };
    let sitl = Sitl::start(&cfg).unwrap();
    let chaos = ChaosProxy::start(sitl.addr(), ChaosConfig::default())
        .await
        .unwrap();
    let core = core_on(chaos.addr()).await;
    let mut events = core.events();
    let seen = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let log = seen.clone();
    tokio::spawn(async move {
        loop {
            match events.recv().await {
                Ok(CoreEvent::VehicleLink {
                    sysid: 1, state, ..
                }) => log.lock().unwrap().push(state),
                Ok(_) | Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {}
                Err(_) => return,
            }
        }
    });
    assert_eq!(
        core.wait_for_vehicle(Duration::from_secs(60)).await,
        Some(1)
    );
    wait_link(&core, VehicleLink::Connected, Duration::from_secs(10)).await;

    chaos.blackout(true);
    let to_stale = wait_link(&core, VehicleLink::Stale, Duration::from_secs(5)).await;
    let to_lost = wait_link(&core, VehicleLink::Lost, Duration::from_secs(8)).await;
    eprintln!(
        "STALE after {:.1} s, LOST {:.1} s later",
        to_stale.as_secs_f32(),
        to_lost.as_secs_f32()
    );
    chaos.blackout(false);
    wait_link(&core, VehicleLink::Connected, Duration::from_secs(10)).await;

    tokio::time::sleep(Duration::from_millis(200)).await;
    let seen = seen.lock().unwrap().clone();
    assert!(
        seen.windows(3).any(|w| w
            == [
                VehicleLink::Stale,
                VehicleLink::Lost,
                VehicleLink::Connected
            ]),
        "events in order: {seen:?}"
    );
}

/// The TCP connection drops; the client link reconnects by itself and the vehicle returns.
#[tokio::test(flavor = "multi_thread")]
async fn reconnects_after_the_connection_is_cut() {
    let Some(cfg) = SitlConfig::copter_or_skip() else {
        return;
    };
    let sitl = Sitl::start(&cfg).unwrap();
    let chaos = ChaosProxy::start(sitl.addr(), ChaosConfig::default())
        .await
        .unwrap();
    let core = core_on(chaos.addr()).await;
    let mut events = core.events();
    assert_eq!(
        core.wait_for_vehicle(Duration::from_secs(60)).await,
        Some(1)
    );

    chaos.cut();
    let mut down = false;
    let mut up_again = false;
    tokio::time::timeout(Duration::from_secs(20), async {
        while !(down && up_again) {
            if let Ok(CoreEvent::Link { up, .. }) = events.recv().await {
                if !up {
                    down = true;
                } else if down {
                    up_again = true;
                }
            }
        }
    })
    .await
    .expect("link down then up again");
    // Heartbeats flow again over the new connection.
    tokio::time::sleep(Duration::from_millis(300)).await;
    wait_link(&core, VehicleLink::Connected, Duration::from_secs(10)).await;
    let age = core
        .fleet()
        .get(1)
        .unwrap()
        .heartbeat
        .as_ref()
        .unwrap()
        .age(Instant::now());
    assert!(
        age < Duration::from_secs(2),
        "fresh heartbeat after reconnect: {age:?}"
    );
}

/// The banner confirms the firmware, so modes are named from the metadata; a mode change is a
/// single action (ADR-0011), labelled with the name and verified by the vehicle's heartbeat.
#[tokio::test(flavor = "multi_thread")]
async fn firmware_is_confirmed_and_a_named_mode_change_is_verified() {
    use dc_core::{Approval, Command, CommandState, Firmware, Request};
    let Some(cfg) = SitlConfig::copter_or_skip() else {
        return;
    };
    let sitl = Sitl::start(&cfg).unwrap();
    let core = core_on(sitl.addr()).await;
    assert_eq!(
        core.wait_for_vehicle(Duration::from_secs(60)).await,
        Some(1)
    );
    tokio::time::timeout(Duration::from_secs(20), async {
        while core
            .fleet()
            .get(1)
            .and_then(|v| v.confirmed_firmware())
            .is_none()
        {
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("version banner within 20 s");
    let v = core.fleet().get(1).unwrap().clone();
    assert_eq!(v.confirmed_firmware(), Some(Firmware::Copter));
    assert_eq!(
        v.mode_name(),
        Some("STABILIZE"),
        "copter.parm starts in STABILIZE"
    );

    let loiter = Firmware::Copter.mode_number("LOITER").unwrap();
    let mode = Command::SetMode {
        custom_mode: loiter,
    };
    assert!(matches!(
        core.request_command(1, mode.clone()).await.unwrap(),
        Request::Ready { .. }
    ));
    let id = core
        .submit_command(1, mode, Approval::None)
        .await
        .expect("single action");
    let status = tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            let s = core.command_status(id).unwrap();
            if s.state.is_final() {
                return s;
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    })
    .await
    .expect("final within the 3 s lifetime");
    assert_eq!(status.state, CommandState::Verified, "{}", status.summary());
    assert_eq!(status.summary(), "MODE LOITER (5): VERIFIED");
    assert_eq!(core.fleet().get(1).unwrap().mode_name(), Some("LOITER"));
}

/// Hazard H16: the GCS mirrors the autopilot's EKF. With GPS the position is trusted; when the
/// GPS fails the core says the position cannot be trusted and raises a Warning.
#[tokio::test(flavor = "multi_thread")]
async fn gps_loss_makes_the_position_untrusted() {
    use dc_core::{AlertLevel, PositionTrust};
    use mavlink_codec::Message;
    use mavlink_codec::messages::ParamSet;
    let Some(cfg) = SitlConfig::copter_or_skip() else {
        return;
    };
    let sitl = Sitl::start(&cfg).unwrap();
    let core = core_on(sitl.addr()).await;
    let mut events = core.events();
    assert_eq!(
        core.wait_for_vehicle(Duration::from_secs(60)).await,
        Some(1)
    );
    let t0 = Instant::now();
    tokio::time::timeout(Duration::from_secs(90), async {
        while core.nav_health(1).trust != PositionTrust::Good {
            tokio::time::sleep(Duration::from_millis(200)).await;
        }
    })
    .await
    .unwrap_or_else(|_| panic!("EKF never good: {:?}", core.nav_health(1)));
    eprintln!("position good after {:.1} s", t0.elapsed().as_secs_f32());
    while events.try_recv().is_ok() {}

    // Fail the simulated GPS.
    let mut param_id = [0u8; 16];
    param_id[..15].copy_from_slice(b"SIM_GPS1_ENABLE");
    core.send(&Message::ParamSet(ParamSet {
        target_system: 1,
        target_component: 1,
        param_id,
        param_value: 0.0,
        param_type: 9,
    }))
    .await;
    let t1 = Instant::now();
    let warning = tokio::time::timeout(Duration::from_secs(60), async {
        loop {
            if let Ok(dc_core::CoreEvent::Alert {
                level: AlertLevel::Warning,
                text,
                ..
            }) = events.recv().await
                && text.starts_with("POSITION UNTRUSTED")
            {
                return text;
            }
        }
    })
    .await
    .unwrap_or_else(|_| panic!("no untrusted warning; health {:?}", core.nav_health(1)));
    eprintln!("{warning} after {:.1} s", t1.elapsed().as_secs_f32());
    assert_eq!(core.nav_health(1).trust, PositionTrust::Untrusted);
}

/// Hazard H4: a mission uploaded through a lossy link (20 % drop each way) is read back and
/// verified item by item on a real ArduCopter.
#[tokio::test(flavor = "multi_thread")]
async fn mission_upload_is_verified_by_read_back_through_loss() {
    use dc_core::{MissionItem, TransferStatus};
    let Some(cfg) = SitlConfig::copter_or_skip() else {
        return;
    };
    let sitl = Sitl::start(&cfg).unwrap();
    let chaos = ChaosProxy::start(
        sitl.addr(),
        ChaosConfig {
            drop: 0.20,
            jitter: Duration::from_millis(20),
            seed: 0x4D15,
            ..Default::default()
        },
    )
    .await
    .unwrap();
    let core = core_on(chaos.addr()).await;
    assert_eq!(
        core.wait_for_vehicle(Duration::from_secs(60)).await,
        Some(1)
    );
    // Wait for a heartbeat that says disarmed (uploads are refused otherwise).
    tokio::time::timeout(Duration::from_secs(20), async {
        // Disarmed, and finished booting (ArduPilot reports MAV_STATE_BOOT until its mission
        // storage is ready; uploading earlier is refused).
        while core.facts(1).armed != Some(false)
            || core
                .fleet()
                .get(1)
                .and_then(|v| v.heartbeat.clone())
                .is_none_or(|h| h.value.system_status == 1)
        {
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("disarmed");
    let wp = |lat: f64, lon: f64, alt: f32| MissionItem {
        frame: 6,
        command: 16,
        params: [0.0; 4],
        x: (lat * 1e7) as i32,
        y: (lon * 1e7) as i32,
        z: alt,
        autocontinue: true,
    };
    let home = core.fleet().get(1).and_then(|v| v.position.clone());
    let (lat0, lon0) = home.map_or((-35.363, 149.165), |p| (p.value.lat_deg, p.value.lon_deg));
    let mut mission = vec![wp(lat0, lon0, 0.0)];
    mission.push(MissionItem {
        command: 22,
        ..wp(0.0, 0.0, 20.0)
    });
    for i in 0..10 {
        mission.push(wp(
            lat0 + 0.0005 * f64::from(i),
            lon0 + 0.0003,
            25.0 + i as f32,
        ));
    }
    mission.push(MissionItem {
        command: 19,
        params: [10.0, 0.0, 0.0, 0.0],
        ..wp(lat0, lon0, 30.0)
    });
    mission.push(MissionItem {
        command: 20,
        frame: 2,
        ..wp(0.0, 0.0, 0.0)
    });
    let t0 = Instant::now();
    let report = core.upload_mission((1, 1), mission.clone()).await.unwrap();
    eprintln!(
        "{} items: upload {:?}, read-back {:?}, {} mismatches, {:.1} s",
        mission.len(),
        report.upload,
        report.readback,
        report.mismatches.len(),
        t0.elapsed().as_secs_f32()
    );
    assert_eq!(report.readback, TransferStatus::Done);
    assert!(report.verified(), "{:?}", report.mismatches);
    assert_eq!(report.on_vehicle.len(), mission.len());
}

/// Wait until `pred` holds, polling every 100 ms; panics with `what` after `within`.
async fn until(within: Duration, what: &str, mut pred: impl FnMut() -> bool) {
    tokio::time::timeout(within, async {
        while !pred() {
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .unwrap_or_else(|_| panic!("{what} not within {within:?}"));
}

/// Submit a single-action (class B) command and wait for its final state.
async fn command(core: &Core, cmd: dc_core::Command) -> dc_core::CommandStatus {
    let id = core
        .submit_command(1, cmd, dc_core::Approval::None)
        .await
        .expect("class B: no confirmation");
    tokio::time::timeout(Duration::from_secs(15), async {
        loop {
            let s = core.command_status(id).unwrap();
            if s.state.is_final() {
                return s;
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    })
    .await
    .expect("final within its lifetime")
}

/// Plan 08 §2 "Flight" and "Failsafe & lost link": a real flight through the command pipeline.
/// GUIDED, arm and take off are single actions (ADR-0011) verified by the vehicle's state; a
/// radio blackout makes the GCS show LOST while the vehicle's own GCS failsafe takes it home
/// (RTL); when the link returns the GCS shows CONNECTED and the vehicle in RTL; it lands and
/// disarms by itself.
#[tokio::test(flavor = "multi_thread")]
async fn flight_with_gcs_failsafe_on_link_loss() {
    use dc_core::{Command, CommandState, PositionTrust};
    use mavlink_codec::Message;
    use mavlink_codec::messages::ParamSet;
    let Some(cfg) = SitlConfig::copter_or_skip() else {
        return;
    };
    let sitl = Sitl::start(&cfg).unwrap();
    let chaos = ChaosProxy::start(sitl.addr(), ChaosConfig::default())
        .await
        .unwrap();
    let core = core_on(chaos.addr()).await;
    assert_eq!(
        core.wait_for_vehicle(Duration::from_secs(60)).await,
        Some(1)
    );
    until(Duration::from_secs(90), "position good (EKF)", || {
        core.nav_health(1).trust == PositionTrust::Good
    })
    .await;
    until(Duration::from_secs(20), "firmware banner", || {
        core.fleet()
            .get(1)
            .and_then(|v| v.confirmed_firmware())
            .is_some()
    })
    .await;

    // Pre-flight setup an operator would do: GCS failsafe on (RTL), 5 s timeout (default).
    // A raw PARAM_SET as test configuration: the product's guarded write path refuses
    // failsafe parameters by design (T0-18).
    let mut param_id = [0u8; 16];
    param_id[..13].copy_from_slice(b"FS_GCS_ENABLE");
    core.send(&Message::ParamSet(ParamSet {
        target_system: 1,
        target_component: 1,
        param_id,
        param_value: 1.0,
        param_type: 2,
    }))
    .await;
    tokio::time::sleep(Duration::from_millis(500)).await;

    let guided = dc_core::Firmware::Copter.mode_number("GUIDED").unwrap();
    let rtl = dc_core::Firmware::Copter.mode_number("RTL").unwrap();
    let s = command(
        &core,
        Command::SetMode {
            custom_mode: guided,
        },
    )
    .await;
    assert_eq!(s.state, CommandState::Verified, "{}", s.summary());

    // Pre-arm checks can take a few seconds after the EKF converges; arming is retried as an
    // operator would, each attempt a single action.
    let t0 = Instant::now();
    loop {
        let s = command(&core, Command::Arm).await;
        if s.state == CommandState::Verified {
            break;
        }
        assert!(
            t0.elapsed() < Duration::from_secs(60),
            "never armed: {}",
            s.summary()
        );
        tokio::time::sleep(Duration::from_secs(2)).await;
    }
    let s = command(&core, Command::Takeoff { alt_m: 15.0 }).await;
    assert!(
        matches!(s.state, CommandState::Verified | CommandState::Accepted),
        "{}",
        s.summary()
    );
    let alt = || {
        core.fleet()
            .get(1)
            .and_then(|v| v.position.clone())
            .map_or(0.0, |p| p.value.alt_rel_m)
    };
    until(Duration::from_secs(60), "climb to 12 m", || alt() > 12.0).await;
    eprintln!("airborne at {:.1} m", alt());

    // Radio blackout, both ways, longer than FS_GCS_TIMEOUT.
    chaos.blackout(true);
    wait_link(&core, VehicleLink::Lost, Duration::from_secs(10)).await;
    tokio::time::sleep(Duration::from_secs(8)).await;
    chaos.blackout(false);
    wait_link(&core, VehicleLink::Connected, Duration::from_secs(10)).await;
    until(
        Duration::from_secs(5),
        "mode RTL shown after the link returns",
        || core.fleet().get(1).and_then(|v| v.mode_name()) == Some("RTL"),
    )
    .await;
    eprintln!("link back: vehicle in RTL at {:.1} m", alt());
    assert_eq!(
        core.fleet()
            .get(1)
            .unwrap()
            .heartbeat
            .as_ref()
            .unwrap()
            .value
            .custom_mode,
        rtl
    );

    // RTL lands at home and disarms by itself.
    until(Duration::from_secs(120), "landed and disarmed", || {
        core.facts(1).armed == Some(false)
    })
    .await;
    assert_eq!(
        core.facts(1).landed_state.map(|(s, _)| s),
        Some(1),
        "ON_GROUND"
    );
}

/// Plan 08 §2 "Flight" with a mission: upload (verified by read-back), arm and climb in GUIDED,
/// then AUTO flies the mission: take off to 20 m, two waypoints, RTL. The GCS's mission
/// progress (`MISSION_CURRENT`, `MISSION_ITEM_REACHED`) follows the vehicle through every item,
/// and the vehicle lands at home and disarms by itself.
#[tokio::test(flavor = "multi_thread")]
async fn auto_mission_flight_reports_progress_to_landing() {
    use dc_core::MissionState;
    use dc_core::{Command, CommandState, MissionItem, PositionTrust};
    let Some(cfg) = SitlConfig::copter_or_skip() else {
        return;
    };
    let sitl = Sitl::start(&cfg).unwrap();
    let core = core_on(sitl.addr()).await;
    assert_eq!(
        core.wait_for_vehicle(Duration::from_secs(60)).await,
        Some(1)
    );
    until(Duration::from_secs(90), "position good (EKF)", || {
        core.nav_health(1).trust == PositionTrust::Good
    })
    .await;
    until(Duration::from_secs(20), "firmware banner, booted", || {
        core.fleet().get(1).is_some_and(|v| {
            v.confirmed_firmware().is_some()
                && v.heartbeat
                    .as_ref()
                    .is_some_and(|h| h.value.system_status != 1)
        })
    })
    .await;

    let (lat0, lon0) = core
        .fleet()
        .get(1)
        .and_then(|v| v.position.clone())
        .map(|p| (p.value.lat_deg, p.value.lon_deg))
        .expect("position");
    // HOME_POSITION: the harness starts SITL at -35.363261, 149.165230, 584 m.
    until(Duration::from_secs(20), "HOME_POSITION", || {
        core.fleet().get(1).is_some_and(|v| v.home.is_some())
    })
    .await;
    let home = core.fleet().get(1).unwrap().home.clone().unwrap().value;
    assert!(
        (home.lat_deg + 35.363_261).abs() < 1e-5
            && (home.lon_deg - 149.165_230).abs() < 1e-5
            && (home.alt_msl_m - 584.0).abs() < 2.0,
        "{home:?}"
    );
    let item = |command: u16, frame: u8, lat: f64, lon: f64, alt: f32| MissionItem {
        frame,
        command,
        params: [0.0; 4],
        x: (lat * 1e7) as i32,
        y: (lon * 1e7) as i32,
        z: alt,
        autocontinue: true,
    };
    // Home (slot 0), TAKEOFF, two waypoints about 55 m apart, RTL.
    let mission = vec![
        item(16, 0, lat0, lon0, 0.0),
        item(22, 3, 0.0, 0.0, 20.0),
        item(16, 3, lat0 + 0.0005, lon0, 20.0),
        item(16, 3, lat0 + 0.0005, lon0 + 0.0006, 20.0),
        item(20, 2, 0.0, 0.0, 0.0),
    ];
    let report = core.upload_mission((1, 1), mission.clone()).await.unwrap();
    assert!(report.verified(), "{:?}", report.mismatches);
    // Saved as a Mission Planner .waypoints file and read again, the mission still matches
    // what the vehicle holds (what `dc-cli mission compare` checks).
    let file = protocols::wpl::to_string(&mission);
    let from_file = protocols::wpl::parse(&file).unwrap();
    assert!(
        protocols::mission::verify(&from_file, &report.on_vehicle, true).is_empty(),
        "{file}"
    );
    until(
        Duration::from_secs(10),
        "MISSION_CURRENT shows the mission",
        || {
            core.fleet()
                .get(1)
                .and_then(|v| v.mission.clone())
                .is_some_and(|m| m.value.total == Some(4))
        },
    )
    .await;

    let mode = |name| dc_core::Firmware::Copter.mode_number(name).unwrap();
    let s = command(
        &core,
        Command::SetMode {
            custom_mode: mode("GUIDED"),
        },
    )
    .await;
    assert_eq!(s.state, CommandState::Verified, "{}", s.summary());
    let t0 = Instant::now();
    loop {
        let s = command(&core, Command::Arm).await;
        if s.state == CommandState::Verified {
            break;
        }
        assert!(
            t0.elapsed() < Duration::from_secs(60),
            "never armed: {}",
            s.summary()
        );
        tokio::time::sleep(Duration::from_secs(2)).await;
    }
    let s = command(&core, Command::Takeoff { alt_m: 10.0 }).await;
    assert!(
        matches!(s.state, CommandState::Verified | CommandState::Accepted),
        "{}",
        s.summary()
    );
    let alt = || {
        core.fleet()
            .get(1)
            .and_then(|v| v.position.clone())
            .map_or(0.0, |p| p.value.alt_rel_m)
    };
    until(Duration::from_secs(60), "climb to 8 m", || alt() > 8.0).await;
    let s = command(
        &core,
        Command::SetMode {
            custom_mode: mode("AUTO"),
        },
    )
    .await;
    assert_eq!(s.state, CommandState::Verified, "{}", s.summary());

    // Follow the mission until the vehicle is down and disarmed, recording what the GCS saw.
    let (mut max_current, mut reached, mut active) = (0u16, Vec::<u16>::new(), false);
    let t0 = Instant::now();
    while core.facts(1).armed != Some(false) {
        assert!(
            t0.elapsed() < Duration::from_secs(240),
            "mission not finished"
        );
        let v = core.fleet().get(1).unwrap().clone();
        if let Some(m) = v.mission {
            if m.value.current > max_current {
                eprintln!(
                    "{:5.1} s: item {} ({:?}), {:.1} m",
                    t0.elapsed().as_secs_f32(),
                    m.value.current,
                    m.value.state,
                    alt()
                );
            }
            max_current = max_current.max(m.value.current);
            active |=
                m.value.state == MissionState::Active && m.value.in_mission_mode == Some(true);
        }
        if let Some(r) = v.mission_reached
            && reached.last() != Some(&r.value)
        {
            reached.push(r.value);
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    eprintln!(
        "disarmed after {:.1} s; reached {reached:?}",
        t0.elapsed().as_secs_f32()
    );
    assert!(active, "MISSION_STATE_ACTIVE in mission mode never seen");
    assert_eq!(max_current, 4, "progress reached the RTL item");
    for wp in [2, 3] {
        assert!(
            reached.contains(&wp),
            "waypoint {wp} reported reached: {reached:?}"
        );
    }
    assert_eq!(
        core.facts(1).landed_state.map(|(s, _)| s),
        Some(1),
        "ON_GROUND"
    );
    until(Duration::from_secs(5), "MISSION_STATE_COMPLETE", || {
        core.fleet()
            .get(1)
            .and_then(|v| v.mission.clone())
            .is_some_and(|m| m.value.state == MissionState::Complete)
    })
    .await;
}

/// Test setup only: put a list on the vehicle with the raw mission protocol (the product has
/// no fence or rally upload; a geofence is a failsafe boundary and needs its own safety
/// review before it gets one).
async fn raw_upload(core: &Core, kind: dc_core::MissionType, items: Vec<dc_core::MissionItem>) {
    use protocols::mission::{MissionUpload, TransferConfig};
    let mut rx = core.subscribe();
    let (mut up, first) = MissionUpload::start(
        (1, 1),
        items,
        kind.mavlink(),
        TransferConfig::default(),
        Instant::now(),
    );
    core.send(&first).await;
    let t0 = Instant::now();
    while matches!(up.status(), dc_core::TransferStatus::InProgress { .. }) {
        assert!(
            t0.elapsed() < Duration::from_secs(30),
            "{kind:?} upload stuck"
        );
        let out = match tokio::time::timeout(Duration::from_millis(100), rx.recv()).await {
            Ok(Ok(inc)) if inc.sysid == 1 && inc.compid == 1 => {
                up.on_message(&inc.msg, Instant::now())
            }
            _ => up.poll(Instant::now()),
        };
        for m in out {
            core.send(&m).await;
        }
    }
    assert_eq!(
        up.status(),
        &dc_core::TransferStatus::Done,
        "{kind:?} upload"
    );
}

/// Geofence and rally points are read back as the vehicle holds them (read-only download,
/// `dc-cli mission download --type fence|rally`).
#[tokio::test(flavor = "multi_thread")]
async fn fence_and_rally_points_are_downloaded() {
    use dc_core::{MissionItem, MissionType, TransferStatus};
    let Some(cfg) = SitlConfig::copter_or_skip() else {
        return;
    };
    let sitl = Sitl::start(&cfg).unwrap();
    let core = core_on(sitl.addr()).await;
    assert_eq!(
        core.wait_for_vehicle(Duration::from_secs(60)).await,
        Some(1)
    );
    until(Duration::from_secs(30), "booted", || {
        core.fleet()
            .get(1)
            .and_then(|v| v.heartbeat.clone())
            .is_some_and(|h| h.value.system_status != 1)
    })
    .await;
    let (lat, lon) = (-35.363261, 149.165230);
    let at = |command: u16, frame: u8, dlat: f64, dlon: f64, p1: f32, z: f32| MissionItem {
        frame,
        command,
        params: [p1, 0.0, 0.0, 0.0],
        x: ((lat + dlat) * 1e7).round() as i32,
        y: ((lon + dlon) * 1e7).round() as i32,
        z,
        autocontinue: false,
    };
    // A 4-vertex inclusion polygon (about 220 m x 180 m), a return point, a 30 m exclusion circle.
    let fence = vec![
        at(5001, 5, -0.001, -0.001, 4.0, 0.0),
        at(5001, 5, 0.001, -0.001, 4.0, 0.0),
        at(5001, 5, 0.001, 0.001, 4.0, 0.0),
        at(5001, 5, -0.001, 0.001, 4.0, 0.0),
        at(5000, 5, 0.0, 0.0005, 0.0, 0.0),
        at(5003, 5, 0.0005, 0.0, 30.0, 0.0),
    ];
    let rally = vec![at(5100, 6, 0.0008, 0.0002, 0.0, 30.0)];
    raw_upload(&core, MissionType::Fence, fence.clone()).await;
    raw_upload(&core, MissionType::Rally, rally.clone()).await;

    let (status, got) = core.download_items((1, 1), MissionType::Fence).await;
    assert_eq!(status, TransferStatus::Done);
    let key = |i: &MissionItem| (i.command, i.x, i.y, i.params[0]);
    assert_eq!(
        got.iter().map(key).collect::<Vec<_>>(),
        fence.iter().map(key).collect::<Vec<_>>(),
        "fence: command, position and param1 (vertex count / radius) as uploaded"
    );
    let (status, got) = core.download_items((1, 1), MissionType::Rally).await;
    assert_eq!(status, TransferStatus::Done);
    assert_eq!(got.len(), 1);
    assert_eq!(
        (got[0].command, got[0].x, got[0].y, got[0].z),
        (5100, rally[0].x, rally[0].y, 30.0)
    );
}

/// ADS-B traffic reaches the GCS: SITL's simulated receiver (uAvionix MAVLink on SERIAL4)
/// reports aircraft, ArduPilot relays them as ADSB_VEHICLE when the GCS asks for the
/// message, and every one arrives marked as simulated.
#[tokio::test(flavor = "multi_thread")]
async fn adsb_traffic_is_reported() {
    let Some(mut cfg) = SitlConfig::copter_or_skip() else {
        return;
    };
    // ADSB_TYPE 1 = uAvionix MAVLink, SERIAL4_PROTOCOL 35 = ADSB (ArduPilot 4.7.1).
    cfg.params = [
        ("ADSB_TYPE", 1.0),
        ("SERIAL4_PROTOCOL", 35.0),
        ("SIM_ADSB_COUNT", 4.0),
        ("SIM_ADSB_RADIUS", 3000.0),
        ("SIM_ADSB_TYPES", 1.0),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_string(), v))
    .collect();
    cfg.args = vec!["--serial4=sim:adsb".into()];
    let sitl = Sitl::start(&cfg).unwrap();
    let core = core_on(sitl.addr()).await;
    assert_eq!(
        core.wait_for_vehicle(Duration::from_secs(60)).await,
        Some(1)
    );
    until(Duration::from_secs(60), "two aircraft", || {
        core.fleet().get(1).is_some_and(|v| v.traffic.len() >= 2)
    })
    .await;
    let v = core.fleet().get(1).unwrap().clone();
    for t in v.traffic.values().map(|t| &t.value) {
        assert!(t.simulated, "{t:?}");
        assert!(
            t.callsign.as_deref().is_some_and(|c| c.starts_with("SIM")),
            "{t:?}"
        );
        // Near home: SITL re-spawns aircraft beyond SIM_ADSB_RADIUS (3 km); 0.1° leaves margin
        // for a random spawn point. With an altitude.
        assert!((t.lat_deg + 35.363_261).abs() < 0.1 && (t.lon_deg - 149.165_23).abs() < 0.1);
        assert!(t.alt_msl_m.is_some(), "{t:?}");
    }
}
