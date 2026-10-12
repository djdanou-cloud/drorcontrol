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
