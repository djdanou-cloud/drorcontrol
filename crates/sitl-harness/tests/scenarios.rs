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
