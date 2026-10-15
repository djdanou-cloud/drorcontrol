//! CommandPolicy through the core (T0-16) against a fake vehicle on TCP: confirmation,
//! delivery, ACK + state verification, ADR-0009 behaviour on a lost link, audit log.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use dc_core::{Core, CoreConfig, CoreEvent, LinkSpec, VehicleLink};
use mavlink_codec::enums::{MavCmd, MavModeFlag, MavResult};
use mavlink_codec::messages::{CommandAck, CommandLong, Heartbeat};
use mavlink_codec::{FrameParser, Header, Message, Version, encode};
use protocols::audit;
use protocols::command::{Approval, Command, CommandState, PolicyConfig, Refusal, Request};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;
use tokio::sync::watch;

#[derive(Default)]
struct Vehicle {
    armed: bool,
    mode: u32,
    /// Heartbeats are sent (radio downlink up).
    silent: bool,
    /// Commands are ignored (uplink down).
    deaf: bool,
    /// Every COMMAND_LONG received.
    received: Vec<CommandLong>,
    /// Heartbeat `system_status` (0 = MAV_STATE_UNINIT, as before).
    system_status: u8,
}

async fn fake_vehicle(
    listener: TcpListener,
    v: Arc<Mutex<Vehicle>>,
    mut stop: watch::Receiver<bool>,
) {
    let (mut sock, _) = listener.accept().await.unwrap();
    let mut parser = FrameParser::new();
    let mut buf = [0u8; 2048];
    let mut seq = 0u8;
    let mut tick = tokio::time::interval(Duration::from_millis(50));
    loop {
        let mut out = Vec::new();
        tokio::select! {
            _ = stop.changed() => return,
            _ = tick.tick() => {
                let s = v.lock().unwrap();
                if !s.silent {
                    let base_mode = MavModeFlag::CUSTOM_MODE_ENABLED.0 | if s.armed { MavModeFlag::SAFETY_ARMED.0 } else { 0 };
                    out.push(Message::Heartbeat(Heartbeat { r#type: 2, autopilot: 3, base_mode, custom_mode: s.mode, system_status: s.system_status, mavlink_version: 3 }));
                }
            }
            n = sock.read(&mut buf) => {
                let Ok(n) = n else { return };
                if n == 0 { return; }
                parser.push(&buf[..n]);
                while let Some(f) = parser.next_frame() {
                    if let Ok(Message::CommandLong(c)) = f.decode() {
                        let mut s = v.lock().unwrap();
                        s.received.push(c.clone());
                        if s.deaf { continue; }
                        let result = match MavCmd(c.command) {
                            MavCmd::COMPONENT_ARM_DISARM => { s.armed = c.param1 == 1.0; MavResult::ACCEPTED }
                            MavCmd::DO_SET_MODE => { s.mode = c.param2 as u32; MavResult::ACCEPTED }
                            _ => MavResult::UNSUPPORTED,
                        };
                        out.push(Message::CommandAck(CommandAck { command: c.command, result: result.0, ..Default::default() }));
                    }
                }
            }
        }
        for m in out {
            let f = encode(
                Version::V2,
                Header {
                    seq,
                    sysid: 1,
                    compid: 1,
                },
                &m,
            )
            .unwrap();
            seq = seq.wrapping_add(1);
            if sock.write_all(f.as_bytes()).await.is_err() {
                return;
            }
        }
    }
}

async fn setup(cfg: CoreConfig) -> (Core, Arc<Mutex<Vehicle>>, watch::Sender<bool>) {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let v = Arc::new(Mutex::new(Vehicle {
        mode: 5,
        ..Default::default()
    }));
    let (stop, stop_rx) = watch::channel(false);
    tokio::spawn(fake_vehicle(listener, v.clone(), stop_rx));
    let core = Core::start(cfg).await.unwrap();
    core.connect(&LinkSpec::Tcp(addr)).await.unwrap();
    assert_eq!(core.wait_for_vehicle(Duration::from_secs(5)).await, Some(1));
    // Let a few heartbeats arrive so the facts are current.
    tokio::time::sleep(Duration::from_millis(150)).await;
    (core, v, stop)
}

async fn until_final(
    events: &mut tokio::sync::broadcast::Receiver<CoreEvent>,
    within: Duration,
) -> Vec<CommandState> {
    let mut seen = Vec::new();
    tokio::time::timeout(within, async {
        loop {
            if let Ok(CoreEvent::Command(s)) = events.recv().await {
                seen.push(s.state);
                if s.state.is_final() {
                    return;
                }
            }
        }
    })
    .await
    .unwrap_or_else(|_| panic!("no final state within {within:?}; saw {seen:?}"));
    seen
}

fn tmp(name: &str) -> std::path::PathBuf {
    std::env::temp_dir().join(format!("dc-core-{name}-{}.log", std::process::id()))
}

#[tokio::test(flavor = "multi_thread")]
async fn arm_is_one_action_disarm_is_confirmed_and_both_are_audited() {
    let log = tmp("audit-arm");
    let _ = std::fs::remove_file(&log);
    let (core, v, stop) = setup(CoreConfig {
        audit_log: Some(log.clone()),
        ..Default::default()
    })
    .await;
    core.set_vehicle_name(1, "WHOOP-3");
    let mut events = core.events();

    // ARM is a single action (ADR-0011): no card, sent at once.
    assert!(matches!(
        core.request_command(1, Command::Arm).await.unwrap(),
        Request::Ready { .. }
    ));
    let id = core
        .submit_command(1, Command::Arm, Approval::None)
        .await
        .unwrap();
    let states = until_final(&mut events, Duration::from_secs(5)).await;
    assert_eq!(
        *states.last().unwrap(),
        CommandState::Verified,
        "{states:?}"
    );
    assert_eq!(core.command_status(id).unwrap().summary(), "ARM: VERIFIED");
    {
        let s = v.lock().unwrap();
        assert!(s.armed);
        let arms: Vec<_> = s.received.iter().filter(|c| c.command == 400).collect();
        assert_eq!(arms.len(), 1, "acknowledged at once: no retransmission");
        assert_eq!((arms[0].param1, arms[0].confirmation), (1.0, 0));
    }

    // DISARM is two-stage: without the card it is refused and nothing is sent.
    assert_eq!(
        core.submit_command(1, Command::Disarm, Approval::None)
            .await
            .unwrap_err(),
        Refusal::ConfirmationRequired
    );
    assert_eq!(
        v.lock()
            .unwrap()
            .received
            .iter()
            .filter(|c| c.command == 400)
            .count(),
        1
    );
    let Request::Confirm(card) = core.request_command(1, Command::Disarm).await.unwrap() else {
        panic!("DISARM is class C")
    };
    assert!(
        card.summary.contains("WHOOP-3") && card.summary.contains("ARMED"),
        "{}",
        card.summary
    );
    let id = core
        .submit_command(1, Command::Disarm, Approval::Confirmed(card.token))
        .await
        .unwrap();
    let states = until_final(&mut events, Duration::from_secs(5)).await;
    assert_eq!(
        *states.last().unwrap(),
        CommandState::Verified,
        "{states:?}"
    );
    assert_eq!(
        core.command_status(id).unwrap().summary(),
        "DISARM: VERIFIED"
    );
    assert!(!v.lock().unwrap().armed);

    // Audit log: complete and chained.
    tokio::time::sleep(Duration::from_millis(100)).await;
    let text = std::fs::read_to_string(&log).unwrap();
    let (n, _) = audit::verify(text.as_bytes()).expect("valid chain");
    assert!(n >= 5, "{text}");
    for want in [
        "request ARM (class B)",
        "accepted ARM",
        "sent ARM attempt 1",
        "ack ARM: accepted",
        "ARM: Verified",
        "refused DISARM: needs confirmation",
        "confirmation issued for DISARM",
        "DISARM: Verified",
    ] {
        assert!(text.contains(want), "audit log lacks {want:?}:\n{text}");
    }
    let _ = stop.send(true);
    drop(core);

    // A restarted core continues the same chain.
    let core2 = Core::start(CoreConfig {
        audit_log: Some(log.clone()),
        ..Default::default()
    })
    .await
    .unwrap();
    assert!(
        core2
            .submit_command(1, Command::Disarm, Approval::None)
            .await
            .is_err()
    );
    tokio::time::sleep(Duration::from_millis(100)).await;
    let (n2, _) = audit::verify(std::fs::read_to_string(&log).unwrap().as_bytes()).unwrap();
    assert!(n2 > n, "appended to the existing chain");
    drop(core2);

    // A tampered log refuses to start: tamper evidence.
    let tampered =
        std::fs::read_to_string(&log)
            .unwrap()
            .replacen("accepted ARM", "accepted DISARM", 1);
    std::fs::write(&log, tampered).unwrap();
    let err = Core::start(CoreConfig {
        audit_log: Some(log.clone()),
        ..Default::default()
    })
    .await
    .err()
    .expect("refused");
    assert_eq!(err.kind(), std::io::ErrorKind::InvalidData);
    let _ = std::fs::remove_file(&log);
}

/// ADR-0009: on a LOST link the command is still sent, retried on every link, and never sent
/// after its lifetime.
#[tokio::test(flavor = "multi_thread")]
async fn lost_link_still_sends_retries_and_expires() {
    let cfg = CoreConfig {
        link_stale: Duration::from_millis(300),
        link_lost: Duration::from_millis(900),
        policy: PolicyConfig {
            lifetime_nav: Duration::from_secs(3),
            ..Default::default()
        },
        ..Default::default()
    };
    let (core, v, stop) = setup(cfg).await;
    let mut events = core.events();

    // Radio gone both ways.
    {
        let mut s = v.lock().unwrap();
        s.silent = true;
        s.deaf = true;
    }
    tokio::time::timeout(Duration::from_secs(5), async {
        while core.vehicle_link(1) != Some(VehicleLink::Lost) {
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    })
    .await
    .expect("link LOST");

    let rtl = Command::Rtl {
        expect_mode: Some(6),
    };
    // RTL is a single action and is never refused for link state.
    assert!(matches!(
        core.request_command(1, rtl.clone())
            .await
            .expect("never refused for link state"),
        Request::Ready { .. }
    ));
    let t0 = std::time::Instant::now();
    core.submit_command(1, rtl, Approval::None)
        .await
        .expect("never refused for link state");

    let states = until_final(&mut events, Duration::from_secs(6)).await;
    assert_eq!(
        *states.last().unwrap(),
        CommandState::Expired { acked: false },
        "{states:?}"
    );
    let expired_after = t0.elapsed();
    assert!(
        expired_after >= Duration::from_secs(3) && expired_after < Duration::from_millis(3400),
        "{expired_after:?}"
    );

    let sent = v
        .lock()
        .unwrap()
        .received
        .iter()
        .filter(|c| c.command == 20)
        .count();
    assert!(sent >= 4, "retransmitted on the lost link: {sent}");
    let confirmations: Vec<u8> = v
        .lock()
        .unwrap()
        .received
        .iter()
        .filter(|c| c.command == 20)
        .map(|c| c.confirmation)
        .collect();
    assert_eq!(confirmations, (0..sent as u8).collect::<Vec<_>>());

    // Nothing more after expiry, even if the uplink comes back.
    v.lock().unwrap().deaf = false;
    tokio::time::sleep(Duration::from_millis(1500)).await;
    let after = v
        .lock()
        .unwrap()
        .received
        .iter()
        .filter(|c| c.command == 20)
        .count();
    assert_eq!(after, sent, "no RTL after its lifetime");
    let _ = stop.send(true);
}

async fn next_alert(
    events: &mut tokio::sync::broadcast::Receiver<CoreEvent>,
    within: Duration,
) -> (dc_core::AlertLevel, String) {
    tokio::time::timeout(within, async {
        loop {
            if let Ok(CoreEvent::Alert { level, text, .. }) = events.recv().await {
                return (level, text);
            }
        }
    })
    .await
    .unwrap_or_else(|_| panic!("no alert within {within:?}"))
}

/// SR-05 / ADR-0011: a mode change the vehicle never confirms is dropped after its 3 s lifetime
/// and raises a Warning for the pilot; a confirmed one raises an Advisory.
#[tokio::test(flavor = "multi_thread")]
async fn unconfirmed_mode_change_alerts_the_pilot() {
    let (core, v, stop) = setup(CoreConfig::default()).await;
    let mut events = core.events();

    // Confirmed: Advisory. (No banner from this fake vehicle, so the mode has no name.)
    core.submit_command(1, Command::SetMode { custom_mode: 4 }, Approval::None)
        .await
        .unwrap();
    let (level, text) = next_alert(&mut events, Duration::from_secs(3)).await;
    assert_eq!(
        (level, text.as_str()),
        (dc_core::AlertLevel::Advisory, "MODE 4 engaged")
    );

    // Uplink down: no ACK, no mode change.
    v.lock().unwrap().deaf = true;
    let t0 = std::time::Instant::now();
    core.submit_command(1, Command::SetMode { custom_mode: 6 }, Approval::None)
        .await
        .unwrap();
    let (level, text) = next_alert(&mut events, Duration::from_secs(5)).await;
    let after = t0.elapsed();
    assert_eq!(level, dc_core::AlertLevel::Warning);
    assert_eq!(text, "MODE 6 NOT CONFIRMED — EXPIRED: check the vehicle");
    assert!(
        after >= Duration::from_secs(3) && after < Duration::from_millis(3400),
        "alert when the 3 s lifetime ends: {after:?}"
    );
    assert_eq!(v.lock().unwrap().mode, 4, "the vehicle never changed mode");
    let _ = stop.send(true);
}

/// A mission upload to an armed vehicle is refused unless the opt-in setting
/// `armed-mission-upload` is on (off by default; ADR-0012 amendment, 2026-10-13).
#[tokio::test(flavor = "multi_thread")]
async fn mission_upload_is_refused_while_armed() {
    let (core, v, stop) = setup(CoreConfig::default()).await;
    v.lock().unwrap().armed = true;
    tokio::time::sleep(Duration::from_millis(300)).await;
    let err = core
        .upload_mission((1, 1), Vec::new())
        .await
        .expect_err("refused while armed");
    assert!(matches!(err, Refusal::Precondition(_)), "{err:?}");
    let _ = stop.send(true);
}

/// Owner, 2026-10-13: in-flight mission upload, fence/rally upload and log erase are opt-in
/// settings, all off by default. Off, each is refused before anything is sent.
#[tokio::test(flavor = "multi_thread")]
async fn vehicle_change_settings_are_off_by_default_and_refuse() {
    use dc_core::{ArmedWrite, MissionType, VehicleChange, VehicleChangeError};
    let (core, v, stop) = setup(CoreConfig::default()).await;
    assert_eq!(
        core.vehicle_changes(),
        dc_core::VehicleChangeSettings::default()
    );
    for kind in [MissionType::Fence, MissionType::Rally] {
        let err = core
            .upload_items((1, 1), kind, Vec::new(), ArmedWrite::Acknowledged)
            .await
            .expect_err("fence/rally upload is off");
        assert_eq!(
            err,
            VehicleChangeError::SettingOff(VehicleChange::FenceRallyUpload)
        );
    }
    assert_eq!(
        core.erase_logs((1, 1)).await.expect_err("log erase is off"),
        VehicleChangeError::SettingOff(VehicleChange::LogErase)
    );
    v.lock().unwrap().armed = true;
    tokio::time::sleep(Duration::from_millis(300)).await;
    let err = core
        .upload_items(
            (1, 1),
            MissionType::Mission,
            Vec::new(),
            ArmedWrite::Acknowledged,
        )
        .await
        .expect_err("armed mission upload is off");
    assert_eq!(
        err,
        VehicleChangeError::SettingOff(VehicleChange::ArmedMissionUpload)
    );
    assert!(err.to_string().contains("armed-mission-upload"), "{err}");
    let _ = stop.send(true);
}

/// With the settings on, an armed vehicle still needs the in-flight warning acknowledged, a
/// log erase still needs it disarmed (ArduPilot ignores log requests while armed), and
/// turning a setting on or off is in the audit log.
#[tokio::test(flavor = "multi_thread")]
async fn vehicle_change_settings_on_still_warn_and_are_audited() {
    use dc_core::{
        ARMED_FENCE_RALLY_WARNING, ARMED_MISSION_UPLOAD_WARNING, ArmedWrite, MissionType,
        VehicleChange, VehicleChangeError,
    };
    let log = tmp("audit-settings");
    let _ = std::fs::remove_file(&log);
    let (core, v, stop) = setup(CoreConfig {
        audit_log: Some(log.clone()),
        ..Default::default()
    })
    .await;
    for c in VehicleChange::ALL {
        core.set_vehicle_change(c, true);
    }
    v.lock().unwrap().armed = true;
    tokio::time::sleep(Duration::from_millis(300)).await;
    let warn = |kind| {
        let core = &core;
        async move {
            core.upload_items((1, 1), kind, Vec::new(), ArmedWrite::DisarmedOnly)
                .await
                .expect_err("needs the warning")
        }
    };
    assert_eq!(
        warn(MissionType::Mission).await,
        VehicleChangeError::ArmedWarningRequired(ARMED_MISSION_UPLOAD_WARNING)
    );
    assert_eq!(
        warn(MissionType::Fence).await,
        VehicleChangeError::ArmedWarningRequired(ARMED_FENCE_RALLY_WARNING)
    );
    let err = core.erase_logs((1, 1)).await.expect_err("armed");
    assert!(
        matches!(&err, VehicleChangeError::Refused(why) if why.contains("armed")),
        "{err:?}"
    );
    core.set_vehicle_change(VehicleChange::LogErase, false);
    core.set_vehicle_change(VehicleChange::LogErase, false); // no change: no record
    tokio::time::sleep(Duration::from_millis(300)).await;
    let text = std::fs::read_to_string(&log).unwrap();
    audit::verify(text.as_bytes()).expect("valid chain");
    for key in ["armed-mission-upload", "fence-rally-upload"] {
        assert!(text.contains(&format!("setting {key} turned ON")), "{text}");
    }
    assert_eq!(
        text.matches("setting log-erase turned").count(),
        2,
        "{text}"
    );
    let _ = stop.send(true);
    let _ = std::fs::remove_file(&log);
}

/// ArduPilot reports MAV_STATE_BOOT until its mission storage is ready; an upload then would be
/// refused as NO_SPACE. The core refuses it first, with the real reason.
#[tokio::test(flavor = "multi_thread")]
async fn mission_upload_waits_for_the_vehicle_to_finish_booting() {
    let (core, v, stop) = setup(CoreConfig::default()).await;
    v.lock().unwrap().system_status = 1; // MAV_STATE_BOOT
    tokio::time::sleep(Duration::from_millis(300)).await;
    let err = core
        .upload_mission((1, 1), Vec::new())
        .await
        .expect_err("refused while booting");
    assert!(
        matches!(&err, Refusal::Precondition(why) if why.contains("booting")),
        "{err:?}"
    );
    let _ = stop.send(true);
}

/// ADR-0013 item 6 through the core: the opt-in settings reach the command policy, a reason
/// is still required, reset stays refused while armed, and settings and uses are audited.
#[tokio::test(flavor = "multi_thread")]
async fn force_arm_and_reset_settings_give_a_single_confirmation() {
    use dc_core::{Simplified, VehicleChange};
    let log = tmp("audit-simple");
    let _ = std::fs::remove_file(&log);
    let (core, v, stop) = setup(CoreConfig {
        audit_log: Some(log.clone()),
        ..Default::default()
    })
    .await;
    core.set_vehicle_name(1, "WHOOP-3");
    assert_eq!(core.vehicle_changes().simplified(), Simplified::default());
    let card = |r: Request| match r {
        Request::Confirm(c) => c,
        r => panic!("{r:?}"),
    };
    // Off: force-arm needs the typed name.
    let c = card(core.request_command(1, Command::ForceArm).await.unwrap());
    assert_eq!(c.challenge.as_deref(), Some("WHOOP-3"));

    core.set_vehicle_change(VehicleChange::SimpleForceArm, true);
    core.set_vehicle_change(VehicleChange::SimpleParamReset, true);
    let c = card(core.request_command(1, Command::ForceArm).await.unwrap());
    assert!(c.challenge.is_none() && c.reason_required && c.single_step);
    assert_eq!(
        core.submit_command(1, Command::ForceArm, Approval::Confirmed(c.token))
            .await
            .unwrap_err(),
        Refusal::ReasonRequired
    );
    let c = card(core.request_command(1, Command::ForceArm).await.unwrap());
    core.submit_command(
        1,
        Command::ForceArm,
        Approval::Reasoned {
            token: c.token,
            reason: "bench test, no props".into(),
        },
    )
    .await
    .unwrap();
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert!(v.lock().unwrap().armed);
    // Armed: reset is still refused, setting or not (SR-08).
    assert!(matches!(
        core.request_command(1, Command::ParamReset).await,
        Err(Refusal::Precondition(_))
    ));

    tokio::time::sleep(Duration::from_millis(100)).await;
    let text = std::fs::read_to_string(&log).unwrap();
    audit::verify(text.as_bytes()).expect("valid chain");
    for want in [
        "setting simple-force-arm turned ON",
        "setting simple-param-reset turned ON",
        "confirmation issued for FORCE ARM (class D)",
        "refused FORCE ARM: a reason is required",
        "accepted FORCE ARM (class C, setting simple-force-arm), reason: bench test, no props",
        "refused request RESET PARAMETERS: not allowed now",
    ] {
        assert!(text.contains(want), "audit log lacks {want:?}:\n{text}");
    }
    let _ = stop.send(true);
}
