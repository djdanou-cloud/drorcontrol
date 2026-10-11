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
                    out.push(Message::Heartbeat(Heartbeat { r#type: 2, autopilot: 3, base_mode, custom_mode: s.mode, mavlink_version: 3, ..Default::default() }));
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

/// Plan 08 §4.1: a mission upload in flight is class C; until that confirmation exists for
/// missions the core refuses it while the vehicle is armed.
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
