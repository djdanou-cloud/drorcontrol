//! Link liveness, recorder health and shutdown (findings of the 2026-10-11 audit check,
//! docs/audit/2026-10-11-external-audit.md).

use std::time::Duration;

use dc_core::{Core, CoreConfig, CoreEvent, LinkSpec, VehicleLink};
use mavlink_codec::messages::Heartbeat;
use mavlink_codec::{Header, Message, Version, encode};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;
use tokio::sync::watch;

/// Fake autopilot that sends a heartbeat every 50 ms while `talking` is true.
async fn vehicle(listener: TcpListener, talking: watch::Receiver<bool>) {
    let (mut sock, _) = listener.accept().await.unwrap();
    let mut buf = [0u8; 1024];
    let mut tick = tokio::time::interval(Duration::from_millis(50));
    let mut seq = 0u8;
    loop {
        tokio::select! {
            _ = tick.tick() => {
                if *talking.borrow() {
                    let hb = Message::Heartbeat(Heartbeat { r#type: 2, autopilot: 3, mavlink_version: 3, ..Default::default() });
                    let f = encode(Version::V2, Header { seq, sysid: 1, compid: 1 }, &hb).unwrap();
                    seq = seq.wrapping_add(1);
                    if sock.write_all(f.as_bytes()).await.is_err() { return; }
                }
            }
            n = sock.read(&mut buf) => if matches!(n, Ok(0) | Err(_)) { return },
        }
    }
}

async fn next_link_state(
    events: &mut tokio::sync::broadcast::Receiver<CoreEvent>,
) -> (VehicleLink, Duration) {
    loop {
        match tokio::time::timeout(Duration::from_secs(5), events.recv())
            .await
            .expect("link state change within 5 s")
            .unwrap()
        {
            CoreEvent::VehicleLink {
                sysid: 1,
                state,
                heartbeat_age,
            } => return (state, heartbeat_age),
            _ => continue,
        }
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn vehicle_goes_stale_then_lost_then_recovers() {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let (talk, talking) = watch::channel(true);
    tokio::spawn(vehicle(listener, talking));

    let core = Core::start(CoreConfig {
        link_stale: Duration::from_millis(300),
        link_lost: Duration::from_millis(900),
        ..Default::default()
    })
    .await
    .unwrap();
    let mut events = core.events();
    core.connect(&LinkSpec::Tcp(addr)).await.unwrap();
    assert_eq!(core.wait_for_vehicle(Duration::from_secs(5)).await, Some(1));
    assert_eq!(core.vehicle_link(1), Some(VehicleLink::Connected));

    // The radio goes silent: STALE after 0.3 s, LOST after 0.9 s, never a false "connected".
    talk.send(false).unwrap();
    let (state, age) = next_link_state(&mut events).await;
    assert_eq!(state, VehicleLink::Stale);
    assert!(age >= Duration::from_millis(300), "{age:?}");
    let (state, age) = next_link_state(&mut events).await;
    assert_eq!(state, VehicleLink::Lost);
    assert!(age >= Duration::from_millis(900), "{age:?}");
    assert_eq!(core.vehicle_link(1), Some(VehicleLink::Lost));

    // Heartbeats return: CONNECTED again.
    talk.send(true).unwrap();
    let (state, _) = next_link_state(&mut events).await;
    assert_eq!(state, VehicleLink::Connected);
    assert_eq!(
        core.vehicle_link(2),
        None,
        "never-seen vehicle has no link state"
    );
}

#[test]
fn link_thresholds_are_bounded() {
    let ok = CoreConfig::default();
    assert_eq!(
        ok.link_stale,
        Duration::from_millis(1500),
        "plan 08 §4.3 default"
    );
    assert_eq!(ok.link_lost, Duration::from_secs(5), "plan 08 §4.3 default");
    assert!(ok.validate().is_ok());
    let bad = |f: fn(&mut CoreConfig)| {
        let mut c = CoreConfig::default();
        f(&mut c);
        c.validate().is_err()
    };
    assert!(bad(|c| c.link_stale = Duration::from_millis(50)));
    assert!(bad(|c| c.link_stale = Duration::from_secs(10)));
    assert!(bad(|c| c.link_lost = c.link_stale));
    assert!(bad(|c| c.link_lost = Duration::from_secs(120)));
    assert!(bad(|c| c.stream_rates = vec![(30, 0.0)]));
    assert!(bad(|c| c.stream_rates = vec![(30, f32::NAN)]));
    // ADR-0011: mode changes expire within 2–3 s.
    assert_eq!(ok.policy.lifetime_mode, Duration::from_secs(3));
    assert!(bad(|c| c.policy.lifetime_mode = Duration::from_millis(3001)));
    assert!(bad(|c| c.policy.lifetime_mode = Duration::from_millis(1999)));
}

#[tokio::test]
async fn bad_config_is_refused_at_start() {
    let cfg = CoreConfig {
        link_lost: Duration::from_millis(100),
        ..Default::default()
    };
    let err = Core::start(cfg).await.err().expect("refused");
    assert_eq!(err.kind(), std::io::ErrorKind::InvalidInput);
}

/// A full disk used to stop recording silently. Now the core says so.
#[cfg(target_os = "linux")]
#[tokio::test(flavor = "multi_thread")]
async fn recorder_failure_is_reported() {
    let core = Core::start(CoreConfig {
        tlog: Some("/dev/full".into()), // every write fails with ENOSPC
        ..Default::default()
    })
    .await
    .unwrap();
    let mut events = core.events();
    loop {
        match tokio::time::timeout(Duration::from_secs(5), events.recv())
            .await
            .expect("recorder event within 5 s")
            .unwrap()
        {
            CoreEvent::Recorder { stopped, detail } => {
                assert!(stopped, "{detail}");
                assert!(detail.contains("/dev/full"), "{detail}");
                break;
            }
            _ => continue,
        }
    }
}

/// Shutting the runtime down flushes the recorder: no frames lost from the last second.
#[test]
fn shutdown_flushes_the_tlog() {
    let tlog = std::env::temp_dir().join(format!("dc-core-shutdown-{}.tlog", std::process::id()));
    let rt = tokio::runtime::Runtime::new().unwrap();
    rt.block_on(async {
        let core = Core::start(CoreConfig {
            tlog: Some(tlog.clone()),
            ..Default::default()
        })
        .await
        .unwrap();
        // The GCS heartbeat (sent at start) is recorded; give it a moment, far below the
        // 1 s periodic flush.
        tokio::time::sleep(Duration::from_millis(200)).await;
        drop(core);
    });
    drop(rt); // cancels the tasks, then waits for the blocking recorder to finish
    let bytes = std::fs::read(&tlog).unwrap();
    let _ = std::fs::remove_file(&tlog);
    let records: Vec<_> = recorder::TlogReader::new(bytes.as_slice())
        .collect::<Result<_, _>>()
        .expect("only complete records");
    assert!(
        !records.is_empty(),
        "the heartbeat sent at start is on disk"
    );
    assert_eq!(records[0].frame.decode().unwrap().name(), "HEARTBEAT");
}

/// A LOST that ends within one watchdog tick must still be reported (found by the SITL
/// fade scenario): the heartbeat gap reveals states the 100 ms sampler skipped.
#[tokio::test(flavor = "multi_thread")]
async fn a_brief_lost_is_never_skipped() {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let (talk, talking) = watch::channel(true);
    tokio::spawn(vehicle(listener, talking));
    let core = Core::start(CoreConfig {
        link_stale: Duration::from_millis(300),
        link_lost: Duration::from_millis(900),
        ..Default::default()
    })
    .await
    .unwrap();
    let mut events = core.events();
    core.connect(&LinkSpec::Tcp(addr)).await.unwrap();
    assert_eq!(core.wait_for_vehicle(Duration::from_secs(5)).await, Some(1));
    for round in 0..5 {
        tokio::time::sleep(Duration::from_millis(300)).await;
        while events.try_recv().is_ok() {}
        // Silent just past the LOST threshold, then back: LOST lasts only a few ms.
        talk.send(false).unwrap();
        tokio::time::sleep(Duration::from_millis(905 + 17 * round)).await;
        talk.send(true).unwrap();
        let mut seen = Vec::new();
        while !seen.contains(&VehicleLink::Connected) {
            let (s, _) = next_link_state(&mut events).await;
            seen.push(s);
        }
        assert_eq!(
            seen,
            [
                VehicleLink::Stale,
                VehicleLink::Lost,
                VehicleLink::Connected
            ],
            "round {round}"
        );
    }
}
