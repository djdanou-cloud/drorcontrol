//! `.tlog` replay links: paced playback, receive-only, and never alongside a live link.

use std::time::{Duration, Instant};

use dc_core::{Core, CoreConfig, CoreEvent, LinkSpec};
use mavlink_codec::messages::{CommandLong, Heartbeat};
use mavlink_codec::{Header, Message, Version, encode};
use recorder::TlogWriter;

/// A 2 s recording: vehicle 7 heartbeats every 200 ms (disarmed, then armed), plus a frame
/// from the recording GCS (255/190), which must not be replayed as received traffic.
fn recording(dir: &std::path::Path) -> std::path::PathBuf {
    let path = dir.join("flight.tlog");
    let mut w = TlogWriter::new(std::fs::File::create(&path).unwrap());
    let t0 = 1_760_000_000_000_000u64;
    for i in 0..=10u64 {
        let hb = Message::Heartbeat(Heartbeat {
            r#type: 2,
            autopilot: 3,
            base_mode: if i >= 5 { 128 } else { 0 },
            mavlink_version: 3,
            ..Default::default()
        });
        let h = Header {
            seq: i as u8,
            sysid: 7,
            compid: 1,
        };
        w.write_raw(
            t0 + i * 200_000,
            encode(Version::V2, h, &hb).unwrap().as_bytes(),
        )
        .unwrap();
    }
    let gcs = Header {
        seq: 0,
        sysid: 255,
        compid: 190,
    };
    let cmd = Message::CommandLong(CommandLong {
        target_system: 7,
        target_component: 1,
        command: 400,
        ..Default::default()
    });
    w.write_raw(
        t0 + 2_000_000,
        encode(Version::V2, gcs, &cmd).unwrap().as_bytes(),
    )
    .unwrap();
    w.flush().unwrap();
    path
}

fn tmpdir(name: &str) -> std::path::PathBuf {
    let d = std::env::temp_dir().join(format!("dc-replay-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

#[test]
fn parses_replay_specs() {
    assert_eq!(
        "tlog:/data/f.tlog".parse(),
        Ok(LinkSpec::Tlog {
            path: "/data/f.tlog".into(),
            speed: 1
        })
    );
    let s: LinkSpec = "tlog:C:\\logs\\a@b.tlog@4".parse().unwrap();
    assert_eq!(
        s,
        LinkSpec::Tlog {
            path: "C:\\logs\\a@b.tlog".into(),
            speed: 4
        }
    );
    assert_eq!(s.to_string(), "tlog:C:\\logs\\a@b.tlog@4");
    assert!("tlog:f.tlog@0".parse::<LinkSpec>().is_err());
    assert!("tlog:f.tlog@101".parse::<LinkSpec>().is_err());
    assert!("tlog:".parse::<LinkSpec>().is_err());
}

#[tokio::test(flavor = "multi_thread")]
async fn replays_with_its_timing_and_only_as_the_sole_link() {
    let dir = tmpdir("play");
    let path = recording(&dir);
    let core = Core::start(CoreConfig::default()).await.unwrap();
    let mut events = core.events();
    let t0 = Instant::now();
    core.connect(&LinkSpec::Tlog {
        path: path.clone(),
        speed: 10,
    })
    .await
    .unwrap();
    assert_eq!(core.wait_for_vehicle(Duration::from_secs(5)).await, Some(7));

    // Live links are refused while a replay is open, and so is a second replay.
    let live = core
        .connect(&"udp:127.0.0.1:0".parse::<dc_core::LinkSpec>().unwrap())
        .await;
    assert_eq!(live.unwrap_err().kind(), std::io::ErrorKind::InvalidInput);
    assert!(
        core.connect(&LinkSpec::Tlog { path, speed: 1 })
            .await
            .is_err()
    );
    // Commands sent to a replay go nowhere (no other link to relay to).
    assert!(core.send(&Message::Heartbeat(Heartbeat::default())).await);

    let finished = tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            if let Ok(CoreEvent::Link {
                up: false, detail, ..
            }) = events.recv().await
            {
                return detail;
            }
        }
    })
    .await
    .expect("replay end reported");
    let took = t0.elapsed();
    assert!(finished.contains("replay finished"), "{finished}");
    // 2 s of recording at 10x: about 0.2 s, not instant.
    assert!(took >= Duration::from_millis(180), "{took:?}");
    let fleet = core.fleet();
    let v = fleet.get(7).unwrap();
    assert!(
        v.heartbeat.as_ref().unwrap().value.armed(),
        "last state replayed"
    );
    assert!(
        fleet.get(255).is_none(),
        "the recording GCS's own frames are not received traffic"
    );
    let _ = std::fs::remove_dir_all(dir);
}

#[tokio::test(flavor = "multi_thread")]
async fn a_live_core_refuses_a_replay_and_a_missing_file_changes_nothing() {
    let dir = tmpdir("live");
    let path = recording(&dir);
    let core = Core::start(CoreConfig::default()).await.unwrap();
    let missing = core
        .connect(&LinkSpec::Tlog {
            path: dir.join("nope.tlog"),
            speed: 1,
        })
        .await;
    assert_eq!(missing.unwrap_err().kind(), std::io::ErrorKind::NotFound);
    core.connect(&"udp:127.0.0.1:0".parse::<dc_core::LinkSpec>().unwrap())
        .await
        .expect("a failed replay does not lock out live links");
    let replay = core.connect(&LinkSpec::Tlog { path, speed: 1 }).await;
    assert_eq!(replay.unwrap_err().kind(), std::io::ErrorKind::InvalidInput);
    let _ = std::fs::remove_dir_all(dir);
}
