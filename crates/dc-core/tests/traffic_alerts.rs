//! ADS-B traffic alerts: an aircraft on a collision course is a Warning (closest approach
//! inside ArduPilot Copter's failure distances), and "Traffic clear" when it turns away.

use std::time::Duration;

use dc_core::{AlertLevel, Core, CoreConfig, CoreEvent, LinkSpec};
use mavlink_codec::messages::{AdsbVehicle, GlobalPositionInt, Heartbeat};
use mavlink_codec::{Header, Message, Version, encode};
use recorder::TlogWriter;

#[tokio::test(flavor = "multi_thread")]
async fn a_head_on_aircraft_is_a_warning_and_clears_when_it_turns() {
    let dir = std::env::temp_dir().join(format!("dc-traffic-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("traffic.tlog");
    let mut w = TlogWriter::new(std::fs::File::create(&path).unwrap());
    let h = Header {
        seq: 0,
        sysid: 1,
        compid: 1,
    };
    let lat0 = -353_632_610;
    let mut callsign = [0u8; 9];
    callsign[..5].copy_from_slice(b"QFA12");
    let traffic = |heading_cdeg: u16| {
        Message::AdsbVehicle(AdsbVehicle {
            ICAO_address: 0x7C_00_01,
            lat: lat0 + 89_932, // 1 km north
            lon: 1_491_652_300,
            altitude: 684_000, // mm, same as the vehicle
            heading: heading_cdeg,
            hor_velocity: 5_000, // 50 m/s
            callsign,
            emitter_type: 1,
            flags: 1 | 2 | 4 | 8 | 16,
            ..Default::default()
        })
    };
    let msgs = [
        Message::Heartbeat(Heartbeat {
            r#type: 2,
            autopilot: 3,
            ..Default::default()
        }),
        Message::GlobalPositionInt(GlobalPositionInt {
            lat: lat0,
            lon: 1_491_652_300,
            alt: 684_000,
            relative_alt: 100_000,
            ..Default::default()
        }),
        traffic(18_000), // flying south, at us
        traffic(18_000), // again: no new alert
        traffic(0),      // turned north, away
    ];
    for (i, m) in msgs.iter().enumerate() {
        w.write_raw(
            1_760_000_000_000_000 + i as u64 * 100_000,
            encode(Version::V2, h, m).unwrap().as_bytes(),
        )
        .unwrap();
    }
    w.flush().unwrap();
    drop(w);

    let core = Core::start(CoreConfig::default()).await.unwrap();
    let mut events = core.events();
    core.connect(&LinkSpec::Tlog { path, speed: 10 })
        .await
        .unwrap();
    let mut alerts = Vec::new();
    let _ = tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            if let Ok(CoreEvent::Alert { level, text, .. }) = events.recv().await {
                alerts.push((level, text));
            }
        }
    })
    .await;
    assert_eq!(alerts.len(), 2, "{alerts:?}");
    assert_eq!(alerts[0].0, AlertLevel::Warning);
    assert!(
        alerts[0]
            .1
            .starts_with("TRAFFIC QFA12: 1000 m N +0 m, closest 0 m in 20 s"),
        "{alerts:?}"
    );
    assert_eq!(
        alerts[1],
        (AlertLevel::Advisory, "Traffic clear: QFA12".to_string())
    );
    // Latched in the annunciator until acknowledged.
    let shown = core.annunciator(1);
    assert_eq!(
        (shown[0].id.as_str(), shown[0].active),
        ("TRAFFIC QFA12", false)
    );
    let _ = std::fs::remove_dir_all(dir);
}
