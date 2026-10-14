//! FENCE_STATUS alerts: a new breach (or another one counted while still outside) is a
//! Warning, back inside is an Advisory; a repeated report of the same breach is quiet.

use std::time::Duration;

use dc_core::{AlertLevel, Core, CoreConfig, CoreEvent, LinkSpec};
use mavlink_codec::messages::{FenceStatus, Heartbeat};
use mavlink_codec::{Header, Message, Version, encode};
use recorder::TlogWriter;

#[tokio::test(flavor = "multi_thread")]
async fn breaches_and_return_are_alerted_once() {
    let dir = std::env::temp_dir().join(format!("dc-fence-alerts-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("fence.tlog");
    let mut w = TlogWriter::new(std::fs::File::create(&path).unwrap());
    let h = Header {
        seq: 0,
        sysid: 3,
        compid: 1,
    };
    let t0 = 1_760_000_000_000_000u64;
    let fence = |breach: u8, count: u16, kind: u8| {
        Message::FenceStatus(FenceStatus {
            breach_status: breach,
            breach_count: count,
            breach_type: kind,
            ..Default::default()
        })
    };
    let msgs = [
        Message::Heartbeat(Heartbeat {
            r#type: 2,
            autopilot: 3,
            ..Default::default()
        }),
        fence(0, 0, 0),
        fence(1, 1, 2), // above the maximum altitude
        fence(1, 1, 2), // the same breach again: quiet
        fence(1, 2, 3), // a second breach while still outside
        fence(0, 2, 3), // back inside
    ];
    for (i, m) in msgs.iter().enumerate() {
        w.write_raw(
            t0 + i as u64 * 100_000,
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
    let mut sounds: Vec<String> = Vec::new();
    let _ = tokio::time::timeout(Duration::from_secs(5), async {
        while alerts.len() < 3 {
            match events.recv().await {
                Ok(CoreEvent::Alert { sysid, level, text }) => {
                    assert_eq!(sysid, 3);
                    alerts.push((level, text));
                }
                Ok(CoreEvent::Sound { id, .. }) => sounds.push(id),
                _ => {}
            }
        }
    })
    .await;
    // Nothing more after the last one.
    tokio::time::sleep(Duration::from_millis(300)).await;
    while let Ok(e) = events.try_recv() {
        match e {
            CoreEvent::Alert { level, text, .. } => alerts.push((level, text)),
            CoreEvent::Sound { id, .. } => sounds.push(id),
            _ => {}
        }
    }
    // The annunciator: FENCE latched (a warning, now cleared) until acknowledged.
    let shown = core.annunciator(3);
    assert_eq!(shown.len(), 1, "{shown:?}");
    assert_eq!((shown[0].id.as_str(), shown[0].active), ("FENCE", false));
    assert_eq!(shown[0].count, 2, "the second breach updated it");
    assert_eq!(
        core.master_alert(3),
        Some(dc_core::annunciator::AlertLevel::Warning)
    );
    core.ack_alert(3, Some("FENCE"));
    assert!(core.annunciator(3).is_empty() && core.master_alert(3).is_none());
    assert_eq!(
        alerts,
        [
            (
                AlertLevel::Warning,
                "FENCE BREACH: above maximum altitude".to_string()
            ),
            (AlertLevel::Warning, "FENCE BREACH: boundary".to_string()),
            (AlertLevel::Advisory, "Back inside the fence".to_string()),
        ]
    );
    // Sounded once: the second breach came within the 10 s quiet time at the same level.
    assert_eq!(sounds, ["FENCE"]);
    let _ = std::fs::remove_dir_all(dir);
}
