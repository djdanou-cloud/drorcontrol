//! STATUSTEXT history: kept per vehicle, oldest first, capped at MESSAGE_HISTORY.

use std::time::Duration;

use dc_core::{Core, CoreConfig, LinkSpec, MESSAGE_HISTORY};
use mavlink_codec::messages::{Heartbeat, Statustext};
use mavlink_codec::{Header, Message, Version, encode};
use recorder::TlogWriter;

#[tokio::test(flavor = "multi_thread")]
async fn keeps_the_last_lines_per_vehicle() {
    let dir = std::env::temp_dir().join(format!("dc-messages-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("texts.tlog");
    let mut w = TlogWriter::new(std::fs::File::create(&path).unwrap());
    let t0 = 1_760_000_000_000_000u64;
    let h = |seq: usize| Header {
        seq: seq as u8,
        sysid: 3,
        compid: 1,
    };
    let hb = Message::Heartbeat(Heartbeat {
        r#type: 2,
        autopilot: 3,
        ..Default::default()
    });
    w.write_raw(t0, encode(Version::V2, h(0), &hb).unwrap().as_bytes())
        .unwrap();
    let n = MESSAGE_HISTORY + 10;
    for i in 0..n {
        let mut text = [0u8; 50];
        let s = format!("line {i}");
        text[..s.len()].copy_from_slice(s.as_bytes());
        let m = Message::Statustext(Statustext {
            severity: 6,
            text,
            ..Default::default()
        });
        w.write_raw(
            t0 + i as u64,
            encode(Version::V2, h(i), &m).unwrap().as_bytes(),
        )
        .unwrap();
    }
    w.flush().unwrap();
    drop(w);

    let core = Core::start(CoreConfig::default()).await.unwrap();
    core.connect(&LinkSpec::Tlog { path, speed: 100 })
        .await
        .unwrap();
    tokio::time::timeout(Duration::from_secs(10), async {
        while core.messages(3).last().map(|m| m.text.text.as_str()) != Some("line 509") {
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    })
    .await
    .expect("all lines delivered");
    let m = core.messages(3);
    assert_eq!(m.len(), MESSAGE_HISTORY);
    assert_eq!(m[0].text.text, "line 10", "oldest dropped first");
    assert!(core.messages(9).is_empty());
    let _ = std::fs::remove_dir_all(dir);
}
