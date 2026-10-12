//! CAMERA_FEEDBACK geotags: kept per vehicle, in order; a repeated report of the same image
//! replaces the earlier one.

use std::time::Duration;

use dc_core::{Core, CoreConfig, LinkSpec};
use mavlink_codec::messages::{CameraFeedback, Heartbeat};
use mavlink_codec::{Header, Message, Version, encode};
use recorder::TlogWriter;

#[tokio::test(flavor = "multi_thread")]
async fn photos_are_kept_once_each() {
    let dir = std::env::temp_dir().join(format!("dc-geotags-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("survey.tlog");
    let mut w = TlogWriter::new(std::fs::File::create(&path).unwrap());
    let h = Header {
        seq: 0,
        sysid: 4,
        compid: 1,
    };
    let t0 = 1_760_000_000_000_000u64;
    let hb = Message::Heartbeat(Heartbeat {
        r#type: 2,
        autopilot: 3,
        ..Default::default()
    });
    w.write_raw(t0, encode(Version::V2, h, &hb).unwrap().as_bytes())
        .unwrap();
    let photo = |img: u16, lat: i32, alt: f32| {
        Message::CameraFeedback(CameraFeedback {
            img_idx: img,
            lat,
            lng: 1_491_652_300,
            alt_msl: 600.0,
            alt_rel: alt,
            yaw: 90.0,
            ..Default::default()
        })
    };
    for (i, m) in [
        photo(1, -353_632_610, 40.0),
        photo(2, -353_622_610, 41.0),
        photo(2, -353_622_611, 41.5), // same image reported again
        photo(3, -353_612_610, 42.0),
    ]
    .iter()
    .enumerate()
    {
        w.write_raw(
            t0 + 1 + i as u64,
            encode(Version::V2, h, m).unwrap().as_bytes(),
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
        while core.geotags(4).last().map(|g| g.img_idx) != Some(3) {
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    })
    .await
    .expect("photos delivered");
    let g = core.geotags(4);
    assert_eq!(g.iter().map(|g| g.img_idx).collect::<Vec<_>>(), [1, 2, 3]);
    assert_eq!(
        g[1].alt_rel_m, 41.5,
        "the repeated report replaced the first"
    );
    assert!((g[2].lat_deg + 35.361_261).abs() < 1e-9);
    let _ = std::fs::remove_dir_all(dir);
}
