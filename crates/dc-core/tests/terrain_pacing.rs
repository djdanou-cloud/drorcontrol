//! Terrain answers are paced for the link they go out on (plan 09 Step 1), and a repeated
//! request whose answer is still being sent is not answered twice.

use std::time::{Duration, Instant};

use dc_core::{Core, CoreConfig, LinkProfile, LinkSpec, LinkTarget};
use mavlink_codec::messages::{Heartbeat, TerrainRequest};
use mavlink_codec::{FrameParser, Header, Message, Version, encode};
use protocols::terrain::{GeoGrid, Terrain};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;

/// A vehicle that asks for 20 terrain sub-grids twice, 300 ms apart, and returns when each
/// TERRAIN_DATA arrived (seconds after the first request).
async fn run(profile: LinkProfile) -> Vec<f64> {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let core = Core::start(CoreConfig::default()).await.unwrap();
    let mut flat = Terrain::default();
    flat.add_grid(
        GeoGrid::new(
            -35.0,
            149.0,
            (0.001, 0.001),
            (1000, 1000),
            vec![600.0; 1_000_000],
            None,
        )
        .unwrap(),
    );
    core.serve_terrain(Some(flat));
    core.connect(&LinkTarget {
        spec: LinkSpec::Tcp(addr),
        profile,
    })
    .await
    .unwrap();
    let (mut sock, _) = listener.accept().await.unwrap();
    let mut seq = 0u8;
    let mut frame = |m: &Message| {
        seq = seq.wrapping_add(1);
        encode(
            Version::V2,
            Header {
                seq,
                sysid: 1,
                compid: 1,
            },
            m,
        )
        .unwrap()
        .as_bytes()
        .to_vec()
    };
    let hb = Message::Heartbeat(Heartbeat {
        r#type: 2,
        autopilot: 3,
        mavlink_version: 3,
        ..Default::default()
    });
    let req = Message::TerrainRequest(TerrainRequest {
        lat: -353_632_610,
        lon: 1_491_652_300,
        grid_spacing: 100,
        mask: (1 << 20) - 1,
    });
    sock.write_all(&frame(&hb)).await.unwrap();
    tokio::time::sleep(Duration::from_millis(200)).await;
    let t0 = Instant::now();
    sock.write_all(&frame(&req)).await.unwrap();
    let mut parser = FrameParser::new();
    let mut buf = [0u8; 4096];
    let mut arrivals = Vec::new();
    let mut repeated = false;
    let end = t0 + Duration::from_secs(5);
    while Instant::now() < end {
        if !repeated && t0.elapsed() > Duration::from_millis(300) {
            sock.write_all(&frame(&hb)).await.unwrap();
            sock.write_all(&frame(&req)).await.unwrap();
            repeated = true;
        }
        let Ok(Ok(n)) = tokio::time::timeout(Duration::from_millis(50), sock.read(&mut buf)).await
        else {
            continue;
        };
        if n == 0 {
            break;
        }
        parser.push(&buf[..n]);
        while let Some(f) = parser.next_frame() {
            if let Ok(Message::TerrainData(_)) = f.decode() {
                arrivals.push(t0.elapsed().as_secs_f64());
            }
        }
    }
    arrivals
}

#[tokio::test(flavor = "multi_thread")]
async fn terrain_answers_are_paced_by_link_profile_and_not_repeated() {
    let high = run(LinkProfile::High).await;
    assert_eq!(
        high.len(),
        20,
        "one answer per sub-grid, the repeat ignored"
    );
    let span = high.last().unwrap() - high.first().unwrap();
    // 10 frames/s: 19 gaps of 100 ms.
    assert!(
        (1.7..2.5).contains(&span),
        "High: {span:.2} s for 20 frames"
    );

    // Full: unpaced, so the first answer is complete before the repeat, which is answered
    // again (only a repeat that arrives while its answer is being sent is ignored).
    let full = run(LinkProfile::Full).await;
    assert_eq!(full.len(), 40);
    let span = full[19] - full[0];
    assert!(span < 0.3, "Full: unpaced, {span:.2} s for the first 20");
}
