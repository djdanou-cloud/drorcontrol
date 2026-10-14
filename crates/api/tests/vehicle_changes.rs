//! UploadItems, EraseLogs and GetVehicleChangeSettings over a real socket (ADR-0012
//! amendment): settings off refuse before anything is sent; with a setting on, an armed
//! vehicle gets the in-flight warning first.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::Duration;

use api::pb::{self, core_client::CoreClient};
use dc_core::{Core, CoreConfig, VehicleChangeSettings};
use mavlink_codec::enums::MavModeFlag;
use mavlink_codec::messages::Heartbeat;
use mavlink_codec::{FrameParser, Header, Message, Version, encode};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;

/// A vehicle that only sends heartbeats and counts mission-protocol and log-erase frames.
async fn vehicle(listener: TcpListener, armed: Arc<AtomicBool>, changes: Arc<AtomicU32>) {
    let (mut sock, _) = listener.accept().await.unwrap();
    let mut parser = FrameParser::new();
    let mut buf = [0u8; 1024];
    let mut tick = tokio::time::interval(Duration::from_millis(100));
    loop {
        tokio::select! {
            _ = tick.tick() => {
                let hb = Message::Heartbeat(Heartbeat {
                    r#type: 2,
                    autopilot: 3,
                    base_mode: if armed.load(Ordering::Relaxed) { MavModeFlag::SAFETY_ARMED.0 } else { 0 },
                    system_status: 3,
                    mavlink_version: 3,
                    ..Default::default()
                });
                let f = encode(Version::V2, Header { seq: 0, sysid: 3, compid: 1 }, &hb).unwrap();
                if sock.write_all(f.as_bytes()).await.is_err() { return; }
            }
            n = sock.read(&mut buf) => {
                let Ok(n) = n else { return };
                if n == 0 { return; }
                parser.push(&buf[..n]);
                while let Some(f) = parser.next_frame() {
                    if matches!(
                        f.decode(),
                        Ok(Message::MissionCount(_) | Message::MissionClearAll(_) | Message::LogErase(_))
                    ) {
                        changes.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }
    }
}

async fn client(
    settings: VehicleChangeSettings,
) -> (
    CoreClient<tonic::transport::Channel>,
    Arc<AtomicBool>,
    Arc<AtomicU32>,
) {
    let armed = Arc::new(AtomicBool::new(false));
    let changes = Arc::new(AtomicU32::new(0));
    let v = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let vaddr = v.local_addr().unwrap();
    tokio::spawn(vehicle(v, armed.clone(), changes.clone()));
    let core = Core::start(CoreConfig {
        vehicle_changes: settings,
        ..Default::default()
    })
    .await
    .unwrap();
    core.connect(&dc_core::LinkSpec::Tcp(vaddr)).await.unwrap();
    assert_eq!(core.wait_for_vehicle(Duration::from_secs(5)).await, Some(3));
    tokio::time::sleep(Duration::from_millis(300)).await;
    let g = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let url = format!("http://{}", g.local_addr().unwrap());
    tokio::spawn(api::serve_on_with(
        core,
        g,
        api::ServeOptions {
            token: None,
            param_dir: None,
        },
    ));
    (CoreClient::connect(url).await.unwrap(), armed, changes)
}

fn upload(kind: pb::ItemKind, ack: bool) -> pb::UploadItemsRequest {
    pb::UploadItemsRequest {
        sysid: 0,
        compid: 0,
        kind: kind as i32,
        items: vec![pb::Item {
            frame: 0,
            command: 16,
            params: vec![0.0; 4],
            x: -353_632_610,
            y: 1_491_652_300,
            z: 0.0,
            autocontinue: true,
        }],
        acknowledge_armed_warning: ack,
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn settings_off_refuse_and_send_nothing() {
    let (mut c, armed, changes) = client(VehicleChangeSettings::default()).await;
    let s = c
        .get_vehicle_change_settings(pb::GetVehicleChangeSettingsRequest {})
        .await
        .unwrap()
        .into_inner();
    assert_eq!(s.settings.len(), 3);
    assert!(s.settings.iter().all(|s| !s.on && s.warning.len() > 40));
    for kind in [pb::ItemKind::Fence, pb::ItemKind::Rally] {
        let e = c.upload_items(upload(kind, true)).await.unwrap_err();
        assert_eq!(e.code(), tonic::Code::FailedPrecondition);
        assert!(
            e.message().contains("fence-rally-upload"),
            "{}",
            e.message()
        );
    }
    let e = c
        .erase_logs(pb::EraseLogsRequest {
            sysid: 0,
            compid: 0,
        })
        .await
        .unwrap_err();
    assert!(e.message().contains("log-erase"), "{}", e.message());
    armed.store(true, Ordering::Relaxed);
    tokio::time::sleep(Duration::from_millis(300)).await;
    let e = c
        .upload_items(upload(pb::ItemKind::Mission, true))
        .await
        .unwrap_err();
    assert!(
        e.message().contains("armed-mission-upload"),
        "{}",
        e.message()
    );
    tokio::time::sleep(Duration::from_millis(200)).await;
    assert_eq!(
        changes.load(Ordering::Relaxed),
        0,
        "nothing reached the vehicle"
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn armed_vehicle_gets_the_warning_first() {
    let all_on = VehicleChangeSettings {
        armed_mission_upload: true,
        fence_rally_upload: true,
        log_erase: true,
    };
    let (mut c, armed, changes) = client(all_on).await;
    armed.store(true, Ordering::Relaxed);
    tokio::time::sleep(Duration::from_millis(300)).await;
    let r = c
        .upload_items(upload(pb::ItemKind::Mission, false))
        .await
        .unwrap()
        .into_inner();
    assert_eq!(r.armed_warning, dc_core::ARMED_MISSION_UPLOAD_WARNING);
    let r = c
        .upload_items(upload(pb::ItemKind::Fence, false))
        .await
        .unwrap()
        .into_inner();
    assert_eq!(r.armed_warning, dc_core::ARMED_FENCE_RALLY_WARNING);
    // Log erase needs a disarmed vehicle whatever the setting.
    let e = c
        .erase_logs(pb::EraseLogsRequest {
            sysid: 0,
            compid: 0,
        })
        .await
        .unwrap_err();
    assert!(e.message().contains("armed"), "{}", e.message());
    tokio::time::sleep(Duration::from_millis(200)).await;
    assert_eq!(
        changes.load(Ordering::Relaxed),
        0,
        "nothing reached the vehicle"
    );
}
