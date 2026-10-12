//! gRPC API over a real socket, with a minimal fake vehicle (3 parameters) on TCP.

use std::time::Duration;

use api::pb::{self, core_client::CoreClient};
use dc_core::{Core, CoreConfig};
use mavlink_codec::enums::MavParamType;
use mavlink_codec::messages::{
    Heartbeat, LogData, LogEntry, MissionCount, MissionItemInt, ParamValue, Statustext,
};

/// The fake vehicle's one dataflash log.
fn log_byte(i: u32) -> u8 {
    (i % 251) as u8
}
const LOG_SIZE: u32 = 200;
use mavlink_codec::{FrameParser, Header, Message, Version, encode};
use protocols::params::encode_id;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;

async fn fake_vehicle(listener: TcpListener) {
    let (mut sock, _) = listener.accept().await.unwrap();
    let mut parser = FrameParser::new();
    let mut buf = [0u8; 1024];
    let mut tick = tokio::time::interval(Duration::from_millis(100));
    loop {
        let mut out = Vec::new();
        tokio::select! {
            _ = tick.tick() => {
                out.push(Message::Heartbeat(Heartbeat { r#type: 2, autopilot: 3, mavlink_version: 3, ..Default::default() }));
                let mut text = [0u8; 50];
                text[..9].copy_from_slice(b"PreArm: x");
                out.push(Message::Statustext(Statustext { severity: 2, text, ..Default::default() }));
            }
            n = sock.read(&mut buf) => {
                let Ok(n) = n else { return };
                if n == 0 { return; }
                parser.push(&buf[..n]);
                while let Some(f) = parser.next_frame() {
                    // Missions: 2 items (home and one waypoint); fence and rally: empty.
                    match f.decode() {
                        Ok(Message::LogRequestList(_)) => out.push(Message::LogEntry(LogEntry {
                            id: 1,
                            num_logs: 1,
                            last_log_num: 1,
                            time_utc: 1_760_000_000,
                            size: LOG_SIZE,
                        })),
                        Ok(Message::LogRequestData(r)) => {
                            let mut ofs = r.ofs;
                            let end = (r.ofs + r.count).min(LOG_SIZE);
                            while ofs < end {
                                let n = (end - ofs).min(90);
                                let mut data = [0u8; 90];
                                for (k, b) in data.iter_mut().take(n as usize).enumerate() {
                                    *b = log_byte(ofs + k as u32);
                                }
                                out.push(Message::LogData(LogData { id: r.id, ofs, count: n as u8, data }));
                                ofs += n;
                            }
                        }
                        Ok(Message::MissionRequestList(r)) => out.push(Message::MissionCount(MissionCount {
                            target_system: 255,
                            target_component: 190,
                            count: if r.mission_type == 0 { 2 } else { 0 },
                            mission_type: r.mission_type,
                        })),
                        Ok(Message::MissionRequestInt(r)) => out.push(Message::MissionItemInt(MissionItemInt {
                            target_system: 255,
                            target_component: 190,
                            seq: r.seq,
                            frame: if r.seq == 0 { 0 } else { 3 },
                            command: 16,
                            x: -353_632_610 + i32::from(r.seq),
                            y: 1_491_652_300,
                            z: 20.0 * f32::from(r.seq),
                            autocontinue: 1,
                            mission_type: r.mission_type,
                            ..Default::default()
                        })),
                        _ => {}
                    }
                    if let Ok(Message::ParamRequestList(_)) = f.decode() {
                        for (i, name) in ["ALPHA", "BETA", "GAMMA"].iter().enumerate() {
                            out.push(Message::ParamValue(ParamValue {
                                param_id: encode_id(name),
                                param_value: i as f32,
                                param_type: MavParamType::INT32.0,
                                param_count: 3,
                                param_index: i as u16,
                            }));
                        }
                    }
                }
            }
        }
        for m in out {
            let f = encode(
                Version::V2,
                Header {
                    seq: 0,
                    sysid: 7,
                    compid: 1,
                },
                &m,
            )
            .unwrap();
            if sock.write_all(f.as_bytes()).await.is_err() {
                return;
            }
        }
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn grpc_api_end_to_end() {
    let core = Core::start(CoreConfig::default()).await.unwrap();
    let grpc = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let url = format!("http://{}", grpc.local_addr().unwrap());
    tokio::spawn(api::serve_on(core.clone(), grpc));
    let mut c = CoreClient::connect(url).await.unwrap();

    let info = c
        .get_info(pb::GetInfoRequest {})
        .await
        .unwrap()
        .into_inner();
    assert_eq!(
        (info.sysid, info.compid, info.dialect.as_str()),
        (255, 190, "ardupilotmega.xml")
    );

    // Errors are typed, not panics.
    let bad = c
        .open_link(pb::OpenLinkRequest {
            spec: "nonsense".into(),
        })
        .await
        .unwrap_err();
    assert_eq!(bad.code(), tonic::Code::InvalidArgument);
    let none = c
        .get_params(pb::GetParamsRequest {
            sysid: 0,
            compid: 0,
        })
        .await
        .unwrap_err();
    assert_eq!(none.code(), tonic::Code::Unavailable, "no vehicle yet");

    // Open a link to the fake vehicle through the API.
    let vehicle = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let spec = format!("tcp:{}", vehicle.local_addr().unwrap());
    tokio::spawn(fake_vehicle(vehicle));
    c.open_link(pb::OpenLinkRequest { spec }).await.unwrap();
    assert_eq!(core.wait_for_vehicle(Duration::from_secs(5)).await, Some(7));

    let list = c
        .list_vehicles(pb::ListVehiclesRequest {})
        .await
        .unwrap()
        .into_inner();
    assert_eq!(list.vehicles.len(), 1);
    let v = &list.vehicles[0];
    assert!(v.heartbeat.is_some());
    assert_eq!(v.link(), pb::vehicle_link_event::State::Connected);
    // The fake vehicle sends no EKF_STATUS_REPORT: trust is UNKNOWN, with the reason given.
    let nav = v.nav.as_ref().expect("nav health is always set by the API");
    assert_eq!(nav.trust(), pb::PositionTrust::Unknown);
    assert!(!nav.reasons.is_empty(), "{nav:?}");

    let params = c
        .get_params(pb::GetParamsRequest {
            sysid: 0,
            compid: 0,
        })
        .await
        .unwrap()
        .into_inner();
    assert_eq!(params.sysid, 7);
    let names: Vec<_> = params.params.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, ["ALPHA", "BETA", "GAMMA"]);

    let items = c
        .download_items(pb::DownloadItemsRequest {
            sysid: 0,
            compid: 0,
            kind: pb::ItemKind::Mission as i32,
        })
        .await
        .unwrap()
        .into_inner();
    assert_eq!(
        (items.sysid, items.compid, items.kind()),
        (7, 1, pb::ItemKind::Mission)
    );
    assert_eq!(items.items.len(), 2);
    assert_eq!(
        (
            items.items[1].frame,
            items.items[1].x,
            items.items[1].z,
            items.items[1].params.len()
        ),
        (3, -353_632_609, 20.0, 4)
    );
    let rally = c
        .download_items(pb::DownloadItemsRequest {
            sysid: 7,
            compid: 1,
            kind: pb::ItemKind::Rally as i32,
        })
        .await
        .unwrap()
        .into_inner();
    assert!(rally.items.is_empty());
    let bad = c
        .download_items(pb::DownloadItemsRequest {
            sysid: 7,
            compid: 1,
            kind: 9,
        })
        .await
        .unwrap_err();
    assert_eq!(bad.code(), tonic::Code::InvalidArgument);

    let msgs = c
        .get_messages(pb::GetMessagesRequest { sysid: 0 })
        .await
        .unwrap()
        .into_inner();
    assert_eq!(msgs.sysid, 7);
    let m = msgs.messages.last().expect("STATUSTEXT kept");
    assert_eq!((m.severity, m.text.as_str(), m.compid), (2, "PreArm: x", 1));
    assert!(m.unix_ms > 1_700_000_000_000, "wall-clock time");

    let stats = c
        .get_link_stats(pb::GetLinkStatsRequest {})
        .await
        .unwrap()
        .into_inner();
    assert_eq!(stats.links.len(), 1, "{stats:?}");
    assert!(
        stats
            .routes
            .iter()
            .any(|r| (r.sysid, r.compid, r.link_id) == (7, 1, stats.links[0].id)),
        "{stats:?}"
    );
    assert!(stats.frames > 0 && stats.app_dropped == 0, "{stats:?}");

    let logs = c
        .list_logs(pb::ListLogsRequest {
            sysid: 0,
            compid: 0,
        })
        .await
        .unwrap()
        .into_inner();
    assert_eq!(logs.logs.len(), 1);
    assert_eq!((logs.logs[0].id, logs.logs[0].size), (1, LOG_SIZE));
    let mut stream = c
        .download_log(pb::DownloadLogRequest {
            sysid: 0,
            compid: 0,
            id: 1,
        })
        .await
        .unwrap()
        .into_inner();
    let mut data = Vec::new();
    loop {
        let u = stream
            .message()
            .await
            .unwrap()
            .expect("stream ends with done");
        data.extend_from_slice(&u.data);
        if u.done {
            break;
        }
    }
    assert_eq!(data, (0..LOG_SIZE).map(log_byte).collect::<Vec<_>>());
    let missing = c
        .download_log(pb::DownloadLogRequest {
            sysid: 0,
            compid: 0,
            id: 9,
        })
        .await
        .unwrap_err();
    assert_eq!(missing.code(), tonic::Code::NotFound);

    let mut tele = c
        .stream_telemetry(pb::StreamTelemetryRequest {
            sysid: 7,
            rate_hz: 20.0,
        })
        .await
        .unwrap()
        .into_inner();
    let first = tokio::time::timeout(Duration::from_secs(5), tele.message())
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    assert_eq!(first.sysid, 7);
}

/// ADR-0012 item 18: with a pre-shared key, calls without it or with a wrong one are refused.
#[tokio::test(flavor = "multi_thread")]
async fn api_key_is_required_when_set() {
    let core = Core::start(CoreConfig::default()).await.unwrap();
    let grpc = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let url = format!("http://{}", grpc.local_addr().unwrap());
    tokio::spawn(api::serve_on_with(
        core.clone(),
        grpc,
        api::ServeOptions {
            token: Some("s3cret-field-key".into()),
        },
    ));
    let channel = tonic::transport::Endpoint::from_shared(url)
        .unwrap()
        .connect()
        .await
        .unwrap();
    let call = |key: Option<&'static str>| {
        let channel = channel.clone();
        async move {
            let mut c = CoreClient::with_interceptor(channel, move |mut req: tonic::Request<()>| {
                if let Some(k) = key {
                    req.metadata_mut()
                        .insert("authorization", format!("Bearer {k}").parse().unwrap());
                }
                Ok(req)
            });
            c.get_info(pb::GetInfoRequest {})
                .await
                .map(|r| r.into_inner())
        }
    };
    assert_eq!(
        call(None).await.unwrap_err().code(),
        tonic::Code::Unauthenticated
    );
    assert_eq!(
        call(Some("s3cret-field-kez")).await.unwrap_err().code(),
        tonic::Code::Unauthenticated
    );
    assert_eq!(
        call(Some("s3cret")).await.unwrap_err().code(),
        tonic::Code::Unauthenticated,
        "a prefix is not the key"
    );
    assert!(call(Some("s3cret-field-key")).await.is_ok());
}
