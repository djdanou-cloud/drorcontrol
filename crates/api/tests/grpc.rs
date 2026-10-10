//! gRPC API over a real socket, with a minimal fake vehicle (3 parameters) on TCP.

use std::time::Duration;

use api::pb::{self, core_client::CoreClient};
use dc_core::{Core, CoreConfig};
use mavlink_codec::enums::MavParamType;
use mavlink_codec::messages::{Heartbeat, ParamValue};
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
            _ = tick.tick() => out.push(Message::Heartbeat(Heartbeat { r#type: 2, autopilot: 3, mavlink_version: 3, ..Default::default() })),
            n = sock.read(&mut buf) => {
                let Ok(n) = n else { return };
                if n == 0 { return; }
                parser.push(&buf[..n]);
                while let Some(f) = parser.next_frame() {
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
    assert!(list.vehicles[0].heartbeat.is_some());

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
