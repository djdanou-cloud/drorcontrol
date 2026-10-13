//! SetParams over a real socket (ADR-0012 item 1): writes off without a baseline directory,
//! the armed warning instead of a write, then a verified write once acknowledged.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use api::pb::{self, core_client::CoreClient};
use dc_core::{Core, CoreConfig};
use mavlink_codec::enums::{MavModeFlag, MavParamType};
use mavlink_codec::messages::{Heartbeat, ParamValue};
use mavlink_codec::{FrameParser, Header, Message, Version, encode};
use protocols::params::{decode_id, encode_id};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;

/// A vehicle with two REAL32 parameters that answers sets, reads and list requests.
async fn vehicle(
    listener: TcpListener,
    armed: Arc<AtomicBool>,
    params: Arc<Mutex<BTreeMap<String, f32>>>,
) {
    let (mut sock, _) = listener.accept().await.unwrap();
    let mut parser = FrameParser::new();
    let mut buf = [0u8; 1024];
    let mut tick = tokio::time::interval(Duration::from_millis(100));
    let value = |name: &str, v: f32, i: u16| {
        Message::ParamValue(ParamValue {
            param_id: encode_id(name),
            param_value: v,
            param_type: MavParamType::REAL32.0,
            param_count: 2,
            param_index: i,
        })
    };
    loop {
        let mut out = Vec::new();
        tokio::select! {
            _ = tick.tick() => out.push(Message::Heartbeat(Heartbeat {
                r#type: 2,
                autopilot: 3,
                base_mode: if armed.load(Ordering::Relaxed) { MavModeFlag::SAFETY_ARMED.0 } else { 0 },
                mavlink_version: 3,
                ..Default::default()
            })),
            n = sock.read(&mut buf) => {
                let Ok(n) = n else { return };
                if n == 0 { return; }
                parser.push(&buf[..n]);
                while let Some(f) = parser.next_frame() {
                    let mut p = params.lock().unwrap();
                    match f.decode() {
                        Ok(Message::ParamRequestList(_)) => {
                            for (i, (k, v)) in p.iter().enumerate() {
                                out.push(value(k, *v, i as u16));
                            }
                        }
                        Ok(Message::ParamSet(s)) => {
                            let name = decode_id(&s.param_id);
                            p.insert(name.clone(), s.param_value);
                            out.push(value(&name, s.param_value, u16::MAX));
                        }
                        Ok(Message::ParamRequestRead(r)) => {
                            let name = decode_id(&r.param_id);
                            if let Some(v) = p.get(&name) {
                                out.push(value(&name, *v, u16::MAX));
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
        for m in out {
            let f = encode(
                Version::V2,
                Header {
                    seq: 0,
                    sysid: 3,
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

async fn client(
    param_dir: Option<std::path::PathBuf>,
) -> (
    CoreClient<tonic::transport::Channel>,
    Arc<AtomicBool>,
    Arc<Mutex<BTreeMap<String, f32>>>,
) {
    let armed = Arc::new(AtomicBool::new(false));
    let params = Arc::new(Mutex::new(BTreeMap::from([
        ("FS_EKF_THRESH".to_string(), 0.8f32),
        ("WPNAV_SPEED".to_string(), 500.0),
    ])));
    let v = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let vaddr = v.local_addr().unwrap();
    tokio::spawn(vehicle(v, armed.clone(), params.clone()));
    let core = Core::start(CoreConfig::default()).await.unwrap();
    core.connect(&dc_core::LinkSpec::Tcp(vaddr)).await.unwrap();
    assert_eq!(core.wait_for_vehicle(Duration::from_secs(5)).await, Some(3));
    let g = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let url = format!("http://{}", g.local_addr().unwrap());
    tokio::spawn(api::serve_on_with(
        core,
        g,
        api::ServeOptions {
            token: None,
            param_dir,
        },
    ));
    (CoreClient::connect(url).await.unwrap(), armed, params)
}

fn request(ack: bool) -> pb::SetParamsRequest {
    pb::SetParamsRequest {
        sysid: 0,
        compid: 0,
        changes: vec![pb::ParamAssignment {
            name: "FS_EKF_THRESH".into(),
            value: 0.9,
        }],
        acknowledge_armed_warning: ack,
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn writes_are_off_without_a_baseline_directory() {
    let (mut c, _, params) = client(None).await;
    let e = c.set_params(request(true)).await.unwrap_err();
    assert_eq!(e.code(), tonic::Code::FailedPrecondition);
    assert_eq!(params.lock().unwrap()["FS_EKF_THRESH"], 0.8);
}

#[tokio::test(flavor = "multi_thread")]
async fn armed_vehicle_gets_the_warning_then_an_acknowledged_verified_write() {
    let dir = std::env::temp_dir().join(format!("dc-setparams-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let (mut c, armed, params) = client(Some(dir.clone())).await;
    armed.store(true, Ordering::Relaxed);
    tokio::time::sleep(Duration::from_millis(400)).await;
    let r = c.set_params(request(false)).await.unwrap().into_inner();
    assert_eq!(r.armed_warning, dc_core::ARMED_WRITE_WARNING);
    assert!(r.results.is_empty() && r.snapshot.is_empty());
    assert_eq!(params.lock().unwrap()["FS_EKF_THRESH"], 0.8, "nothing sent");
    let r = c.set_params(request(true)).await.unwrap().into_inner();
    assert!(r.verified, "{r:?}");
    assert!(r.armed_warning.is_empty());
    assert_eq!(r.results.len(), 1);
    assert_eq!(r.results[0].status(), pb::ParamWriteStatus::Verified);
    assert!((r.results[0].value - 0.9).abs() < 1e-6);
    assert_eq!(params.lock().unwrap()["FS_EKF_THRESH"], 0.9);
    let snap = std::path::Path::new(&r.snapshot);
    assert!(snap.starts_with(&dir) && snap.exists(), "{}", r.snapshot);
    let bad = c
        .set_params(pb::SetParamsRequest {
            changes: vec![],
            ..request(true)
        })
        .await
        .unwrap_err();
    assert_eq!(bad.code(), tonic::Code::InvalidArgument);
    std::fs::remove_dir_all(dir).unwrap();
}
