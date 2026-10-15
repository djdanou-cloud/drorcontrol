//! What `RC_CHANNELS.rssi` measures (plan 09 Step 2): the vehicle's parameters, then its own
//! CRSF/ELRS messages, decide whether it is RSSI or link quality.

use std::time::Duration;

use dc_core::{Core, CoreConfig, LinkSpec, SignalKind};
use mavlink_codec::messages::{Heartbeat, ParamValue, RcChannels, Statustext};
use mavlink_codec::{Header, Message, Version, encode};
use tokio::io::AsyncWriteExt;
use tokio::net::TcpListener;

fn param(name: &str, value: f32) -> Message {
    let mut id = [0u8; 16];
    id[..name.len()].copy_from_slice(name.as_bytes());
    Message::ParamValue(ParamValue {
        param_id: id,
        param_value: value,
        param_type: 6, // INT32
        param_count: 1,
        param_index: 0,
    })
}

fn text(t: &str) -> Message {
    let mut b = [0u8; 50];
    b[..t.len()].copy_from_slice(t.as_bytes());
    Message::Statustext(Statustext {
        severity: 6,
        text: b,
        ..Default::default()
    })
}

async fn until(core: &Core, f: impl Fn(&Core) -> bool) {
    tokio::time::timeout(Duration::from_secs(3), async {
        while !f(core) {
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    })
    .await
    .expect("condition within 3 s");
}

#[tokio::test(flavor = "multi_thread")]
async fn rc_rssi_is_labelled_by_parameters_then_by_the_crsf_message() {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let core = Core::start(CoreConfig::default()).await.unwrap();
    core.connect(&LinkSpec::Tcp(addr)).await.unwrap();
    let (mut sock, _) = listener.accept().await.unwrap();
    let mut seq = 0u8;
    let mut send = async |m: Message| {
        seq = seq.wrapping_add(1);
        let f = encode(
            Version::V2,
            Header {
                seq,
                sysid: 1,
                compid: 1,
            },
            &m,
        )
        .unwrap();
        sock.write_all(f.as_bytes()).await.unwrap();
    };
    send(Message::Heartbeat(Heartbeat {
        r#type: 2,
        autopilot: 3,
        mavlink_version: 3,
        ..Default::default()
    }))
    .await;
    send(Message::RcChannels(RcChannels {
        chancount: 8,
        rssi: 249,
        ..Default::default()
    }))
    .await;
    until(&core, |c| c.rc_signal(1).is_some()).await;
    assert_eq!(core.rc_signal(1).unwrap().kind, SignalKind::Unknown);

    // RSSI_TYPE 3 (receiver protocol), RC_OPTIONS without bit 11: RSSI.
    send(param("RSSI_TYPE", 3.0)).await;
    send(param("RC_OPTIONS", 32.0)).await;
    until(&core, |c| c.rc_signal(1).unwrap().kind == SignalKind::Rssi).await;
    let s = core.rc_signal(1).unwrap();
    assert!((s.pct - 98.0).abs() < 0.1, "{}", s.pct);

    // The receiver code says it shows LQ: that wins; the rates come with it.
    send(text("ELRS: RSSI now displays  as LQ")).await;
    send(text("ELRS: Link rate 250Hz, Telemetry rate 62Hz")).await;
    until(&core, |c| {
        c.rc_signal(1)
            .unwrap()
            .link
            .is_some_and(|l| l.link_rate_hz.is_some())
    })
    .await;
    let s = core.rc_signal(1).unwrap();
    assert_eq!(s.kind, SignalKind::LinkQuality);
    assert!(s.pct > 99.9, "LQ 100 % arrives as 249: {}", s.pct);
    let l = s.link.unwrap();
    assert_eq!(
        (
            l.protocol.as_str(),
            l.link_rate_hz,
            l.telemetry_rate_hz,
            l.rssi_as_lq
        ),
        ("ELRS", Some(250), Some(62), Some(true))
    );
}
