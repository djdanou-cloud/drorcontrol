//! `Core` against a scripted fake vehicle on TCP (no SITL needed): heartbeat detection,
//! stream-rate requests, telemetry into the fleet snapshot, STATUSTEXT events, and a
//! parameter download that has to recover values the "radio" dropped.

use std::collections::BTreeMap;
use std::time::Duration;

use dc_core::{Core, CoreConfig, CoreEvent, LinkSpec};
use mavlink_codec::enums::{MavCmd, MavParamType};
use mavlink_codec::messages::{Attitude, Heartbeat, ParamValue, Statustext};
use mavlink_codec::{FrameParser, Header, Message, Version, encode};
use protocols::params::encode_id;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;
use tokio::sync::mpsc;

const PARAMS: usize = 40;

fn param_value(i: usize) -> Message {
    Message::ParamValue(ParamValue {
        param_id: encode_id(&format!("TEST_P{i:02}")),
        param_value: i as f32 * 0.5,
        param_type: MavParamType::REAL32.0,
        param_count: PARAMS as u16,
        param_index: i as u16,
    })
}

/// Fake autopilot (sysid 1, compid 1). Reports every command it receives on `seen`.
async fn fake_vehicle(listener: TcpListener, seen: mpsc::UnboundedSender<Message>) {
    let (mut sock, _) = listener.accept().await.unwrap();
    let mut parser = FrameParser::new();
    let mut seq = 0u8;
    let mut buf = [0u8; 2048];
    let mut tick = tokio::time::interval(Duration::from_millis(100));
    loop {
        let mut out: Vec<Message> = Vec::new();
        tokio::select! {
            _ = tick.tick() => {
                out.push(Message::Heartbeat(Heartbeat { r#type: 2, autopilot: 3, custom_mode: 5, mavlink_version: 3, ..Default::default() }));
                out.push(Message::Attitude(Attitude { roll: 0.1, pitch: -0.2, yaw: 1.5, ..Default::default() }));
            }
            n = sock.read(&mut buf) => {
                let n = n.unwrap();
                if n == 0 { return; }
                parser.push(&buf[..n]);
                while let Some(f) = parser.next_frame() {
                    let msg = f.decode().unwrap();
                    match &msg {
                        // Full list, but every 7th value is "lost" on the link.
                        Message::ParamRequestList(_) => {
                            out.extend((0..PARAMS).filter(|i| i % 7 != 3).map(param_value));
                            let mut s = Statustext { severity: 6, ..Default::default() };
                            s.text[..11].copy_from_slice(b"ArduCopter!");
                            out.push(Message::Statustext(s));
                        }
                        Message::ParamRequestRead(r) => out.push(param_value(r.param_index as usize)),
                        _ => {}
                    }
                    let _ = seen.send(msg);
                }
            }
        }
        for m in out {
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
            seq = seq.wrapping_add(1);
            sock.write_all(f.as_bytes()).await.unwrap();
        }
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn core_talks_to_a_vehicle() {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let (seen_tx, mut seen) = mpsc::unbounded_channel();
    tokio::spawn(fake_vehicle(listener, seen_tx));

    let tlog = std::env::temp_dir().join(format!("dc-core-test-{}.tlog", std::process::id()));
    let core = Core::start(CoreConfig {
        tlog: Some(tlog.clone()),
        ..Default::default()
    })
    .await
    .unwrap();
    let mut events = core.events();
    core.connect(&LinkSpec::Tcp(addr)).await.unwrap();

    assert_eq!(core.wait_for_vehicle(Duration::from_secs(5)).await, Some(1));

    // We announce ourselves as a GCS and ask for telemetry rates.
    let mut got_heartbeat = false;
    let mut intervals = Vec::new();
    while !(got_heartbeat && intervals.len() == core.config().stream_rates.len()) {
        match tokio::time::timeout(Duration::from_secs(5), seen.recv())
            .await
            .unwrap()
            .unwrap()
        {
            Message::Heartbeat(h) => got_heartbeat |= h.r#type == 6,
            Message::CommandLong(c) if c.command == MavCmd::SET_MESSAGE_INTERVAL.0 => {
                intervals.push(c.param1 as u32)
            }
            _ => {}
        }
    }
    assert!(intervals.contains(&30), "ATTITUDE requested");

    // Telemetry lands in the snapshot. ATTITUDE follows the HEARTBEAT that made the vehicle
    // known, so wait for it rather than assume it was applied already (seen racing on an M1).
    let att = tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            if let Some(a) = core.fleet().get(1).and_then(|v| v.attitude.clone()) {
                return a;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    })
    .await
    .expect("attitude within 2 s");
    assert!((att.value.yaw - 1.5).abs() < 1e-6);

    // Parameter download recovers the dropped values.
    let mut progress = Vec::new();
    let params = core
        .download_params((1, 1), |r, t| progress.push((r, t)))
        .await
        .unwrap();
    assert_eq!(params.len(), PARAMS);
    assert_eq!(
        params["TEST_P03"].value, 1.5,
        "a value that was dropped and re-requested"
    );
    assert!(progress.iter().any(|&(_, t)| t == PARAMS));
    let rereads: Vec<u16> = std::iter::from_fn(|| seen.try_recv().ok())
        .filter_map(|m| match m {
            Message::ParamRequestRead(r) => Some(r.param_index as u16),
            _ => None,
        })
        .collect();
    let mut dropped: Vec<u16> = (0..PARAMS as u16).filter(|i| i % 7 == 3).collect();
    let mut asked = rereads.clone();
    asked.sort_unstable();
    asked.dedup();
    dropped.sort_unstable();
    assert_eq!(asked, dropped, "only missing indices are re-requested");

    // STATUSTEXT surfaces as an event.
    let text = loop {
        if let CoreEvent::StatusText { text, .. } =
            tokio::time::timeout(Duration::from_secs(5), events.recv())
                .await
                .unwrap()
                .unwrap()
        {
            break text;
        }
    };
    assert_eq!(text.text, "ArduCopter!");

    // The tlog holds what we exchanged.
    drop(core);
    tokio::time::sleep(Duration::from_millis(1200)).await;
    let bytes = std::fs::read(&tlog).unwrap();
    let mut by_type: BTreeMap<String, usize> = BTreeMap::new();
    // The writer is still live, so the file may end mid-record: read the complete prefix.
    for r in recorder::TlogReader::new(bytes.as_slice()).map_while(Result::ok) {
        let f = r.frame;
        *by_type
            .entry(f.decode().unwrap().name().to_string())
            .or_default() += 1;
    }
    assert!(by_type.get("PARAM_VALUE").copied().unwrap_or(0) >= PARAMS);
    assert!(by_type.contains_key("HEARTBEAT"));
    let _ = std::fs::remove_file(&tlog);
}
