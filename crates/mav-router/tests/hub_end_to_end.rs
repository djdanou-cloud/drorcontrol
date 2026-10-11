//! End-to-end over real sockets: a simulated vehicle on UDP, a third-party tool on TCP,
//! and the GCS application behind the hub. Checks local delivery, relaying, targeted
//! routing, loop prevention and a byte-exact `.tlog` of everything the hub saw.

use std::time::Duration;

use link_core::LinkIds;
use link_net::{UdpMode, spawn_tcp_server, spawn_udp};
use mav_router::{HubOutput, LinkPolicy, RouterConfig, spawn_hub};
use mavlink_codec::messages::{CommandLong, Heartbeat, ParamRequestList};
use mavlink_codec::{Frame, FrameParser, Header, Message, Version, encode};
use recorder::{TlogReader, TlogWriter};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpStream, UdpSocket};
use tokio::time::timeout;

const T: Duration = Duration::from_secs(5);

fn frame(sysid: u8, compid: u8, msg: Message) -> Frame {
    encode(
        Version::V2,
        Header {
            seq: 0,
            sysid,
            compid,
        },
        &msg,
    )
    .unwrap()
}

fn heartbeat(sysid: u8) -> Frame {
    frame(
        sysid,
        1,
        Message::Heartbeat(Heartbeat {
            custom_mode: u32::from(sysid),
            ..Default::default()
        }),
    )
}

fn command(from: (u8, u8), target: (u8, u8)) -> Frame {
    let m = Message::CommandLong(CommandLong {
        target_system: target.0,
        target_component: target.1,
        command: 400,
        ..Default::default()
    });
    frame(from.0, from.1, m)
}

async fn udp_recv(sock: &UdpSocket) -> Frame {
    let mut buf = [0u8; 512];
    let n = timeout(T, sock.recv(&mut buf))
        .await
        .expect("udp timeout")
        .unwrap();
    Frame::from_bytes(&buf[..n]).unwrap()
}

async fn tcp_recv(stream: &mut TcpStream, parser: &mut FrameParser) -> Frame {
    let mut buf = [0u8; 512];
    loop {
        if let Some(f) = parser.next_frame() {
            return f;
        }
        let n = timeout(T, stream.read(&mut buf))
            .await
            .expect("tcp timeout")
            .unwrap();
        assert!(n > 0, "tcp closed");
        parser.push(&buf[..n]);
    }
}

/// Wait until the app receives `want` (other local deliveries in between are fine).
async fn expect_local(app: &mut tokio::sync::mpsc::Receiver<HubOutput>, want: &Frame) {
    loop {
        match timeout(T, app.recv()).await.expect("app timeout").unwrap() {
            HubOutput::Frame { frame, .. } if &frame == want => return,
            _ => continue,
        }
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn routes_between_udp_vehicle_tcp_tool_and_app() {
    let (hub, mut out) = spawn_hub(RouterConfig::default(), LinkPolicy::relay_all());
    let ids = LinkIds::new();

    // GCS UDP server (like port 14550) and a TCP server for third-party tools.
    let (udp_link, udp_addr) = spawn_udp(
        ids.next(),
        UdpMode::Server {
            bind: "127.0.0.1:0".parse().unwrap(),
        },
        hub.events.clone(),
    )
    .await
    .unwrap();
    let tcp_addr = spawn_tcp_server(
        ids.clone(),
        "127.0.0.1:0".parse().unwrap(),
        hub.events.clone(),
    )
    .await
    .unwrap();

    // Vehicle (sysid 1) on UDP announces itself; the app gets it.
    let vehicle = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    vehicle.connect(udp_addr).await.unwrap();
    vehicle.send(heartbeat(1).as_bytes()).await.unwrap();
    expect_local(&mut out.app, &heartbeat(1)).await;

    // Tool (sysid 200) connects over TCP; vehicle traffic is relayed to it.
    let mut tool = TcpStream::connect(tcp_addr).await.unwrap();
    let mut tool_rx = FrameParser::new();
    let tool_link = loop {
        match timeout(T, out.links.recv()).await.unwrap().unwrap() {
            HubOutput::LinkAdded { link, name } if name.starts_with("tcp-server") => break link,
            _ => continue,
        }
    };
    vehicle.send(heartbeat(1).as_bytes()).await.unwrap();
    assert_eq!(tcp_recv(&mut tool, &mut tool_rx).await, heartbeat(1));

    // Tool announces itself and commands the vehicle: relayed to UDP only, not to the app.
    tool.write_all(heartbeat(200).as_bytes()).await.unwrap();
    assert_eq!(udp_recv(&vehicle).await, heartbeat(200));
    let cmd = command((200, 1), (1, 1));
    tool.write_all(cmd.as_bytes()).await.unwrap();
    assert_eq!(udp_recv(&vehicle).await, cmd);

    // The app asks vehicle 1 for parameters: goes to the vehicle, not to the tool.
    let req = frame(
        255,
        190,
        Message::ParamRequestList(ParamRequestList {
            target_system: 1,
            target_component: 1,
        }),
    );
    assert!(hub.send(req.clone()).await);
    assert_eq!(udp_recv(&vehicle).await, req);

    // A command addressed to the GCS is delivered locally and not relayed.
    let to_us = command((1, 1), (255, 190));
    vehicle.send(to_us.as_bytes()).await.unwrap();
    expect_local(&mut out.app, &to_us).await;

    // Loop: a radio echoing our own frame back is dropped.
    vehicle.send(req.as_bytes()).await.unwrap();
    // Barrier: a later vehicle heartbeat proves the echo was processed before we look.
    vehicle.send(heartbeat(1).as_bytes()).await.unwrap();
    expect_local(&mut out.app, &heartbeat(1)).await;
    assert_eq!(
        tcp_recv(&mut tool, &mut tool_rx).await,
        heartbeat(1),
        "tool saw no echo, only the heartbeat"
    );

    let snap = hub.snapshot().await.unwrap();
    assert_eq!(snap.stats.loops_dropped, 1);
    assert!(snap.routes.contains(&((1, 1), udp_link.id)));
    assert!(snap.routes.contains(&((200, 1), tool_link)));

    // Everything the hub saw, in order, round-trips through a .tlog byte-exactly.
    let mut tapped = Vec::new();
    while let Ok(t) = out.tap.try_recv() {
        tapped.push(t);
    }
    let expected = [
        heartbeat(1),
        heartbeat(1),
        heartbeat(200),
        cmd,
        req.clone(),
        to_us,
        req,
        heartbeat(1),
    ];
    assert_eq!(
        tapped.iter().map(|t| t.frame.clone()).collect::<Vec<_>>(),
        expected
    );

    let mut w = TlogWriter::new(Vec::new());
    for t in &tapped {
        w.write(t.time, &t.frame).unwrap();
    }
    let log = w.into_inner().unwrap();
    let replay: Vec<Frame> = TlogReader::new(log.as_slice())
        .map(|r| r.unwrap().frame)
        .collect();
    assert_eq!(replay, expected);
}

/// Audit finding: link notifications used to share the frame queue and were dropped when a
/// telemetry burst filled it, so a "link down" could be lost. They now have their own queue.
#[tokio::test]
async fn link_down_survives_a_full_frame_queue() {
    use link_core::{LinkEvent, LinkHandle, LinkId};
    use mav_router::hub::APP_QUEUE;
    use std::time::SystemTime;

    let (hub, mut out) = spawn_hub(RouterConfig::default(), LinkPolicy::relay_all());
    let (handle, _outbound) = LinkHandle::new(LinkId(7), "radio");
    hub.events.send(LinkEvent::Added(handle)).await.unwrap();
    // Flood more frames than the application queue holds; nobody drains `out.app`.
    for i in 0..APP_QUEUE + 500 {
        let ev = LinkEvent::Frame {
            link: LinkId(7),
            frame: heartbeat(1 + (i % 2) as u8),
            rx_time: SystemTime::now(),
        };
        hub.events.send(ev).await.unwrap();
    }
    hub.events
        .send(LinkEvent::Status {
            link: LinkId(7),
            up: false,
            detail: "radio lost".into(),
        })
        .await
        .unwrap();

    let snap = hub.snapshot().await.unwrap();
    assert!(
        snap.app_dropped > 0,
        "the frame queue overflowed (router also drops duplicates)"
    );
    assert_eq!(snap.links_dropped, 0);
    assert!(matches!(
        timeout(T, out.links.recv()).await.unwrap(),
        Some(HubOutput::LinkAdded {
            link: LinkId(7),
            ..
        })
    ));
    match timeout(T, out.links.recv()).await.unwrap() {
        Some(HubOutput::LinkStatus { link, up, detail }) => {
            assert_eq!(
                (link, up, detail.as_str()),
                (LinkId(7), false, "radio lost")
            )
        }
        other => panic!("expected link-down notice, got {other:?}"),
    }
}
