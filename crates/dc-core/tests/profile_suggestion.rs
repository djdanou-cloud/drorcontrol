//! Link-profile suggestion (setting `suggest-link-profile`, plan 09 §8 decision 3): a lossy
//! link gets a leaner profile suggested, never applied without the operator.

use std::time::Duration;

use dc_core::{
    Core, CoreConfig, CoreEvent, LinkProfile, LinkSpec, ProfileSuggestion, VehicleChangeSettings,
};
use mavlink_codec::messages::{Attitude, Heartbeat};
use mavlink_codec::{Header, Message, Version, encode};
use tokio::io::AsyncWriteExt;
use tokio::net::TcpListener;

/// A vehicle whose frames arrive with every fourth sequence number missing (25 % loss).
async fn lossy_vehicle(listener: TcpListener) {
    let (mut sock, _) = listener.accept().await.unwrap();
    let mut seq = 0u8;
    for i in 0u32.. {
        let msg = if i % 10 == 0 {
            Message::Heartbeat(Heartbeat {
                r#type: 2,
                autopilot: 3,
                mavlink_version: 3,
                ..Default::default()
            })
        } else {
            Message::Attitude(Attitude::default())
        };
        seq = seq.wrapping_add(if i % 4 == 3 { 2 } else { 1 });
        let f = encode(
            Version::V2,
            Header {
                seq,
                sysid: 1,
                compid: 1,
            },
            &msg,
        )
        .unwrap();
        if sock.write_all(f.as_bytes()).await.is_err() {
            return;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

async fn start(suggest: bool) -> (Core, tokio::sync::broadcast::Receiver<CoreEvent>) {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(lossy_vehicle(listener));
    let core = Core::start(CoreConfig {
        vehicle_changes: VehicleChangeSettings {
            suggest_link_profile: suggest,
            ..Default::default()
        },
        ..Default::default()
    })
    .await
    .unwrap();
    let events = core.events();
    core.connect(&LinkSpec::Tcp(addr)).await.unwrap();
    (core, events)
}

async fn suggestion(
    events: &mut tokio::sync::broadcast::Receiver<CoreEvent>,
    within: Duration,
) -> Option<ProfileSuggestion> {
    tokio::time::timeout(within, async {
        loop {
            if let Ok(CoreEvent::LinkProfileSuggested(s)) = events.recv().await {
                return s;
            }
        }
    })
    .await
    .ok()
}

#[tokio::test(flavor = "multi_thread")]
async fn off_by_default_nothing_is_suggested() {
    let (core, mut events) = start(false).await;
    assert!(
        suggestion(&mut events, Duration::from_secs(5))
            .await
            .is_none()
    );
    assert!(core.link_profile_suggestions().is_empty());
}

#[tokio::test(flavor = "multi_thread")]
async fn a_lossy_link_gets_a_leaner_profile_suggested_and_the_operator_decides() {
    let (core, mut events) = start(true).await;
    let s = suggestion(&mut events, Duration::from_secs(8))
        .await
        .expect("suggestion on a 25 % lossy link");
    assert_eq!(
        (s.sysid, s.current, s.suggested),
        (1, LinkProfile::Full, LinkProfile::High)
    );
    assert!(s.reason.contains("packet loss"), "{}", s.reason);
    // Nothing changed by itself.
    assert_eq!(core.link_profile(s.link), LinkProfile::Full);
    assert_eq!(core.link_profile_suggestions(), std::slice::from_ref(&s));

    assert_eq!(
        core.accept_link_profile_suggestion(s.link).await,
        Some(LinkProfile::High)
    );
    assert_eq!(core.link_profile(s.link), LinkProfile::High);
    assert_eq!(core.accept_link_profile_suggestion(s.link).await, None);
}

#[tokio::test(flavor = "multi_thread")]
async fn a_dismissed_suggestion_is_not_repeated() {
    let (core, mut events) = start(true).await;
    let s = suggestion(&mut events, Duration::from_secs(8))
        .await
        .expect("suggestion");
    assert!(core.dismiss_link_profile_suggestion(s.link));
    assert!(!core.dismiss_link_profile_suggestion(s.link));
    assert_eq!(core.link_profile(s.link), LinkProfile::Full);
    // Still lossy, but the dismissed profile is not suggested again for this link.
    assert!(
        suggestion(&mut events, Duration::from_secs(4))
            .await
            .is_none()
    );
}
