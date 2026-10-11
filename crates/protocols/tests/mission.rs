//! Mission upload / download / read-back verification (hazard H4) against an in-process
//! vehicle that behaves like ArduPilot, over a lossy link.

use std::time::{Duration, Instant};

use mavlink_codec::Message;
use mavlink_codec::messages::{MissionAck, MissionCount, MissionRequest};
use protocols::mission::*;

const T: (u8, u8) = (1, 1);

fn wp(lat: f64, lon: f64, alt: f32) -> MissionItem {
    MissionItem {
        frame: 6, // GLOBAL_RELATIVE_ALT_INT
        command: 16,
        params: [0.0; 4],
        x: (lat * 1e7) as i32,
        y: (lon * 1e7) as i32,
        z: alt,
        autocontinue: true,
    }
}

fn mission() -> Vec<MissionItem> {
    let mut m = vec![wp(32.82, 34.98, 0.0)]; // home slot
    m.push(MissionItem {
        command: 22, // TAKEOFF
        ..wp(0.0, 0.0, 30.0)
    });
    for i in 0..8 {
        m.push(wp(
            32.82 + 0.001 * f64::from(i),
            34.98,
            40.0 + i as f32 * 0.5,
        ));
    }
    m.push(MissionItem {
        command: 19, // LOITER_TIME
        params: [12.0, 0.0, 0.0, 0.0],
        ..wp(32.83, 34.99, 45.0)
    });
    m.push(MissionItem {
        command: 20, // RTL
        frame: 2,
        ..wp(0.0, 0.0, 0.0)
    });
    m
}

/// ArduPilot-like mission store: item 0 is home; altitude kept in cm; frames read back
/// without `_INT`; requests re-sent after 1 s of silence.
struct Vehicle {
    home: MissionItem,
    stored: Vec<MissionItem>,
    receiving: Option<(u16, u16)>, // (next seq, count)
    last_request: Instant,
    refuse: Option<u8>,
    /// Store param1 truncated to whole seconds (a vehicle that keeps less precision).
    truncate_param1: bool,
}

impl Vehicle {
    fn new(now: Instant) -> Self {
        Vehicle {
            home: wp(32.82, 34.98, 0.0),
            stored: Vec::new(),
            receiving: None,
            last_request: now,
            refuse: None,
            truncate_param1: false,
        }
    }

    fn request(&mut self, seq: u16, now: Instant) -> Message {
        self.last_request = now;
        Message::MissionRequest(MissionRequest {
            target_system: 255,
            target_component: 190,
            seq,
            mission_type: 0,
        })
    }

    fn handle(&mut self, msg: &Message, now: Instant) -> Vec<Message> {
        match msg {
            Message::MissionCount(c) => {
                if let Some(r) = self.refuse {
                    return vec![ack(r)];
                }
                self.receiving = Some((0, c.count));
                self.stored = vec![self.home; usize::from(c.count)];
                vec![self.request(0, now)]
            }
            Message::MissionItemInt(m) => match self.receiving {
                Some((next, count)) if m.seq == next => {
                    let mut item = MissionItem::from_message(m);
                    item.z = (item.z * 100.0).round() / 100.0;
                    if self.truncate_param1 {
                        item.params[0] = item.params[0].trunc();
                    }
                    if m.seq != 0 {
                        self.stored[usize::from(m.seq)] = item;
                    }
                    if next + 1 == count {
                        self.receiving = None;
                        vec![ack(0)]
                    } else {
                        self.receiving = Some((next + 1, count));
                        vec![self.request(next + 1, now)]
                    }
                }
                // ArduPilot: out-of-sequence item -> INVALID_SEQUENCE, transfer continues.
                Some(_) => vec![ack(13)],
                None => Vec::new(),
            },
            Message::MissionRequestList(_) => vec![Message::MissionCount(MissionCount {
                target_system: 255,
                target_component: 190,
                count: self.stored.len() as u16,
                mission_type: 0,
            })],
            Message::MissionRequestInt(r) => {
                let Some(item) = self.stored.get(usize::from(r.seq)) else {
                    return Vec::new();
                };
                let mut item = if r.seq == 0 { self.home } else { *item };
                item.frame = match item.frame {
                    5 => 0,
                    6 => 3,
                    11 => 10,
                    f => f,
                };
                vec![item.to_message((255, 190), r.seq, 0)]
            }
            _ => Vec::new(),
        }
    }

    fn tick(&mut self, now: Instant) -> Vec<Message> {
        match self.receiving {
            Some((next, _)) if now.duration_since(self.last_request) >= Duration::from_secs(1) => {
                vec![self.request(next, now)]
            }
            _ => Vec::new(),
        }
    }
}

fn ack(r: u8) -> Message {
    Message::MissionAck(MissionAck {
        target_system: 255,
        target_component: 190,
        r#type: r,
        mission_type: 0,
    })
}

/// Deterministic loss.
struct Link {
    state: u64,
    loss: f64,
}

impl Link {
    fn pass(&mut self) -> bool {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.state >> 33) as f64 / f64::from(u32::MAX >> 1)) >= self.loss
    }
}

/// Run a GCS state machine against the vehicle; returns elapsed simulated time.
fn run<G>(
    gcs: &mut G,
    first: Message,
    (v, link): (&mut Vehicle, &mut Link),
    t0: Instant,
    on_msg: impl Fn(&mut G, &Message, Instant) -> Vec<Message>,
    poll: impl Fn(&mut G, Instant) -> Vec<Message>,
    done: impl Fn(&G) -> bool,
) -> Duration {
    let mut to_vehicle = vec![first];
    let mut now = t0;
    while !done(gcs) && now < t0 + Duration::from_secs(120) {
        let mut to_gcs = Vec::new();
        for m in to_vehicle.drain(..) {
            if link.pass() {
                to_gcs.extend(v.handle(&m, now));
            }
        }
        to_gcs.extend(v.tick(now));
        for m in to_gcs {
            if link.pass() {
                to_vehicle.extend(on_msg(gcs, &m, now));
            }
        }
        to_vehicle.extend(poll(gcs, now));
        now += Duration::from_millis(10);
    }
    now - t0
}

fn upload(v: &mut Vehicle, items: &[MissionItem], link: &mut Link, t0: Instant) -> TransferStatus {
    let (mut up, first) = MissionUpload::start(
        T,
        items.to_vec(),
        MISSION_TYPE_MISSION,
        TransferConfig::default(),
        t0,
    );
    run(
        &mut up,
        first,
        (v, link),
        t0,
        |g, m, n| g.on_message(m, n),
        |g, n| g.poll(n),
        |g| !matches!(g.status(), TransferStatus::InProgress { .. }),
    );
    up.status().clone()
}

fn download(v: &mut Vehicle, link: &mut Link, t0: Instant) -> (TransferStatus, Vec<MissionItem>) {
    let (mut dl, first) =
        MissionDownload::start(T, MISSION_TYPE_MISSION, TransferConfig::default(), t0);
    run(
        &mut dl,
        first,
        (v, link),
        t0,
        |g, m, n| g.on_message(m, n),
        |g, n| g.poll(n),
        |g| !matches!(g.status(), TransferStatus::InProgress { .. }),
    );
    (dl.status().clone(), dl.items().to_vec())
}

#[test]
fn upload_then_read_back_verifies() {
    let t0 = Instant::now();
    let mut v = Vehicle::new(t0);
    let mut link = Link {
        state: 1,
        loss: 0.0,
    };
    let m = mission();
    assert_eq!(upload(&mut v, &m, &mut link, t0), TransferStatus::Done);
    let (st, back) = download(&mut v, &mut link, t0);
    assert_eq!(st, TransferStatus::Done);
    assert_eq!(back.len(), m.len());
    assert_eq!(back[2].frame, 3, "read back without _INT");
    assert_eq!(
        verify(&m, &back, true),
        vec![],
        "home skipped, frames and cm handled"
    );
    // Item 0 really is home on the vehicle, not what we sent.
    let mut wrong_home = m.clone();
    wrong_home[0].x += 1000;
    assert_eq!(verify(&wrong_home, &back, true), vec![]);
}

#[test]
fn completes_through_30_percent_loss_both_ways() {
    let m = mission();
    for seed in 1..=20 {
        let t0 = Instant::now();
        let mut v = Vehicle::new(t0);
        let mut link = Link {
            state: seed,
            loss: 0.30,
        };
        let up = upload(&mut v, &m, &mut link, t0);
        // A lost final ACK fails the upload on the GCS side, but the read-back decides.
        let (st, back) = download(&mut v, &mut link, t0);
        assert_eq!(st, TransferStatus::Done, "seed {seed}");
        assert_eq!(
            verify(&m, &back, true),
            vec![],
            "seed {seed} (upload said {up:?})"
        );
    }
}

#[test]
fn read_back_catches_what_the_vehicle_changed() {
    let t0 = Instant::now();
    let mut v = Vehicle::new(t0);
    v.truncate_param1 = true;
    let mut m = mission();
    let loiter = m.iter().position(|i| i.command == 19).unwrap();
    m[loiter].params[0] = 12.5;
    let mut link = Link {
        state: 7,
        loss: 0.0,
    };
    assert_eq!(
        upload(&mut v, &m, &mut link, t0),
        TransferStatus::Done,
        "the ACK alone says fine"
    );
    let (_, back) = download(&mut v, &mut link, t0);
    let diff = verify(&m, &back, true);
    assert_eq!(
        diff,
        vec![Mismatch {
            seq: Some(loiter as u16),
            field: "param1",
            draft: "12.5".into(),
            vehicle: "12".into(),
        }]
    );
    // A shorter mission on the vehicle is caught too.
    assert_eq!(verify(&m, &back[..5], true)[0].field, "count");
}

#[test]
fn refusal_and_silence_fail_with_a_reason() {
    let t0 = Instant::now();
    let mut v = Vehicle::new(t0);
    v.refuse = Some(4); // MAV_MISSION_NO_SPACE
    let mut link = Link {
        state: 3,
        loss: 0.0,
    };
    assert_eq!(
        upload(&mut v, &mission(), &mut link, t0),
        TransferStatus::Failed("vehicle refused: NO_SPACE".into())
    );
    let mut dead = Link {
        state: 3,
        loss: 1.0,
    };
    assert_eq!(
        upload(&mut Vehicle::new(t0), &mission(), &mut dead, t0),
        TransferStatus::Failed("vehicle never requested an item".into())
    );
    assert_eq!(
        download(&mut Vehicle::new(t0), &mut dead, t0).0,
        TransferStatus::Failed("no MISSION_COUNT".into())
    );
}
