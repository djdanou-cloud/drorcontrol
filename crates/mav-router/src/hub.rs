//! The async hub: owns the [`Router`], receives [`LinkEvent`]s from every link, forwards
//! frames, delivers local traffic to the application and taps everything for recording.

use std::collections::BTreeMap;
use std::time::{Instant, SystemTime};

use link_core::{LinkEvent, LinkHandle, LinkId};
use mavlink_codec::Frame;
use tokio::sync::{mpsc, oneshot};

use crate::router::{LinkPolicy, Router, RouterConfig, RouterStats};

/// Queue depth towards the application. If the application falls this far behind,
/// frames are dropped and counted rather than stalling routing.
pub const APP_QUEUE: usize = 4096;

/// What the hub tells the application.
// Frames are kept inline (~290 bytes) on purpose: boxing would cost a heap
// allocation per frame on the hot path.
#[allow(clippy::large_enum_variant)]
#[derive(Debug)]
pub enum HubOutput {
    /// A frame for us (broadcast or addressed to our system).
    Frame {
        /// Arrival link.
        link: LinkId,
        /// The frame.
        frame: Frame,
        /// Host time of receipt.
        rx_time: SystemTime,
    },
    /// A link was added.
    LinkAdded {
        /// Link id.
        link: LinkId,
        /// Description.
        name: String,
    },
    /// A link was removed.
    LinkRemoved(LinkId),
    /// A link's transport went up or down.
    LinkStatus {
        /// Link id.
        link: LinkId,
        /// Usable?
        up: bool,
        /// Detail text.
        detail: String,
    },
}

/// A copy of every frame crossing the hub, in order, for the `.tlog` recorder.
/// Unbounded on purpose: recording must be lossless (the recorder reports backlog).
#[derive(Debug, Clone)]
pub struct Tapped {
    /// Host time (receipt, or send time for local frames).
    pub time: SystemTime,
    /// The frame.
    pub frame: Frame,
}

/// Point-in-time view of the hub, for diagnostics and tests.
#[derive(Debug, Clone)]
pub struct HubSnapshot {
    /// Links and their names.
    pub links: Vec<(LinkId, String)>,
    /// Fresh routes.
    pub routes: Vec<((u8, u8), LinkId)>,
    /// Router counters.
    pub stats: RouterStats,
    /// Frames dropped because the application queue was full.
    pub app_dropped: u64,
}

enum Control {
    SetPolicy(LinkId, LinkPolicy),
    Snapshot(oneshot::Sender<HubSnapshot>),
}

/// Handle used by links (`events`) and by the application (`send`, `set_policy`, `snapshot`).
#[derive(Clone)]
pub struct HubHandle {
    /// Give this to every link.
    pub events: mpsc::Sender<LinkEvent>,
    local: mpsc::Sender<Frame>,
    control: mpsc::Sender<Control>,
}

impl HubHandle {
    /// Send a frame we originate (routed by target like any other frame).
    pub async fn send(&self, frame: Frame) -> bool {
        self.local.send(frame).await.is_ok()
    }

    /// Change a link's policy (links start with the hub's default policy).
    pub async fn set_policy(&self, link: LinkId, policy: LinkPolicy) {
        let _ = self.control.send(Control::SetPolicy(link, policy)).await;
    }

    /// Current links, routes and counters.
    pub async fn snapshot(&self) -> Option<HubSnapshot> {
        let (tx, rx) = oneshot::channel();
        self.control.send(Control::Snapshot(tx)).await.ok()?;
        rx.await.ok()
    }
}

/// Receivers the application keeps.
pub struct HubOutputs {
    /// Local deliveries and link notifications.
    pub app: mpsc::Receiver<HubOutput>,
    /// Every frame, for recording.
    pub tap: mpsc::UnboundedReceiver<Tapped>,
}

/// Start the hub on the current tokio runtime. `default_policy` applies to new links.
pub fn spawn_hub(cfg: RouterConfig, default_policy: LinkPolicy) -> (HubHandle, HubOutputs) {
    let (ev_tx, mut ev_rx) = mpsc::channel::<LinkEvent>(1024);
    let (local_tx, mut local_rx) = mpsc::channel::<Frame>(1024);
    let (ctl_tx, mut ctl_rx) = mpsc::channel::<Control>(16);
    let (app_tx, app_rx) = mpsc::channel(APP_QUEUE);
    let (tap_tx, tap_rx) = mpsc::unbounded_channel();

    tokio::spawn(async move {
        let mut router = Router::new(cfg);
        let mut links: BTreeMap<LinkId, LinkHandle> = BTreeMap::new();
        let mut app_dropped = 0u64;
        let forward = |links: &BTreeMap<LinkId, LinkHandle>, to: &[LinkId], frame: &Frame| {
            for l in to {
                if let Some(h) = links.get(l) {
                    h.send(frame.clone());
                }
            }
        };
        loop {
            tokio::select! {
                ev = ev_rx.recv() => {
                    let Some(ev) = ev else { break };
                    match ev {
                        LinkEvent::Added(h) => {
                            router.add_link(h.id, default_policy.clone());
                            let note = HubOutput::LinkAdded { link: h.id, name: h.name.clone() };
                            links.insert(h.id, h);
                            let _ = app_tx.try_send(note);
                        }
                        LinkEvent::Removed(id) => {
                            router.remove_link(id);
                            links.remove(&id);
                            let _ = app_tx.try_send(HubOutput::LinkRemoved(id));
                        }
                        LinkEvent::Status { link, up, detail } => {
                            let _ = app_tx.try_send(HubOutput::LinkStatus { link, up, detail });
                        }
                        LinkEvent::Frame { link, frame, rx_time } => {
                            let _ = tap_tx.send(Tapped { time: rx_time, frame: frame.clone() });
                            let d = router.decide(Some(link), &frame, Instant::now());
                            forward(&links, &d.forward, &frame);
                            if d.deliver_local && app_tx.try_send(HubOutput::Frame { link, frame, rx_time }).is_err() {
                                app_dropped += 1;
                            }
                        }
                    }
                }
                f = local_rx.recv() => {
                    let Some(frame) = f else { break };
                    let _ = tap_tx.send(Tapped { time: SystemTime::now(), frame: frame.clone() });
                    let d = router.decide(None, &frame, Instant::now());
                    forward(&links, &d.forward, &frame);
                }
                c = ctl_rx.recv() => match c {
                    Some(Control::SetPolicy(link, p)) => router.add_link(link, p),
                    Some(Control::Snapshot(reply)) => {
                        let _ = reply.send(HubSnapshot {
                            links: links.iter().map(|(id, h)| (*id, h.name.clone())).collect(),
                            routes: router.routes(Instant::now()),
                            stats: router.stats().clone(),
                            app_dropped,
                        });
                    }
                    None => {}
                },
            }
        }
    });

    (
        HubHandle {
            events: ev_tx,
            local: local_tx,
            control: ctl_tx,
        },
        HubOutputs {
            app: app_rx,
            tap: tap_rx,
        },
    )
}
