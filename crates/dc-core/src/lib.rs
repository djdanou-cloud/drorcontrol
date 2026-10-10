//! DrorControl headless core.
//!
//! [`Core`] owns the router hub, the links, the vehicle state ([`Fleet`]), the GCS
//! heartbeat and the `.tlog` recorder, and exposes protocol operations (parameter download,
//! stream-rate requests). The desktop UI, `dc-cli`, the gRPC daemon and CI test pilots all
//! drive the same `Core`.
//!
//! Threading: link I/O and routing run on tokio tasks; vehicle state is published as an
//! immutable snapshot (`arc-swap`), so readers (the UI at frame rate) never block ingest.

use std::collections::BTreeMap;
use std::fmt;
use std::io;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::str::FromStr;
use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;
use link_core::{LinkId, LinkIds};
use link_net::{UdpMode, spawn_tcp_client, spawn_tcp_server, spawn_udp};
use mav_router::{HubHandle, HubOutput, LinkPolicy, RouterConfig, spawn_hub};
use mavlink_codec::enums::{MavAutopilot, MavCmd, MavState, MavType};
use mavlink_codec::messages::{CommandLong, Heartbeat};
use mavlink_codec::{Frame, Header, Message, Version, encode};
use protocols::params::{DownloadConfig, DownloadStatus, Param, ParamDownload};
use tokio::sync::broadcast;

pub use vehicle::{Fleet, StatusText, VehicleState};

/// How to reach a vehicle. Parsed from strings such as `tcp:127.0.0.1:5760`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LinkSpec {
    /// `tcp:HOST:PORT` — connect (and keep reconnecting) to a TCP server, e.g. SITL.
    Tcp(SocketAddr),
    /// `tcpin:ADDR:PORT` — accept TCP clients.
    TcpIn(SocketAddr),
    /// `udp:ADDR:PORT` — listen for UDP (GCS port 14550 style).
    Udp(SocketAddr),
    /// `udpout:HOST:PORT` — send UDP to a fixed address.
    UdpOut(SocketAddr),
    /// `serial:PATH:BAUD` — serial port.
    Serial {
        /// Device path, e.g. `/dev/ttyACM0`, `/dev/cu.usbmodem1101`, `COM3`.
        path: String,
        /// Baud rate.
        baud: u32,
    },
}

impl FromStr for LinkSpec {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, String> {
        let (kind, rest) = s
            .split_once(':')
            .ok_or_else(|| format!("`{s}`: expected KIND:ADDRESS"))?;
        let addr = || {
            rest.parse::<SocketAddr>()
                .map_err(|e| format!("`{rest}`: {e}"))
        };
        Ok(match kind {
            "tcp" => LinkSpec::Tcp(addr()?),
            "tcpin" => LinkSpec::TcpIn(addr()?),
            "udp" => LinkSpec::Udp(addr()?),
            "udpout" => LinkSpec::UdpOut(addr()?),
            "serial" => {
                let (path, baud) = rest.rsplit_once(':').ok_or("serial:PATH:BAUD")?;
                LinkSpec::Serial {
                    path: path.to_string(),
                    baud: baud.parse().map_err(|_| format!("bad baud `{baud}`"))?,
                }
            }
            _ => {
                return Err(format!(
                    "unknown link kind `{kind}` (tcp, tcpin, udp, udpout, serial)"
                ));
            }
        })
    }
}

impl fmt::Display for LinkSpec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LinkSpec::Tcp(a) => write!(f, "tcp:{a}"),
            LinkSpec::TcpIn(a) => write!(f, "tcpin:{a}"),
            LinkSpec::Udp(a) => write!(f, "udp:{a}"),
            LinkSpec::UdpOut(a) => write!(f, "udpout:{a}"),
            LinkSpec::Serial { path, baud } => write!(f, "serial:{path}:{baud}"),
        }
    }
}

/// Core configuration.
#[derive(Debug, Clone)]
pub struct CoreConfig {
    /// Our MAVLink system id.
    pub sysid: u8,
    /// Our component id.
    pub compid: u8,
    /// Write every frame to a `.tlog` file at this path.
    pub tlog: Option<PathBuf>,
    /// Telemetry requested from each new vehicle: `(message id, rate Hz)`.
    pub stream_rates: Vec<(u32, f32)>,
}

impl Default for CoreConfig {
    fn default() -> Self {
        CoreConfig {
            sysid: 255,
            compid: 190,
            tlog: None,
            // ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD, SYS_STATUS, GPS_RAW_INT
            stream_rates: vec![(30, 10.0), (33, 5.0), (74, 5.0), (1, 2.0), (24, 2.0)],
        }
    }
}

/// Things worth telling the user about.
#[derive(Debug, Clone, PartialEq)]
pub enum CoreEvent {
    /// First heartbeat from a vehicle.
    VehicleSeen {
        /// System id.
        sysid: u8,
    },
    /// `STATUSTEXT`.
    StatusText {
        /// System id.
        sysid: u8,
        /// Text and severity.
        text: StatusText,
    },
    /// A link went up/down.
    Link {
        /// Link id.
        link: LinkId,
        /// Up?
        up: bool,
        /// Detail.
        detail: String,
    },
}

/// A decoded frame as seen by protocol drivers.
#[derive(Debug, Clone)]
pub struct Incoming {
    /// Sender system.
    pub sysid: u8,
    /// Sender component.
    pub compid: u8,
    /// Decoded message.
    pub msg: Message,
}

/// Parameter download failure.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamError {
    /// Indices never received.
    pub missing: Vec<u16>,
    /// Parameters that did arrive.
    pub partial: usize,
}

impl fmt::Display for ParamError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.partial == 0 {
            write!(
                f,
                "no PARAM_VALUE received (wrong target, link or SYSID_MYGCS?)"
            )
        } else {
            write!(
                f,
                "parameter download incomplete: {} received, {} missing",
                self.partial,
                self.missing.len()
            )
        }
    }
}

impl std::error::Error for ParamError {}

/// The headless core. Cheap to clone; all clones share state.
#[derive(Clone)]
pub struct Core {
    cfg: Arc<CoreConfig>,
    hub: HubHandle,
    ids: LinkIds,
    seq: Arc<AtomicU8>,
    fleet: Arc<ArcSwap<Fleet>>,
    incoming: broadcast::Sender<Arc<Incoming>>,
    events: broadcast::Sender<CoreEvent>,
}

impl Core {
    /// Start the core on the current tokio runtime.
    pub async fn start(cfg: CoreConfig) -> io::Result<Core> {
        let router = RouterConfig {
            sysid: cfg.sysid,
            compid: cfg.compid,
            ..Default::default()
        };
        let (hub, outputs) = spawn_hub(router, LinkPolicy::relay_all());
        let (incoming, _) = broadcast::channel(4096);
        let (events, _) = broadcast::channel(1024);
        let core = Core {
            cfg: Arc::new(cfg),
            hub,
            ids: LinkIds::new(),
            seq: Arc::default(),
            fleet: Arc::new(ArcSwap::from_pointee(Fleet::new())),
            incoming,
            events,
        };

        // Recorder: lossless, on a blocking thread so disk latency never touches routing.
        let mut tap = outputs.tap;
        if let Some(path) = core.cfg.tlog.clone() {
            let file = std::fs::File::create(&path)?;
            tokio::task::spawn_blocking(move || {
                let mut w = recorder::TlogWriter::new(file);
                let mut last_flush = Instant::now();
                while let Some(t) = tap.blocking_recv() {
                    if w.write(t.time, &t.frame).is_err() {
                        break;
                    }
                    if last_flush.elapsed() >= Duration::from_secs(1) {
                        let _ = w.flush();
                        last_flush = Instant::now();
                    }
                }
                let _ = w.flush();
            });
        } else {
            tokio::spawn(async move { while tap.recv().await.is_some() {} });
        }

        // Dispatcher: decode, update the fleet snapshot, publish to protocol drivers.
        let c = core.clone();
        let mut app = outputs.app;
        tokio::spawn(async move {
            while let Some(out) = app.recv().await {
                match out {
                    HubOutput::Frame { frame, .. } => c.dispatch(&frame).await,
                    HubOutput::LinkStatus { link, up, detail } => {
                        let _ = c.events.send(CoreEvent::Link { link, up, detail });
                    }
                    HubOutput::LinkAdded { .. } | HubOutput::LinkRemoved(_) => {}
                }
            }
        });

        // GCS heartbeat, 1 Hz: tells vehicles a GCS is present (GCS failsafe).
        let c = core.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(1));
            loop {
                tick.tick().await;
                let hb = Message::Heartbeat(Heartbeat {
                    r#type: MavType::GCS.0,
                    autopilot: MavAutopilot::INVALID.0,
                    base_mode: 0,
                    custom_mode: 0,
                    system_status: MavState::ACTIVE.0,
                    mavlink_version: 3,
                });
                if !c.send(&hb).await {
                    break;
                }
            }
        });
        Ok(core)
    }

    async fn dispatch(&self, frame: &Frame) {
        let Ok(msg) = frame.decode() else { return };
        let (sysid, compid) = (frame.sysid(), frame.compid());
        let now = Instant::now();
        let first_heartbeat = matches!(msg, Message::Heartbeat(ref h) if h.r#type != MavType::GCS.0)
            && self
                .fleet
                .load()
                .get(sysid)
                .is_none_or(|v| v.heartbeat.is_none());
        let mut text = None;
        self.fleet.rcu(|f| {
            let mut f = Fleet::clone(f);
            text = f.apply(sysid, compid, &msg, now);
            f
        });
        if first_heartbeat && compid == vehicle::AUTOPILOT_COMPID {
            let _ = self.events.send(CoreEvent::VehicleSeen { sysid });
            self.request_streams(sysid, compid).await;
        }
        if let Some(text) = text {
            let _ = self.events.send(CoreEvent::StatusText { sysid, text });
        }
        let _ = self
            .incoming
            .send(Arc::new(Incoming { sysid, compid, msg }));
    }

    /// Configuration in use.
    pub fn config(&self) -> &CoreConfig {
        &self.cfg
    }

    /// Open a link.
    pub async fn connect(&self, spec: &LinkSpec) -> io::Result<LinkId> {
        let ev = self.hub.events.clone();
        Ok(match spec {
            LinkSpec::Tcp(a) => spawn_tcp_client(self.ids.next(), *a, ev).await.id,
            LinkSpec::TcpIn(a) => {
                spawn_tcp_server(self.ids.clone(), *a, ev).await?;
                LinkId(0)
            }
            LinkSpec::Udp(a) => {
                spawn_udp(self.ids.next(), UdpMode::Server { bind: *a }, ev)
                    .await?
                    .0
                    .id
            }
            LinkSpec::UdpOut(a) => {
                spawn_udp(self.ids.next(), UdpMode::Client { remote: *a }, ev)
                    .await?
                    .0
                    .id
            }
            LinkSpec::Serial { path, baud } => {
                link_serial::spawn_serial(self.ids.next(), path, *baud, ev)
                    .await?
                    .id
            }
        })
    }

    /// Send a message from us (MAVLink 2, our ids, next sequence number).
    pub async fn send(&self, msg: &Message) -> bool {
        let seq = self.seq.fetch_add(1, Ordering::Relaxed);
        let header = Header {
            seq,
            sysid: self.cfg.sysid,
            compid: self.cfg.compid,
        };
        match encode(Version::V2, header, msg) {
            Ok(frame) => self.hub.send(frame).await,
            Err(_) => false,
        }
    }

    /// Latest vehicle state (wait-free snapshot).
    pub fn fleet(&self) -> Arc<Fleet> {
        self.fleet.load_full()
    }

    /// Decoded messages as they arrive (for protocol drivers and tools).
    pub fn subscribe(&self) -> broadcast::Receiver<Arc<Incoming>> {
        self.incoming.subscribe()
    }

    /// User-facing events.
    pub fn events(&self) -> broadcast::Receiver<CoreEvent> {
        self.events.subscribe()
    }

    /// Wait for the first vehicle (autopilot heartbeat); returns its system id.
    pub async fn wait_for_vehicle(&self, timeout: Duration) -> Option<u8> {
        let mut rx = self.events.subscribe();
        let pick = |f: &Fleet| {
            f.vehicles()
                .find(|v| v.heartbeat.is_some())
                .map(|v| v.sysid)
        };
        if let Some(id) = pick(&self.fleet()) {
            return Some(id);
        }
        tokio::time::timeout(timeout, async {
            loop {
                match rx.recv().await {
                    Ok(CoreEvent::VehicleSeen { sysid }) => return Some(sysid),
                    Ok(_) | Err(broadcast::error::RecvError::Lagged(_)) => {
                        if let Some(id) = pick(&self.fleet()) {
                            return Some(id);
                        }
                    }
                    Err(broadcast::error::RecvError::Closed) => return None,
                }
            }
        })
        .await
        .ok()
        .flatten()
    }

    /// Ask the vehicle for the configured telemetry rates (`MAV_CMD_SET_MESSAGE_INTERVAL`).
    pub async fn request_streams(&self, sysid: u8, compid: u8) {
        for &(msgid, hz) in &self.cfg.stream_rates {
            let cmd = Message::CommandLong(CommandLong {
                target_system: sysid,
                target_component: compid,
                command: MavCmd::SET_MESSAGE_INTERVAL.0,
                confirmation: 0,
                param1: msgid as f32,
                param2: 1e6 / hz,
                ..Default::default()
            });
            self.send(&cmd).await;
        }
    }

    /// Download every parameter from `target`. `progress(received, total)` is called as
    /// values arrive.
    pub async fn download_params(
        &self,
        target: (u8, u8),
        mut progress: impl FnMut(usize, usize),
    ) -> Result<BTreeMap<String, Param>, ParamError> {
        let mut rx = self.incoming.subscribe();
        let (mut dl, req) = ParamDownload::start(target, DownloadConfig::default(), Instant::now());
        self.send(&req).await;
        let mut tick = tokio::time::interval(Duration::from_millis(50));
        let mut last = (usize::MAX, 0);
        loop {
            tokio::select! {
                m = rx.recv() => match m {
                    Ok(inc) if inc.sysid == target.0 && inc.compid == target.1 => dl.on_message(&inc.msg, Instant::now()),
                    Ok(_) | Err(broadcast::error::RecvError::Lagged(_)) => {}
                    Err(broadcast::error::RecvError::Closed) => break,
                },
                _ = tick.tick() => {
                    for m in dl.poll(Instant::now()) {
                        self.send(&m).await;
                    }
                }
            }
            match dl.status() {
                DownloadStatus::Complete => return Ok(dl.params()),
                DownloadStatus::Failed => break,
                DownloadStatus::Running { received, total } => {
                    if (received, total) != last {
                        progress(received, total);
                        last = (received, total);
                    }
                }
            }
        }
        Err(ParamError {
            missing: dl.missing(),
            partial: dl.params().len(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_link_specs() {
        assert_eq!(
            "tcp:127.0.0.1:5760".parse(),
            Ok(LinkSpec::Tcp("127.0.0.1:5760".parse().unwrap()))
        );
        assert_eq!(
            "udp:0.0.0.0:14550".parse(),
            Ok(LinkSpec::Udp("0.0.0.0:14550".parse().unwrap()))
        );
        assert_eq!(
            "serial:/dev/ttyACM0:115200".parse(),
            Ok(LinkSpec::Serial {
                path: "/dev/ttyACM0".into(),
                baud: 115_200
            })
        );
        assert_eq!(
            "serial:COM3:57600".parse::<LinkSpec>().unwrap().to_string(),
            "serial:COM3:57600"
        );
        assert!("bogus:1".parse::<LinkSpec>().is_err());
        assert!("tcp:nohost".parse::<LinkSpec>().is_err());
    }
}
