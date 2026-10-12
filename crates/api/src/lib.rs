//! DrorControl gRPC API v1 (`proto/dc/v1/core.proto`).
//!
//! * [`pb`]: generated message types, client and server stubs.
//! * [`CoreService`]: the server implementation over [`dc_core::Core`].
//! * [`serve`]: run the server on an address.
//!
//! The API is read-only in v1 (no parameter writes or commands) until the command-safety
//! layer exists, so remote clients cannot bypass interlocks (plan 08 §4).

use std::net::SocketAddr;
use std::pin::Pin;
use std::time::{Duration, Instant};

use dc_core::{AlertLevel, Core, CoreEvent, PositionTrust, VehicleLink, VehicleState};
use tokio_stream::Stream;
use tonic::{Request, Response, Status};

/// Generated protobuf types and gRPC stubs.
#[allow(missing_docs, clippy::all)]
pub mod pb {
    tonic::include_proto!("dc.v1");
}

use pb::core_server::{Core as CoreApi, CoreServer};

/// gRPC service backed by a running core.
#[derive(Clone)]
pub struct CoreService {
    core: Core,
}

impl CoreService {
    /// Wrap a core.
    pub fn new(core: Core) -> Self {
        CoreService { core }
    }

    /// Resolve "0 = first vehicle heard".
    fn pick_vehicle(&self, sysid: u32) -> Result<u8, Status> {
        if sysid != 0 {
            return u8::try_from(sysid)
                .map_err(|_| Status::invalid_argument("sysid must be 1..=255"));
        }
        self.core
            .fleet()
            .vehicles()
            .find(|v| v.heartbeat.is_some())
            .map(|v| v.sysid)
            .ok_or_else(|| Status::unavailable("no vehicle heard yet"))
    }
}

fn ms(d: Duration) -> u32 {
    u32::try_from(d.as_millis()).unwrap_or(u32::MAX)
}

fn mission_state_to_pb(s: dc_core::MissionState) -> pb::MissionState {
    use dc_core::MissionState as M;
    match s {
        M::Unknown => pb::MissionState::Unknown,
        M::NoMission => pb::MissionState::NoMission,
        M::NotStarted => pb::MissionState::NotStarted,
        M::Active => pb::MissionState::Active,
        M::Paused => pb::MissionState::Paused,
        M::Complete => pb::MissionState::Complete,
    }
}

fn link_to_pb(l: VehicleLink) -> pb::vehicle_link_event::State {
    use pb::vehicle_link_event::State;
    match l {
        VehicleLink::Connected => State::Connected,
        VehicleLink::Stale => State::Stale,
        VehicleLink::Lost => State::Lost,
    }
}

/// [`vehicle_to_pb`] plus what the core derives: navigation health and link state.
pub fn vehicle_with_health(core: &Core, v: &VehicleState, now: Instant) -> pb::Vehicle {
    let mut pb = vehicle_to_pb(v, now);
    let nav = core.nav_health(v.sysid);
    pb.nav = Some(pb::NavHealth {
        trust: match nav.trust {
            PositionTrust::Unknown => pb::PositionTrust::Unknown,
            PositionTrust::Good => pb::PositionTrust::Good,
            PositionTrust::Degraded => pb::PositionTrust::Degraded,
            PositionTrust::Untrusted => pb::PositionTrust::Untrusted,
        } as i32,
        reasons: nav.reasons.iter().map(|r| r.to_string()).collect(),
    });
    if let Some(l) = core.vehicle_link(v.sysid) {
        pb.link = link_to_pb(l) as i32;
    }
    pb
}

/// Convert core state to the wire type (vehicle state only: `nav` and `link` are left unset;
/// see [`vehicle_with_health`]).
pub fn vehicle_to_pb(v: &VehicleState, now: Instant) -> pb::Vehicle {
    pb::Vehicle {
        sysid: u32::from(v.sysid),
        heartbeat: v.heartbeat.as_ref().map(|h| pb::HeartbeatState {
            age_ms: ms(h.age(now)),
            mav_type: u32::from(h.value.mav_type),
            autopilot: u32::from(h.value.autopilot),
            custom_mode: h.value.custom_mode,
            armed: h.value.armed(),
            system_status: u32::from(h.value.system_status),
            mode_name: v.mode_name().map(str::to_string),
        }),
        attitude: v.attitude.as_ref().map(|a| pb::AttitudeState {
            age_ms: ms(a.age(now)),
            roll_rad: a.value.roll,
            pitch_rad: a.value.pitch,
            yaw_rad: a.value.yaw,
        }),
        position: v.position.as_ref().map(|p| pb::PositionState {
            age_ms: ms(p.age(now)),
            lat_deg: p.value.lat_deg,
            lon_deg: p.value.lon_deg,
            alt_msl_m: p.value.alt_msl_m,
            alt_rel_m: p.value.alt_rel_m,
            heading_deg: p.value.heading_deg,
        }),
        power: v.power.as_ref().map(|p| pb::PowerState {
            age_ms: ms(p.age(now)),
            voltage_v: p.value.voltage,
            current_a: p.value.current,
            remaining_pct: p.value.remaining_pct.map(u32::from),
        }),
        gps: v.gps.as_ref().map(|g| pb::GpsState {
            age_ms: ms(g.age(now)),
            fix_type: u32::from(g.value.fix_type),
            satellites: g.value.satellites.map(u32::from),
            hdop: g.value.hdop,
        }),
        mission: v.mission.as_ref().map(|m| pb::MissionProgress {
            age_ms: ms(m.age(now)),
            current: u32::from(m.value.current),
            total: m.value.total.map(u32::from),
            state: mission_state_to_pb(m.value.state) as i32,
            in_mission_mode: m.value.in_mission_mode,
            last_reached: v.mission_reached.as_ref().map(|r| u32::from(r.value)),
        }),
        landed: v.extended.as_ref().map(|e| pb::LandedState {
            age_ms: ms(e.age(now)),
            landed_state: u32::from(e.value.landed.to_mavlink()),
        }),
        nav: None,
        link: 0,
        home: v.home.as_ref().map(|h| pb::HomeState {
            age_ms: ms(h.age(now)),
            lat_deg: h.value.lat_deg,
            lon_deg: h.value.lon_deg,
            alt_msl_m: h.value.alt_msl_m,
        }),
    }
}

type BoxStream<T> = Pin<Box<dyn Stream<Item = Result<T, Status>> + Send + 'static>>;

#[tonic::async_trait]
impl CoreApi for CoreService {
    async fn get_info(&self, _: Request<pb::GetInfoRequest>) -> Result<Response<pb::Info>, Status> {
        let cfg = self.core.config();
        Ok(Response::new(pb::Info {
            version: env!("CARGO_PKG_VERSION").to_string(),
            sysid: u32::from(cfg.sysid),
            compid: u32::from(cfg.compid),
            dialect: mavlink_codec::dialect::DIALECT_ROOT.to_string(),
        }))
    }

    async fn open_link(
        &self,
        req: Request<pb::OpenLinkRequest>,
    ) -> Result<Response<pb::OpenLinkReply>, Status> {
        let spec = req
            .into_inner()
            .spec
            .parse()
            .map_err(Status::invalid_argument)?;
        let id = self
            .core
            .connect(&spec)
            .await
            .map_err(|e| Status::unavailable(e.to_string()))?;
        Ok(Response::new(pb::OpenLinkReply { link_id: id.0 }))
    }

    async fn list_vehicles(
        &self,
        _: Request<pb::ListVehiclesRequest>,
    ) -> Result<Response<pb::VehicleList>, Status> {
        let now = Instant::now();
        let vehicles = self
            .core
            .fleet()
            .vehicles()
            .map(|v| vehicle_with_health(&self.core, v, now))
            .collect();
        Ok(Response::new(pb::VehicleList { vehicles }))
    }

    async fn get_params(
        &self,
        req: Request<pb::GetParamsRequest>,
    ) -> Result<Response<pb::ParamList>, Status> {
        let r = req.into_inner();
        let sysid = self.pick_vehicle(r.sysid)?;
        let compid = if r.compid == 0 {
            1
        } else {
            u8::try_from(r.compid).map_err(|_| Status::invalid_argument("compid"))?
        };
        let params = self
            .core
            .download_params((sysid, compid), |_, _| {})
            .await
            .map_err(|e| Status::deadline_exceeded(e.to_string()))?;
        Ok(Response::new(pb::ParamList {
            sysid: u32::from(sysid),
            compid: u32::from(compid),
            params: params
                .into_iter()
                .map(|(name, p)| pb::Param {
                    name,
                    value: p.value,
                    mav_param_type: u32::from(p.ty),
                    index: u32::from(p.index),
                })
                .collect(),
        }))
    }

    type StreamTelemetryStream = BoxStream<pb::Vehicle>;

    async fn stream_telemetry(
        &self,
        req: Request<pb::StreamTelemetryRequest>,
    ) -> Result<Response<Self::StreamTelemetryStream>, Status> {
        let r = req.into_inner();
        let sysid = self.pick_vehicle(r.sysid)?;
        let hz = if r.rate_hz <= 0.0 {
            1.0
        } else {
            r.rate_hz.min(50.0)
        };
        let core = self.core.clone();
        let (tx, rx) = tokio::sync::mpsc::channel(4);
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs_f32(1.0 / hz));
            tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            loop {
                tick.tick().await;
                let fleet = core.fleet();
                let Some(v) = fleet.get(sysid) else { continue };
                if tx
                    .send(Ok(vehicle_with_health(&core, v, Instant::now())))
                    .await
                    .is_err()
                {
                    break; // client went away
                }
            }
        });
        Ok(Response::new(Box::pin(
            tokio_stream::wrappers::ReceiverStream::new(rx),
        )))
    }

    type StreamEventsStream = BoxStream<pb::Event>;

    async fn stream_events(
        &self,
        _: Request<pb::StreamEventsRequest>,
    ) -> Result<Response<Self::StreamEventsStream>, Status> {
        let mut events = self.core.events();
        let (tx, rx) = tokio::sync::mpsc::channel(64);
        tokio::spawn(async move {
            use pb::event::Kind;
            loop {
                let kind = match events.recv().await {
                    Ok(CoreEvent::StatusText { sysid, text }) => {
                        Kind::StatusText(pb::StatusTextEvent {
                            sysid: u32::from(sysid),
                            severity: u32::from(text.severity),
                            text: text.text,
                        })
                    }
                    Ok(CoreEvent::Link { link, up, detail }) => Kind::Link(pb::LinkEvent {
                        link_id: link.0,
                        up,
                        detail,
                    }),
                    Ok(CoreEvent::VehicleSeen { sysid }) => Kind::VehicleSeen(u32::from(sysid)),
                    Ok(CoreEvent::VehicleLink {
                        sysid,
                        state,
                        heartbeat_age,
                    }) => Kind::VehicleLink(pb::VehicleLinkEvent {
                        sysid: u32::from(sysid),
                        state: link_to_pb(state) as i32,
                        heartbeat_age_ms: heartbeat_age.as_millis().min(u32::MAX as u128) as u32,
                    }),
                    Ok(CoreEvent::Recorder { stopped, detail }) => {
                        Kind::Recorder(pb::RecorderEvent { stopped, detail })
                    }
                    Ok(CoreEvent::Command(st)) => Kind::Command(pb::CommandEvent {
                        id: st.id.0,
                        sysid: u32::from(st.target.sysid),
                        state: format!("{:?}", st.state),
                        r#final: st.state.is_final(),
                        attempts: st.attempts,
                        summary: st.summary(),
                    }),
                    Ok(CoreEvent::Alert { sysid, level, text }) => {
                        use pb::alert_event::Level;
                        Kind::Alert(pb::AlertEvent {
                            sysid: u32::from(sysid),
                            level: match level {
                                AlertLevel::Advisory => Level::Advisory,
                                AlertLevel::Caution => Level::Caution,
                                AlertLevel::Warning => Level::Warning,
                            } as i32,
                            text,
                        })
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
                };
                if tx.send(Ok(pb::Event { kind: Some(kind) })).await.is_err() {
                    break;
                }
            }
        });
        Ok(Response::new(Box::pin(
            tokio_stream::wrappers::ReceiverStream::new(rx),
        )))
    }
}

/// Serve the API on `addr` until the future is dropped.
pub async fn serve(core: Core, addr: SocketAddr) -> Result<(), tonic::transport::Error> {
    tonic::transport::Server::builder()
        .add_service(CoreServer::new(CoreService::new(core)))
        .serve(addr)
        .await
}

/// Serve on an already-bound listener (lets callers bind port 0 and learn the port).
pub async fn serve_on(
    core: Core,
    listener: tokio::net::TcpListener,
) -> Result<(), tonic::transport::Error> {
    let incoming = tokio_stream::wrappers::TcpListenerStream::new(listener);
    tonic::transport::Server::builder()
        .add_service(CoreServer::new(CoreService::new(core)))
        .serve_with_incoming(incoming)
        .await
}

#[cfg(test)]
mod tests {
    use super::*;
    use dc_core::Fleet;
    use mavlink_codec::Message;
    use mavlink_codec::messages::{
        ExtendedSysState, Heartbeat, MissionCurrent, MissionItemReached, Statustext,
    };

    #[test]
    fn vehicle_carries_mode_name_mission_and_landed_state() {
        let now = Instant::now();
        let mut f = Fleet::new();
        let hb = Message::Heartbeat(Heartbeat {
            r#type: 2,
            autopilot: 3,
            custom_mode: 3, // Copter AUTO
            ..Default::default()
        });
        f.apply(1, 1, &hb, now);
        let pb = vehicle_to_pb(f.get(1).unwrap(), now);
        assert_eq!(pb.heartbeat.unwrap().mode_name, None, "no banner, no name");
        assert!(pb.mission.is_none() && pb.landed.is_none());

        let mut text = [0u8; 50];
        text[..23].copy_from_slice(b"ArduCopter V4.7.1 (abc)");
        f.apply(
            1,
            1,
            &Message::Statustext(Statustext {
                severity: 6,
                text,
                ..Default::default()
            }),
            now,
        );
        f.apply(
            1,
            1,
            &Message::MissionCurrent(MissionCurrent {
                seq: 2,
                total: 4,
                mission_state: 3,
                mission_mode: 1,
            }),
            now,
        );
        f.apply(
            1,
            1,
            &Message::MissionItemReached(MissionItemReached { seq: 1 }),
            now,
        );
        f.apply(
            1,
            1,
            &Message::ExtendedSysState(ExtendedSysState {
                vtol_state: 0,
                landed_state: 2,
            }),
            now,
        );
        let pb = vehicle_to_pb(f.get(1).unwrap(), now);
        assert_eq!(pb.heartbeat.unwrap().mode_name.as_deref(), Some("AUTO"));
        let m = pb.mission.unwrap();
        assert_eq!(
            (
                m.current,
                m.total,
                m.state(),
                m.in_mission_mode,
                m.last_reached
            ),
            (2, Some(4), pb::MissionState::Active, Some(true), Some(1))
        );
        assert_eq!(pb.landed.unwrap().landed_state, 2);
        assert!(pb.home.is_none());
        f.apply(
            1,
            1,
            &Message::HomePosition(mavlink_codec::messages::HomePosition {
                latitude: -353_632_610,
                longitude: 1_491_652_300,
                altitude: 584_090,
                ..Default::default()
            }),
            now,
        );
        let home = vehicle_to_pb(f.get(1).unwrap(), now).home.unwrap();
        assert_eq!((home.lat_deg, home.alt_msl_m), (-35.363_261, 584.09));
    }
}
