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
    fn pick_component(compid: u32) -> Result<u8, Status> {
        if compid == 0 {
            return Ok(1);
        }
        u8::try_from(compid).map_err(|_| Status::invalid_argument("compid must be 0..=255"))
    }

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
        traffic: v
            .traffic
            .values()
            .map(|t| pb::TrafficState {
                age_ms: ms(t.age(now)),
                icao: t.value.icao,
                lat_deg: t.value.lat_deg,
                lon_deg: t.value.lon_deg,
                alt_msl_m: t.value.alt_msl_m.map(|(a, _)| a),
                alt_geometric: t.value.alt_msl_m.is_some_and(|(_, g)| g),
                heading_deg: t.value.heading_deg,
                speed_ms: t.value.speed_ms,
                climb_ms: t.value.climb_ms,
                callsign: t.value.callsign.clone(),
                squawk: t.value.squawk.map(u32::from),
                emitter_type: u32::from(t.value.emitter_type),
                tslc_s: u32::from(t.value.tslc_s),
                simulated: t.value.simulated,
            })
            .collect(),
        batteries: v
            .batteries
            .values()
            .map(|b| pb::BatteryState {
                age_ms: ms(b.age(now)),
                id: u32::from(b.value.id),
                function: u32::from(b.value.function),
                chemistry: u32::from(b.value.chemistry),
                temperature_c: b.value.temperature_c,
                voltage_v: b.value.voltage_v,
                cells_v: b.value.cells_v.clone(),
                current_a: b.value.current_a,
                consumed_mah: b.value.consumed_mah,
                consumed_j: b.value.consumed_j,
                remaining_pct: b.value.remaining_pct.map(u32::from),
                time_remaining_s: b.value.time_remaining_s,
                charge_state: u32::from(b.value.charge_state),
                fault_bitmask: b.value.fault_bitmask,
            })
            .collect(),
        vibration: v.vibration.as_ref().map(|t| pb::VibrationState {
            age_ms: ms(t.age(now)),
            levels_ms2: t.value.levels.to_vec(),
            clipping: t.value.clipping.to_vec(),
        }),
        wind: v.wind.as_ref().map(|t| pb::WindState {
            age_ms: ms(t.age(now)),
            from_deg: t.value.from_deg,
            speed_ms: t.value.speed_ms,
            speed_z_ms: t.value.speed_z_ms,
        }),
        rc: v.rc.as_ref().map(|t| pb::RcState {
            age_ms: ms(t.age(now)),
            channels_us: t.value.channels.iter().map(|&c| u32::from(c)).collect(),
            count: u32::from(t.value.count),
            rssi: t.value.rssi.map(u32::from),
        }),
        servos: v
            .servos
            .iter()
            .map(|(port, t)| pb::ServoState {
                age_ms: ms(t.age(now)),
                port: u32::from(*port),
                outputs_us: t.value.outputs.iter().map(|&o| u32::from(o)).collect(),
            })
            .collect(),
        radio: v.radio.as_ref().map(|t| {
            let r = t.value;
            pb::RadioState {
                age_ms: ms(t.age(now)),
                compid: u32::from(r.compid),
                rssi: r.rssi.map(u32::from),
                remote_rssi: r.remote_rssi.map(u32::from),
                noise: r.noise.map(u32::from),
                remote_noise: r.remote_noise.map(u32::from),
                txbuf_pct: u32::from(r.txbuf_pct),
                rx_errors: u32::from(r.rx_errors),
                corrected: u32::from(r.corrected),
            }
        }),
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
        let compid = Self::pick_component(r.compid)?;
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

    async fn download_items(
        &self,
        req: Request<pb::DownloadItemsRequest>,
    ) -> Result<Response<pb::ItemList>, Status> {
        let r = req.into_inner();
        let sysid = self.pick_vehicle(r.sysid)?;
        let compid = Self::pick_component(r.compid)?;
        let kind = pb::ItemKind::try_from(r.kind)
            .map_err(|_| Status::invalid_argument(format!("unknown kind {}", r.kind)))?;
        let mt = match kind {
            pb::ItemKind::Mission => dc_core::MissionType::Mission,
            pb::ItemKind::Fence => dc_core::MissionType::Fence,
            pb::ItemKind::Rally => dc_core::MissionType::Rally,
        };
        let (status, items) = self.core.download_items((sysid, compid), mt).await;
        if status != dc_core::TransferStatus::Done {
            return Err(Status::deadline_exceeded(format!(
                "{mt:?} download did not complete: {status:?}"
            )));
        }
        Ok(Response::new(pb::ItemList {
            sysid: u32::from(sysid),
            compid: u32::from(compid),
            kind: kind as i32,
            items: items
                .iter()
                .map(|i| pb::Item {
                    frame: u32::from(i.frame),
                    command: u32::from(i.command),
                    params: i.params.to_vec(),
                    x: i.x,
                    y: i.y,
                    z: i.z,
                    autocontinue: i.autocontinue,
                })
                .collect(),
        }))
    }

    async fn get_messages(
        &self,
        req: Request<pb::GetMessagesRequest>,
    ) -> Result<Response<pb::MessageList>, Status> {
        let sysid = self.pick_vehicle(req.into_inner().sysid)?;
        let messages = self
            .core
            .messages(sysid)
            .into_iter()
            .map(|m| pb::LoggedMessage {
                unix_ms: m
                    .time
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_or(0, |d| d.as_millis() as u64),
                compid: u32::from(m.compid),
                severity: u32::from(m.text.severity),
                text: m.text.text,
            })
            .collect();
        Ok(Response::new(pb::MessageList {
            sysid: u32::from(sysid),
            messages,
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
        let mut callsign = [0u8; 9];
        callsign[..4].copy_from_slice(b"TEST");
        f.apply(
            1,
            156,
            &Message::AdsbVehicle(mavlink_codec::messages::AdsbVehicle {
                ICAO_address: 42,
                lat: -353_600_000,
                lon: 1_491_700_000,
                callsign,
                flags: 1 | 16 | 64,
                ..Default::default()
            }),
            now,
        );
        let mut volts = [u16::MAX; 10];
        volts[0] = 12_600;
        f.apply(
            1,
            1,
            &Message::BatteryStatus(mavlink_codec::messages::BatteryStatus {
                id: 0,
                voltages: volts,
                temperature: i16::MAX,
                current_battery: -1,
                current_consumed: -1,
                energy_consumed: -1,
                battery_remaining: 80,
                ..Default::default()
            }),
            now,
        );
        let b = &vehicle_to_pb(f.get(1).unwrap(), now).batteries[0];
        assert_eq!(
            (b.voltage_v, b.cells_v.len(), b.remaining_pct, b.current_a),
            (Some(12.6), 0, Some(80), None)
        );
        let traffic = vehicle_to_pb(f.get(1).unwrap(), now).traffic;
        assert_eq!(traffic.len(), 1);
        assert_eq!(
            (
                traffic[0].icao,
                traffic[0].callsign.as_deref(),
                traffic[0].alt_msl_m,
                traffic[0].simulated
            ),
            (42, Some("TEST"), None, true)
        );
        assert_eq!((home.lat_deg, home.alt_msl_m), (-35.363_261, 584.09));
    }
}
