//! MAVLink mission protocol (sans-I/O): upload, download and read-back verification
//! (hazard H4: a corrupt or partial mission upload).
//!
//! Upload: send `MISSION_COUNT`; answer each `MISSION_REQUEST_INT` / `MISSION_REQUEST` with
//! `MISSION_ITEM_INT`; finish on `MISSION_ACK`. The vehicle drives the transfer (ArduPilot
//! re-requests an item after 1 s); the GCS re-sends `MISSION_COUNT` while nothing has been
//! requested and gives up after its retry budget.
//!
//! Download: `MISSION_REQUEST_LIST` → `MISSION_COUNT`, then `MISSION_REQUEST_INT` for each
//! item in order, re-requested on silence, finished with `MISSION_ACK`.
//!
//! An upload is only trusted after the mission has been downloaded again and [`verify`] finds
//! no difference: a lost final ACK, a vehicle that stores a value differently, or a partial
//! write all show up there. ArduPilot specifics handled by [`verify`]: item 0 is the home
//! position (an uploaded item 0 is ignored, a download returns home), downloads use the
//! non-`_INT` frame numbers, and altitude is stored in centimetres.

use std::time::{Duration, Instant};

use mavlink_codec::Message;
use mavlink_codec::messages::{
    MissionAck, MissionCount, MissionItemInt, MissionRequestInt, MissionRequestList,
};

/// `MAV_MISSION_TYPE_MISSION`.
pub const MISSION_TYPE_MISSION: u8 = 0;

/// Which list a transfer is about (`MAV_MISSION_TYPE`). The same protocol carries all three.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionType {
    /// The mission (on ArduPilot item 0 is home).
    Mission,
    /// Geofence: polygon vertices, circles and the return point (`FENCE_*` items).
    Fence,
    /// Rally points (`MAV_CMD_NAV_RALLY_POINT`).
    Rally,
}

impl MissionType {
    /// The `MAV_MISSION_TYPE` value.
    pub fn mavlink(self) -> u8 {
        match self {
            MissionType::Mission => MISSION_TYPE_MISSION,
            MissionType::Fence => 1,
            MissionType::Rally => 2,
        }
    }
}
/// `MAV_MISSION_ACCEPTED`.
const MISSION_ACCEPTED: u8 = 0;
/// `MAV_MISSION_INVALID_SEQUENCE`: ArduPilot's answer to an out-of-sequence item. The upload
/// continues (`MissionItemProtocol::handle_mission_item` just returns), so it is not final.
const MISSION_INVALID_SEQUENCE: u8 = 13;
/// Silence after the last item before giving up on the final ACK (the read-back decides).
const FINAL_ACK_RETRIES: u32 = 3;

/// One mission item, as `MISSION_ITEM_INT` carries it (without addressing).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MissionItem {
    /// `MAV_FRAME`.
    pub frame: u8,
    /// `MAV_CMD`.
    pub command: u16,
    /// Params 1–4.
    pub params: [f32; 4],
    /// Latitude (degE7) or local x.
    pub x: i32,
    /// Longitude (degE7) or local y.
    pub y: i32,
    /// Altitude (m) or local z.
    pub z: f32,
    /// Continue to the next item automatically.
    pub autocontinue: bool,
}

impl MissionItem {
    /// From a received item.
    pub fn from_message(m: &MissionItemInt) -> Self {
        MissionItem {
            frame: m.frame,
            command: m.command,
            params: [m.param1, m.param2, m.param3, m.param4],
            x: m.x,
            y: m.y,
            z: m.z,
            autocontinue: m.autocontinue != 0,
        }
    }

    /// The item to send as `seq`.
    pub fn to_message(&self, target: (u8, u8), seq: u16, mission_type: u8) -> Message {
        Message::MissionItemInt(MissionItemInt {
            target_system: target.0,
            target_component: target.1,
            seq,
            frame: self.frame,
            command: self.command,
            current: 0,
            autocontinue: u8::from(self.autocontinue),
            param1: self.params[0],
            param2: self.params[1],
            param3: self.params[2],
            param4: self.params[3],
            x: self.x,
            y: self.y,
            z: self.z,
            mission_type,
        })
    }
}

/// Retry policy.
#[derive(Debug, Clone, Copy)]
pub struct TransferConfig {
    /// Silence before re-sending the last message.
    pub retry_after: Duration,
    /// Re-sends of one step before giving up.
    pub max_retries: u32,
}

impl Default for TransferConfig {
    fn default() -> Self {
        // At 30 % loss each way a round trip succeeds about half the time; 16 attempts make
        // a stuck item unlikely (≈ 2e-5) while a dead link still fails within ~16 s
        // (plan 08 §2: transfers must complete at 30 % loss).
        TransferConfig {
            retry_after: Duration::from_secs(1),
            max_retries: 15,
        }
    }
}

/// Transfer state.
#[derive(Debug, Clone, PartialEq)]
pub enum TransferStatus {
    /// Running: items transferred so far and the total.
    InProgress {
        /// Items done.
        done: u16,
        /// Items in total (0 until known for a download).
        total: u16,
    },
    /// Finished and acknowledged.
    Done,
    /// Gave up or the vehicle refused; the text says why.
    Failed(String),
}

fn result_name(r: u8) -> String {
    mavlink_codec::enums::MavMissionResult(r)
        .name()
        .map(|n| n.trim_start_matches("MAV_MISSION_").to_string())
        .unwrap_or_else(|| format!("result {r}"))
}

/// Upload state machine.
#[derive(Debug)]
pub struct MissionUpload {
    target: (u8, u8),
    mission_type: u8,
    items: Vec<MissionItem>,
    cfg: TransferConfig,
    last_activity: Instant,
    retries: u32,
    requested_any: bool,
    highest_sent: Option<u16>,
    status: TransferStatus,
}

impl MissionUpload {
    /// Start: returns the `MISSION_COUNT` to send. For `MISSION_TYPE_MISSION` on ArduPilot,
    /// `items[0]` is the home slot (ignored by the vehicle).
    pub fn start(
        target: (u8, u8),
        items: Vec<MissionItem>,
        mission_type: u8,
        cfg: TransferConfig,
        now: Instant,
    ) -> (Self, Message) {
        let up = MissionUpload {
            target,
            mission_type,
            cfg,
            last_activity: now,
            retries: 0,
            requested_any: false,
            highest_sent: None,
            status: TransferStatus::InProgress {
                done: 0,
                total: items.len() as u16,
            },
            items,
        };
        let count = up.count_message();
        (up, count)
    }

    fn count_message(&self) -> Message {
        Message::MissionCount(MissionCount {
            target_system: self.target.0,
            target_component: self.target.1,
            count: self.items.len() as u16,
            mission_type: self.mission_type,
        })
    }

    /// Feed a message from the target; returns what to send in reply.
    pub fn on_message(&mut self, msg: &Message, now: Instant) -> Vec<Message> {
        if !matches!(self.status, TransferStatus::InProgress { .. }) {
            return Vec::new();
        }
        let requested = match msg {
            Message::MissionRequestInt(r) if r.mission_type == self.mission_type => Some(r.seq),
            Message::MissionRequest(r) if r.mission_type == self.mission_type => Some(r.seq),
            Message::MissionAck(a)
                if a.mission_type == self.mission_type && a.r#type == MISSION_INVALID_SEQUENCE =>
            {
                return Vec::new();
            }
            Message::MissionAck(a) if a.mission_type == self.mission_type => {
                self.status = if a.r#type == MISSION_ACCEPTED {
                    TransferStatus::Done
                } else {
                    TransferStatus::Failed(format!("vehicle refused: {}", result_name(a.r#type)))
                };
                return Vec::new();
            }
            _ => None,
        };
        let Some(seq) = requested else {
            return Vec::new();
        };
        let Some(item) = self.items.get(usize::from(seq)) else {
            self.status = TransferStatus::Failed(format!(
                "vehicle asked for item {seq} of {}",
                self.items.len()
            ));
            return Vec::new();
        };
        self.requested_any = true;
        self.last_activity = now;
        self.retries = 0;
        self.highest_sent = Some(self.highest_sent.map_or(seq, |h| h.max(seq)));
        self.status = TransferStatus::InProgress {
            done: seq,
            total: self.items.len() as u16,
        };
        vec![item.to_message(self.target, seq, self.mission_type)]
    }

    /// Timeouts: re-send `MISSION_COUNT` while nothing was requested; once requests flow the
    /// vehicle drives re-requests, so the GCS only waits (re-sending items it may have moved
    /// past only provokes `INVALID_SEQUENCE`). Gives up after the retry budget, or 3 s after
    /// the last item if the final ACK never comes.
    pub fn poll(&mut self, now: Instant) -> Vec<Message> {
        if !matches!(self.status, TransferStatus::InProgress { .. })
            || now.saturating_duration_since(self.last_activity) < self.cfg.retry_after
        {
            return Vec::new();
        }
        let last_sent = self.highest_sent == Some(self.items.len().saturating_sub(1) as u16);
        if self.retries >= self.cfg.max_retries || (last_sent && self.retries >= FINAL_ACK_RETRIES)
        {
            self.status = TransferStatus::Failed(if self.requested_any {
                "no MISSION_ACK (vehicle stopped requesting)".into()
            } else {
                "vehicle never requested an item".into()
            });
            return Vec::new();
        }
        self.retries += 1;
        self.last_activity = now;
        if self.requested_any {
            Vec::new()
        } else {
            vec![self.count_message()]
        }
    }

    /// Status.
    pub fn status(&self) -> &TransferStatus {
        &self.status
    }
}

/// Download state machine.
#[derive(Debug)]
pub struct MissionDownload {
    target: (u8, u8),
    mission_type: u8,
    cfg: TransferConfig,
    count: Option<u16>,
    items: Vec<MissionItem>,
    last_activity: Instant,
    retries: u32,
    status: TransferStatus,
}

impl MissionDownload {
    /// Start: returns the `MISSION_REQUEST_LIST` to send.
    pub fn start(
        target: (u8, u8),
        mission_type: u8,
        cfg: TransferConfig,
        now: Instant,
    ) -> (Self, Message) {
        let d = MissionDownload {
            target,
            mission_type,
            cfg,
            count: None,
            items: Vec::new(),
            last_activity: now,
            retries: 0,
            status: TransferStatus::InProgress { done: 0, total: 0 },
        };
        let req = d.list_message();
        (d, req)
    }

    fn list_message(&self) -> Message {
        Message::MissionRequestList(MissionRequestList {
            target_system: self.target.0,
            target_component: self.target.1,
            mission_type: self.mission_type,
        })
    }

    fn request(&self, seq: u16) -> Message {
        Message::MissionRequestInt(MissionRequestInt {
            target_system: self.target.0,
            target_component: self.target.1,
            seq,
            mission_type: self.mission_type,
        })
    }

    fn ack(&self) -> Message {
        Message::MissionAck(MissionAck {
            target_system: self.target.0,
            target_component: self.target.1,
            r#type: MISSION_ACCEPTED,
            mission_type: self.mission_type,
        })
    }

    /// The next message to send now that the state advanced.
    fn next(&mut self) -> Vec<Message> {
        let count = self.count.unwrap_or(0);
        let done = self.items.len() as u16;
        if done >= count {
            self.status = TransferStatus::Done;
            return vec![self.ack()];
        }
        self.status = TransferStatus::InProgress { done, total: count };
        vec![self.request(done)]
    }

    /// Feed a message from the target; returns what to send.
    pub fn on_message(&mut self, msg: &Message, now: Instant) -> Vec<Message> {
        if !matches!(self.status, TransferStatus::InProgress { .. }) {
            return Vec::new();
        }
        match msg {
            Message::MissionCount(c)
                if c.mission_type == self.mission_type && self.count.is_none() =>
            {
                self.count = Some(c.count);
            }
            Message::MissionItemInt(m)
                if m.mission_type == self.mission_type
                    && self.count.is_some()
                    && m.seq == self.items.len() as u16 =>
            {
                self.items.push(MissionItem::from_message(m));
            }
            // Duplicates and out-of-order items are ignored; the next request asks again.
            _ => return Vec::new(),
        }
        self.last_activity = now;
        self.retries = 0;
        self.next()
    }

    /// Timeouts: re-send the outstanding request; give up after the retry budget.
    pub fn poll(&mut self, now: Instant) -> Vec<Message> {
        if !matches!(self.status, TransferStatus::InProgress { .. })
            || now.saturating_duration_since(self.last_activity) < self.cfg.retry_after
        {
            return Vec::new();
        }
        if self.retries >= self.cfg.max_retries {
            self.status = TransferStatus::Failed(match self.count {
                None => "no MISSION_COUNT".into(),
                Some(n) => format!("item {} of {n} never arrived", self.items.len()),
            });
            return Vec::new();
        }
        self.retries += 1;
        self.last_activity = now;
        match self.count {
            None => vec![self.list_message()],
            Some(_) => vec![self.request(self.items.len() as u16)],
        }
    }

    /// Status.
    pub fn status(&self) -> &TransferStatus {
        &self.status
    }

    /// Items received (complete once [`TransferStatus::Done`]).
    pub fn items(&self) -> &[MissionItem] {
        &self.items
    }
}

/// A difference between what was uploaded and what the vehicle holds.
#[derive(Debug, Clone, PartialEq)]
pub struct Mismatch {
    /// Item number (`None` for the item count).
    pub seq: Option<u16>,
    /// Field name.
    pub field: &'static str,
    /// Uploaded value.
    pub draft: String,
    /// Value on the vehicle.
    pub vehicle: String,
}

/// `*_INT` frames come back as their plain equivalents (ArduPilot).
fn normalise_frame(f: u8) -> u8 {
    match f {
        5 => 0,   // GLOBAL_INT → GLOBAL
        6 => 3,   // GLOBAL_RELATIVE_ALT_INT → GLOBAL_RELATIVE_ALT
        11 => 10, // GLOBAL_TERRAIN_ALT_INT → GLOBAL_TERRAIN_ALT
        f => f,
    }
}

/// Commands ArduPilot stores with a location (`AP_Mission::stored_in_location`, 4.7.1).
/// Others keep no frame, position or altitude: those fields read back as 0.
const ARDUPILOT_LOCATION_COMMANDS: [u16; 26] = [
    16, 17, 18, 19, 21, 22, 30, 31, 82, 92, 179, 188, 189, 191, 195, 201, 84, 85, 94, 5001, 5002,
    5003, 5004, 5000, 5100, 36,
];

/// Whether ArduPilot treats `command`'s x and y as latitude and longitude
/// (`AP_Mission::cmd_has_location`, 4.7.1). For other commands they are plain integers.
pub fn has_location(command: u16) -> bool {
    ARDUPILOT_LOCATION_COMMANDS.contains(&command)
}

/// The item as ArduPilot stores it, where its encoding is known to differ without changing
/// meaning (checked against `AP_Mission.cpp` 4.7.1 and SITL). Everything else is compared
/// as sent, so an unknown difference is reported, never assumed harmless.
fn ardupilot_canonical(mut i: MissionItem) -> MissionItem {
    i.frame = normalise_frame(i.frame);
    if !has_location(i.command) {
        (i.frame, i.x, i.y, i.z) = (0, 0, 0, 0.0);
    }
    if i.command == 19 {
        // NAV_LOITER_TIME: only the direction (param3 < 0 = counter-clockwise) and the
        // cross-track flag (param4 > 0) are stored.
        i.params[2] = if i.params[2] < 0.0 { -1.0 } else { 1.0 };
        i.params[3] = if i.params[3] > 0.0 { 1.0 } else { 0.0 };
    }
    i
}

fn same_param(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || (a - b).abs() <= 1e-4 * a.abs().max(1.0)
}

/// Compare the uploaded mission with the one read back (ArduPilot storage rules applied, see
/// `ardupilot_canonical`). `skip_home`: item 0 is the home slot
/// (ArduPilot `MISSION_TYPE_MISSION`) and is not compared. Empty result = verified.
pub fn verify(draft: &[MissionItem], vehicle: &[MissionItem], skip_home: bool) -> Vec<Mismatch> {
    let mut out = Vec::new();
    if draft.len() != vehicle.len() {
        out.push(Mismatch {
            seq: None,
            field: "count",
            draft: draft.len().to_string(),
            vehicle: vehicle.len().to_string(),
        });
    }
    let first = usize::from(skip_home);
    for (i, (d, v)) in draft.iter().zip(vehicle).enumerate().skip(first) {
        let (d, v) = (&ardupilot_canonical(*d), &ardupilot_canonical(*v));
        let seq = Some(i as u16);
        let mut diff = |field, dv: String, vv: String| {
            out.push(Mismatch {
                seq,
                field,
                draft: dv,
                vehicle: vv,
            })
        };
        if d.command != v.command {
            diff("command", d.command.to_string(), v.command.to_string());
        }
        if d.frame != v.frame {
            diff("frame", d.frame.to_string(), v.frame.to_string());
        }
        for (k, name) in ["param1", "param2", "param3", "param4"]
            .into_iter()
            .enumerate()
        {
            if !same_param(d.params[k], v.params[k]) {
                diff(name, d.params[k].to_string(), v.params[k].to_string());
            }
        }
        if d.x != v.x {
            diff("x", d.x.to_string(), v.x.to_string());
        }
        if d.y != v.y {
            diff("y", d.y.to_string(), v.y.to_string());
        }
        // ArduPilot stores altitude in whole centimetres.
        if !((d.z - v.z).abs() <= 0.011 || (d.z.is_nan() && v.z.is_nan())) {
            diff("z", d.z.to_string(), v.z.to_string());
        }
    }
    out
}

/// A mission flying `points` (latitude, longitude) in order at `alt_m` above home: home at
/// the first point, TAKEOFF there, a waypoint per point, RTL. Same layout as a survey.
pub fn route(points: &[(f64, f64)], alt_m: f32) -> Vec<MissionItem> {
    let item = |command: u16, frame: u8, (lat, lon): (f64, f64), z: f32| MissionItem {
        frame,
        command,
        params: [0.0; 4],
        x: (lat * 1e7).round() as i32,
        y: (lon * 1e7).round() as i32,
        z,
        autocontinue: true,
    };
    let Some(&first) = points.first() else {
        return Vec::new();
    };
    let mut m = vec![item(16, 0, first, 0.0), item(22, 3, (0.0, 0.0), alt_m)];
    m.extend(points.iter().map(|p| item(16, 3, *p, alt_m)));
    m.push(item(20, 2, (0.0, 0.0), 0.0));
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_route_is_home_takeoff_waypoints_rtl() {
        let r = route(&[(-35.0, 149.0), (-35.001, 149.0)], 30.0);
        assert_eq!(
            r.iter().map(|i| (i.command, i.frame)).collect::<Vec<_>>(),
            [(16, 0), (22, 3), (16, 3), (16, 3), (20, 2)]
        );
        assert_eq!((r[3].x, r[3].z), (-350_010_000, 30.0));
        assert!(route(&[], 30.0).is_empty());
    }

    fn item(command: u16, frame: u8, params: [f32; 4]) -> MissionItem {
        MissionItem {
            frame,
            command,
            params,
            x: 328_200_000,
            y: 349_800_000,
            z: 30.0,
            autocontinue: true,
        }
    }

    #[test]
    fn known_ardupilot_encodings_are_not_differences() {
        // RTL keeps no location; ArduPilot reads it back as frame 0, position 0.
        let rtl = item(20, 2, [0.0; 4]);
        let back = MissionItem {
            frame: 0,
            x: 0,
            y: 0,
            z: 0.0,
            ..rtl
        };
        assert_eq!(verify(&[rtl, rtl], &[rtl, back], true), vec![]);
        // LOITER_TIME direction 0 (clockwise) reads back as +1.
        let lt = item(19, 6, [12.0, 0.0, 0.0, 0.0]);
        let back = MissionItem {
            frame: 3,
            params: [12.0, 0.0, 1.0, 0.0],
            ..lt
        };
        assert_eq!(verify(&[lt, lt], &[lt, back], true), vec![]);
    }

    #[test]
    fn real_differences_are_still_reported() {
        let lt = item(19, 6, [12.0, 0.0, -1.0, 0.0]);
        let back = MissionItem {
            frame: 3,
            params: [12.0, 0.0, 1.0, 0.0],
            ..lt
        };
        assert_eq!(
            verify(&[lt, lt], &[lt, back], true)[0].field,
            "param3",
            "direction flipped"
        );
        // A waypoint keeps its location: a moved one is a difference.
        let wp = item(16, 6, [0.0; 4]);
        let moved = MissionItem {
            frame: 3,
            x: wp.x + 1,
            ..wp
        };
        assert_eq!(verify(&[wp, wp], &[wp, moved], true)[0].field, "x");
        // LOITER_TURNS packs a radius into param3: compared exactly.
        let turns = item(18, 6, [2.0, 0.0, 0.0, 0.0]);
        let back = MissionItem {
            frame: 3,
            params: [2.0, 0.0, 1.0, 0.0],
            ..turns
        };
        assert_eq!(
            verify(&[turns, turns], &[turns, back], true)[0].field,
            "param3"
        );
    }
}

/// Summary of a mission for the planner (an estimate; the vehicle's own path will differ
/// with turns, splines and wind).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MissionStats {
    /// Navigation items (commands up to `MAV_CMD_NAV_LAST`, 95), home excluded.
    pub nav_items: usize,
    /// Horizontal path length, m: home, then every navigation item with a location, back to
    /// home on RTL. `DO_JUMP` repeats are not followed.
    pub path_m: f64,
    /// Highest altitude of a navigation item, by frame: above home (frames 3, 6), above
    /// MSL (0, 5) and above terrain (10, 11). Altitudes in different frames cannot be
    /// compared without terrain data, so they are kept apart.
    pub max_alt_rel_m: Option<f32>,
    /// See `max_alt_rel_m`.
    pub max_alt_msl_m: Option<f32>,
    /// See `max_alt_rel_m`.
    pub max_alt_terrain_m: Option<f32>,
    /// The last navigation command: 20 (RTL), 21 (LAND) or another.
    pub last_nav: Option<u16>,
    /// The mission contains `DO_JUMP` (177): the path counts each item once.
    pub has_jumps: bool,
}

/// `MAV_CMD_NAV_LAST`: navigation commands have ids up to this (`AP_Mission::is_nav_cmd`
/// adds a few higher ones without a location).
const MAV_CMD_NAV_LAST: u16 = 95;
const MAV_CMD_NAV_RETURN_TO_LAUNCH: u16 = 20;
const MAV_CMD_DO_JUMP: u16 = 177;

/// Great-circle distance, m (mean Earth radius).
fn distance_m(a: (f64, f64), b: (f64, f64)) -> f64 {
    const R: f64 = 6_371_008.8;
    let (la1, lo1, la2, lo2) = (
        a.0.to_radians(),
        a.1.to_radians(),
        b.0.to_radians(),
        b.1.to_radians(),
    );
    let h = ((la2 - la1) / 2.0).sin().powi(2)
        + la1.cos() * la2.cos() * ((lo2 - lo1) / 2.0).sin().powi(2);
    2.0 * R * h.sqrt().asin()
}

/// Statistics of an ArduPilot mission (`items[0]` is home). A navigation item at 0, 0 means
/// "where the vehicle is" (`ModeAuto::get_loc_from_cmd`), so it adds no distance.
pub fn stats(items: &[MissionItem]) -> MissionStats {
    let mut s = MissionStats::default();
    let Some(home) = items.first() else {
        return s;
    };
    let deg = |i: &MissionItem| (f64::from(i.x) * 1e-7, f64::from(i.y) * 1e-7);
    let home = deg(home);
    let mut at = home;
    for i in &items[1..] {
        s.has_jumps |= i.command == MAV_CMD_DO_JUMP;
        if i.command > MAV_CMD_NAV_LAST {
            continue;
        }
        s.nav_items += 1;
        s.last_nav = Some(i.command);
        if i.command == MAV_CMD_NAV_RETURN_TO_LAUNCH {
            s.path_m += distance_m(at, home);
            at = home;
            continue;
        }
        if !has_location(i.command) {
            continue;
        }
        let max = |m: &mut Option<f32>| *m = Some(m.map_or(i.z, |v| v.max(i.z)));
        match i.frame {
            3 | 6 => max(&mut s.max_alt_rel_m),
            0 | 5 => max(&mut s.max_alt_msl_m),
            10 | 11 => max(&mut s.max_alt_terrain_m),
            _ => {}
        }
        if i.x == 0 && i.y == 0 {
            continue;
        }
        let p = deg(i);
        s.path_m += distance_m(at, p);
        at = p;
    }
    s
}

/// Why a mission edit is refused.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EditError {
    /// `DO_JUMP` targets item numbers, which the edit would invalidate.
    HasJumps,
    /// Nothing to edit (no items after home).
    Empty,
}

impl std::fmt::Display for EditError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            EditError::HasJumps => "the mission has DO_JUMP items, whose targets would break",
            EditError::Empty => "the mission has no items after home",
        })
    }
}

/// The route flown backwards: home (item 0), a leading `TAKEOFF` and a trailing `RTL` or
/// `LAND` stay where they are; every item between them is reversed in order. Commands that
/// configure what follows (speed, camera, servo) move with the reversal, so review them.
pub fn reverse(items: &[MissionItem]) -> Result<Vec<MissionItem>, EditError> {
    if items.len() < 2 {
        return Err(EditError::Empty);
    }
    if items.iter().any(|i| i.command == MAV_CMD_DO_JUMP) {
        return Err(EditError::HasJumps);
    }
    let start = if items[1].command == 22 { 2 } else { 1 };
    let end = if matches!(items.last().map(|i| i.command), Some(20 | 21)) && items.len() > start {
        items.len() - 1
    } else {
        items.len()
    };
    let mut out = items.to_vec();
    out[start..end].reverse();
    Ok(out)
}

/// Every item with a location moved `north_m` and `east_m` metres (home, item 0, too when
/// `include_home`). Items without a location, or at 0, 0 (meaning "here"), are unchanged.
pub fn shift(
    items: &[MissionItem],
    north_m: f64,
    east_m: f64,
    include_home: bool,
) -> Vec<MissionItem> {
    const R: f64 = 6_371_008.8;
    items
        .iter()
        .enumerate()
        .map(|(k, i)| {
            let mut i = *i;
            if (k > 0 || include_home) && has_location(i.command) && !(i.x == 0 && i.y == 0) {
                let lat = f64::from(i.x) * 1e-7;
                let lon = f64::from(i.y) * 1e-7;
                let nlat = lat + (north_m / R).to_degrees();
                let nlon = lon + (east_m / (R * lat.to_radians().cos())).to_degrees();
                i.x = (nlat * 1e7).round() as i32;
                i.y = (nlon * 1e7).round() as i32;
            }
            i
        })
        .collect()
}

/// One corner of a mission's flight path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathPoint {
    /// Item it comes from (0 = home).
    pub seq: u16,
    /// Latitude and longitude, degrees.
    pub lat: f64,
    /// See `lat`.
    pub lon: f64,
    /// Height above home, m. MSL altitudes are converted with home's MSL altitude when the
    /// file has one; heights above terrain are used as they are (approximate).
    pub alt_rel_m: f64,
}

/// ArduCopter's default RTL altitude, m above home (4.7.1 `RTL_ALT_M` = 15; 4.6.3
/// `RTL_ALT` = 1500 cm).
pub const RTL_ALT_DEFAULT_M: f64 = 15.0;

/// The path a mission flies, for drawing: home, takeoff straight up, every navigation item
/// with a location, landing straight down, and RTL back to home at its altitude (raised to
/// `RTL_ALT_DEFAULT_M` if lower, as the vehicle climbs first) then down.
/// `DO_JUMP` repeats are not followed.
pub fn path(items: &[MissionItem]) -> Vec<PathPoint> {
    let Some(home) = items.first() else {
        return Vec::new();
    };
    let deg = |i: &MissionItem| (f64::from(i.x) * 1e-7, f64::from(i.y) * 1e-7);
    let (hlat, hlon) = deg(home);
    // A downloaded home carries its MSL altitude; a planned file often has 0 there.
    let home_msl = (home.z != 0.0).then_some(f64::from(home.z));
    let mut out = vec![PathPoint {
        seq: 0,
        lat: hlat,
        lon: hlon,
        alt_rel_m: 0.0,
    }];
    let mut at = out[0];
    for (seq, i) in items.iter().enumerate().skip(1) {
        let seq = seq as u16;
        let rel = match i.frame {
            0 | 5 => home_msl.map_or(f64::from(i.z), |h| f64::from(i.z) - h),
            _ => f64::from(i.z),
        };
        let mut push = |p: PathPoint, at: &mut PathPoint| {
            out.push(p);
            *at = p;
        };
        match i.command {
            22 => push(
                PathPoint {
                    seq,
                    alt_rel_m: rel,
                    ..at
                },
                &mut at,
            ),
            20 => {
                // ArduCopter climbs to at least the RTL altitude (default 15 m above home) before
                // turning for home (4.7.1 `ModeRTL::compute_return_target`).
                if at.alt_rel_m < RTL_ALT_DEFAULT_M {
                    push(
                        PathPoint {
                            seq,
                            alt_rel_m: RTL_ALT_DEFAULT_M,
                            ..at
                        },
                        &mut at,
                    );
                }
                push(
                    PathPoint {
                        seq,
                        lat: hlat,
                        lon: hlon,
                        ..at
                    },
                    &mut at,
                );
                push(
                    PathPoint {
                        seq,
                        alt_rel_m: 0.0,
                        ..at
                    },
                    &mut at,
                );
            }
            21 => {
                if i.x != 0 || i.y != 0 {
                    let (lat, lon) = deg(i);
                    push(
                        PathPoint {
                            seq,
                            lat,
                            lon,
                            ..at
                        },
                        &mut at,
                    );
                }
                push(
                    PathPoint {
                        seq,
                        alt_rel_m: 0.0,
                        ..at
                    },
                    &mut at,
                );
            }
            c if c <= MAV_CMD_NAV_LAST && has_location(c) && (i.x != 0 || i.y != 0) => {
                let (lat, lon) = deg(i);
                push(
                    PathPoint {
                        seq,
                        lat,
                        lon,
                        alt_rel_m: rel,
                    },
                    &mut at,
                );
            }
            _ => {}
        }
    }
    out
}

/// Speeds a Copter flies a mission at, for [`estimate_time`]. Defaults are ArduCopter
/// 4.7.1's (`AC_WPNav.cpp`, `mode_land.cpp`, `ArduCopter/config.h`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Speeds {
    /// Horizontal speed between waypoints (`WP_SPD`), m/s.
    pub horizontal_ms: f64,
    /// Climb and descent speed (`WP_SPD_UP`, `WP_SPD_DN`), m/s.
    pub up_ms: f64,
    /// See `up_ms`.
    pub down_ms: f64,
    /// Final landing speed (`LAND_SPD_MS`), m/s.
    pub land_ms: f64,
    /// Landing speed above `land_alt_low_m` (`LAND_SPD_HIGH_MS`; 0 = `down_ms`), m/s.
    pub land_high_ms: f64,
    /// Height below which landing slows to `land_ms` (`LAND_ALT_LOW_M`), m.
    pub land_alt_low_m: f64,
    /// RTL return altitude (`RTL_ALT_M`; 0 = current altitude), m.
    pub rtl_alt_m: f64,
    /// RTL horizontal speed (`RTL_SPEED_MS`; 0 = `horizontal_ms`), m/s.
    pub rtl_speed_ms: f64,
}

impl Default for Speeds {
    fn default() -> Self {
        Speeds {
            horizontal_ms: 10.0,
            up_ms: 2.5,
            down_ms: 1.5,
            land_ms: 0.5,
            land_high_ms: 0.0,
            land_alt_low_m: 10.0,
            rtl_alt_m: 15.0,
            rtl_speed_ms: 0.0,
        }
    }
}

impl Speeds {
    /// From a vehicle's parameters: 4.7 names (metres), else the 4.6-and-earlier names
    /// (centimetres: `WPNAV_SPEED`, `WPNAV_SPEED_UP`, `WPNAV_SPEED_DN`, `LAND_SPEED`,
    /// `LAND_SPEED_HIGH`, `LAND_ALT_LOW`, `RTL_ALT`, `RTL_SPEED`; renamed in 4.7.1's
    /// `AC_WPNav::convert_parameters`, `mode_land.cpp`, `mode_rtl.cpp`), else the default.
    pub fn from_params(p: &std::collections::BTreeMap<String, f64>) -> Speeds {
        let d = Speeds::default();
        let get = |new: &str, old: &str, default: f64| {
            p.get(new)
                .copied()
                .or_else(|| p.get(old).map(|v| v / 100.0))
                .unwrap_or(default)
        };
        Speeds {
            horizontal_ms: get("WP_SPD", "WPNAV_SPEED", d.horizontal_ms),
            up_ms: get("WP_SPD_UP", "WPNAV_SPEED_UP", d.up_ms),
            down_ms: get("WP_SPD_DN", "WPNAV_SPEED_DN", d.down_ms),
            land_ms: get("LAND_SPD_MS", "LAND_SPEED", d.land_ms),
            land_high_ms: get("LAND_SPD_HIGH_MS", "LAND_SPEED_HIGH", d.land_high_ms),
            land_alt_low_m: get("LAND_ALT_LOW_M", "LAND_ALT_LOW", d.land_alt_low_m),
            rtl_alt_m: get("RTL_ALT_M", "RTL_ALT", d.rtl_alt_m),
            rtl_speed_ms: get("RTL_SPEED_MS", "RTL_SPEED", d.rtl_speed_ms),
        }
    }
}

/// Rough flight time of a mission.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TimeEstimate {
    /// Total, s.
    pub total_s: f64,
    /// Of which holding (waypoint delays, `LOITER_TIME`, `DELAY`), s.
    pub hold_s: f64,
    /// Some altitudes are above MSL or terrain and could not be compared with home: their
    /// climbs and descents are not counted.
    pub alt_unknown: bool,
    /// The mission contains `LOITER_UNLIM` (it never ends by itself) or `DO_JUMP` (repeats
    /// are not counted).
    pub open_ended: bool,
}

/// Rough flight time: each leg flown in a straight line at the waypoint speeds (whichever
/// of the horizontal and vertical distance takes longer), plus holds, RTL and landing as
/// Copter flies them. Acceleration, turns and wind are ignored, so this is a lower bound:
/// a short SITL test mission (about 190 m) took 1.5 times as long.
pub fn estimate_time(items: &[MissionItem], sp: &Speeds) -> TimeEstimate {
    let mut e = TimeEstimate::default();
    let Some(home) = items.first() else {
        return e;
    };
    let deg = |i: &MissionItem| (f64::from(i.x) * 1e-7, f64::from(i.y) * 1e-7);
    let home = deg(home);
    let (mut at, mut alt) = (home, 0.0f64);
    let leg = |from: (f64, f64), to: (f64, f64), dz: f64, h_speed: f64| {
        let h = distance_m(from, to) / h_speed;
        let v = if dz >= 0.0 {
            dz / sp.up_ms
        } else {
            -dz / sp.down_ms
        };
        h.max(v)
    };
    let land = |alt: f64| {
        let low = alt.min(sp.land_alt_low_m).max(0.0);
        let high_speed = if sp.land_high_ms > 0.0 {
            sp.land_high_ms
        } else {
            sp.down_ms
        };
        (alt - low).max(0.0) / high_speed.max(sp.land_ms) + low / sp.land_ms
    };
    for i in &items[1..] {
        let target_alt = match i.frame {
            3 | 6 => Some(f64::from(i.z)),
            _ => None,
        };
        let has_pos = has_location(i.command) && !(i.x == 0 && i.y == 0);
        let to = if has_pos { deg(i) } else { at };
        match i.command {
            177 => e.open_ended = true,
            17 => e.open_ended = true,
            // TAKEOFF: straight up.
            22 => {
                if let Some(z) = target_alt {
                    e.total_s += (z - alt).max(0.0) / sp.up_ms;
                    alt = z;
                } else {
                    e.alt_unknown = true;
                }
            }
            // WAYPOINT, LOITER_TIME, LOITER_TURNS, SPLINE_WAYPOINT.
            16 | 19 | 18 | 82 => {
                let dz = match target_alt {
                    Some(z) => z - alt,
                    None => {
                        e.alt_unknown |= has_location(i.command);
                        0.0
                    }
                };
                e.total_s += leg(at, to, dz, sp.horizontal_ms);
                alt += dz;
                at = to;
                let hold = match i.command {
                    16 | 19 => f64::from(i.params[0]).max(0.0),
                    _ => 0.0,
                };
                e.hold_s += hold;
                e.total_s += hold;
                if i.command == 18 {
                    let circle = 2.0 * std::f64::consts::PI * f64::from(i.params[2]).abs();
                    e.total_s += f64::from(i.params[0]).max(0.0) * circle / sp.horizontal_ms;
                }
            }
            // DELAY.
            93 => {
                let d = f64::from(i.params[0]).max(0.0);
                e.hold_s += d;
                e.total_s += d;
            }
            // LAND: fly there at the current altitude, then descend.
            21 => {
                e.total_s += leg(at, to, 0.0, sp.horizontal_ms) + land(alt);
                at = to;
                alt = 0.0;
            }
            // RTL: climb to the return altitude, fly home, land (RTL_ALT_FINAL_M 0).
            20 => {
                let ret = if sp.rtl_alt_m > 0.0 {
                    alt.max(sp.rtl_alt_m)
                } else {
                    alt
                };
                e.total_s += (ret - alt) / sp.up_ms;
                let speed = if sp.rtl_speed_ms > 0.0 {
                    sp.rtl_speed_ms
                } else {
                    sp.horizontal_ms
                };
                e.total_s += leg(at, home, 0.0, speed) + land(ret);
                at = home;
                alt = 0.0;
            }
            _ => {}
        }
    }
    e
}

#[cfg(test)]
mod edit_tests {
    use super::*;

    fn it(command: u16, lat: f64) -> MissionItem {
        MissionItem {
            frame: 3,
            command,
            params: [0.0; 4],
            x: (lat * 1e7).round() as i32,
            y: if lat == 0.0 { 0 } else { 1_490_000_000 },
            z: 20.0,
            autocontinue: true,
        }
    }

    #[test]
    fn reverse_keeps_home_takeoff_and_rtl() {
        let m = [
            it(16, -35.0),
            it(22, 0.0),
            it(16, -34.999),
            it(16, -34.998),
            it(16, -34.997),
            it(20, 0.0),
        ];
        let r = reverse(&m).unwrap();
        let lats: Vec<i32> = r.iter().map(|i| i.x).collect();
        assert_eq!(
            lats,
            [m[0].x, 0, m[4].x, m[3].x, m[2].x, 0],
            "home, takeoff, the three waypoints backwards, RTL"
        );
        assert_eq!(r[1].command, 22);
        assert_eq!(r[5].command, 20);
        let mut jump = m.to_vec();
        jump.insert(3, it(177, 0.0));
        assert_eq!(reverse(&jump), Err(EditError::HasJumps));
    }

    #[test]
    fn shift_moves_located_items_by_metres() {
        let m = [it(16, -35.0), it(22, 0.0), it(16, -35.0)];
        let s = shift(&m, 111.19, 0.0, false);
        assert_eq!(s[0], m[0], "home stays");
        assert_eq!(s[1], m[1], "takeoff at 0, 0 stays");
        assert!(
            (f64::from(s[2].x - m[2].x) * 1e-7 - 0.001).abs() < 1e-6,
            "{:?}",
            s[2]
        );
        let e = shift(&m, 0.0, 100.0, true);
        let dlon = f64::from(e[0].y - m[0].y) * 1e-7;
        let metres = dlon.to_radians() * 6_371_008.8 * (-35f64).to_radians().cos();
        assert!((metres - 100.0).abs() < 0.05, "{metres}");
    }
}

#[cfg(test)]
mod path_tests {
    use super::*;

    #[test]
    fn path_goes_up_out_and_back_down() {
        let it = |command: u16, frame: u8, lat: f64, lon: f64, z: f32| MissionItem {
            frame,
            command,
            params: [0.0; 4],
            x: (lat * 1e7).round() as i32,
            y: (lon * 1e7).round() as i32,
            z,
            autocontinue: true,
        };
        let m = [
            it(16, 0, -35.0, 149.0, 584.0),
            it(22, 3, 0.0, 0.0, 20.0),
            it(16, 3, -34.999, 149.0, 30.0),
            it(178, 0, 0.0, 0.0, 0.0),          // DO_CHANGE_SPEED: no point
            it(16, 0, -34.999, 149.001, 624.0), // MSL: 40 m above home
            it(20, 0, 0.0, 0.0, 0.0),
        ];
        let p: Vec<(u16, f64, f64)> = path(&m)
            .iter()
            .map(|p| (p.seq, (p.lat * 1e7).round() / 1e7, p.alt_rel_m))
            .collect();
        assert_eq!(
            p,
            [
                (0, -35.0, 0.0),
                (1, -35.0, 20.0),
                (2, -34.999, 30.0),
                (4, -34.999, 40.0),
                (5, -35.0, 40.0),
                (5, -35.0, 0.0),
            ]
        );
    }
}

#[cfg(test)]
mod time_tests {
    use super::*;

    fn it(command: u16, lat: f64, z: f32, p1: f32) -> MissionItem {
        MissionItem {
            frame: if command == 16 && lat == -35.0 && z == 0.0 {
                0
            } else {
                3
            },
            command,
            params: [p1, 0.0, 0.0, 0.0],
            x: (lat * 1e7).round() as i32,
            y: if lat == 0.0 { 0 } else { 1_490_000_000 },
            z,
            autocontinue: true,
        }
    }

    #[test]
    fn takeoff_out_hold_and_rtl() {
        // 0.009 deg of latitude is about 1000.8 m.
        let m = [
            it(16, -35.0, 0.0, 0.0),     // home
            it(22, 0.0, 20.0, 0.0),      // takeoff to 20 m: 8 s
            it(16, -34.991, 20.0, 10.0), // 1000.8 m at 10 m/s, hold 10 s
            it(20, 0.0, 0.0, 0.0),       // RTL: stays at 20 m (> 15), back, land
        ];
        let e = estimate_time(&m, &Speeds::default());
        let d = distance_m((-35.0, 149.0), (-34.991, 149.0));
        // Landing from 20 m: 10 m at 1.5 m/s, then 10 m at 0.5 m/s.
        let want = 8.0 + d / 10.0 + 10.0 + d / 10.0 + 10.0 / 1.5 + 20.0;
        assert!((e.total_s - want).abs() < 1e-6, "{e:?} vs {want}");
        assert_eq!(e.hold_s, 10.0);
        assert!(!e.alt_unknown && !e.open_ended);
    }

    #[test]
    fn climbs_limit_short_legs_and_params_of_both_versions_are_read() {
        let m = [
            it(16, -35.0, 0.0, 0.0),
            it(22, 0.0, 10.0, 0.0),
            // 111 m away but 50 m higher: the climb at 2.5 m/s (20 s) takes longer.
            it(16, -34.999, 60.0, 0.0),
        ];
        let e = estimate_time(&m, &Speeds::default());
        assert!((e.total_s - (4.0 + 20.0)).abs() < 1e-6, "{e:?}");
        let mut p = std::collections::BTreeMap::new();
        p.insert("WPNAV_SPEED".to_string(), 500.0); // 4.6: cm/s
        p.insert("WP_SPD_UP".to_string(), 5.0); // 4.7: m/s
        let s = Speeds::from_params(&p);
        assert_eq!((s.horizontal_ms, s.up_ms, s.down_ms), (5.0, 5.0, 1.5));
    }
}

#[cfg(test)]
mod stats_tests {
    use super::*;

    fn it(command: u16, frame: u8, lat: f64, lon: f64, z: f32) -> MissionItem {
        MissionItem {
            frame,
            command,
            params: [0.0; 4],
            x: (lat * 1e7).round() as i32,
            y: (lon * 1e7).round() as i32,
            z,
            autocontinue: true,
        }
    }

    #[test]
    fn path_and_altitudes() {
        // 0.001° of latitude is about 111.2 m.
        let m = [
            it(16, 0, -35.0, 149.0, 584.0),
            it(22, 3, 0.0, 0.0, 20.0),        // take off where it is
            it(16, 3, -34.999, 149.0, 30.0),  // 111 m north
            it(178, 2, 0.0, 0.0, 0.0),        // DO_CHANGE_SPEED: not navigation
            it(177, 2, 0.0, 0.0, 0.0),        // DO_JUMP
            it(16, 0, -34.998, 149.0, 650.0), // another 111 m, MSL altitude
            it(20, 2, 0.0, 0.0, 0.0),         // RTL: 222 m back
        ];
        let s = stats(&m);
        assert_eq!(s.nav_items, 4);
        assert!((s.path_m - 444.8).abs() < 1.0, "{}", s.path_m);
        assert_eq!(
            (s.max_alt_rel_m, s.max_alt_msl_m, s.max_alt_terrain_m),
            (Some(30.0), Some(650.0), None)
        );
        assert_eq!((s.last_nav, s.has_jumps), (Some(20), true));
        assert_eq!(stats(&[]), MissionStats::default());
    }
}

/// How serious a [`Finding`] is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LintLevel {
    /// Worth knowing.
    Note,
    /// Very likely a mistake.
    Warning,
}

/// One remark about a mission.
#[derive(Debug, Clone, PartialEq)]
pub struct Finding {
    /// Item number (`None`: the mission as a whole).
    pub seq: Option<u16>,
    /// Level.
    pub level: LintLevel,
    /// What and why.
    pub text: String,
}

const MAV_CMD_NAV_TAKEOFF: u16 = 22;
const MAV_CMD_NAV_LAND: u16 = 21;

/// Check an ArduPilot mission (`items[0]` is home) for likely mistakes before it is
/// uploaded. Nothing here changes the mission.
pub fn lint(items: &[MissionItem]) -> Vec<Finding> {
    let mut out = Vec::new();
    let mut add = |seq: Option<usize>, level, text: String| {
        out.push(Finding {
            seq: seq.map(|s| s as u16),
            level,
            text,
        });
    };
    if items.len() <= 1 {
        add(
            None,
            LintLevel::Warning,
            "no mission items after home".into(),
        );
        return out;
    }
    let nav: Vec<(usize, &MissionItem)> = items
        .iter()
        .enumerate()
        .skip(1)
        .filter(|(_, i)| i.command <= MAV_CMD_NAV_LAST)
        .collect();
    // As `AP_Mission::is_takeoff_next` (4.7.1): the first navigation command, NAV_DELAY
    // (93) skipped, must be NAV_TAKEOFF, NAV_TAKEOFF_LOCAL (24) or NAV_VTOL_TAKEOFF (84).
    match nav.iter().find(|(_, i)| i.command != 93) {
        Some((_, i)) if matches!(i.command, MAV_CMD_NAV_TAKEOFF | 24 | 84) => {}
        Some((s, _)) => add(
            Some(*s),
            LintLevel::Note,
            "the first navigation command is not TAKEOFF: Copter refuses AUTO while landed \
             and armed (\"Auto: Missing Takeoff Cmd\"); fine only if AUTO starts in the air"
                .into(),
        ),
        None => add(None, LintLevel::Warning, "no navigation commands".into()),
    }
    if let Some((s, i)) = nav.last()
        && !matches!(i.command, MAV_CMD_NAV_RETURN_TO_LAUNCH | MAV_CMD_NAV_LAND)
    {
        add(
            Some(*s),
            LintLevel::Note,
            "the mission does not end with RTL or LAND: what the vehicle does at the end \
             depends on its settings"
                .into(),
        );
    }
    let mut frames = std::collections::BTreeSet::new();
    for (s, i) in items.iter().enumerate().skip(1) {
        if i.command == MAV_CMD_DO_JUMP {
            let target = i.params[0];
            if target < 1.0 || target as usize >= items.len() || target.fract() != 0.0 {
                add(
                    Some(s),
                    LintLevel::Warning,
                    format!("DO_JUMP to item {target}, which does not exist"),
                );
            }
        }
        if !has_location(i.command) || i.command > MAV_CMD_NAV_LAST {
            continue;
        }
        if i.x != 0 || i.y != 0 {
            let (lat, lon) = (f64::from(i.x) * 1e-7, f64::from(i.y) * 1e-7);
            if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
                add(
                    Some(s),
                    LintLevel::Warning,
                    format!("position {lat}, {lon} is invalid"),
                );
            }
        }
        let relative = matches!(i.frame, 3 | 6);
        if i.command == MAV_CMD_NAV_TAKEOFF && relative && i.z <= 0.0 {
            add(
                Some(s),
                LintLevel::Warning,
                format!("TAKEOFF to {} m above home", i.z),
            );
        } else if relative && i.z <= 0.0 && i.command != MAV_CMD_NAV_LAND {
            add(
                Some(s),
                LintLevel::Warning,
                format!("waypoint at {} m above home: at or below the ground", i.z),
            );
        }
        frames.insert(match i.frame {
            0 | 5 => "above MSL",
            3 | 6 => "above home",
            10 | 11 => "above terrain",
            _ => "other",
        });
        if matches!(i.frame, 10 | 11) {
            add(
                Some(s),
                LintLevel::Note,
                "altitude above terrain: the vehicle needs terrain data for this location".into(),
            );
        }
    }
    if frames.len() > 1 {
        add(
            None,
            LintLevel::Note,
            format!(
                "altitudes use more than one reference ({}): check they mean what you intend",
                frames.into_iter().collect::<Vec<_>>().join(", ")
            ),
        );
    }
    out
}

#[cfg(test)]
mod lint_tests {
    use super::*;

    fn it(command: u16, frame: u8, z: f32) -> MissionItem {
        MissionItem {
            frame,
            command,
            params: [0.0; 4],
            x: -353_632_610,
            y: 1_491_652_300,
            z,
            autocontinue: true,
        }
    }

    #[test]
    fn a_clean_mission_has_no_findings() {
        let m = [
            it(16, 0, 584.0),
            it(22, 3, 20.0),
            it(16, 3, 30.0),
            it(20, 2, 0.0),
        ];
        assert_eq!(lint(&m), []);
        // A NAV_DELAY and a DO command before the takeoff are allowed (is_takeoff_next).
        let m = [
            it(16, 0, 584.0),
            it(93, 2, 0.0),
            it(178, 2, 0.0),
            it(22, 3, 20.0),
            it(20, 2, 0.0),
        ];
        assert_eq!(lint(&m), []);
    }

    #[test]
    fn finds_likely_mistakes() {
        let mut jump = it(177, 2, 0.0);
        jump.params[0] = 9.0;
        let m = [
            it(16, 0, 584.0),
            it(16, 3, 0.0),   // no takeoff first; at ground level
            jump,             // jump to a missing item
            it(16, 0, 650.0), // MSL among relative altitudes
        ];
        let f = lint(&m);
        let texts: Vec<_> = f.iter().map(|f| (f.seq, f.level)).collect();
        assert!(texts.contains(&(Some(1), LintLevel::Note)), "{f:#?}");
        assert!(texts.contains(&(Some(1), LintLevel::Warning)), "{f:#?}");
        assert!(texts.contains(&(Some(2), LintLevel::Warning)), "{f:#?}");
        assert!(
            texts.contains(&(Some(3), LintLevel::Note)),
            "no RTL/LAND at the end: {f:#?}"
        );
        assert!(
            f.iter()
                .any(|f| f.seq.is_none() && f.text.contains("more than one reference"))
        );
        assert_eq!(lint(&m[..1])[0].text, "no mission items after home");
    }
}
