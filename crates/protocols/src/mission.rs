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

/// The item as ArduPilot stores it, where its encoding is known to differ without changing
/// meaning (checked against `AP_Mission.cpp` 4.7.1 and SITL). Everything else is compared
/// as sent, so an unknown difference is reported, never assumed harmless.
fn ardupilot_canonical(mut i: MissionItem) -> MissionItem {
    i.frame = normalise_frame(i.frame);
    if !ARDUPILOT_LOCATION_COMMANDS.contains(&i.command) {
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

#[cfg(test)]
mod tests {
    use super::*;

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
