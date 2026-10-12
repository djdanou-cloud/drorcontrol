//! `.tlog` review: what a recorded session contains and what happened in it.
//!
//! Messages are folded through the same [`dc_core::Fleet`] the live core uses, so mode names
//! follow the same rule: a mode is named only once the firmware banner confirmed the firmware.

use std::collections::BTreeMap;
use std::io::{self, Read};
use std::time::{Duration, Instant};

use dc_core::Fleet;
use mavlink_codec::Message;
use recorder::TlogReader;

/// What happened, at microseconds since the Unix epoch.
#[derive(Debug, Clone, PartialEq)]
pub struct Event {
    /// Time of the record.
    pub unix_us: u64,
    /// System id.
    pub sysid: u8,
    /// Description.
    pub text: String,
}

/// Summary of one `.tlog`.
#[derive(Debug, Default)]
pub struct Summary {
    /// Records read.
    pub records: u64,
    /// First and last record time (µs since the Unix epoch).
    pub span: Option<(u64, u64)>,
    /// Frames per (sysid, compid).
    pub sources: BTreeMap<(u8, u8), u64>,
    /// Frames received and lost per (sysid, compid), from MAVLink sequence numbers (as the
    /// live core counts them). Frames our side sent are recorded too.
    pub sequences: BTreeMap<(u8, u8), dc_core::SeqCount>,
    /// Messages by name (`#id` for ids the dialect does not know).
    pub messages: BTreeMap<String, u64>,
    /// Mode changes, arming, firmware and STATUSTEXT (severity 0–6), in order.
    pub events: Vec<Event>,
    /// Highest altitude above home per system, m.
    pub max_alt_rel_m: BTreeMap<u8, f32>,
    /// Armed time, ground speed and distance per system.
    pub flights: BTreeMap<u8, FlightStats>,
    /// Telemetry radio reports (`RADIO_STATUS`) per sender.
    pub radios: BTreeMap<(u8, u8), RadioStats>,
    /// Link gaps per vehicle: times (µs since the Unix epoch) and lengths (s) of each gap
    /// between the autopilot's heartbeats longer than the core's STALE limit
    /// (`CoreConfig::link_stale`, 1.5 s); those over `link_lost` (5 s) are what the live
    /// core would have shown as LOST. A gap still open at the end of the recording is
    /// included.
    pub link_gaps: BTreeMap<u8, Vec<(u64, f64)>>,
    last_heartbeat: BTreeMap<u8, u64>,
    /// Why reading stopped early (a truncated or corrupt record), if it did.
    pub stopped: Option<String>,
}

/// A telemetry radio over the recording, from `RADIO_STATUS`. Signal and noise are in the
/// radio's own units (MAVLink: "device dependent"; about 2 per dB on SiK radios); 255 means
/// unknown and is skipped. Local = the radio sending the report, remote = the other end.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct RadioStats {
    /// Reports.
    pub reports: u64,
    /// Lowest and mean local and remote signal.
    pub rssi: Option<(u8, f64)>,
    /// See `rssi`.
    pub remrssi: Option<(u8, f64)>,
    /// Smallest margin of signal over noise, local and remote (fade margin).
    pub margin: Option<i16>,
    /// See `margin`.
    pub rem_margin: Option<i16>,
    /// Receive errors and corrected packets during the recording (counters since the radio's
    /// boot; a reset is counted from zero).
    pub rx_errors: u64,
    /// See `rx_errors`.
    pub fixed: u64,
    /// Lowest free transmit buffer, %.
    pub min_txbuf: Option<u8>,
    sums: (f64, u64, f64, u64),
    last_counters: Option<(u16, u16)>,
}

impl RadioStats {
    fn add(&mut self, r: &mavlink_codec::messages::RadioStatus) {
        self.reports += 1;
        let low = |cur: Option<(u8, f64)>, v: u8| cur.map_or(v, |c| c.0.min(v));
        if r.rssi != u8::MAX {
            self.sums.0 += f64::from(r.rssi);
            self.sums.1 += 1;
            self.rssi = Some((low(self.rssi, r.rssi), self.sums.0 / self.sums.1 as f64));
            if r.noise != u8::MAX {
                let m = i16::from(r.rssi) - i16::from(r.noise);
                self.margin = Some(self.margin.map_or(m, |x| x.min(m)));
            }
        }
        if r.remrssi != u8::MAX {
            self.sums.2 += f64::from(r.remrssi);
            self.sums.3 += 1;
            self.remrssi = Some((
                low(self.remrssi, r.remrssi),
                self.sums.2 / self.sums.3 as f64,
            ));
            if r.remnoise != u8::MAX {
                let m = i16::from(r.remrssi) - i16::from(r.remnoise);
                self.rem_margin = Some(self.rem_margin.map_or(m, |x| x.min(m)));
            }
        }
        let grow = |last: u16, now: u16| u64::from(if now >= last { now - last } else { now });
        if let Some((e, f)) = self.last_counters {
            self.rx_errors += grow(e, r.rxerrors);
            self.fixed += grow(f, r.fixed);
        }
        self.last_counters = Some((r.rxerrors, r.fixed));
        self.min_txbuf = Some(self.min_txbuf.map_or(r.txbuf, |t| t.min(r.txbuf)));
    }
}

/// What one vehicle did while armed, from its heartbeats and `GLOBAL_POSITION_INT`.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct FlightStats {
    /// Armed time, s (an armed period open at the end counts to the end of the recording).
    pub armed_s: f64,
    /// Times armed.
    pub arms: u32,
    /// Horizontal distance while armed, m.
    pub distance_m: f64,
    /// Highest ground speed, m/s.
    pub max_speed_ms: f32,
    armed_since: Option<u64>,
    last_pos: Option<(f64, f64)>,
}

fn distance_m(a: (f64, f64), b: (f64, f64)) -> f64 {
    const R: f64 = 6_371_008.8;
    let (la1, la2) = (a.0.to_radians(), b.0.to_radians());
    let h = ((la2 - la1) / 2.0).sin().powi(2)
        + la1.cos() * la2.cos() * ((b.1 - a.1).to_radians() / 2.0).sin().powi(2);
    2.0 * R * h.sqrt().asin()
}

#[derive(Default, PartialEq)]
struct Seen {
    mode: Option<String>,
    armed: Option<bool>,
    firmware: bool,
}

/// Read a whole `.tlog`. A bad record ends the summary there (reported in `stopped`); what
/// was read before it is kept.
/// One CSV cell for a MAVLink field: numbers in their shortest form (floats sent as 32-bit
/// written as such), text up to its first NUL and quoted, arrays quoted and space-separated.
fn csv_cell(v: &mavlink_codec::FieldValue) -> String {
    use mavlink_codec::FieldValue as F;
    let float = |x: f64| {
        if f64::from(x as f32) == x {
            format!("{}", x as f32)
        } else {
            format!("{x}")
        }
    };
    let join = |parts: Vec<String>| format!("\"{}\"", parts.join(" "));
    match v {
        F::UInt(x) => x.to_string(),
        F::Int(x) => x.to_string(),
        F::Float(x) => float(*x),
        F::UIntArray(a) => join(a.iter().map(u64::to_string).collect()),
        F::IntArray(a) => join(a.iter().map(i64::to_string).collect()),
        F::FloatArray(a) => join(a.iter().map(|x| float(*x)).collect()),
        F::Chars(b) => {
            let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
            format!(
                "\"{}\"",
                String::from_utf8_lossy(&b[..end]).replace('"', "\"\"")
            )
        }
    }
}

/// Every `msg` message (name, any case) in the recording as CSV: receive time (µs since the
/// Unix epoch), sender, then the message's fields in declaration order. Only messages from
/// `sysid` when given. Returns the CSV and the row count; a bad record ends the read there.
pub fn csv(input: impl Read, msg: &str, sysid: Option<u8>) -> io::Result<(String, usize)> {
    let mut r = TlogReader::new(input);
    let mut out = String::new();
    let mut rows = 0;
    while let Ok(Some(rec)) = r.next_record() {
        let f = &rec.frame;
        if sysid.is_some_and(|s| s != f.sysid()) {
            continue;
        }
        let Ok(m) = f.decode() else { continue };
        if !m.name().eq_ignore_ascii_case(msg) {
            continue;
        }
        let fields = m.fields();
        if rows == 0 {
            out.push_str("time_usec,sysid,compid");
            for (name, _) in &fields {
                out.push(',');
                out.push_str(name);
            }
            out.push('\n');
        }
        out.push_str(&format!("{},{},{}", rec.unix_us, f.sysid(), f.compid()));
        for (_, v) in &fields {
            out.push(',');
            out.push_str(&csv_cell(v));
        }
        out.push('\n');
        rows += 1;
    }
    Ok((out, rows))
}

pub fn summarise(input: impl Read) -> io::Result<Summary> {
    let mut r = TlogReader::new(input);
    let mut s = Summary::default();
    let mut fleet = Fleet::new();
    let base = Instant::now();
    let mut seen: BTreeMap<u8, Seen> = BTreeMap::new();
    loop {
        let rec = match r.next_record() {
            Ok(Some(rec)) => rec,
            Ok(None) => break,
            Err(e) => {
                s.stopped = Some(e.to_string());
                break;
            }
        };
        s.records += 1;
        let t = rec.unix_us;
        s.span = Some(s.span.map_or((t, t), |(a, b)| (a.min(t), b.max(t))));
        let f = &rec.frame;
        let (sysid, compid) = (f.sysid(), f.compid());
        *s.sources.entry((sysid, compid)).or_default() += 1;
        s.sequences
            .entry((sysid, compid))
            .or_default()
            .on_seq(f.seq());
        let Ok(msg) = f.decode() else {
            *s.messages.entry(format!("#{}", f.msgid())).or_default() += 1;
            continue;
        };
        *s.messages.entry(msg.name().to_string()).or_default() += 1;
        let at = base + Duration::from_micros(t - s.span.map_or(t, |(a, _)| a));
        if let Message::RadioStatus(r) = &msg {
            s.radios.entry((sysid, compid)).or_default().add(r);
        }
        if matches!(msg, Message::Heartbeat(_)) && compid == 1 {
            let stale = dc_core::CoreConfig::default().link_stale;
            if let Some(prev) = s.last_heartbeat.insert(sysid, t) {
                let gap = t.saturating_sub(prev);
                if gap > stale.as_micros() as u64 {
                    s.link_gaps
                        .entry(sysid)
                        .or_default()
                        .push((prev, gap as f64 / 1e6));
                }
            }
        }
        let text = fleet.apply(sysid, compid, &msg, at);
        let mut event = |text: String| {
            s.events.push(Event {
                unix_us: t,
                sysid,
                text,
            })
        };
        if let Some(st) = text
            && st.severity <= 6
        {
            event(format!("sev{} {}", st.severity, st.text));
        }
        let Some(v) = fleet.get(sysid) else { continue };
        let prev = seen.entry(sysid).or_default();
        if !prev.firmware
            && let Some(fw) = v.confirmed_firmware()
        {
            prev.firmware = true;
            event(format!("firmware {fw:?} (banner)"));
        }
        if let Some(hb) = &v.heartbeat {
            let mode = v.mode_name().map_or_else(
                || format!("number {}", hb.value.custom_mode),
                str::to_string,
            );
            if prev.mode.as_ref() != Some(&mode) {
                event(format!("mode {mode}"));
                prev.mode = Some(mode);
            }
            let armed = hb.value.armed();
            if prev.armed != Some(armed) {
                event(if armed { "ARMED" } else { "disarmed" }.to_string());
                prev.armed = Some(armed);
                let f = s.flights.entry(sysid).or_default();
                if armed {
                    f.arms += 1;
                    f.armed_since = Some(t);
                } else if let Some(since) = f.armed_since.take() {
                    f.armed_s += t.saturating_sub(since) as f64 / 1e6;
                    f.last_pos = None;
                }
            }
        }
        if let Some(p) = &v.position {
            let m = s.max_alt_rel_m.entry(sysid).or_insert(f32::MIN);
            *m = m.max(p.value.alt_rel_m);
        }
        if let (Message::GlobalPositionInt(_), Some(p)) = (&msg, &v.position) {
            let f = s.flights.entry(sysid).or_default();
            let speed = p.value.vel_ned[0].hypot(p.value.vel_ned[1]);
            f.max_speed_ms = f.max_speed_ms.max(speed);
            let here = (p.value.lat_deg, p.value.lon_deg);
            if f.armed_since.is_some() && here != (0.0, 0.0) {
                if let Some(last) = f.last_pos {
                    f.distance_m += distance_m(last, here);
                }
                f.last_pos = Some(here);
            }
        }
    }
    if let Some((_, end)) = s.span {
        let stale = dc_core::CoreConfig::default().link_stale.as_micros() as u64;
        for (sys, last) in &s.last_heartbeat {
            if end.saturating_sub(*last) > stale {
                s.link_gaps
                    .entry(*sys)
                    .or_default()
                    .push((*last, end.saturating_sub(*last) as f64 / 1e6));
            }
        }
        for f in s.flights.values_mut() {
            if let Some(since) = f.armed_since {
                f.armed_s += end.saturating_sub(since) as f64 / 1e6;
            }
        }
    }
    Ok(s)
}

/// The parameters a recording saw from one vehicle's autopilot (`PARAM_VALUE`, last value
/// per name), as Mission Planner's "tlog → param" does. Default vehicle: the first that sent
/// any. Returns the system id and the parameters.
pub fn params(
    input: impl Read,
    vehicle: Option<u8>,
) -> io::Result<Option<(u8, BTreeMap<String, protocols::params::Param>)>> {
    let mut r = TlogReader::new(input);
    let mut chosen = vehicle;
    let mut out = BTreeMap::new();
    while let Ok(Some(rec)) = r.next_record() {
        let f = &rec.frame;
        if f.compid() != 1 {
            // Autopilot component only (vehicle::AUTOPILOT_COMPID).

            continue;
        }
        let Ok(Message::ParamValue(p)) = f.decode() else {
            continue;
        };
        if *chosen.get_or_insert(f.sysid()) != f.sysid() {
            continue;
        }
        out.insert(
            protocols::params::decode_id(&p.param_id),
            protocols::params::Param {
                value: p.param_value,
                ty: p.param_type,
                index: p.param_index,
            },
        );
    }
    Ok(chosen.map(|s| (s, out)))
}

/// A mission and who sent it: (system id, component id).
pub type SentMission = ((u8, u8), Vec<dc_core::MissionItem>);

/// The last complete list of `mission_type` (`MAV_MISSION_TYPE`: 0 mission, 1 fence, 2 rally)
/// seen in a recording (`MISSION_COUNT` then every item, from one sender): the vehicle's
/// answer to a download, or the GCS's upload. Items
/// in `MISSION_ITEM` (not `_INT`) are converted as ArduPilot does (x, y × 1e7 for commands
/// with a location). Returns the sender and the items.
pub fn mission(input: impl Read, mission_type: u8) -> io::Result<Option<SentMission>> {
    use protocols::mission::{MissionItem, has_location};
    let mut r = TlogReader::new(input);
    // Per sender: the announced count and the items so far.
    let mut open: BTreeMap<(u8, u8), (u16, BTreeMap<u16, MissionItem>)> = BTreeMap::new();
    let mut last = None;
    while let Ok(Some(rec)) = r.next_record() {
        let f = &rec.frame;
        let from = (f.sysid(), f.compid());
        let (seq, item) = match f.decode() {
            Ok(Message::MissionCount(c)) if c.mission_type == mission_type => {
                if c.count == 0 {
                    // An empty mission is complete as announced.
                    last = Some((from, Vec::new()));
                    open.remove(&from);
                } else {
                    open.insert(from, (c.count, BTreeMap::new()));
                }
                continue;
            }
            Ok(Message::MissionItemInt(m)) if m.mission_type == mission_type => {
                (m.seq, MissionItem::from_message(&m))
            }
            Ok(Message::MissionItem(m)) if m.mission_type == mission_type => {
                let scale = |v: f32| {
                    if has_location(m.command) {
                        (f64::from(v) * 1e7).round() as i32
                    } else {
                        v as i32
                    }
                };
                let item = MissionItem {
                    frame: m.frame,
                    command: m.command,
                    params: [m.param1, m.param2, m.param3, m.param4],
                    x: scale(m.x),
                    y: scale(m.y),
                    z: m.z,
                    autocontinue: m.autocontinue != 0,
                };
                (m.seq, item)
            }
            _ => continue,
        };
        let Some((count, items)) = open.get_mut(&from) else {
            continue;
        };
        if seq < *count {
            items.insert(seq, item);
        }
        if items.len() == usize::from(*count) {
            last = Some((from, items.values().copied().collect()));
            open.remove(&from);
        }
    }
    Ok(last)
}

/// `YYYY-MM-DD HH:MM:SS.mmm` UTC.
pub fn utc(unix_us: u64) -> String {
    let secs = unix_us / 1_000_000;
    let ms = (unix_us / 1000) % 1000;
    let days = (secs / 86_400) as i64;
    let rem = secs % 86_400;
    // Civil date from days since 1970-01-01 (Howard Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}-{m:02}-{d:02} {:02}:{:02}:{:02}.{ms:03}",
        rem / 3600,
        (rem / 60) % 60,
        rem % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::messages::{GlobalPositionInt, Heartbeat, Statustext};
    use mavlink_codec::{Header, Message, Version, encode};
    use recorder::TlogWriter;

    fn tlog(records: &[(u64, u8, Message)]) -> Vec<u8> {
        let mut w = TlogWriter::new(Vec::new());
        for (i, (us, compid, m)) in records.iter().enumerate() {
            let h = Header {
                seq: i as u8,
                sysid: 1,
                compid: *compid,
            };
            let f = encode(Version::V2, h, m).unwrap();
            w.write_raw(*us, f.as_bytes()).unwrap();
        }
        w.into_inner().unwrap()
    }

    fn hb(custom_mode: u32, armed: bool) -> Message {
        Message::Heartbeat(Heartbeat {
            r#type: 2,
            autopilot: 3,
            base_mode: if armed { 128 } else { 0 },
            custom_mode,
            ..Default::default()
        })
    }

    #[test]
    fn summarises_a_flight() {
        let mut text = [0u8; 50];
        text[..23].copy_from_slice(b"ArduCopter V4.7.1 (abc)");
        let banner = Message::Statustext(Statustext {
            severity: 6,
            text,
            ..Default::default()
        });
        let t0 = 1_760_000_000_000_000; // 2025-10-09 08:53:20 UTC
        let pos = |alt_mm| {
            Message::GlobalPositionInt(GlobalPositionInt {
                relative_alt: alt_mm,
                ..Default::default()
            })
        };
        let bytes = tlog(&[
            (t0, 1, hb(0, false)),
            (t0 + 100_000, 1, banner),
            (t0 + 1_000_000, 1, hb(4, false)),
            (t0 + 2_000_000, 1, hb(4, true)),
            (t0 + 3_000_000, 1, pos(15_500)),
            (t0 + 4_000_000, 1, pos(9_000)),
            (t0 + 5_000_000, 1, hb(6, true)),
            (t0 + 9_000_000, 1, hb(6, false)),
        ]);
        let s = summarise(&bytes[..]).unwrap();
        assert_eq!(s.records, 8);
        assert_eq!(s.span, Some((t0, t0 + 9_000_000)));
        assert_eq!(s.sources[&(1, 1)], 8);
        assert_eq!(s.messages["HEARTBEAT"], 5);
        assert_eq!(s.max_alt_rel_m[&1], 15.5);
        let f = &s.flights[&1];
        assert_eq!(
            (f.arms, f.armed_s),
            (1, 7.0),
            "armed at +2 s, disarmed at +9 s"
        );
        let texts: Vec<&str> = s.events.iter().map(|e| e.text.as_str()).collect();
        assert_eq!(
            texts,
            [
                "mode number 0",
                "disarmed",
                "sev6 ArduCopter V4.7.1 (abc)",
                "firmware Copter (banner)",
                "mode STABILIZE",
                "mode GUIDED",
                "ARMED",
                "mode RTL",
                "disarmed",
            ]
        );
        assert_eq!(s.stopped, None);
        assert_eq!(utc(t0), "2025-10-09 08:53:20.000");
    }

    #[test]
    fn extracts_params_and_the_last_complete_mission() {
        use mavlink_codec::messages::{MissionCount, MissionItemInt, ParamValue};
        let pv = |name: &str, v: f32| {
            Message::ParamValue(ParamValue {
                param_id: protocols::params::encode_id(name),
                param_value: v,
                param_type: 9,
                param_count: 2,
                param_index: 0,
            })
        };
        let item = |seq: u16, x: i32| {
            Message::MissionItemInt(MissionItemInt {
                seq,
                command: 16,
                frame: 3,
                x,
                y: 1_491_652_300,
                z: 20.0,
                ..Default::default()
            })
        };
        let count = |n: u16| {
            Message::MissionCount(MissionCount {
                count: n,
                ..Default::default()
            })
        };
        let bytes = tlog(&[
            (1, 1, pv("WP_SPD", 5.0)),
            (2, 1, pv("WP_SPD", 6.5)),
            (3, 1, pv("RTL_ALT", 1500.0)),
            (4, 1, count(2)),
            (5, 1, item(0, -353_632_610)),
            (6, 1, item(1, -353_622_610)),
            (7, 1, count(3)), // a later download, cut short: not complete
            (8, 1, item(0, -353_632_610)),
        ]);
        let (sys, p) = params(&bytes[..], None).unwrap().unwrap();
        assert_eq!(sys, 1);
        assert_eq!(p["WP_SPD"].value, 6.5, "last value wins");
        assert_eq!(p.len(), 2);
        let (from, m) = mission(&bytes[..], 0).unwrap().unwrap();
        assert_eq!((from, m.len()), ((1, 1), 2));
        assert_eq!(m[1].x, -353_622_610);
        // A later empty mission is the last complete one.
        let mut more = bytes.clone();
        more.extend(tlog(&[(9, 1, count(0))]));
        assert_eq!(mission(&more[..], 0).unwrap().unwrap().1.len(), 0);
        // A rally list (type 2) is kept apart from the mission.
        assert!(mission(&bytes[..], 2).unwrap().is_none());
        let rally = tlog(&[
            (
                10,
                1,
                Message::MissionCount(MissionCount {
                    count: 1,
                    mission_type: 2,
                    ..Default::default()
                }),
            ),
            (
                11,
                1,
                Message::MissionItemInt(MissionItemInt {
                    command: 5100,
                    frame: 3,
                    x: -353_620_000,
                    y: 1_491_630_000,
                    z: 25.0,
                    mission_type: 2,
                    ..Default::default()
                }),
            ),
        ]);
        let mut all = bytes.clone();
        all.extend(rally);
        let (_, r) = mission(&all[..], 2).unwrap().unwrap();
        assert_eq!((r.len(), r[0].command, r[0].z), (1, 5100, 25.0));
        assert_eq!(
            mission(&all[..], 0).unwrap().unwrap().1.len(),
            2,
            "mission unchanged"
        );
    }

    #[test]
    fn exports_one_message_type_as_csv() {
        let mut text = [0u8; 50];
        text[..12].copy_from_slice(b"say \"hi\", ok");
        let st = Message::Statustext(Statustext {
            severity: 4,
            text,
            ..Default::default()
        });
        let pos = Message::GlobalPositionInt(GlobalPositionInt {
            lat: -353_632_610,
            hdg: 9000,
            ..Default::default()
        });
        let bytes = tlog(&[(10, 1, hb(0, false)), (20, 1, st), (30, 1, pos)]);
        let (text, rows) = csv(&bytes[..], "statustext", None).unwrap();
        assert_eq!(rows, 1);
        let mut lines = text.lines();
        assert_eq!(
            lines.next(),
            Some("time_usec,sysid,compid,severity,text,id,chunk_seq")
        );
        assert_eq!(lines.next(), Some("20,1,1,4,\"say \"\"hi\"\", ok\",0,0"));
        let (text, _) = csv(&bytes[..], "GLOBAL_POSITION_INT", Some(1)).unwrap();
        assert!(
            text.lines()
                .nth(1)
                .unwrap()
                .starts_with("30,1,1,0,-353632610,"),
            "{text}"
        );
        assert_eq!(
            csv(&bytes[..], "HEARTBEAT", Some(2)).unwrap().1,
            0,
            "other system"
        );
        assert_eq!(
            csv_cell(&mavlink_codec::FieldValue::Float(f64::from(0.3f32))),
            "0.3"
        );
        assert_eq!(
            csv_cell(&mavlink_codec::FieldValue::UIntArray(vec![1, 2])),
            "\"1 2\""
        );
    }

    #[test]
    fn radio_status_over_a_recording() {
        use mavlink_codec::messages::RadioStatus;
        let rs = |rssi: u8, remrssi: u8, noise: u8, rxerrors: u16| {
            Message::RadioStatus(RadioStatus {
                rssi,
                remrssi,
                noise,
                remnoise: 40,
                rxerrors,
                fixed: 0,
                txbuf: 90,
            })
        };
        let bytes = tlog(&[
            (1, 68, rs(200, 190, 50, 3)),
            (2, 68, rs(120, 255, 60, 7)), // remote unknown this time
            (3, 68, rs(160, 170, 55, 2)), // radio rebooted: counter restarted
        ]);
        let s = summarise(&bytes[..]).unwrap();
        let r = &s.radios[&(1, 68)];
        assert_eq!(r.reports, 3);
        assert_eq!(r.rssi, Some((120, 160.0)));
        assert_eq!(r.remrssi, Some((170, 180.0)));
        assert_eq!((r.margin, r.rem_margin), (Some(60), Some(130)));
        assert_eq!(r.rx_errors, 4 + 2);
        assert_eq!(r.min_txbuf, Some(90));
    }

    #[test]
    fn heartbeat_gaps_are_link_drops() {
        let s = 1_000_000;
        // Heartbeats every second; a 3 s gap (STALE), a 7 s gap (LOST), then silence for 2 s
        // to the end of the recording (another vehicle's frame).
        let mut recs: Vec<(u64, u8, Message)> = [0, 1, 4, 11, 12]
            .iter()
            .map(|t| (t * s, 1, hb(0, false)))
            .collect();
        recs.push((14 * s, 2, hb(0, false))); // component 2: not the autopilot
        let sum = summarise(&tlog(&recs)[..]).unwrap();
        assert_eq!(
            sum.link_gaps[&1],
            [(s, 3.0), (4 * s, 7.0), (12 * s, 2.0)],
            "{:?}",
            sum.link_gaps
        );
    }

    #[test]
    fn counts_lost_frames_per_source() {
        let mut w = TlogWriter::new(Vec::new());
        // 0, 1, 2, then 5 (3 and 4 lost), 6, then 4 arrives late, then 6 again (duplicate).
        for (i, seq) in [0u8, 1, 2, 5, 6, 4, 6].iter().enumerate() {
            let h = Header {
                seq: *seq,
                sysid: 1,
                compid: 1,
            };
            let f = encode(Version::V2, h, &hb(0, false)).unwrap();
            w.write_raw(1_000 + i as u64, f.as_bytes()).unwrap();
        }
        let s = summarise(&w.into_inner().unwrap()[..]).unwrap();
        let q = s.sequences[&(1, 1)];
        assert_eq!((q.received, q.lost), (6, 1));
        assert_eq!(
            s.sources[&(1, 1)],
            7,
            "every record is still counted as a frame"
        );
        assert!((q.loss_pct().unwrap() - 100.0 / 7.0).abs() < 1e-9);
    }

    #[test]
    fn a_truncated_file_keeps_what_was_read() {
        let mut bytes = tlog(&[(1, 1, hb(0, false)), (2, 1, hb(0, false))]);
        bytes.truncate(bytes.len() - 3);
        let s = summarise(&bytes[..]).unwrap();
        assert_eq!(s.records, 1);
        assert!(s.stopped.is_some());
    }
}
