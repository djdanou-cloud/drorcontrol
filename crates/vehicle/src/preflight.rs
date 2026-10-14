//! Pre-flight checklist from what the vehicle reports: the automatic part of a checklist
//! (Mission Planner's "Pre-Flight" checks), shown before arming. Advice only: nothing here
//! arms, refuses or changes anything; the autopilot's own pre-arm checks stay the arming
//! gate (ADR-0011).

use std::time::{Duration, Instant};

use crate::nav::{NavHealth, PositionTrust};
use crate::{Threat, VehicleState};

/// Result of one item.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Status {
    /// Fine.
    Pass,
    /// Worth knowing; not a reason to stop.
    Info,
    /// Not reported (yet): check by other means.
    Unknown,
    /// Do not fly until this is understood.
    Fail,
}

/// One checklist line.
#[derive(Debug, Clone, PartialEq)]
pub struct Item {
    /// Short name.
    pub name: &'static str,
    /// Result.
    pub status: Status,
    /// What was seen.
    pub detail: String,
}

/// GPS: fewest satellites for a pass.
pub const MIN_SATELLITES: u8 = 6;
/// GPS: highest HDOP for a pass (ArduPilot's `GPS_HDOP_GOOD` default, 140 = 1.4).
pub const MAX_HDOP: f32 = 1.4;
/// Battery: lowest remaining capacity for a pass, percent.
pub const MIN_BATTERY_PCT: u8 = 30;
/// Vibration: highest level for a pass, m/s² (the level ArduPilot's own vibration check and
/// the log health check treat as too high).
pub const MAX_VIBRATION: f32 = 30.0;
/// A message older than this counts as not reported.
pub const STALE: Duration = Duration::from_secs(5);

/// The checklist for `v` at `now`; `nav` is its navigation health
/// ([`crate::nav::assess`]).
pub fn checklist(v: &VehicleState, nav: &NavHealth, now: Instant) -> Vec<Item> {
    let fresh = |at: Instant| now.saturating_duration_since(at) < STALE;
    let mut out = Vec::new();
    let mut push = |name, status, detail: String| {
        out.push(Item {
            name,
            status,
            detail,
        })
    };

    match &v.heartbeat {
        Some(h) if fresh(h.at) => {
            let armed = h.value.armed();
            push(
                "link",
                Status::Pass,
                format!("heartbeat {:.1} s ago", h.age(now).as_secs_f32()),
            );
            push(
                "disarmed",
                if armed { Status::Info } else { Status::Pass },
                if armed {
                    "ARMED".into()
                } else {
                    "disarmed".into()
                },
            );
        }
        Some(h) => push(
            "link",
            Status::Fail,
            format!("no heartbeat for {:.0} s", h.age(now).as_secs_f32()),
        ),
        None => push("link", Status::Fail, "no heartbeat".into()),
    }
    push(
        "firmware",
        match v.firmware_tested() {
            Some(true) => Status::Pass,
            Some(false) => Status::Info,
            None => Status::Unknown,
        },
        match (&v.autopilot_version, v.firmware_tested()) {
            (Some(a), Some(true)) => format!("{} (covered by the tests)", a.value.version_string()),
            (Some(a), Some(false)) => format!(
                "{}: not covered by DrorControl's tests (ArduCopter 4.6 and 4.7)",
                a.value.version_string()
            ),
            _ => "version not reported yet".into(),
        },
    );
    match v.sensors.as_ref().filter(|s| fresh(s.at)).map(|s| s.value) {
        Some(s) => {
            push(
                "pre-arm checks",
                match s.prearm_ok() {
                    Some(true) => Status::Pass,
                    Some(false) => Status::Fail,
                    None => Status::Info,
                },
                match s.prearm_ok() {
                    Some(true) => "the autopilot's pre-arm checks pass".into(),
                    Some(false) => {
                        "the autopilot's pre-arm checks fail (see its PreArm messages)".into()
                    }
                    None => "arming checks disabled on the vehicle (ARMING_CHECK 0)".into(),
                },
            );
            let bad = s.unhealthy();
            push(
                "sensors",
                if bad.is_empty() {
                    Status::Pass
                } else {
                    Status::Fail
                },
                if bad.is_empty() {
                    "all enabled sensors healthy".into()
                } else {
                    format!("unhealthy: {}", bad.join(", "))
                },
            );
            let rc = s.present & crate::SENSOR_RC_RECEIVER != 0;
            push(
                "RC receiver",
                match (rc, s.health & crate::SENSOR_RC_RECEIVER != 0) {
                    (true, true) => Status::Pass,
                    (true, false) => Status::Fail,
                    (false, _) => Status::Info,
                },
                match (rc, s.health & crate::SENSOR_RC_RECEIVER != 0) {
                    (true, true) => "receiving".into(),
                    (true, false) => "in RC failsafe".into(),
                    (false, _) => "no RC input seen since boot".into(),
                },
            );
        }
        None => push(
            "pre-arm checks",
            Status::Unknown,
            "SYS_STATUS not received".into(),
        ),
    }
    match v.gps.as_ref().filter(|g| fresh(g.at)).map(|g| g.value) {
        Some(g) => {
            let sats = g.satellites.unwrap_or(0);
            let hdop = g.hdop.unwrap_or(f32::INFINITY);
            let ok = g.fix_type >= 3 && sats >= MIN_SATELLITES && hdop <= MAX_HDOP;
            push(
                "GPS",
                if ok { Status::Pass } else { Status::Fail },
                format!(
                    "fix type {}, {sats} satellites, HDOP {} (want 3D, {MIN_SATELLITES}+, ≤ {MAX_HDOP})",
                    g.fix_type,
                    g.hdop.map_or("?".into(), |h| format!("{h:.2}"))
                ),
            );
        }
        None => push("GPS", Status::Unknown, "GPS_RAW_INT not received".into()),
    }
    push(
        "position (EKF)",
        match nav.trust {
            PositionTrust::Good => Status::Pass,
            PositionTrust::Unknown => Status::Unknown,
            _ => Status::Fail,
        },
        if nav.reasons.is_empty() {
            format!("{:?}", nav.trust).to_lowercase()
        } else {
            format!("{:?}: {}", nav.trust, nav.reasons.join(", ")).to_lowercase()
        },
    );
    push(
        "home",
        if v.home.is_some() {
            Status::Pass
        } else {
            Status::Fail
        },
        v.home.as_ref().map_or("not set".into(), |h| {
            format!("{:.7},{:.7}", h.value.lat_deg, h.value.lon_deg)
        }),
    );
    match v.power.as_ref().filter(|p| fresh(p.at)).map(|p| p.value) {
        Some(p) => {
            let detail = format!(
                "{}{}",
                p.voltage
                    .map_or("voltage ?".into(), |v| format!("{v:.2} V")),
                p.remaining_pct
                    .map_or(String::new(), |r| format!(", {r} %"))
            );
            push(
                "battery",
                match p.remaining_pct {
                    Some(r) if r < MIN_BATTERY_PCT => Status::Fail,
                    Some(_) => Status::Pass,
                    None => Status::Info,
                },
                if p.remaining_pct.is_none() {
                    format!("{detail} (no capacity estimate: check the voltage)")
                } else {
                    detail
                },
            );
        }
        None => push("battery", Status::Unknown, "not reported".into()),
    }
    match v
        .vibration
        .as_ref()
        .filter(|x| fresh(x.at))
        .map(|x| x.value)
    {
        Some(x) => {
            let worst = x.levels.iter().copied().fold(0.0, f32::max);
            push(
                "vibration",
                if worst < MAX_VIBRATION {
                    Status::Pass
                } else {
                    Status::Fail
                },
                format!("{worst:.1} m/s² (limit {MAX_VIBRATION})"),
            );
        }
        None => push(
            "vibration",
            Status::Unknown,
            "VIBRATION not received".into(),
        ),
    }
    push(
        "mission",
        Status::Info,
        match v.mission.as_ref().and_then(|m| m.value.total) {
            Some(0) | None => "no mission on the vehicle".into(),
            Some(n) => format!("{n} items on the vehicle"),
        },
    );
    push(
        "fence",
        match v.fence.as_ref().filter(|f| fresh(f.at)) {
            Some(f) if f.value.breached => Status::Fail,
            _ => Status::Info,
        },
        match v.fence.as_ref().filter(|f| fresh(f.at)) {
            Some(f) if f.value.breached => format!("BREACHED ({})", f.value.breach_type.label()),
            Some(_) => "enabled, inside".into(),
            None => "not enabled (no FENCE_STATUS)".into(),
        },
    );
    let threats: Vec<String> = v
        .position
        .as_ref()
        .map(|own| {
            v.traffic
                .values()
                .filter(|t| t.value.approach(&own.value).threat > Threat::None)
                .map(|t| {
                    t.value
                        .callsign
                        .clone()
                        .unwrap_or_else(|| format!("{:06X}", t.value.icao))
                })
                .collect()
        })
        .unwrap_or_default();
    push(
        "traffic",
        if threats.is_empty() {
            Status::Pass
        } else {
            Status::Fail
        },
        if threats.is_empty() {
            format!("{} aircraft, none converging", v.traffic.len())
        } else {
            format!("converging: {}", threats.join(", "))
        },
    );
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Fleet, nav::NavThresholds};
    use mavlink_codec::Message;
    use mavlink_codec::messages::{GpsRawInt, Heartbeat, SysStatus, Vibration};

    fn status(items: &[Item], name: &str) -> Status {
        items.iter().find(|i| i.name == name).unwrap().status
    }

    #[test]
    fn checklist_from_reported_state() {
        let mut f = Fleet::new();
        let now = Instant::now();
        let apply = |f: &mut Fleet, m: Message| f.apply(1, 1, &m, now);
        apply(
            &mut f,
            Message::Heartbeat(Heartbeat {
                r#type: 2,
                autopilot: 3,
                ..Default::default()
            }),
        );
        let present = (1 << 28) | (1 << 2) | (1 << 16);
        apply(
            &mut f,
            Message::SysStatus(SysStatus {
                onboard_control_sensors_present: present,
                onboard_control_sensors_enabled: present,
                // Compass unhealthy, pre-arm failing, RC fine.
                onboard_control_sensors_health: 1 << 16,
                voltage_battery: 12_100,
                current_battery: -1,
                battery_remaining: 22,
                ..Default::default()
            }),
        );
        apply(
            &mut f,
            Message::GpsRawInt(GpsRawInt {
                fix_type: 3,
                satellites_visible: 11,
                eph: 90,
                ..Default::default()
            }),
        );
        apply(
            &mut f,
            Message::Vibration(Vibration {
                vibration_x: 8.0,
                vibration_y: 9.0,
                vibration_z: 35.0,
                ..Default::default()
            }),
        );
        let v = f.get(1).unwrap();
        let nav = crate::nav::assess(v, &NavThresholds::default(), now);
        let c = checklist(v, &nav, now);
        assert_eq!(status(&c, "link"), Status::Pass);
        assert_eq!(status(&c, "disarmed"), Status::Pass);
        assert_eq!(status(&c, "pre-arm checks"), Status::Fail);
        let sensors = c.iter().find(|i| i.name == "sensors").unwrap();
        assert_eq!(
            (sensors.status, sensors.detail.as_str()),
            (Status::Fail, "unhealthy: 3D_MAG")
        );
        assert_eq!(status(&c, "RC receiver"), Status::Pass);
        assert_eq!(status(&c, "GPS"), Status::Pass);
        assert_eq!(status(&c, "home"), Status::Fail);
        assert_eq!(status(&c, "battery"), Status::Fail, "22 % < 30 %");
        assert_eq!(status(&c, "vibration"), Status::Fail, "35 > 30");
        assert_eq!(status(&c, "fence"), Status::Info);
        assert_eq!(status(&c, "traffic"), Status::Pass);
        // Ten seconds later nothing new arrived: the link fails, readings are unknown.
        let later = checklist(v, &nav, now + Duration::from_secs(10));
        assert_eq!(status(&later, "link"), Status::Fail);
        assert_eq!(status(&later, "GPS"), Status::Unknown);
    }
}
