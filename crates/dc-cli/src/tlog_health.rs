//! Health checks on a `.tlog`, for when only the telemetry recording is at hand (no dataflash
//! log): the same checks and thresholds as `logs::health`, from the MAVLink messages the
//! vehicle streamed. Judged while armed (`HEARTBEAT` base_mode `MAV_MODE_FLAG_SAFETY_ARMED`)
//! for the first autopilot in the recording.
//!
//! * `VIBRATION` (m/s², cumulative clipping counts) against the vibration guidance;
//! * battery: `SYS_STATUS.voltage_battery` (mV) against `BATT_LOW_VOLT`/`BATT_CRT_VOLT`;
//! * `EKF_STATUS_REPORT` velocity, horizontal position and compass variances against
//!   `FS_EKF_THRESH` (the variances Copter's EKF check compares, `ekf_check.cpp`);
//! * `GPS_RAW_INT` after the first 3D fix: HDOP (`eph`, ×100) against `GPS_HDOP_GOOD`, fix
//!   losses;
//! * `STATUSTEXT` errors (severity 0–3) and arming-check failures.
//!
//! Parameters come from `PARAM_VALUE` in the recording (a parameter download, or the values
//! the autopilot announces); without them the defaults named in `logs::health` apply. The
//! telemetry rate is lower than the dataflash rate, so short peaks can be missed.

use std::collections::BTreeMap;
use std::io::{self, Read};

use logs::health::{Finding, Level, battery_level, ekf_level, gps_level, vibration_level};
use mavlink_codec::Message;
use recorder::TlogReader;

const ARMED: u8 = 128;
const AUTOPILOT_INVALID: u8 = 8;

fn text(b: &[u8]) -> String {
    let end = b.iter().position(|c| *c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).into_owned()
}

#[derive(Default)]
struct State {
    vehicle: Option<u8>,
    armed: bool,
    params: BTreeMap<String, f64>,
    vibe: Option<(f64, u64)>,
    clips: [Option<(u32, u32)>; 3],
    volts: Option<(f64, u64)>,
    ekf: Option<(f64, u64, &'static str)>,
    gps_fixed: bool,
    gps_losses: Vec<u64>,
    last_fix: u8,
    hdop: Option<(f64, u64)>,
    texts: Vec<(u64, u8, String)>,
}

fn max_at(cur: &mut Option<(f64, u64)>, v: f64, t: u64) {
    if cur.is_none_or(|c| v > c.0) {
        *cur = Some((v, t));
    }
}

/// Health findings for the first autopilot in a `.tlog`.
pub fn assess(input: impl Read) -> io::Result<Vec<Finding>> {
    let mut r = TlogReader::new(input);
    let mut s = State::default();
    let mut first = None::<u64>;
    while let Some(rec) = r.next_record()? {
        let t = rec.unix_us;
        first.get_or_insert(t);
        let f = &rec.frame;
        let Ok(m) = f.decode() else { continue };
        if let Message::Heartbeat(h) = &m
            && f.compid() == 1
            && h.autopilot != AUTOPILOT_INVALID
        {
            s.vehicle.get_or_insert(f.sysid());
            if s.vehicle == Some(f.sysid()) {
                s.armed = h.base_mode & ARMED != 0;
            }
        }
        if s.vehicle != Some(f.sysid()) {
            continue;
        }
        match &m {
            Message::ParamValue(p) => {
                s.params.insert(text(&p.param_id), f64::from(p.param_value));
            }
            Message::Statustext(st) => s.texts.push((t, st.severity, text(&st.text))),
            _ if !s.armed => {}
            Message::Vibration(v) => {
                let worst = v.vibration_x.max(v.vibration_y).max(v.vibration_z);
                max_at(&mut s.vibe, f64::from(worst), t);
                for (k, c) in [v.clipping_0, v.clipping_1, v.clipping_2]
                    .iter()
                    .enumerate()
                {
                    let e = s.clips[k].get_or_insert((*c, *c));
                    e.1 = *c;
                }
            }
            Message::SysStatus(st) if st.voltage_battery != u16::MAX && st.voltage_battery > 0 => {
                let v = f64::from(st.voltage_battery) / 1000.0;
                if s.volts.is_none_or(|c| v < c.0) {
                    s.volts = Some((v, t));
                }
            }
            Message::EkfStatusReport(e) => {
                for (v, name) in [
                    (e.velocity_variance, "velocity"),
                    (e.pos_horiz_variance, "position"),
                    (e.compass_variance, "compass"),
                ] {
                    let v = f64::from(v);
                    if s.ekf.is_none_or(|c| v > c.0) {
                        s.ekf = Some((v, t, name));
                    }
                }
            }
            Message::GpsRawInt(g) => {
                if g.fix_type >= 3 {
                    s.gps_fixed = true;
                    if g.eph != u16::MAX {
                        max_at(&mut s.hdop, f64::from(g.eph) / 100.0, t);
                    }
                } else if s.gps_fixed && s.last_fix >= 3 {
                    s.gps_losses.push(t);
                }
                s.last_fix = g.fix_type;
            }
            _ => {}
        }
    }
    let t0 = first.unwrap_or(0);
    let at = |t: u64| Some(t.saturating_sub(t0));
    let mut out = Vec::new();
    if let Some((v, t)) = s.vibe {
        out.push(Finding {
            check: "vibration",
            level: vibration_level(v),
            text: format!(
                "highest vibration {v:.1} m/s² (guidance: < 30 fine, > 60 nearly always a problem)"
            ),
            at_us: at(t),
        });
    }
    for (k, c) in s.clips.iter().enumerate() {
        if let Some((a, b)) = c {
            let n = b.saturating_sub(*a);
            out.push(Finding {
                check: "clipping",
                level: if n > 0 { Level::Warning } else { Level::Ok },
                text: format!("accelerometer {k} clipped {n} times while armed"),
                at_us: None,
            });
        }
    }
    if let Some((v, t)) = s.volts {
        let p = |n: &str| s.params.get(n).copied().filter(|v| *v > 0.0);
        let (level, why) = battery_level(v, p("BATT_LOW_VOLT"), p("BATT_CRT_VOLT"));
        out.push(Finding {
            check: "battery",
            level,
            text: format!("lowest {v:.2} V while armed, {why}"),
            at_us: at(t),
        });
    }
    if let Some((v, t, name)) = s.ekf {
        let thresh = s.params.get("FS_EKF_THRESH").copied().unwrap_or(0.8);
        out.push(Finding {
            check: "ekf",
            level: ekf_level(v, thresh),
            text: format!("highest {name} variance {v:.2} (FS_EKF_THRESH {thresh:.2})"),
            at_us: at(t),
        });
    }
    if let Some((h, t)) = s.hdop {
        let good = s.params.get("GPS_HDOP_GOOD").copied().unwrap_or(140.0);
        out.push(Finding {
            check: "gps",
            level: gps_level(h, s.gps_losses.len(), good),
            text: format!(
                "highest HDOP {h:.2} while armed (GPS_HDOP_GOOD {good:.0}), {} fix losses",
                s.gps_losses.len()
            ),
            at_us: s.gps_losses.first().map_or(at(t), |l| at(*l)),
        });
    }
    let texts: Vec<(u64, String)> = s.texts.iter().map(|(t, _, x)| (*t, x.clone())).collect();
    for (text, t, n) in logs::health::arming_messages(&texts) {
        out.push(Finding {
            check: "arming",
            level: Level::Caution,
            text: if n > 1 {
                format!("{text} ({n} times)")
            } else {
                text
            },
            at_us: at(t),
        });
    }
    for (t, sev, text) in s.texts.iter().filter(|(_, sev, _)| *sev <= 3) {
        out.push(Finding {
            check: "error",
            level: Level::Warning,
            text: format!("STATUSTEXT severity {sev}: {text}"),
            at_us: at(*t),
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::messages::{
        EkfStatusReport, GpsRawInt, Heartbeat, ParamValue, Statustext, SysStatus, Vibration,
    };
    use mavlink_codec::{Header, Version, encode};
    use recorder::{TlogWriter, from_unix_micros};

    fn tlog(msgs: &[Message]) -> Vec<u8> {
        let mut w = TlogWriter::new(Vec::new());
        for (i, m) in msgs.iter().enumerate() {
            let h = Header {
                seq: i as u8,
                sysid: 1,
                compid: 1,
            };
            let f = encode(Version::V2, h, m).unwrap();
            w.write(from_unix_micros(1_000_000 * (i as u64 + 1)), &f)
                .unwrap();
        }
        w.into_inner().unwrap()
    }

    fn hb(armed: bool) -> Message {
        Message::Heartbeat(Heartbeat {
            autopilot: 3,
            base_mode: if armed { ARMED } else { 0 },
            ..Default::default()
        })
    }

    #[test]
    fn findings_while_armed_against_the_vehicle_parameters() {
        let mut id = [0u8; 16];
        id[..13].copy_from_slice(b"BATT_LOW_VOLT");
        let mut txt = [0u8; 50];
        txt[..17].copy_from_slice(b"PreArm: Throttle!");
        let input = tlog(&[
            hb(false),
            Message::ParamValue(ParamValue {
                param_id: id,
                param_value: 10.5,
                ..Default::default()
            }),
            Message::Statustext(Statustext {
                severity: 2,
                text: txt,
                ..Default::default()
            }),
            Message::SysStatus(SysStatus {
                voltage_battery: 9000, // disarmed: not counted
                ..Default::default()
            }),
            hb(true),
            Message::SysStatus(SysStatus {
                voltage_battery: 10_400,
                ..Default::default()
            }),
            Message::Vibration(Vibration {
                vibration_z: 35.0,
                clipping_0: 4,
                ..Default::default()
            }),
            Message::Vibration(Vibration {
                vibration_z: 12.0,
                clipping_0: 6,
                ..Default::default()
            }),
            Message::EkfStatusReport(EkfStatusReport {
                compass_variance: 0.6,
                ..Default::default()
            }),
            Message::GpsRawInt(GpsRawInt {
                fix_type: 3,
                eph: 90,
                ..Default::default()
            }),
            Message::GpsRawInt(GpsRawInt {
                fix_type: 1,
                eph: 9999,
                ..Default::default()
            }),
        ]);
        let f = assess(&input[..]).unwrap();
        let get = |c: &str| f.iter().find(|x| x.check == c).unwrap();
        assert_eq!(get("vibration").level, Level::Caution);
        assert_eq!(get("clipping").level, Level::Warning);
        assert!(get("clipping").text.contains("clipped 2 times"));
        assert_eq!(get("battery").level, Level::Caution, "{:?}", get("battery"));
        assert!(get("battery").text.contains("10.40 V"));
        assert_eq!(get("ekf").level, Level::Caution);
        assert_eq!(get("gps").level, Level::Warning, "a fix loss");
        assert_eq!(get("arming").text, "PreArm: Throttle!");
        assert_eq!(get("error").level, Level::Warning);
    }
}
