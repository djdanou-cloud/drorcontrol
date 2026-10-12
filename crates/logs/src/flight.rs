//! Flight-level views of a dataflash log: armed periods and a flight summary, an event
//! timeline, and parameters changed after boot.
//!
//! Record layouts (ArduPilot 4.7.1, `AP_Logger/LogStructure.h` and per-library files):
//! `ARM` = TimeUS, ArmState, ArmChecks, Forced, Method; `EV` = TimeUS, Id (`LogEvent` in
//! `AP_Logger.h`: 10 ARMED, 11 DISARMED, …); `MODE` = TimeUS, Mode, ModeNum, Rsn;
//! `POS` = TimeUS, Lat, Lng, Alt, RelHomeAlt, …; `GPS` = …, Spd; `BAT` = …, Volt, …,
//! CurrTot (mAh); `PARM` = TimeUS, Name, Value, Default; `MSG` = TimeUS, Message.

use std::collections::BTreeMap;

use crate::dataflash::{Kind, Log};
use crate::health::err_subsystem;

/// One armed period, µs since boot (`end` is `None` if the log ends armed).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmedPeriod {
    /// Armed at.
    pub start_us: u64,
    /// Disarmed at.
    pub end_us: Option<u64>,
}

/// Earliest timestamp among the records a log starts with (`PARM`, `ATT`, `POS`), µs.
fn log_start_us(log: &Log) -> u64 {
    ["PARM", "ATT", "POS"]
        .iter()
        .filter_map(|m| log.column(m, "TimeUS", None))
        .filter_map(|c| c.values.first().copied())
        .reduce(f64::min)
        .unwrap_or(0.0) as u64
}

/// Arm and disarm times: from `ARM` records, or `EV` 10/11 in logs without them. A log
/// whose first state change is a disarm was armed from its start: ArduPilot opens the log
/// on arming (unless `LOG_DISARMED`), and the arm record itself can precede the file.
pub fn armed_periods(log: &Log) -> Vec<ArmedPeriod> {
    let changes: Vec<(u64, bool)> = match log.column("ARM", "ArmState", None) {
        Some(c) if !c.values.is_empty() => c
            .time_us
            .iter()
            .zip(&c.values)
            .map(|(t, v)| (*t, *v != 0.0))
            .collect(),
        _ => log
            .column("EV", "Id", None)
            .map(|c| {
                c.time_us
                    .iter()
                    .zip(&c.values)
                    .filter(|(_, id)| **id == 10.0 || **id == 11.0)
                    .map(|(t, id)| (*t, *id == 10.0))
                    .collect()
            })
            .unwrap_or_default(),
    };
    let mut out: Vec<ArmedPeriod> = Vec::new();
    if changes.first().is_some_and(|(_, armed)| !armed) {
        out.push(ArmedPeriod {
            start_us: log_start_us(log),
            end_us: None,
        });
    }
    for (t, armed) in changes {
        match (armed, out.last_mut()) {
            (true, Some(p)) if p.end_us.is_none() => {} // already armed
            (true, _) => out.push(ArmedPeriod {
                start_us: t,
                end_us: None,
            }),
            (false, Some(p)) if p.end_us.is_none() => p.end_us = Some(t),
            (false, _) => {}
        }
    }
    out
}

/// What a flight log says about the flight.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FlightSummary {
    /// Armed periods.
    pub armed: Vec<ArmedPeriod>,
    /// Total armed time, s (to the end of the log for a period still open).
    pub armed_s: f64,
    /// Highest altitude above home (`POS.RelHomeAlt`), m.
    pub max_alt_rel_m: Option<f64>,
    /// Highest GPS ground speed (`GPS.Spd`, first GPS), m/s.
    pub max_speed_ms: Option<f64>,
    /// Horizontal distance flown while armed (`POS`), m.
    pub distance_m: f64,
    /// Charge used (`BAT.CurrTot`, first battery), mAh.
    pub used_mah: Option<f64>,
    /// Lowest measured battery voltage (first battery), V.
    pub min_voltage: Option<f64>,
}

fn distance_m(a: (f64, f64), b: (f64, f64)) -> f64 {
    const R: f64 = 6_371_008.8;
    let (la1, la2) = (a.0.to_radians(), b.0.to_radians());
    let h = ((la2 - la1) / 2.0).sin().powi(2)
        + la1.cos() * la2.cos() * ((b.1 - a.1).to_radians() / 2.0).sin().powi(2);
    2.0 * R * h.sqrt().asin()
}

/// Summary of the flight(s) in a log.
pub fn summary(log: &Log) -> FlightSummary {
    let armed = armed_periods(log);
    let end_of_log = log
        .column("POS", "TimeUS", None)
        .and_then(|c| c.values.last().copied())
        .unwrap_or(0.0) as u64;
    let in_flight = |t: u64| {
        armed
            .iter()
            .any(|p| t >= p.start_us && p.end_us.is_none_or(|e| t <= e))
    };
    let armed_s = armed.iter().fold(0.0, |s, p| {
        s + p.end_us.unwrap_or(end_of_log).saturating_sub(p.start_us) as f64 / 1e6
    });
    let max = |msg: &str, field: &str, inst: Option<(&str, f64)>| {
        log.column(msg, field, inst).and_then(|c| {
            c.values
                .into_iter()
                .filter(|v| v.is_finite())
                .reduce(f64::max)
        })
    };
    let mut distance_m_total = 0.0;
    if let (Some(lat), Some(lon)) = (
        log.column("POS", "Lat", None),
        log.column("POS", "Lng", None),
    ) {
        let mut prev: Option<(f64, f64)> = None;
        for ((la, lo), t) in lat.values.iter().zip(&lon.values).zip(&lat.time_us) {
            let here = (*la, *lo);
            if in_flight(*t) && !(*la == 0.0 && *lo == 0.0) {
                if let Some(p) = prev {
                    distance_m_total += distance_m(p, here);
                }
                prev = Some(here);
            } else {
                prev = None;
            }
        }
    }
    let bat = Some(("Inst", 0.0));
    let used_mah = log.column("BAT", "CurrTot", bat).and_then(|c| {
        let first = c.values.iter().copied().find(|v| v.is_finite())?;
        let last = c.values.iter().copied().rev().find(|v| v.is_finite())?;
        Some(last - first)
    });
    let min_voltage = log.column("BAT", "Volt", bat).and_then(|c| {
        c.values
            .into_iter()
            .filter(|v| v.is_finite() && *v > 0.1)
            .reduce(f64::min)
    });
    FlightSummary {
        armed,
        armed_s,
        max_alt_rel_m: max("POS", "RelHomeAlt", None),
        max_speed_ms: max("GPS", "Spd", Some(("I", 0.0))),
        distance_m: distance_m_total,
        used_mah,
        min_voltage,
    }
}

/// One timeline entry.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// Flight mode change: mode number and `ModeReason`.
    Mode {
        /// Mode number.
        mode: u8,
        /// Why (`ModeReason` number).
        reason: u8,
    },
    /// `ARM` record.
    Arm {
        /// Armed (true) or disarmed.
        armed: bool,
        /// Forced (checks skipped).
        forced: bool,
    },
    /// `EV` record (`LogEvent` id).
    Ev(u8),
    /// `ERR` record: subsystem name and code.
    Err(&'static str, u8),
    /// `MSG` text.
    Text(String),
}

fn text_field(log: &Log, msg: &str, field: &str, i: usize) -> Option<String> {
    let fmt = log.format_by_name(msg)?;
    let f = &fmt.fields[fmt.field(field)?];
    if f.kind != Kind::Text {
        return None;
    }
    let rec = log.record(msg, i)?;
    let raw = &rec[f.offset..f.offset + f.size];
    let end = raw.iter().position(|&c| c == 0).unwrap_or(raw.len());
    Some(String::from_utf8_lossy(&raw[..end]).into_owned())
}

/// `LogEvent` name for the ids worth showing (AP_Logger.h 4.7.1); others print as numbers.
pub fn ev_name(id: u8) -> Option<&'static str> {
    Some(match id {
        10 => "ARMED",
        11 => "DISARMED",
        15 => "AUTO_ARMED",
        17 => "LAND_COMPLETE_MAYBE",
        18 => "LAND_COMPLETE",
        19 => "LOST_GPS",
        25 => "SET_HOME",
        28 => "NOT_LANDED",
        _ => return None,
    })
}

/// Mode changes, arming, events, errors and messages, in time order (µs since boot).
pub fn events(log: &Log) -> Vec<(u64, Event)> {
    let mut out = Vec::new();
    let col = |m: &str, f: &str| log.column(m, f, None);
    if let (Some(m), Some(r)) = (col("MODE", "ModeNum"), col("MODE", "Rsn")) {
        for ((t, mode), reason) in m.time_us.iter().zip(&m.values).zip(&r.values) {
            out.push((
                *t,
                Event::Mode {
                    mode: *mode as u8,
                    reason: *reason as u8,
                },
            ));
        }
    }
    if let (Some(a), Some(f)) = (col("ARM", "ArmState"), col("ARM", "Forced")) {
        for ((t, armed), forced) in a.time_us.iter().zip(&a.values).zip(&f.values) {
            out.push((
                *t,
                Event::Arm {
                    armed: *armed != 0.0,
                    forced: *forced != 0.0,
                },
            ));
        }
    }
    if let Some(e) = col("EV", "Id") {
        out.extend(
            e.time_us
                .iter()
                .zip(&e.values)
                .map(|(t, id)| (*t, Event::Ev(*id as u8))),
        );
    }
    if let (Some(s), Some(c)) = (col("ERR", "Subsys"), col("ERR", "ECode")) {
        for ((t, sub), code) in s.time_us.iter().zip(&s.values).zip(&c.values) {
            out.push((*t, Event::Err(err_subsystem(*sub as u8), *code as u8)));
        }
    }
    if let Some(m) = col("MSG", "TimeUS") {
        for (i, t) in m.values.iter().enumerate() {
            if let Some(text) = text_field(log, "MSG", "Message", i) {
                out.push((*t as u64, Event::Text(text)));
            }
        }
    }
    out.sort_by_key(|(t, _)| *t);
    out
}

/// A parameter whose logged value changed after it was first logged.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamChange {
    /// When, µs since boot.
    pub time_us: u64,
    /// Parameter name.
    pub name: String,
    /// Value before and after.
    pub old: f64,
    /// See `old`.
    pub new: f64,
    /// A value the autopilot updates itself, not a change someone made: the `AP_Stats`
    /// counters (`STAT_*` except `STAT_RESET`, which a user sets), and `MOT_THST_HOVER`,
    /// which Copter learns and saves at disarm when `MOT_HOVER_LEARN` is 2 (the default;
    /// `AP_MotorsMulticopter::save_params_on_disarm`).
    pub automatic: bool,
}

/// Parameters the autopilot writes itself (4.7.1).
fn automatic(name: &str) -> bool {
    matches!(
        name,
        "STAT_BOOTCNT"
            | "STAT_FLTTIME"
            | "STAT_RUNTIME"
            | "STAT_FLTCNT"
            | "STAT_DISTFLWN"
            | "MOT_THST_HOVER"
    )
}

/// Parameters changed during the log: every `PARM` record whose value differs from the same
/// parameter's previous record (ArduPilot logs all parameters at start and each change).
pub fn param_changes(log: &Log) -> Vec<ParamChange> {
    let Some(values) = log.column("PARM", "Value", None) else {
        return Vec::new();
    };
    let mut last: BTreeMap<String, f64> = BTreeMap::new();
    let mut out = Vec::new();
    for (i, (v, t)) in values.values.iter().zip(&values.time_us).enumerate() {
        let Some(name) = text_field(log, "PARM", "Name", i) else {
            continue;
        };
        if let Some(old) = last.insert(name.clone(), *v)
            && (old as f32) != (*v as f32)
        {
            out.push(ParamChange {
                time_us: *t,
                automatic: automatic(&name),
                name,
                old,
                new: *v,
            });
        }
    }
    out
}
