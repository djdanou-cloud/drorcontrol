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

/// Periods in the air, µs since boot: armed, between Copter's land detector reporting
/// `NOT_LANDED` (EV 28) and `LAND_COMPLETE` (EV 18) (`Copter::set_land_complete`). The bool
/// is false when the log has no land-detector events and the armed periods are used instead.
pub fn flying_periods(log: &Log) -> (Vec<ArmedPeriod>, bool) {
    let armed = armed_periods(log);
    let land: Vec<(u64, bool)> = log
        .column("EV", "Id", None)
        .map(|c| {
            c.time_us
                .iter()
                .zip(&c.values)
                .filter(|(_, id)| **id == 18.0 || **id == 28.0)
                .map(|(t, id)| (*t, *id == 28.0))
                .collect()
        })
        .unwrap_or_default();
    if !land.iter().any(|(_, flying)| *flying) {
        return (armed, false);
    }
    let end = log_end_us(log);
    let mut air: Vec<(u64, u64)> = Vec::new();
    let mut since: Option<u64> = None;
    for (t, flying) in land {
        match (flying, since) {
            (true, None) => since = Some(t),
            (false, Some(s)) => {
                air.push((s, t));
                since = None;
            }
            _ => {}
        }
    }
    if let Some(s) = since {
        air.push((s, end));
    }
    let mut out = Vec::new();
    for p in &armed {
        let pe = p.end_us.unwrap_or(end);
        for (a, b) in &air {
            let (s, e) = (p.start_us.max(*a), pe.min(*b));
            if e > s {
                out.push(ArmedPeriod {
                    start_us: s,
                    end_us: Some(e),
                });
            }
        }
    }
    (out, true)
}

/// Whether `t` falls in one of `periods` (an open period runs to the end of the log).
pub fn in_periods(periods: &[ArmedPeriod], t: u64) -> bool {
    periods
        .iter()
        .any(|p| t >= p.start_us && p.end_us.is_none_or(|e| t <= e))
}

/// `CTUN.CRt`, the climb rate, in m/s: Copter 4.7 logs it as float m/s, 4.6 and earlier as
/// int16 cm/s (the owner's 4.6.2 log).
pub fn climb_rate(log: &Log) -> Option<crate::dataflash::Column> {
    let fmt = log.format_by_name("CTUN")?;
    let scale = match fmt.fields.get(fmt.field("CRt")?)?.kind {
        Kind::F32 | Kind::F64 => 1.0,
        _ => 0.01,
    };
    let mut c = log.column("CTUN", "CRt", None)?;
    for v in &mut c.values {
        *v *= scale;
    }
    Some(c)
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
    /// Furthest horizontal distance from home while armed (`POS` against the latest `ORGN`
    /// home, type 1, at that time), m.
    pub max_from_home_m: Option<f64>,
    /// Fastest climb and descent while flying (`CTUN.CRt`), m/s (descent positive).
    pub max_climb_ms: Option<f64>,
    /// See `max_climb_ms`.
    pub max_descent_ms: Option<f64>,
}

fn distance_m(a: (f64, f64), b: (f64, f64)) -> f64 {
    const R: f64 = 6_371_008.8;
    let (la1, la2) = (a.0.to_radians(), b.0.to_radians());
    let h = ((la2 - la1) / 2.0).sin().powi(2)
        + la1.cos() * la2.cos() * ((b.1 - a.1).to_radians() / 2.0).sin().powi(2);
    2.0 * R * h.sqrt().asin()
}

/// Latest `TimeUS` in the log (the last record of each message type), µs.
pub(crate) fn log_end_us(log: &Log) -> u64 {
    log.counts()
        .iter()
        .filter(|(_, n)| **n > 0)
        .filter_map(|(name, n)| {
            let fmt = log.format_by_name(name)?;
            fmt.u64_value(log.record(name, n - 1)?, fmt.field("TimeUS")?)
        })
        .max()
        .unwrap_or(0)
}

/// Summary of the flight(s) in a log.
pub fn summary(log: &Log) -> FlightSummary {
    let armed = armed_periods(log);
    let end_of_log = log_end_us(log);
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
    // Home: ORGN type 1 (`AP_AHRS::LogOriginType::ahrs_home`); Copter sets it again at arming.
    let homes: Vec<(u64, (f64, f64))> = match (
        log.column("ORGN", "Type", None),
        log.column("ORGN", "Lat", None),
        log.column("ORGN", "Lng", None),
    ) {
        (Some(ty), Some(la), Some(lo)) => ty
            .time_us
            .iter()
            .enumerate()
            .filter(|(i, _)| ty.values[*i] == 1.0)
            .map(|(i, t)| (*t, (la.values[i], lo.values[i])))
            .collect(),
        _ => Vec::new(),
    };
    let mut max_from_home_m: Option<f64> = None;
    if let (Some(lat), Some(lon)) = (
        log.column("POS", "Lat", None),
        log.column("POS", "Lng", None),
    ) {
        let mut h = 0;
        for ((la, lo), t) in lat.values.iter().zip(&lon.values).zip(&lat.time_us) {
            while h + 1 < homes.len() && homes[h + 1].0 <= *t {
                h += 1;
            }
            let Some(home) = homes.get(h).filter(|home| home.0 <= *t) else {
                continue;
            };
            if in_flight(*t) && !(*la == 0.0 && *lo == 0.0) {
                let d = distance_m(home.1, (*la, *lo));
                max_from_home_m = Some(max_from_home_m.map_or(d, |m| m.max(d)));
            }
        }
    }
    let (flying, _) = flying_periods(log);
    let rates: Vec<f64> = climb_rate(log)
        .map(|c| {
            c.time_us
                .iter()
                .zip(&c.values)
                .filter(|(t, v)| in_periods(&flying, **t) && v.is_finite())
                .map(|(_, v)| *v)
                .collect()
        })
        .unwrap_or_default();
    let max_climb_ms = rates.iter().copied().reduce(f64::max).map(|v| v.max(0.0));
    let max_descent_ms = rates
        .iter()
        .copied()
        .reduce(f64::min)
        .map(|v| (-v).max(0.0));
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
        max_from_home_m,
        max_climb_ms,
        max_descent_ms,
    }
}

/// Armed time in each flight mode (`MODE.ModeNum`), s, in order of first use. Time before
/// the first `MODE` record is not counted (its mode is unknown).
pub fn mode_times(log: &Log) -> Vec<(u8, f64)> {
    let Some(modes) = log.column("MODE", "ModeNum", None) else {
        return Vec::new();
    };
    let armed = armed_periods(log);
    let end = log_end_us(log);
    let mut out: Vec<(u8, f64)> = Vec::new();
    for (i, (t, m)) in modes.time_us.iter().zip(&modes.values).enumerate() {
        let next = modes.time_us.get(i + 1).copied().unwrap_or(end).max(*t);
        let s = armed.iter().fold(0.0, |s, p| {
            let (a, b) = (p.start_us.max(*t), p.end_us.unwrap_or(end).min(next));
            s + b.saturating_sub(a) as f64 / 1e6
        });
        let mode = *m as u8;
        match out.iter_mut().find(|(k, _)| *k == mode) {
            Some(e) => e.1 += s,
            None => out.push((mode, s)),
        }
    }
    out
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{
        LogBuilder,
        V::{N, T},
    };

    #[test]
    fn mode_time_counts_only_armed_time() {
        let mut b = LogBuilder::default();
        b.format("ATT", "Qf", "TimeUS,Roll")
            .format("MODE", "QMBB", "TimeUS,Mode,ModeNum,Rsn")
            .format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method");
        let s = |x: f64| x * 1e6;
        // STABILIZE (0) from 1 s; armed at 2 s; LOITER (5) at 5 s; AUTO (3) at 9 s;
        // disarmed at 12 s; back to STABILIZE at 13 s; log ends at 20 s.
        b.rec("ATT", &[N(s(0.5)), N(0.0)])
            .rec("MODE", &[N(s(1.0)), N(0.0), N(0.0), N(0.0)])
            .rec("ARM", &[N(s(2.0)), N(1.0), N(0.0), N(0.0), N(0.0)])
            .rec("MODE", &[N(s(5.0)), N(5.0), N(5.0), N(1.0)])
            .rec("MODE", &[N(s(9.0)), N(3.0), N(3.0), N(1.0)])
            .rec("ARM", &[N(s(12.0)), N(0.0), N(0.0), N(0.0), N(0.0)])
            .rec("MODE", &[N(s(13.0)), N(0.0), N(0.0), N(1.0)])
            .rec("ATT", &[N(s(20.0)), N(0.0)]);
        let log = b.open();
        let t = mode_times(&log);
        let want = [(0u8, 3.0), (5, 4.0), (3, 3.0)];
        assert_eq!(t.len(), 3, "{t:?}");
        for ((m, got), (wm, w)) in t.iter().zip(want) {
            assert_eq!(*m, wm);
            assert!((got - w).abs() < 1e-6, "{t:?}");
        }
        assert!((summary(&log).armed_s - 10.0).abs() < 1e-6);
    }

    #[test]
    fn param_changes_skip_unchanged_and_mark_automatic() {
        let mut b = LogBuilder::default();
        b.format("PARM", "QNff", "TimeUS,Name,Value,Default");
        for (t, name, v) in [
            (1.0, "WPNAV_SPEED", 1000.0),
            (1.0, "STAT_BOOTCNT", 3.0),
            (5.0, "WPNAV_SPEED", 1000.0),
            (6.0, "WPNAV_SPEED", 500.0),
            (7.0, "STAT_BOOTCNT", 4.0),
        ] {
            b.rec("PARM", &[N(t * 1e6), T(name), N(v), N(0.0)]);
        }
        let c = param_changes(&b.open());
        assert_eq!(c.len(), 2, "{c:?}");
        assert_eq!(
            (c[0].name.as_str(), c[0].old, c[0].new, c[0].automatic),
            ("WPNAV_SPEED", 1000.0, 500.0, false)
        );
        assert!(c[1].automatic && c[1].time_us == 7_000_000);
    }

    #[test]
    fn distance_from_the_latest_home_and_climb_rates() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("EV", "QB", "TimeUS,Id")
            .format("ORGN", "QBLLe", "TimeUS,Type,Lat,Lng,Alt")
            .format("POS", "QLLff", "TimeUS,Lat,Lng,Alt,RelHomeAlt")
            .format("CTUN", "Qh", "TimeUS,CRt"); // 4.6 layout: int16 cm/s
        let s = |x: f64| x * 1e6;
        // Home first at -35.0 (boot), set again at -35.001 on arming.
        b.rec("ORGN", &[N(s(0.5)), N(1.0), N(-35.0), N(149.0), N(584.0)]);
        b.rec("ORGN", &[N(s(1.0)), N(1.0), N(-35.001), N(149.0), N(584.0)]);
        b.rec("ARM", &[N(s(1.0)), N(1.0), N(0.0), N(0.0), N(0.0)]);
        b.rec("EV", &[N(s(1.5)), N(28.0)]);
        // 0.002 deg north of the new home (about 222 m), then back.
        for (t, lat) in [(2.0, -35.001), (3.0, -34.999), (4.0, -35.001)] {
            b.rec("POS", &[N(s(t)), N(lat), N(149.0), N(600.0), N(16.0)]);
        }
        for (t, crt) in [(2.0, 250.0), (3.0, 0.0), (4.0, -120.0)] {
            b.rec("CTUN", &[N(s(t)), N(crt)]);
        }
        b.rec("EV", &[N(s(5.0)), N(18.0)]);
        let sm = summary(&b.open());
        let d = sm.max_from_home_m.unwrap();
        assert!((d - 222.4).abs() < 0.5, "{sm:?}");
        assert_eq!((sm.max_climb_ms, sm.max_descent_ms), (Some(2.5), Some(1.2)));
    }
}
