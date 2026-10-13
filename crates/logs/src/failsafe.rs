//! Every failsafe in a log, in order: what triggered (the autopilot's `ERR` records for its
//! failsafe and safety checks, `ECode` 0 = resolved) and what the vehicle did (mode changes
//! whose `ModeReason` is a failsafe).
//!
//! Sources, ArduPilot 4.7.1 and 4.6.3: `ModeReason` (`AP_Vehicle/ModeReason.h`): 3 radio,
//! 4 battery, 5 GCS, 6 EKF, 7 GPS glitch, 10 fence breached, 11 terrain, 17 terminate,
//! 19 crash, 25 failsafe (general), 29 leak, 50 dead reckoning, 51 takeoff; `ERR`
//! subsystems as in [`crate::health::err_subsystem`] (`FAILSAFE_*`, `CRASH_CHECK`,
//! `EKFCHECK`, `THRUST_LOSS_CHECK`).

use crate::dataflash::Log;
use crate::flight::{Event, events};

/// A failsafe reason for a mode change, if `reason` is one.
pub fn failsafe_reason(reason: u8) -> Option<&'static str> {
    Some(match reason {
        3 => "radio failsafe",
        4 => "battery failsafe",
        5 => "GCS failsafe",
        6 => "EKF failsafe",
        7 => "GPS glitch",
        10 => "fence breached",
        11 => "terrain failsafe",
        17 => "terminate",
        19 => "crash",
        25 => "failsafe",
        29 => "leak failsafe",
        50 => "dead-reckoning failsafe",
        51 => "takeoff failsafe",
        _ => return None,
    })
}

fn safety_subsystem(name: &str) -> bool {
    name.starts_with("FAILSAFE_")
        || matches!(name, "CRASH_CHECK" | "EKFCHECK" | "THRUST_LOSS_CHECK")
}

/// One entry of the failsafe timeline.
#[derive(Debug, Clone, PartialEq)]
pub enum FailsafeEvent {
    /// A failsafe or safety check triggered (`ERR` with a non-zero code).
    Triggered {
        /// Subsystem name.
        what: &'static str,
        /// `ECode`.
        code: u8,
    },
    /// It cleared (`ECode` 0).
    Resolved {
        /// Subsystem name.
        what: &'static str,
    },
    /// The vehicle changed mode because of a failsafe.
    ModeChange {
        /// New mode number.
        mode: u8,
        /// Why.
        reason: &'static str,
    },
}

/// The failsafe timeline: (µs since boot, event).
pub fn failsafes(log: &Log) -> Vec<(u64, FailsafeEvent)> {
    // The cause is logged at the same time as the mode change: list it first.
    let rank = |e: &FailsafeEvent| match e {
        FailsafeEvent::Triggered { .. } => 0,
        FailsafeEvent::ModeChange { .. } => 1,
        FailsafeEvent::Resolved { .. } => 2,
    };
    let mut out: Vec<(u64, FailsafeEvent)> = events(log)
        .into_iter()
        .filter_map(|(t, e)| match e {
            Event::Err(what, 0) if safety_subsystem(what) => {
                Some((t, FailsafeEvent::Resolved { what }))
            }
            Event::Err(what, code) if safety_subsystem(what) => {
                Some((t, FailsafeEvent::Triggered { what, code }))
            }
            Event::Mode { mode, reason } => failsafe_reason(reason)
                .map(|reason| (t, FailsafeEvent::ModeChange { mode, reason })),
            _ => None,
        })
        .collect();
    out.sort_by_key(|(t, e)| (*t, rank(e)));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn errors_and_failsafe_mode_changes_in_order() {
        let mut b = LogBuilder::default();
        b.format("ERR", "QBB", "TimeUS,Subsys,ECode").format(
            "MODE",
            "QBBB",
            "TimeUS,Mode,ModeNum,Rsn",
        );
        b.rec("MODE", &[N(1e6), N(5.0), N(5.0), N(1.0)]); // pilot: not a failsafe
        b.rec("ERR", &[N(2e6), N(6.0), N(1.0)]); // battery failsafe
        b.rec("MODE", &[N(2e6), N(6.0), N(6.0), N(4.0)]); // RTL, logged at the same time
        b.rec("ERR", &[N(3e6), N(3.0), N(1.0)]); // compass error: not a failsafe
        b.rec("ERR", &[N(4e6), N(6.0), N(0.0)]);
        let f = failsafes(&b.open());
        assert_eq!(
            f.iter().map(|(_, e)| e.clone()).collect::<Vec<_>>(),
            [
                FailsafeEvent::Triggered {
                    what: "FAILSAFE_BATT",
                    code: 1
                },
                FailsafeEvent::ModeChange {
                    mode: 6,
                    reason: "battery failsafe"
                },
                FailsafeEvent::Resolved {
                    what: "FAILSAFE_BATT"
                },
            ]
        );
    }
}
