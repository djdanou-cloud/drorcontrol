//! EKF lane switches: when EKF3 changed its primary core (lane), usually because the
//! current lane's sensors looked worse. A switch in flight is worth investigating (IMU,
//! compass or GPS trouble on one lane).
//!
//! `XKF4.PI` = primary core index (`AP_NavEKF3/LogStructure.h`, 4.7.1), read from core 0's
//! records (field `C`).

use crate::dataflash::Log;
use crate::flight::{armed_periods, in_periods};

/// One change of primary lane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaneSwitch {
    /// When, µs since boot.
    pub time_us: u64,
    /// Primary core before and after.
    pub from: u8,
    /// See `from`.
    pub to: u8,
    /// The vehicle was armed.
    pub armed: bool,
}

/// Every primary-lane change in the log.
pub fn lane_switches(log: &Log) -> Vec<LaneSwitch> {
    let Some(pi) = log.column("XKF4", "PI", Some(("C", 0.0))) else {
        return Vec::new();
    };
    let armed = armed_periods(log);
    pi.time_us
        .iter()
        .zip(&pi.values)
        .zip(pi.values.iter().skip(1).zip(pi.time_us.iter().skip(1)))
        .filter(|((_, a), (b, _))| a != b)
        .map(|((_, a), (b, t))| LaneSwitch {
            time_us: *t,
            from: *a as u8,
            to: *b as u8,
            armed: in_periods(&armed, *t),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn primary_index_changes_are_switches() {
        let mut b = LogBuilder::default();
        b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("XKF4", "QBb", "TimeUS,C,PI");
        b.rec("XKF4", &[N(1e6), N(0.0), N(0.0)]);
        b.rec("XKF4", &[N(1.5e6), N(0.0), N(1.0)]); // on the ground
        b.rec("ARM", &[N(2e6), N(1.0), N(0.0), N(0.0), N(0.0)]);
        b.rec("XKF4", &[N(3e6), N(0.0), N(1.0)]);
        b.rec("XKF4", &[N(3e6), N(1.0), N(0.0)]); // core 1's record: ignored
        b.rec("XKF4", &[N(4e6), N(0.0), N(0.0)]); // in flight
        let s = lane_switches(&b.open());
        assert_eq!(
            s,
            [
                LaneSwitch {
                    time_us: 1_500_000,
                    from: 0,
                    to: 1,
                    armed: false
                },
                LaneSwitch {
                    time_us: 4_000_000,
                    from: 1,
                    to: 0,
                    armed: true
                },
            ]
        );
    }
}
