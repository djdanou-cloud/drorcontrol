//! Navigation health (hazard H16: misleading position in GPS-degraded operation).
//!
//! The GCS never decides that a position is good on its own: it mirrors what the autopilot's
//! EKF reports (`EKF_STATUS_REPORT` flags and variances) and ArduPilot's own EKF check
//! (`ArduCopter/ekf_check.cpp`: two of compass / velocity / position variance at or above
//! `FS_EKF_THRESH`, default 0.8, or velocity variance at twice it). Anything the EKF does not
//! vouch for is shown as such, never as a good position.

use std::time::{Duration, Instant};

use crate::{Timed, VehicleState};

/// `EKF_STATUS_FLAGS` bits used here (ardupilotmega.xml).
pub mod flags {
    /// Horizontal absolute position is good.
    pub const POS_HORIZ_ABS: u16 = 16;
    /// In constant-position mode: no position aiding at all.
    pub const CONST_POS_MODE: u16 = 128;
    /// EKF not initialised.
    pub const UNINITIALIZED: u16 = 1024;
    /// GPS glitching.
    pub const GPS_GLITCHING: u16 = 32768;
}

/// From `EKF_STATUS_REPORT` (autopilot component only).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ekf {
    /// `EKF_STATUS_FLAGS`.
    pub flags: u16,
    /// Velocity variance (normalised: 1.0 = innovation at its test limit).
    pub velocity_var: f32,
    /// Horizontal position variance.
    pub pos_horiz_var: f32,
    /// Vertical position variance.
    pub pos_vert_var: f32,
    /// Compass variance.
    pub compass_var: f32,
}

/// How far the displayed position can be trusted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum PositionTrust {
    /// The EKF vouches for an absolute position with acceptable variances.
    Good,
    /// Usable but degraded: high variance, or no 3D GPS fix.
    Degraded,
    /// Do not steer by it: no absolute position, glitching GPS, or variances over the
    /// autopilot's own failsafe threshold.
    Untrusted,
    /// No current EKF report.
    Unknown,
}

/// Thresholds (defaults mirror ArduPilot's; the failsafe one should follow the vehicle's
/// `FS_EKF_THRESH` once parameters are known).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavThresholds {
    /// ArduPilot `FS_EKF_THRESH` (default 0.8; ≤ 0 disables that check, as on the vehicle).
    pub ekf_fail: f32,
    /// Degraded from this variance (the caution band of common EKF displays).
    pub ekf_caution: f32,
    /// EKF report older than this is not current.
    pub max_age: Duration,
}

impl Default for NavThresholds {
    fn default() -> Self {
        NavThresholds {
            ekf_fail: 0.8,
            ekf_caution: 0.5,
            max_age: Duration::from_secs(3),
        }
    }
}

/// Assessment with the reasons, for display.
#[derive(Debug, Clone, PartialEq)]
pub struct NavHealth {
    /// Trust level.
    pub trust: PositionTrust,
    /// Why (empty when Good).
    pub reasons: Vec<&'static str>,
}

/// ArduPilot's `ekf_over_threshold` (without the optical-flow exception, which the GCS cannot
/// see; it only makes the vehicle stricter than this).
fn over_failsafe(e: &Ekf, thresh: f32) -> bool {
    if thresh <= 0.0 {
        return false;
    }
    let mut count = 0;
    if e.compass_var >= thresh {
        count += 1;
    }
    if e.velocity_var >= 2.0 * thresh {
        count += 2;
    } else if e.velocity_var >= thresh {
        count += 1;
    }
    (e.pos_horiz_var >= thresh && count >= 1) || count >= 2
}

/// Assess one vehicle's navigation health now.
pub fn assess(v: &VehicleState, t: &NavThresholds, now: Instant) -> NavHealth {
    let Some(Timed { value: e, at }) = v.ekf.as_ref() else {
        return NavHealth {
            trust: PositionTrust::Unknown,
            reasons: vec!["no EKF status"],
        };
    };
    if now.saturating_duration_since(*at) > t.max_age {
        return NavHealth {
            trust: PositionTrust::Unknown,
            reasons: vec!["EKF status not current"],
        };
    }
    let mut untrusted = Vec::new();
    if e.flags & flags::UNINITIALIZED != 0 {
        untrusted.push("EKF not initialised");
    }
    if e.flags & flags::POS_HORIZ_ABS == 0 {
        untrusted.push("no absolute position");
    }
    if e.flags & flags::CONST_POS_MODE != 0 {
        untrusted.push("no position aiding (constant-position mode)");
    }
    if e.flags & flags::GPS_GLITCHING != 0 {
        untrusted.push("GPS glitch");
    }
    if over_failsafe(e, t.ekf_fail) {
        untrusted.push("EKF variance over failsafe threshold");
    }
    if !untrusted.is_empty() {
        return NavHealth {
            trust: PositionTrust::Untrusted,
            reasons: untrusted,
        };
    }
    let mut degraded = Vec::new();
    if [e.velocity_var, e.pos_horiz_var, e.compass_var]
        .iter()
        .any(|&x| x >= t.ekf_caution)
    {
        degraded.push("EKF variance high");
    }
    if v.gps.as_ref().is_some_and(|g| g.value.fix_type < 3) {
        degraded.push("GPS: no 3D fix");
    }
    NavHealth {
        trust: if degraded.is_empty() {
            PositionTrust::Good
        } else {
            PositionTrust::Degraded
        },
        reasons: degraded,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Gps;

    const GOOD: u16 = 1 | 2 | 4 | 8 | flags::POS_HORIZ_ABS | 32;

    fn vehicle(e: Ekf, fix: u8, now: Instant) -> VehicleState {
        VehicleState {
            ekf: Some(Timed { value: e, at: now }),
            gps: Some(Timed {
                value: Gps {
                    fix_type: fix,
                    satellites: Some(12),
                    hdop: Some(0.8),
                },
                at: now,
            }),
            ..Default::default()
        }
    }

    fn ekf(flags: u16, vel: f32, pos: f32, mag: f32) -> Ekf {
        Ekf {
            flags,
            velocity_var: vel,
            pos_horiz_var: pos,
            pos_vert_var: 0.1,
            compass_var: mag,
        }
    }

    #[test]
    fn mirrors_the_autopilot() {
        let now = Instant::now();
        let t = NavThresholds::default();
        let trust = |e, fix| assess(&vehicle(e, fix, now), &t, now).trust;
        assert_eq!(trust(ekf(GOOD, 0.1, 0.1, 0.1), 3), PositionTrust::Good);
        assert_eq!(trust(ekf(GOOD, 0.6, 0.1, 0.1), 3), PositionTrust::Degraded);
        assert_eq!(
            trust(ekf(GOOD, 0.1, 0.1, 0.1), 1),
            PositionTrust::Degraded,
            "no 3D fix"
        );
        // ArduPilot's rule: two of three at the threshold, or velocity at twice it.
        assert_eq!(trust(ekf(GOOD, 0.8, 0.1, 0.8), 3), PositionTrust::Untrusted);
        assert_eq!(trust(ekf(GOOD, 0.8, 0.8, 0.1), 3), PositionTrust::Untrusted);
        assert_eq!(trust(ekf(GOOD, 1.6, 0.1, 0.1), 3), PositionTrust::Untrusted);
        assert_eq!(
            trust(ekf(GOOD, 0.1, 0.9, 0.1), 3),
            PositionTrust::Degraded,
            "position alone"
        );
        // Flags.
        assert_eq!(
            trust(ekf(GOOD & !flags::POS_HORIZ_ABS, 0.1, 0.1, 0.1), 3),
            PositionTrust::Untrusted
        );
        assert_eq!(
            trust(ekf(GOOD | flags::GPS_GLITCHING, 0.1, 0.1, 0.1), 3),
            PositionTrust::Untrusted
        );
        assert_eq!(
            trust(ekf(GOOD | flags::CONST_POS_MODE, 0.1, 0.1, 0.1), 3),
            PositionTrust::Untrusted
        );
        assert_eq!(
            trust(ekf(GOOD | flags::UNINITIALIZED, 0.1, 0.1, 0.1), 3),
            PositionTrust::Untrusted
        );
        // Old or missing report: unknown, never good.
        let old = vehicle(ekf(GOOD, 0.1, 0.1, 0.1), 3, now);
        assert_eq!(
            assess(&old, &t, now + Duration::from_secs(4)).trust,
            PositionTrust::Unknown
        );
        assert_eq!(
            assess(&VehicleState::default(), &t, now).trust,
            PositionTrust::Unknown
        );
        // FS_EKF_THRESH = 0 disables the variance failsafe, as on the vehicle.
        let off = NavThresholds { ekf_fail: 0.0, ..t };
        assert_eq!(
            assess(&vehicle(ekf(GOOD, 1.6, 0.9, 0.9), 3, now), &off, now).trust,
            PositionTrust::Degraded
        );
    }
}
