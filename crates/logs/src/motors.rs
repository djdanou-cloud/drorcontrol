//! Motor outputs in flight: how hard each motor works compared with the others. A motor that
//! persistently works harder than the rest points at a weak motor or propeller, a bent arm, or
//! the centre of gravity being off towards it. On a quad, the motors spinning one way all
//! working harder than the others points at a yaw imbalance (twisted motor mounts).
//!
//! Sources (ArduPilot 4.7.1):
//! * `RCOU` = TimeUS, C1..C14 (PWM, µs): `AP_Logger/LogStructure.h`. Outputs 15 and up are in
//!   `RCO2`/`RCO3` and are not read here.
//! * Motor functions `SERVOn_FUNCTION` 33..40 = motors 1..8, 82..85 = motors 9..12, 160..161 =
//!   motors 13..14 (`SRV_Channel.h`).
//! * Output range `MOT_PWM_MIN`/`MOT_PWM_MAX`, defaults 1000/2000 (`AP_MotorsMulticopter.cpp`).
//! * Quad plus and X (`FRAME_CLASS` 1, `FRAME_TYPE` 0 or 1): motors 1 and 2 spin
//!   counter-clockwise, 3 and 4 clockwise (`AP_MotorsMatrix::setup_quad_matrix`).

use crate::dataflash::Log;
use crate::flight::{flying_periods, in_periods};

/// One motor's average output while flying.
#[derive(Debug, Clone, PartialEq)]
pub struct MotorOutput {
    /// Motor number (1-based, as in `SERVOn_FUNCTION`).
    pub motor: u8,
    /// Output channel (`SERVOn`).
    pub channel: u8,
    /// Mean PWM while flying, µs.
    pub mean_pwm: f64,
    /// Mean output as a fraction of `MOT_PWM_MIN`..`MOT_PWM_MAX` (0..1).
    pub mean_output: f64,
}

/// Motor outputs while flying.
#[derive(Debug, Clone, PartialEq)]
pub struct MotorBalance {
    /// Each motor, by motor number.
    pub motors: Vec<MotorOutput>,
    /// `RCOU` samples used.
    pub samples: usize,
    /// Whether the samples were chosen with the land detector (else: whenever armed).
    pub land_detector: bool,
    /// Quad plus or X only: mean output of the clockwise motors (3, 4) minus the counter-clockwise
    /// ones (1, 2), as a fraction of the output range.
    pub cw_minus_ccw: Option<f64>,
}

impl MotorBalance {
    /// Largest deviation of one motor from the mean of all, as a fraction of the mean output
    /// (motor number, deviation; positive = works harder).
    pub fn worst(&self) -> Option<(u8, f64)> {
        let n = self.motors.len() as f64;
        let mean = self.motors.iter().map(|m| m.mean_output).sum::<f64>() / n;
        if mean <= 0.0 {
            return None;
        }
        self.motors
            .iter()
            .map(|m| (m.motor, (m.mean_output - mean) / mean))
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
    }
}

/// Motor number for a `SERVOn_FUNCTION` value.
fn motor_number(function: f64) -> Option<u8> {
    let f = function as i32;
    match f {
        33..=40 => Some((f - 32) as u8),
        82..=85 => Some((f - 73) as u8),
        160..=161 => Some((f - 147) as u8),
        _ => None,
    }
}

/// Mean motor outputs while flying; `None` without motor outputs (`SERVOn_FUNCTION`) or
/// `RCOU` samples in flight.
pub fn motor_balance(log: &Log) -> Option<MotorBalance> {
    let params = log.params();
    let p = |name: &str, default: f64| params.get(name).copied().unwrap_or(default);
    let (lo, hi) = (p("MOT_PWM_MIN", 1000.0), p("MOT_PWM_MAX", 2000.0));
    if hi <= lo {
        return None;
    }
    let mut map: Vec<(u8, u8)> = (1..=14u8)
        .filter_map(|ch| {
            let f = params.get(&format!("SERVO{ch}_FUNCTION"))?;
            Some((motor_number(*f)?, ch))
        })
        .collect();
    map.sort();
    if map.is_empty() {
        return None;
    }
    let (flying, land_detector) = flying_periods(log);
    let mut motors = Vec::new();
    let mut samples = 0;
    for (motor, ch) in map {
        let c = log.column("RCOU", &format!("C{ch}"), None)?;
        let v: Vec<f64> = c
            .time_us
            .iter()
            .zip(&c.values)
            .filter(|(t, _)| in_periods(&flying, **t))
            .map(|(_, v)| *v)
            .collect();
        if v.is_empty() {
            return None;
        }
        samples = v.len();
        let mean_pwm = v.iter().sum::<f64>() / v.len() as f64;
        motors.push(MotorOutput {
            motor,
            channel: ch,
            mean_pwm,
            mean_output: (mean_pwm - lo) / (hi - lo),
        });
    }
    let quad = p("FRAME_CLASS", 0.0) == 1.0 && matches!(p("FRAME_TYPE", -1.0), 0.0 | 1.0);
    let out = |n: u8| motors.iter().find(|m| m.motor == n).map(|m| m.mean_output);
    let cw_minus_ccw = match (quad, out(1), out(2), out(3), out(4)) {
        (true, Some(a), Some(b), Some(c), Some(d)) => Some((c + d) / 2.0 - (a + b) / 2.0),
        _ => None,
    };
    Some(MotorBalance {
        motors,
        samples,
        land_detector,
        cw_minus_ccw,
    })
}

/// Hover throttle: what Copter learned and what it used in level flight.
///
/// Copter learns `MOT_THST_HOVER` while flying in modes with automatic throttle, holding
/// height (vertical speed under 0.6 m/s) and level, and keeps it within 0.125..0.6875
/// (`Copter::update_throttle_hover`, `AP_MotorsMulticopter.h`, 4.7.1). The learned value is
/// logged as `CTUN.ThH`.
#[derive(Debug, Clone, PartialEq)]
pub struct HoverThrottle {
    /// Last `CTUN.ThH` while flying.
    pub learned: Option<f64>,
    /// Mean `CTUN.ThO` while flying with |climb rate| < 0.6 m/s. Needs the land detector's
    /// events: armed time alone includes time on the ground.
    pub mean_level: Option<f64>,
    /// `CTUN` samples in that mean.
    pub samples: usize,
    /// `MOT_THST_HOVER` as last logged.
    pub param: Option<f64>,
    /// The learned value sits at a learning limit: `Some("upper")` (the vehicle may be
    /// underpowered or overloaded) or `Some("lower")` (very overpowered).
    pub at_limit: Option<&'static str>,
}

/// Learning limits (`AP_MOTORS_THST_HOVER_MIN`/`MAX`, 4.7.1).
pub const HOVER_LEARN_RANGE: (f64, f64) = (0.125, 0.6875);

/// Hover throttle while flying; `None` without `CTUN`.
pub fn hover_throttle(log: &Log) -> Option<HoverThrottle> {
    let (flying, land_detector) = flying_periods(log);
    let tho = log.column("CTUN", "ThO", None)?;
    let thh = log.column("CTUN", "ThH", None)?;
    let crt = crate::flight::climb_rate(log)?;
    let mut sum = 0.0;
    let mut samples = 0;
    let mut learned = None;
    for (((t, o), h), c) in tho
        .time_us
        .iter()
        .zip(&tho.values)
        .zip(&thh.values)
        .zip(&crt.values)
    {
        if !in_periods(&flying, *t) {
            continue;
        }
        learned = Some(*h);
        if land_detector && c.abs() < 0.6 {
            sum += o;
            samples += 1;
        }
    }
    let at_limit = learned.and_then(|h| {
        if h >= HOVER_LEARN_RANGE.1 - 0.005 {
            Some("upper")
        } else if h <= HOVER_LEARN_RANGE.0 + 0.005 {
            Some("lower")
        } else {
            None
        }
    });
    Some(HoverThrottle {
        learned,
        mean_level: (samples > 0).then(|| sum / samples as f64),
        samples,
        param: log.params().get("MOT_THST_HOVER").copied(),
        at_limit,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{
        LogBuilder,
        V::{N, T},
    };

    #[test]
    fn motor_numbers_follow_the_servo_functions() {
        assert_eq!(motor_number(33.0), Some(1));
        assert_eq!(motor_number(40.0), Some(8));
        assert_eq!(motor_number(82.0), Some(9));
        assert_eq!(motor_number(85.0), Some(12));
        assert_eq!(motor_number(161.0), Some(14));
        assert_eq!(motor_number(70.0), None, "throttle is not a motor");
    }

    #[test]
    fn averages_only_while_flying() {
        let mut b = LogBuilder::default();
        b.format("PARM", "QNff", "TimeUS,Name,Value,Default")
            .format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
            .format("EV", "QB", "TimeUS,Id")
            .format("RCOU", "QHHHHH", "TimeUS,C1,C2,C3,C4,C5");
        // Motors on outputs 1-4 in the default quad X order; output 5 is something else.
        for (name, v) in [
            ("FRAME_CLASS", 1.0),
            ("FRAME_TYPE", 1.0),
            ("SERVO1_FUNCTION", 33.0),
            ("SERVO2_FUNCTION", 34.0),
            ("SERVO3_FUNCTION", 35.0),
            ("SERVO4_FUNCTION", 36.0),
            ("SERVO5_FUNCTION", 0.0),
        ] {
            b.rec("PARM", &[N(0.0), T(name), N(v), N(0.0)]);
        }
        let s = |x: f64| x * 1e6;
        b.rec("ARM", &[N(s(1.0)), N(1.0), N(0.0), N(0.0), N(0.0)]);
        // On the ground while armed: idle, ignored.
        b.rec(
            "RCOU",
            &[
                N(s(1.5)),
                N(1100.0),
                N(1100.0),
                N(1100.0),
                N(1100.0),
                N(900.0),
            ],
        );
        b.rec("EV", &[N(s(2.0)), N(28.0)]);
        for i in 0..10 {
            let t = s(3.0 + f64::from(i));
            // Motor 3 works 10 % harder; CW (3, 4) average above CCW (1, 2).
            b.rec(
                "RCOU",
                &[N(t), N(1500.0), N(1500.0), N(1550.0), N(1500.0), N(900.0)],
            );
        }
        b.rec("EV", &[N(s(14.0)), N(18.0)]);
        b.rec(
            "RCOU",
            &[
                N(s(15.0)),
                N(1100.0),
                N(1100.0),
                N(1100.0),
                N(1100.0),
                N(900.0),
            ],
        );
        b.rec("ARM", &[N(s(16.0)), N(0.0), N(0.0), N(0.0), N(0.0)]);
        let m = motor_balance(&b.open()).unwrap();
        assert!(m.land_detector);
        assert_eq!(m.samples, 10);
        assert_eq!(m.motors.len(), 4);
        assert_eq!((m.motors[2].motor, m.motors[2].mean_pwm), (3, 1550.0));
        assert!((m.motors[0].mean_output - 0.5).abs() < 1e-9);
        let (motor, dev) = m.worst().unwrap();
        assert_eq!(motor, 3);
        assert!((dev - (0.55 - 0.5125) / 0.5125).abs() < 1e-9, "{dev}");
        assert!((m.cw_minus_ccw.unwrap() - 0.025).abs() < 1e-9);
    }

    #[test]
    fn hover_throttle_reads_both_climb_rate_units() {
        for (code, unit) in [("f", 1.0), ("h", 100.0)] {
            let mut b = LogBuilder::default();
            b.format("ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
                .format("EV", "QB", "TimeUS,Id")
                .format("CTUN", &format!("Qfff{code}"), "TimeUS,ThO,ThH,ThI,CRt");
            let s = |x: f64| x * 1e6;
            b.rec("ARM", &[N(s(1.0)), N(1.0), N(0.0), N(0.0), N(0.0)]);
            b.rec("CTUN", &[N(s(1.2)), N(0.0), N(0.35), N(0.0), N(0.0)]); // on the ground
            b.rec("EV", &[N(s(1.5)), N(28.0)]);
            // Climbing at 2 m/s (not level), then holding height.
            b.rec("CTUN", &[N(s(2.0)), N(0.6), N(0.35), N(0.0), N(2.0 * unit)]);
            for i in 0..4 {
                let t = s(3.0 + f64::from(i));
                b.rec("CTUN", &[N(t), N(0.3), N(0.31), N(0.0), N(0.1 * unit)]);
            }
            b.rec("CTUN", &[N(s(8.0)), N(0.4), N(0.69), N(0.0), N(0.0)]);
            let h = hover_throttle(&b.open()).unwrap();
            assert_eq!(h.samples, 5, "{code}");
            assert!((h.mean_level.unwrap() - 0.32).abs() < 1e-6, "{code}: {h:?}");
            assert!((h.learned.unwrap() - 0.69).abs() < 1e-6);
            assert_eq!(h.at_limit, Some("upper"));
        }
    }
}
