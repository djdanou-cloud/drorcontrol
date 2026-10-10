//! The QML-facing telemetry object: one `tick()` per rendered frame copies the latest
//! snapshot into properties and records frame/tick timing.

use std::pin::Pin;
use std::time::{Duration, Instant};

use cxx_qt::CxxQtType;

#[cxx_qt::bridge]
pub mod qobject {
    extern "RustQt" {
        #[qobject]
        #[qml_element]
        #[qproperty(f64, roll)]
        #[qproperty(f64, pitch)]
        #[qproperty(f64, yaw)]
        #[qproperty(f64, alt)]
        #[qproperty(f64, msgs_per_sec)]
        #[qproperty(f64, frame_p99_ms)]
        #[qproperty(f64, tick_p99_us)]
        #[qproperty(i32, dropped_frames)]
        type Telemetry = super::TelemetryRust;

        /// Called once per frame by QML. Returns true when the run is over.
        #[qinvokable]
        fn tick(self: Pin<&mut Telemetry>) -> bool;
    }
}

/// Timing samples and published values.
#[derive(Default)]
pub struct TelemetryRust {
    roll: f64,
    pitch: f64,
    yaw: f64,
    alt: f64,
    msgs_per_sec: f64,
    frame_p99_ms: f64,
    tick_p99_us: f64,
    dropped_frames: i32,
    started: Option<Instant>,
    last_frame: Option<Instant>,
    frame_ms: Vec<f64>,
    tick_us: Vec<f64>,
    load_us: Vec<f64>,
    last_rate: Option<(Instant, u64)>,
    rates: Vec<f64>,
}

/// A frame interval above this is a dropped frame at 60 Hz (1.5 × 16.7 ms).
const DROPPED_MS: f64 = 25.0;

pub fn percentile(v: &[f64], p: f64) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    let mut s = v.to_vec();
    s.sort_by(|a, b| a.total_cmp(b));
    s[((s.len() - 1) as f64 * p).round() as usize]
}

impl qobject::Telemetry {
    fn tick(mut self: Pin<&mut Self>) -> bool {
        let t0 = Instant::now();
        let warmup = Duration::from_secs(crate::warmup_s());
        let duration = Duration::from_secs(crate::duration_s());

        // Coalesced update: one wait-free snapshot load, one property write per field.
        let fleet = crate::core().fleet();
        let load_us = t0.elapsed().as_secs_f64() * 1e6;
        let total: u64 = fleet
            .vehicles()
            .map(|v| v.message_counts.values().sum::<u64>())
            .sum();
        // S01_FAKE=1: animate values with no ingest at all, to separate QML binding cost
        // from ingest contention.
        if std::env::var_os("S01_FAKE").is_some() {
            let t = t0
                .duration_since(*crate::START.get_or_init(Instant::now))
                .as_secs_f64();
            self.as_mut().set_roll((t * 0.7).sin() * 0.5);
            self.as_mut().set_pitch((t * 0.5).sin() * 0.2);
            self.as_mut().set_yaw(t % std::f64::consts::TAU);
            self.as_mut().set_alt(50.0 + t.sin());
        } else if let Some(v) = fleet.get(1) {
            if let Some(a) = &v.attitude {
                self.as_mut().set_roll(f64::from(a.value.roll));
                self.as_mut().set_pitch(f64::from(a.value.pitch));
                self.as_mut().set_yaw(f64::from(a.value.yaw));
            }
            if let Some(p) = &v.position {
                self.as_mut().set_alt(f64::from(p.value.alt_rel_m));
            }
        }

        let now = Instant::now();
        let (started, last_frame, last_rate) = {
            let r = self.as_mut().rust_mut();
            let r = r.get_mut();
            let started = *r.started.get_or_insert(now);
            (started, r.last_frame.replace(now), r.last_rate)
        };
        let elapsed = now.duration_since(started);
        let measuring = elapsed >= warmup;

        // Ingest rate, sampled once per second.
        let mut new_rate = None;
        match last_rate {
            Some((t, n)) if now.duration_since(t) >= Duration::from_secs(1) => {
                new_rate = Some((n, total, now.duration_since(t).as_secs_f64(), now));
            }
            None => {
                self.as_mut().rust_mut().get_mut().last_rate = Some((now, total));
            }
            _ => {}
        }
        if let Some((n0, n1, dt, at)) = new_rate {
            let rate = (n1 - n0) as f64 / dt;
            self.as_mut().set_msgs_per_sec(rate);
            let r = self.as_mut().rust_mut();
            let r = r.get_mut();
            r.last_rate = Some((at, n1));
            if measuring {
                r.rates.push(rate);
            }
        }

        let tick_us = t0.elapsed().as_secs_f64() * 1e6;
        if measuring {
            let r = self.as_mut().rust_mut();
            let r = r.get_mut();
            if let Some(prev) = last_frame {
                r.frame_ms
                    .push(now.duration_since(prev).as_secs_f64() * 1e3);
            }
            r.tick_us.push(tick_us);
            r.load_us.push(load_us);
            if r.frame_ms.len().is_multiple_of(60) {
                let (f, t, d) = (
                    percentile(&r.frame_ms, 0.99),
                    percentile(&r.tick_us, 0.99),
                    r.frame_ms.iter().filter(|&&x| x > DROPPED_MS).count() as i32,
                );
                self.as_mut().set_frame_p99_ms(f);
                self.as_mut().set_tick_p99_us(t);
                self.as_mut().set_dropped_frames(d);
            }
        }

        if elapsed >= warmup + duration {
            let r = self.rust();
            crate::report(&r.frame_ms, &r.tick_us, &r.load_us, &r.rates);
            return true;
        }
        false
    }
}
