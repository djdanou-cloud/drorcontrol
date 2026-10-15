//! Link quality per vehicle (plan 08 §4.3 DEGRADED, plan 09 Step 2): packet loss from
//! MAVLink sequence numbers and, when the vehicle reports it, RC link quality.
//!
//! This only informs: it raises a Caution ("LINK DEGRADED") and clears it. It does not change
//! how commands are delivered — the command policy keeps using the heartbeat state
//! ([`crate::VehicleLink`]); feeding DEGRADED into delivery is a command-pipeline change
//! left for the owner (ADR-0009).

use std::collections::HashMap;
use std::time::{Duration, Instant};

use crate::{Core, RcSignal, SignalKind};
use protocols::command::AlertLevel;

/// Good, degraded, or not enough data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LinkQualityState {
    /// Not enough frames to judge, and no link-quality report.
    #[default]
    Unknown,
    /// Within limits.
    Good,
    /// Loss or link quality outside limits.
    Degraded,
}

/// Limits, with hysteresis so the state does not flicker.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkQualityThresholds {
    /// Degraded at this loss over the window, % (plan 08 §4.3: 5 %).
    pub loss_enter_pct: f32,
    /// Good again below this loss, %.
    pub loss_exit_pct: f32,
    /// Frames (received + lost) needed in the window before loss counts.
    pub min_frames: u32,
    /// Degraded below this RC link quality, % (only when the vehicle reports LQ).
    pub lq_enter_pct: f32,
    /// Good again at or above this RC link quality, %.
    pub lq_exit_pct: f32,
}

impl Default for LinkQualityThresholds {
    fn default() -> Self {
        LinkQualityThresholds {
            loss_enter_pct: 5.0,
            loss_exit_pct: 3.0,
            min_frames: 20,
            lq_enter_pct: 70.0,
            lq_exit_pct: 80.0,
        }
    }
}

/// One assessment.
#[derive(Debug, Clone, PartialEq)]
pub struct LinkQuality {
    /// State.
    pub state: LinkQualityState,
    /// Loss over the window, % (`None` below `min_frames`).
    pub loss_pct: Option<f32>,
    /// RC link quality, % (only when `RC_CHANNELS.rssi` is known to carry LQ).
    pub rc_lq_pct: Option<f32>,
    /// Why it is degraded (empty otherwise).
    pub reasons: Vec<String>,
}

/// Assess from `(received, lost)` over the window and the RC signal, given the previous state
/// (for hysteresis).
pub fn assess(
    prev: LinkQualityState,
    frames: (u32, u32),
    rc: Option<&RcSignal>,
    th: &LinkQualityThresholds,
) -> LinkQuality {
    let (rx, lost) = frames;
    let loss_pct = (rx + lost >= th.min_frames).then(|| lost as f32 * 100.0 / (rx + lost) as f32);
    let rc_lq_pct = rc
        .filter(|s| s.kind == SignalKind::LinkQuality)
        .map(|s| s.pct);
    if loss_pct.is_none() && rc_lq_pct.is_none() {
        return LinkQuality {
            state: LinkQualityState::Unknown,
            loss_pct,
            rc_lq_pct,
            reasons: Vec::new(),
        };
    }
    let was_degraded = prev == LinkQualityState::Degraded;
    let mut reasons = Vec::new();
    if let Some(l) = loss_pct {
        let limit = if was_degraded {
            th.loss_exit_pct
        } else {
            th.loss_enter_pct
        };
        if l >= limit {
            reasons.push(format!("loss {l:.0}%"));
        }
    }
    if let Some(q) = rc_lq_pct {
        let limit = if was_degraded {
            th.lq_exit_pct
        } else {
            th.lq_enter_pct
        };
        if q < limit {
            reasons.push(format!("RC LQ {q:.0}%"));
        }
    }
    LinkQuality {
        state: if reasons.is_empty() {
            LinkQualityState::Good
        } else {
            LinkQualityState::Degraded
        },
        loss_pct,
        rc_lq_pct,
        reasons,
    }
}

/// The text of the Caution raised on DEGRADED (annunciator id `LINK QUALITY`).
pub fn degraded_text(q: &LinkQuality) -> String {
    format!("LINK DEGRADED: {}", q.reasons.join(", "))
}

/// The text that clears it.
pub const RECOVERED_TEXT: &str = "Link quality good again";

/// Per-vehicle state for the core.
pub(crate) type LinkQualitySeen = std::sync::Arc<std::sync::Mutex<HashMap<u8, LinkQualityState>>>;

impl Core {
    /// Link quality of `sysid` now: autopilot packet loss over the last 10 s and RC link
    /// quality ([`Core::rc_signal`]).
    pub fn link_quality(&self, sysid: u8) -> LinkQuality {
        let prev = self
            .link_quality_seen
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&sysid)
            .copied()
            .unwrap_or_default();
        self.link_quality_from(sysid, prev)
    }

    fn link_quality_from(&self, sysid: u8, prev: LinkQualityState) -> LinkQuality {
        let frames = self
            .stream_quality(sysid)
            .into_iter()
            .find(|(c, _)| *c == ::vehicle::AUTOPILOT_COMPID)
            .map_or((0, 0), |(_, q)| (q.recent_received, q.recent_lost));
        assess(
            prev,
            frames,
            self.rc_signal(sysid).as_ref(),
            &LinkQualityThresholds::default(),
        )
    }

    /// Re-assess every vehicle; alert on DEGRADED and when it clears.
    pub(crate) fn check_link_quality(&self, now: Instant) {
        for v in self.fleet().vehicles() {
            // Only vehicles still heard: a lost link is LINK LOST, not degraded.
            if v.heartbeat
                .as_ref()
                .is_none_or(|h| h.age(now) > Duration::from_secs(5))
            {
                continue;
            }
            let prev = self
                .link_quality_seen
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .get(&v.sysid)
                .copied()
                .unwrap_or_default();
            let q = self.link_quality_from(v.sysid, prev);
            if q.state == LinkQualityState::Unknown || q.state == prev {
                continue;
            }
            self.link_quality_seen
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(v.sysid, q.state);
            match (prev, q.state) {
                (_, LinkQualityState::Degraded) => {
                    self.alert(v.sysid, AlertLevel::Caution, degraded_text(&q));
                    if let Some(l) = q
                        .loss_pct
                        .filter(|_| q.reasons.iter().any(|r| r.starts_with("loss")))
                    {
                        self.maybe_suggest_profile(
                            v.sysid,
                            &format!("{l:.0}% packet loss over 10 s"),
                        );
                    }
                }
                (LinkQualityState::Degraded, LinkQualityState::Good) => {
                    self.alert(v.sysid, AlertLevel::Advisory, RECOVERED_TEXT.into())
                }
                _ => {}
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use LinkQualityState::*;

    fn lq(pct: f32) -> RcSignal {
        RcSignal {
            value: 0,
            pct,
            kind: SignalKind::LinkQuality,
            link: None,
        }
    }

    #[test]
    fn loss_and_lq_with_hysteresis() {
        let th = LinkQualityThresholds::default();
        assert_eq!(
            assess(Unknown, (10, 0), None, &th).state,
            Unknown,
            "too few frames"
        );
        assert_eq!(assess(Unknown, (96, 4), None, &th).state, Good, "4 %");
        let d = assess(Good, (95, 5), None, &th);
        assert_eq!(
            (d.state, d.reasons.clone()),
            (Degraded, vec!["loss 5%".to_string()])
        );
        assert_eq!(degraded_text(&d), "LINK DEGRADED: loss 5%");
        // Hysteresis: 4 % stays degraded, below 3 % clears.
        assert_eq!(assess(Degraded, (96, 4), None, &th).state, Degraded);
        assert_eq!(assess(Degraded, (98, 2), None, &th).state, Good);
        // RC LQ counts only when the vehicle reports LQ.
        assert_eq!(assess(Good, (0, 0), Some(&lq(65.0)), &th).state, Degraded);
        assert_eq!(
            assess(Degraded, (0, 0), Some(&lq(75.0)), &th).state,
            Degraded
        );
        assert_eq!(assess(Degraded, (0, 0), Some(&lq(85.0)), &th).state, Good);
        let rssi = RcSignal {
            kind: SignalKind::Rssi,
            ..lq(10.0)
        };
        assert_eq!(assess(Good, (0, 0), Some(&rssi), &th).state, Unknown);
        let both = assess(Good, (90, 10), Some(&lq(50.0)), &th);
        assert_eq!(both.reasons, ["loss 10%", "RC LQ 50%"]);
    }

    #[test]
    fn annunciator_latches_and_clears_it() {
        use crate::annunciator::{Effect, classify};
        assert_eq!(
            classify("LINK DEGRADED: loss 12%"),
            Effect::Condition("LINK QUALITY".into())
        );
        assert_eq!(
            classify(RECOVERED_TEXT),
            Effect::Clear("LINK QUALITY".into())
        );
    }
}
