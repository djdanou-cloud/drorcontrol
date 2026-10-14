//! Annunciator state (plan 04 §2 "Annunciators", plan 08 hazard H9 "alert fatigue / missed
//! alert"): which alerts the display shows, in what order, and when to sound them.
//!
//! * Priority: warning > caution > advisory > status; within a level, oldest first, so an
//!   alert keeps its place while it is shown.
//! * Latching: a warning or caution stays shown after its condition clears until the
//!   operator acknowledges it, so a brief one is not missed. Advisories and status go away
//!   with their condition.
//! * De-duplication: raising an alert that is already shown updates it (text, count) instead
//!   of adding another.
//! * Rate limiting: an alert that comes back within `repeat_quiet` of its last sounding is
//!   shown again but not sounded again; escalating to a higher level always sounds.
//!
//! Display only: nothing here decides or blocks a command (ADR-0009).

use std::time::{Duration, Instant};

/// What one of the core's alerts (`CoreEvent::Alert`) does to the annunciator.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Effect {
    /// A condition that lasts until a later alert clears it (fence breach, position trust).
    Condition(String),
    /// A one-off event (a command expired or was rejected): raised and cleared at once, so a
    /// warning or caution stays latched until acknowledged and an advisory is not kept.
    Event(String),
    /// The condition behind this id is over.
    Clear(String),
}

/// Map a core alert's text to its annunciator id and effect. The ids are the fixed labels:
/// `FENCE`, `POSITION`, and for one-off events the text before `:` or ` — `.
pub fn classify(text: &str) -> Effect {
    match text {
        "Back inside the fence" => return Effect::Clear("FENCE".into()),
        "Position good again" => return Effect::Clear("POSITION".into()),
        _ => {}
    }
    if let Some(rest) = text.strip_prefix("Traffic clear: ") {
        return Effect::Clear(format!("TRAFFIC {rest}"));
    }
    if let Some(rest) = text.strip_prefix("TRAFFIC ") {
        let who = rest.split(':').next().unwrap_or(rest).trim();
        return Effect::Condition(format!("TRAFFIC {who}"));
    }
    if text.starts_with("FENCE BREACH") {
        return Effect::Condition("FENCE".into());
    }
    if text.starts_with("POSITION ") {
        return Effect::Condition("POSITION".into());
    }
    let end = [text.find(" — "), text.find(':')]
        .into_iter()
        .flatten()
        .min()
        .unwrap_or(text.len());
    Effect::Event(text[..end].trim().to_string())
}

/// How urgent an alert is (ordered: `Warning` is the highest).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AlertLevel {
    /// State worth showing (e.g. `RTK FLOAT`).
    Status,
    /// Worth knowing, no action needed now.
    Advisory,
    /// Needs attention soon.
    Caution,
    /// Needs action now.
    Warning,
}

impl AlertLevel {
    /// Warnings and cautions are sounded and latch.
    pub fn is_master(self) -> bool {
        self >= AlertLevel::Caution
    }
}

/// One alert as shown.
#[derive(Debug, Clone, PartialEq)]
pub struct Alert {
    /// Stable key, e.g. `LINK LOST` (also its fixed label).
    pub id: String,
    /// Current level.
    pub level: AlertLevel,
    /// Latest detail text.
    pub text: String,
    /// When it was first raised (this showing).
    pub raised: Instant,
    /// Times raised while shown.
    pub count: u32,
    /// The condition is still present.
    pub active: bool,
    /// The operator has acknowledged it.
    pub acked: bool,
    last_sounded: Option<Instant>,
}

/// What the display should do now about a raise.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sound {
    /// Alert id.
    pub id: String,
    /// Its level (choose tone or speech).
    pub level: AlertLevel,
}

/// The set of alerts.
#[derive(Debug, Clone)]
pub struct Annunciator {
    alerts: Vec<Alert>,
    /// Minimum time between soundings of the same alert at the same level.
    pub repeat_quiet: Duration,
}

impl Default for Annunciator {
    fn default() -> Self {
        Annunciator {
            alerts: Vec::new(),
            repeat_quiet: Duration::from_secs(10),
        }
    }
}

impl Annunciator {
    /// The condition behind `id` is present. Returns what to sound, if anything.
    pub fn raise(
        &mut self,
        id: &str,
        level: AlertLevel,
        text: &str,
        now: Instant,
    ) -> Option<Sound> {
        let quiet = self.repeat_quiet;
        let sound = |a: &mut Alert, force: bool| {
            let due = a
                .last_sounded
                .is_none_or(|t| now.saturating_duration_since(t) >= quiet);
            if a.level.is_master() && (force || due) {
                a.last_sounded = Some(now);
                Some(Sound {
                    id: a.id.clone(),
                    level: a.level,
                })
            } else {
                None
            }
        };
        if let Some(a) = self.alerts.iter_mut().find(|a| a.id == id) {
            let escalated = level > a.level;
            let returned = !a.active;
            a.text = text.to_string();
            a.count += 1;
            a.active = true;
            if escalated {
                a.level = level;
            }
            if escalated || returned {
                // Escalation or a return needs the operator again.
                a.acked = false;
                return sound(a, escalated);
            }
            return None;
        }
        let mut a = Alert {
            id: id.to_string(),
            level,
            text: text.to_string(),
            raised: now,
            count: 1,
            active: true,
            acked: false,
            last_sounded: None,
        };
        let s = sound(&mut a, true);
        self.alerts.push(a);
        s
    }

    /// The condition behind `id` is gone. Warnings and cautions stay until acknowledged;
    /// advisories, status and acknowledged alerts go.
    pub fn clear(&mut self, id: &str) {
        if let Some(a) = self.alerts.iter_mut().find(|a| a.id == id) {
            a.active = false;
        }
        self.alerts
            .retain(|a| a.active || (a.level.is_master() && !a.acked));
    }

    /// The operator acknowledges `id`: it stops asking for attention, and goes if its
    /// condition has cleared.
    pub fn ack(&mut self, id: &str) {
        if let Some(a) = self.alerts.iter_mut().find(|a| a.id == id) {
            a.acked = true;
        }
        self.alerts.retain(|a| a.active || !a.acked);
    }

    /// Acknowledge everything shown.
    pub fn ack_all(&mut self) {
        for a in &mut self.alerts {
            a.acked = true;
        }
        self.alerts.retain(|a| a.active);
    }

    /// The alerts to show, highest level first, then oldest first.
    pub fn shown(&self) -> Vec<&Alert> {
        let mut v: Vec<&Alert> = self.alerts.iter().collect();
        v.sort_by(|a, b| b.level.cmp(&a.level).then(a.raised.cmp(&b.raised)));
        v
    }

    /// The master light: the highest level among unacknowledged warnings and cautions.
    pub fn master(&self) -> Option<AlertLevel> {
        self.alerts
            .iter()
            .filter(|a| a.level.is_master() && !a.acked)
            .map(|a| a.level)
            .max()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use AlertLevel::*;

    fn ids(a: &Annunciator) -> Vec<&str> {
        a.shown().iter().map(|a| a.id.as_str()).collect()
    }

    #[test]
    fn priority_then_age() {
        let t = Instant::now();
        let s = |k: u64| t + Duration::from_secs(k);
        let mut a = Annunciator::default();
        assert!(
            a.raise("RTK FLOAT", Status, "", s(0)).is_none(),
            "status is silent"
        );
        assert!(a.raise("VIBRATION", Advisory, "", s(1)).is_none());
        assert_eq!(
            a.raise("LOW BATT", Caution, "22%", s(2)).unwrap().level,
            Caution
        );
        assert!(a.raise("LINK LOST", Warning, "", s(3)).is_some());
        assert!(a.raise("FENCE BREACH", Warning, "", s(4)).is_some());
        assert_eq!(
            ids(&a),
            [
                "LINK LOST",
                "FENCE BREACH",
                "LOW BATT",
                "VIBRATION",
                "RTK FLOAT"
            ]
        );
        assert_eq!(a.master(), Some(Warning));
    }

    #[test]
    fn latching_and_acknowledge() {
        let t = Instant::now();
        let mut a = Annunciator::default();
        a.raise("LINK LOST", Warning, "", t);
        a.raise("VIBRATION", Advisory, "", t);
        a.clear("LINK LOST");
        a.clear("VIBRATION");
        // The brief warning stays (latched); the advisory is gone.
        assert_eq!(ids(&a), ["LINK LOST"]);
        assert!(!a.shown()[0].active);
        a.ack("LINK LOST");
        assert!(a.shown().is_empty() && a.master().is_none());
        // Acknowledged while still present: shown, but the master light is off.
        a.raise("LOW BATT", Caution, "", t);
        a.ack("LOW BATT");
        assert_eq!(ids(&a), ["LOW BATT"]);
        assert_eq!(a.master(), None);
        a.clear("LOW BATT");
        assert!(a.shown().is_empty(), "acknowledged and cleared: gone");
        a.raise("A", Warning, "", t);
        a.raise("B", Caution, "", t);
        a.clear("A");
        a.ack_all();
        assert_eq!(ids(&a), ["B"]);
    }

    #[test]
    fn dedupe_rate_limit_and_escalation() {
        let t = Instant::now();
        let s = |k: u64| t + Duration::from_secs(k);
        let mut a = Annunciator::default();
        assert!(a.raise("LOW BATT", Caution, "25%", s(0)).is_some());
        // Still present: one entry, updated, not sounded again.
        assert!(a.raise("LOW BATT", Caution, "24%", s(1)).is_none());
        assert_eq!(a.shown().len(), 1);
        assert_eq!((a.shown()[0].count, a.shown()[0].text.as_str()), (2, "24%"));
        // Flapping: back within 10 s of the last sounding is shown but silent ...
        a.clear("LOW BATT");
        assert!(a.raise("LOW BATT", Caution, "24%", s(5)).is_none());
        assert!(!a.shown()[0].acked, "needs the operator again");
        // ... and sounds again once the quiet time has passed.
        a.clear("LOW BATT");
        assert!(a.raise("LOW BATT", Caution, "23%", s(12)).is_some());
        // Escalation sounds at once.
        let e = a.raise("LOW BATT", Warning, "critical", s(13)).unwrap();
        assert_eq!(e.level, Warning);
        assert_eq!(a.shown()[0].level, Warning);
        // A lower level does not downgrade a shown alert.
        assert!(a.raise("LOW BATT", Caution, "", s(14)).is_none());
        assert_eq!(a.shown()[0].level, Warning);
    }

    #[test]
    fn core_alerts_map_to_fixed_ids() {
        assert_eq!(
            classify("FENCE BREACH: boundary"),
            Effect::Condition("FENCE".into())
        );
        assert_eq!(
            classify("Back inside the fence"),
            Effect::Clear("FENCE".into())
        );
        assert_eq!(
            classify("POSITION UNTRUSTED: EKF variance"),
            Effect::Condition("POSITION".into())
        );
        assert_eq!(
            classify("Position good again"),
            Effect::Clear("POSITION".into())
        );
        assert_eq!(
            classify("MODE 6 NOT CONFIRMED — EXPIRED: check the vehicle"),
            Effect::Event("MODE 6 NOT CONFIRMED".into())
        );
        assert_eq!(
            classify("MODE 4 engaged"),
            Effect::Event("MODE 4 engaged".into())
        );
        assert_eq!(
            classify("TRAFFIC QFA12: 900 m N, closest 0 m in 18 s"),
            Effect::Condition("TRAFFIC QFA12".into())
        );
        assert_eq!(
            classify("Traffic clear: QFA12"),
            Effect::Clear("TRAFFIC QFA12".into())
        );
    }
}
