//! Link profiles (plan 09 Step 1): how much telemetry each link is asked to carry, and the
//! per-link stream-rate request.
//!
//! ArduPilot keeps stream rates per link, so the request for a link goes out on that link
//! only ([`HubHandle::send_on`](mav_router::HubHandle::send_on)). It runs when a vehicle's
//! autopilot heartbeat first arrives on a link, and again after the vehicle was silent on it
//! for the core's LOST time (a reboot or a long dropout resets the rates).

use std::fmt;
use std::ops::Deref;
use std::str::FromStr;
use std::sync::Arc;
use std::time::{Duration, Instant};

use link_core::LinkId;
use mavlink_codec::{Header, Message, Version, encode};
pub use protocols::streams::{LinkProfile, RateOutcome, RateResult};
use protocols::streams::{StreamRequest, StreamRequestConfig};
use tokio::sync::broadcast;

use crate::{Core, CoreEvent, LinkSpec};

/// A link and its profile, from strings like `serial:/dev/ttyACM0:460800?profile=constrained-elrs`.
/// Without `?profile=` the profile is [`LinkProfile::Full`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LinkTarget {
    /// Where.
    pub spec: LinkSpec,
    /// How much telemetry to ask for on it.
    pub profile: LinkProfile,
}

impl Deref for LinkTarget {
    type Target = LinkSpec;

    fn deref(&self) -> &LinkSpec {
        &self.spec
    }
}

impl From<LinkSpec> for LinkTarget {
    fn from(spec: LinkSpec) -> Self {
        LinkTarget {
            spec,
            profile: LinkProfile::Full,
        }
    }
}

impl From<&LinkSpec> for LinkTarget {
    fn from(spec: &LinkSpec) -> Self {
        spec.clone().into()
    }
}

impl From<&LinkTarget> for LinkTarget {
    fn from(t: &LinkTarget) -> Self {
        t.clone()
    }
}

impl FromStr for LinkTarget {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, String> {
        let (spec, profile) = match s.rsplit_once("?profile=") {
            Some((spec, key)) => (
                spec,
                LinkProfile::from_key(key).ok_or_else(|| {
                    let keys: Vec<_> = LinkProfile::ALL.iter().map(|p| p.key()).collect();
                    format!("unknown link profile `{key}` ({})", keys.join(", "))
                })?,
            ),
            None => (s, LinkProfile::Full),
        };
        let spec: LinkSpec = spec.parse()?;
        match (&spec, profile) {
            (_, LinkProfile::Full) => {}
            (LinkSpec::Tlog { .. }, _) => {
                return Err("a .tlog replay has no stream rates: no ?profile=".into());
            }
            (LinkSpec::TcpIn(_), _) => {
                return Err(
                    "tcpin opens one link per client: a profile is not supported there".into(),
                );
            }
            _ => {}
        }
        Ok(LinkTarget { spec, profile })
    }
}

impl fmt::Display for LinkTarget {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.profile {
            LinkProfile::Full => write!(f, "{}", self.spec),
            p => write!(f, "{}?profile={}", self.spec, p.key()),
        }
    }
}

/// The outcome of one link's stream-rate request.
#[derive(Debug, Clone, PartialEq)]
pub struct StreamReport {
    /// Vehicle.
    pub sysid: u8,
    /// Link.
    pub link: LinkId,
    /// Profile requested.
    pub profile: LinkProfile,
    /// Per message, in request order.
    pub results: Vec<RateResult>,
}

impl StreamReport {
    /// Every rate was set and read back.
    pub fn all_set(&self) -> bool {
        self.results.iter().all(|r| r.outcome == RateOutcome::Set)
    }
}

impl Core {
    /// The profile of `link` ([`LinkProfile::Full`] unless the link was opened with one).
    pub fn link_profile(&self, link: LinkId) -> LinkProfile {
        self.link_profiles
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&link)
            .copied()
            .unwrap_or_default()
    }

    /// Change `link`'s profile and request it again from every vehicle heard on the link.
    pub async fn set_link_profile(&self, link: LinkId, profile: LinkProfile) {
        self.link_profiles
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert(link, profile);
        let vehicles: Vec<u8> = self
            .link_heard
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .keys()
            .filter(|(_, l)| *l == link)
            .map(|(s, _)| *s)
            .collect();
        for sysid in vehicles {
            self.request_link_streams(sysid, vehicle::AUTOPILOT_COMPID, link);
        }
    }

    /// The last stream-rate request on each link to `sysid`.
    pub fn stream_reports(&self, sysid: u8) -> Vec<StreamReport> {
        self.stream_reports
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .iter()
            .filter(|((s, _), _)| *s == sysid)
            .map(|(_, r)| r.clone())
            .collect()
    }

    /// An autopilot heartbeat from `sysid` arrived on `link`: request that link's rates if
    /// it is the first one there or the vehicle was silent on it for the LOST time.
    pub(crate) fn on_link_heartbeat(&self, sysid: u8, compid: u8, link: LinkId, now: Instant) {
        let due = {
            let mut heard = self.link_heard.lock().unwrap_or_else(|e| e.into_inner());
            let last = heard.insert((sysid, link), now);
            last.is_none_or(|t| now.saturating_duration_since(t) >= self.cfg.link_lost)
        };
        if due {
            self.request_link_streams(sysid, compid, link);
        }
    }

    /// Run `link`'s profile request for `(sysid, compid)` in the background; the outcome is
    /// kept ([`Core::stream_reports`]) and announced with [`CoreEvent::LinkStreams`].
    pub fn request_link_streams(&self, sysid: u8, compid: u8, link: LinkId) {
        let profile = self.link_profile(link);
        let rates = profile.rates(&self.cfg.stream_rates).to_vec();
        let core = self.clone();
        tokio::spawn(async move {
            let mut rx = core.incoming.subscribe();
            let (mut req, first) = StreamRequest::start(
                (sysid, compid),
                &rates,
                profile.stops_other_streams(),
                StreamRequestConfig::default(),
                Instant::now(),
            );
            for m in first {
                core.send_on(link, &m).await;
            }
            let mut tick = tokio::time::interval(Duration::from_millis(50));
            while !req.is_done() {
                let out = tokio::select! {
                    m = rx.recv() => match m {
                        Ok(inc) if inc.sysid == sysid && inc.compid == compid && inc.link == link => {
                            req.on_message(&inc.msg, Instant::now())
                        }
                        Ok(_) | Err(broadcast::error::RecvError::Lagged(_)) => Vec::new(),
                        Err(broadcast::error::RecvError::Closed) => return,
                    },
                    _ = tick.tick() => req.poll(Instant::now()),
                };
                for m in out {
                    core.send_on(link, &m).await;
                }
            }
            let report = StreamReport {
                sysid,
                link,
                profile,
                results: req.results().to_vec(),
            };
            core.stream_reports
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert((sysid, link), report);
            let _ = core.events.send(CoreEvent::LinkStreams {
                sysid,
                link,
                profile,
                problems: req.problems(),
            });
        });
    }

    /// Send a message from us on one link only.
    pub(crate) async fn send_on(&self, link: LinkId, msg: &Message) -> bool {
        let seq = self.seq.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let header = Header {
            seq,
            sysid: self.cfg.sysid,
            compid: self.cfg.compid,
        };
        match encode(Version::V2, header, msg) {
            Ok(frame) => self.hub.send_on(link, frame).await,
            Err(_) => false,
        }
    }
}

/// Shared per-core state for this module.
pub(crate) type LinkHeard = Arc<std::sync::Mutex<std::collections::HashMap<(u8, LinkId), Instant>>>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn link_targets_parse_and_print() {
        let t: LinkTarget = "serial:/dev/ttyACM0:460800?profile=constrained-elrs"
            .parse()
            .unwrap();
        assert_eq!(t.profile, LinkProfile::Constrained);
        assert_eq!(
            t.spec,
            LinkSpec::Serial {
                path: "/dev/ttyACM0".into(),
                baud: 460_800
            }
        );
        assert_eq!(
            t.to_string(),
            "serial:/dev/ttyACM0:460800?profile=constrained-elrs"
        );
        let plain: LinkTarget = "udp:0.0.0.0:14550".parse().unwrap();
        assert_eq!(plain.profile, LinkProfile::Full);
        assert_eq!(plain.to_string(), "udp:0.0.0.0:14550");
        assert_eq!(
            "tcp:127.0.0.1:5763?profile=high"
                .parse::<LinkTarget>()
                .unwrap()
                .profile,
            LinkProfile::High
        );
        // Explicit full prints without the suffix.
        assert_eq!(
            "tcp:127.0.0.1:5760?profile=full"
                .parse::<LinkTarget>()
                .unwrap()
                .to_string(),
            "tcp:127.0.0.1:5760"
        );
        for bad in [
            "tcp:127.0.0.1:5760?profile=elrs",
            "tlog:x.tlog?profile=high",
            "tcpin:0.0.0.0:5760?profile=high",
        ] {
            assert!(bad.parse::<LinkTarget>().is_err(), "{bad}");
        }
        // A plain LinkSpec still converts.
        let s = LinkSpec::Tcp("127.0.0.1:5760".parse().unwrap());
        assert_eq!(LinkTarget::from(&s).profile, LinkProfile::Full);
    }
}
