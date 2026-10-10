//! Routing decisions, independent of any I/O (unit-testable).
//!
//! Rules (MAVLink routing convention, as implemented by ArduPilot and mavlink-router):
//! * Learn `(sysid, compid) → link` from every frame received on a link.
//! * Never send a frame back out of the link it arrived on.
//! * Drop frames that claim to come from our own `(sysid, compid)` (an echo or a loop).
//! * Broadcast (no target field, or `target_system == 0`): to every other link.
//! * Targeted: to links where the exact `(system, component)` was seen; else where the
//!   system was seen; else (unknown system) to every other link if `flood_unknown`.
//! * Deliver locally when broadcast or addressed to our system (component 0 or ours).
//! * Per-link policy: `relay` (does this link get third-party traffic?) and an optional
//!   message-id allow-list (e.g. only `GLOBAL_POSITION_INT` to an antenna tracker).

use std::collections::{BTreeMap, HashMap, HashSet};
use std::time::{Duration, Instant};

use link_core::LinkId;
use mavlink_codec::Frame;

/// Router configuration.
#[derive(Debug, Clone)]
pub struct RouterConfig {
    /// Our own system id (GCS default 255).
    pub sysid: u8,
    /// Our own component id (default 190, `MAV_COMP_ID_MISSIONPLANNER`).
    pub compid: u8,
    /// Routes not refreshed for this long are ignored.
    pub route_ttl: Duration,
    /// Forward frames for systems we have never seen to every link.
    pub flood_unknown: bool,
}

impl Default for RouterConfig {
    fn default() -> Self {
        RouterConfig {
            sysid: 255,
            compid: 190,
            route_ttl: Duration::from_secs(10),
            flood_unknown: true,
        }
    }
}

/// What a link may receive.
#[derive(Debug, Clone, Default)]
pub struct LinkPolicy {
    /// Receive frames relayed from other links (not only our own traffic).
    pub relay: bool,
    /// If set, only these message ids are sent on this link.
    pub allow: Option<HashSet<u32>>,
}

impl LinkPolicy {
    /// A normal link: relays everything.
    pub fn relay_all() -> Self {
        LinkPolicy {
            relay: true,
            allow: None,
        }
    }

    /// Only our own traffic, no relaying (e.g. a vehicle link that should not see a
    /// second vehicle's traffic).
    pub fn local_only() -> Self {
        LinkPolicy {
            relay: false,
            allow: None,
        }
    }

    fn permits(&self, from_local: bool, msgid: u32) -> bool {
        (from_local || self.relay) && self.allow.as_ref().is_none_or(|a| a.contains(&msgid))
    }
}

/// The outcome for one frame.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Decision {
    /// Hand the frame to the local application.
    pub deliver_local: bool,
    /// Links to send it to (sorted, never including the source link).
    pub forward: Vec<LinkId>,
}

/// Counters.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct RouterStats {
    /// Frames routed.
    pub frames: u64,
    /// Frames dropped because they claimed our own identity.
    pub loops_dropped: u64,
    /// Targeted frames for unknown systems that were flooded.
    pub flooded: u64,
}

/// Routing table plus per-link policies.
#[derive(Debug)]
pub struct Router {
    cfg: RouterConfig,
    links: BTreeMap<LinkId, LinkPolicy>,
    routes: HashMap<(u8, u8), (LinkId, Instant)>,
    stats: RouterStats,
}

impl Router {
    /// Empty router.
    pub fn new(cfg: RouterConfig) -> Self {
        Router {
            cfg,
            links: BTreeMap::new(),
            routes: HashMap::new(),
            stats: RouterStats::default(),
        }
    }

    /// Configuration in use.
    pub fn config(&self) -> &RouterConfig {
        &self.cfg
    }

    /// Counters since creation.
    pub fn stats(&self) -> &RouterStats {
        &self.stats
    }

    /// Register (or update the policy of) a link.
    pub fn add_link(&mut self, link: LinkId, policy: LinkPolicy) {
        self.links.insert(link, policy);
    }

    /// Forget a link and every route through it.
    pub fn remove_link(&mut self, link: LinkId) {
        self.links.remove(&link);
        self.routes.retain(|_, (l, _)| *l != link);
    }

    /// Link on which `(sysid, compid)` was last heard, if fresh.
    pub fn route(&self, sysid: u8, compid: u8, now: Instant) -> Option<LinkId> {
        self.routes
            .get(&(sysid, compid))
            .filter(|(_, t)| now.duration_since(*t) <= self.cfg.route_ttl)
            .map(|(l, _)| *l)
    }

    /// Every fresh `(sysid, compid) → link` entry.
    pub fn routes(&self, now: Instant) -> Vec<((u8, u8), LinkId)> {
        let mut v: Vec<_> = self
            .routes
            .iter()
            .filter(|(_, (_, t))| now.duration_since(*t) <= self.cfg.route_ttl)
            .map(|(k, (l, _))| (*k, *l))
            .collect();
        v.sort();
        v
    }

    /// Decide where `frame` goes. `from` is the arrival link, or `None` for frames the
    /// local application is sending.
    pub fn decide(&mut self, from: Option<LinkId>, frame: &Frame, now: Instant) -> Decision {
        self.stats.frames += 1;
        let own = (self.cfg.sysid, self.cfg.compid);
        let src = (frame.sysid(), frame.compid());
        if let Some(link) = from {
            if src == own {
                self.stats.loops_dropped += 1;
                return Decision::default();
            }
            self.routes.insert(src, (link, now));
        }

        let msgid = frame.msgid();
        let local = from.is_none();
        let eligible = |l: &LinkId, p: &LinkPolicy| Some(*l) != from && p.permits(local, msgid);

        let (deliver_local, targets): (bool, Vec<LinkId>) = match frame.target() {
            None | Some((0, _)) => (
                !local,
                self.links
                    .iter()
                    .filter(|(l, p)| eligible(l, p))
                    .map(|(l, _)| *l)
                    .collect(),
            ),
            Some((sys, comp)) => {
                let to_us = sys == own.0 && (comp == 0 || comp == own.1);
                let fresh = |k: &(u8, u8), t: &Instant| {
                    now.duration_since(*t) <= self.cfg.route_ttl && k.0 == sys
                };
                let exact: HashSet<LinkId> = self
                    .routes
                    .iter()
                    .filter(|(k, (_, t))| fresh(k, t) && comp != 0 && k.1 == comp)
                    .map(|(_, (l, _))| *l)
                    .collect();
                let system: HashSet<LinkId> = self
                    .routes
                    .iter()
                    .filter(|(k, (_, t))| fresh(k, t))
                    .map(|(_, (l, _))| *l)
                    .collect();
                let chosen = if !exact.is_empty() { exact } else { system };
                let links: Vec<LinkId> = if !chosen.is_empty() {
                    self.links
                        .iter()
                        .filter(|(l, p)| chosen.contains(l) && eligible(l, p))
                        .map(|(l, _)| *l)
                        .collect()
                } else if self.cfg.flood_unknown && !to_us {
                    self.stats.flooded += 1;
                    self.links
                        .iter()
                        .filter(|(l, p)| eligible(l, p))
                        .map(|(l, _)| *l)
                        .collect()
                } else {
                    Vec::new()
                };
                (!local && to_us, links)
            }
        };
        Decision {
            deliver_local,
            forward: targets,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::messages::{CommandLong, GlobalPositionInt, Heartbeat, ParamRequestList};
    use mavlink_codec::{Header, Message, Version, encode};

    const A: LinkId = LinkId(1);
    const B: LinkId = LinkId(2);
    const C: LinkId = LinkId(3);

    fn hb(sysid: u8, compid: u8) -> Frame {
        let m = Message::Heartbeat(Heartbeat::default());
        encode(
            Version::V2,
            Header {
                seq: 0,
                sysid,
                compid,
            },
            &m,
        )
        .unwrap()
    }

    fn cmd(sysid: u8, compid: u8, target: (u8, u8)) -> Frame {
        let m = Message::CommandLong(CommandLong {
            target_system: target.0,
            target_component: target.1,
            command: 400,
            ..Default::default()
        });
        encode(
            Version::V2,
            Header {
                seq: 0,
                sysid,
                compid,
            },
            &m,
        )
        .unwrap()
    }

    fn router() -> (Router, Instant) {
        let mut r = Router::new(RouterConfig::default());
        for l in [A, B, C] {
            r.add_link(l, LinkPolicy::relay_all());
        }
        (r, Instant::now())
    }

    #[test]
    fn broadcast_goes_everywhere_except_source_and_is_delivered() {
        let (mut r, now) = router();
        let d = r.decide(Some(A), &hb(1, 1), now);
        assert_eq!(
            d,
            Decision {
                deliver_local: true,
                forward: vec![B, C]
            }
        );
        assert_eq!(r.route(1, 1, now), Some(A));
    }

    #[test]
    fn local_broadcast_goes_to_all_links_not_back_to_us() {
        let (mut r, now) = router();
        let d = r.decide(None, &hb(255, 190), now);
        assert_eq!(
            d,
            Decision {
                deliver_local: false,
                forward: vec![A, B, C]
            }
        );
    }

    #[test]
    fn targeted_uses_learned_routes() {
        let (mut r, now) = router();
        r.decide(Some(A), &hb(1, 1), now); // vehicle 1 autopilot on A
        r.decide(Some(B), &hb(1, 100), now); // vehicle 1 camera on B
        r.decide(Some(C), &hb(2, 1), now); // vehicle 2 on C

        // From us to vehicle 1 autopilot: exact route only.
        assert_eq!(r.decide(None, &cmd(255, 190, (1, 1)), now).forward, vec![A]);
        // To the whole of system 1: every link where it was seen.
        assert_eq!(
            r.decide(None, &cmd(255, 190, (1, 0)), now).forward,
            vec![A, B]
        );
        // Unknown component of a known system: system routes.
        assert_eq!(
            r.decide(None, &cmd(255, 190, (1, 42)), now).forward,
            vec![A, B]
        );
        // Third party on C commanding vehicle 1: relayed to A, not delivered to us.
        assert_eq!(
            r.decide(Some(C), &cmd(2, 1, (1, 1)), now),
            Decision {
                deliver_local: false,
                forward: vec![A]
            }
        );
    }

    #[test]
    fn addressed_to_us_is_delivered_not_forwarded() {
        let (mut r, now) = router();
        let d = r.decide(Some(A), &cmd(1, 1, (255, 190)), now);
        assert_eq!(
            d,
            Decision {
                deliver_local: true,
                forward: vec![]
            }
        );
        let d = r.decide(Some(A), &cmd(1, 1, (255, 0)), now);
        assert!(d.deliver_local && d.forward.is_empty());
        let d = r.decide(Some(A), &cmd(1, 1, (255, 1)), now);
        assert!(!d.deliver_local, "other component of our system");
    }

    #[test]
    fn unknown_targets_flood_unless_disabled() {
        let (mut r, now) = router();
        let f = cmd(255, 190, (9, 1));
        assert_eq!(r.decide(None, &f, now).forward, vec![A, B, C]);
        assert_eq!(r.stats().flooded, 1);
        r.cfg.flood_unknown = false;
        assert!(r.decide(None, &f, now).forward.is_empty());
    }

    #[test]
    fn loops_and_echoes_are_dropped() {
        let (mut r, now) = router();
        // Our own frame coming back in on a link (echoing radio, routing loop).
        assert_eq!(r.decide(Some(B), &hb(255, 190), now), Decision::default());
        assert_eq!(r.stats().loops_dropped, 1);
        assert_eq!(
            r.route(255, 190, now),
            None,
            "must not learn a route to ourselves"
        );
        // A frame is never sent back to its source, even if the target lives there.
        r.decide(Some(A), &hb(1, 1), now);
        assert_eq!(
            r.decide(Some(A), &cmd(1, 2, (1, 1)), now).forward,
            Vec::<LinkId>::new()
        );
    }

    #[test]
    fn routes_expire_and_move() {
        let (mut r, t0) = router();
        r.decide(Some(A), &hb(1, 1), t0);
        let later = t0 + Duration::from_secs(11);
        assert_eq!(r.route(1, 1, later), None);
        r.decide(Some(B), &hb(1, 1), later);
        assert_eq!(
            r.route(1, 1, later),
            Some(B),
            "vehicle moved to another link"
        );
        r.remove_link(B);
        assert_eq!(r.route(1, 1, later), None);
        assert!(!r.decide(None, &hb(255, 190), later).forward.contains(&B));
    }

    #[test]
    fn policies_filter_relay_and_message_ids() {
        let (mut r, now) = router();
        // C is an antenna tracker: only GLOBAL_POSITION_INT, relayed from the vehicle.
        r.add_link(
            C,
            LinkPolicy {
                relay: true,
                allow: Some([33].into()),
            },
        );
        // B is a second vehicle link that should not get relayed traffic.
        r.add_link(B, LinkPolicy::local_only());

        let pos = encode(
            Version::V2,
            Header {
                seq: 0,
                sysid: 1,
                compid: 1,
            },
            &Message::GlobalPositionInt(GlobalPositionInt::default()),
        )
        .unwrap();
        assert_eq!(r.decide(Some(A), &pos, now).forward, vec![C]);
        assert_eq!(
            r.decide(Some(A), &hb(1, 1), now).forward,
            Vec::<LinkId>::new()
        );
        // Our own traffic still goes to the local-only link B.
        let req = Message::ParamRequestList(ParamRequestList {
            target_system: 0,
            target_component: 0,
        });
        let req = encode(
            Version::V2,
            Header {
                seq: 0,
                sysid: 255,
                compid: 190,
            },
            &req,
        )
        .unwrap();
        assert_eq!(r.decide(None, &req, now).forward, vec![A, B]);
    }
}
