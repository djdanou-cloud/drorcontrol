//! Where RTL goes from a given position: home or a rally point, as ArduCopter chooses
//! (`AP_Rally::calc_best_rally_or_home_location` and `find_nearest_rally_point`, 4.6.3 and
//! 4.7.1):
//!
//! 1. the nearest rally point that is valid (on Copter: inside the fence, when one is
//!    enabled: `AP_Rally_Copter::is_valid`);
//! 2. home instead if that point is farther than `RALLY_LIMIT_KM` (0 = no limit);
//! 3. with `RALLY_INCL_HOME`, home instead if home is nearer.
//!
//! Copter defaults: `RALLY_LIMIT_KM` 0.3, `RALLY_INCL_HOME` 1.

use std::collections::BTreeMap;

use crate::fence::{Fence, FenceParams, breach_at};

/// Copter's rally settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RallyParams {
    /// `RALLY_LIMIT_KM`.
    pub limit_km: f64,
    /// `RALLY_INCL_HOME`.
    pub incl_home: bool,
}

impl Default for RallyParams {
    fn default() -> Self {
        RallyParams {
            limit_km: 0.3,
            incl_home: true,
        }
    }
}

impl RallyParams {
    /// From a parameter set (Copter defaults for what it lacks).
    pub fn from_params(p: &BTreeMap<String, f64>) -> RallyParams {
        let d = RallyParams::default();
        RallyParams {
            limit_km: p.get("RALLY_LIMIT_KM").copied().unwrap_or(d.limit_km),
            incl_home: p.get("RALLY_INCL_HOME").map_or(d.incl_home, |v| *v != 0.0),
        }
    }
}

/// The RTL target and why.
#[derive(Debug, Clone, PartialEq)]
pub enum RtlTarget {
    /// Home.
    Home {
        /// Why not a rally point.
        why: &'static str,
    },
    /// Rally point `index` (0-based, in the order uploaded).
    Rally {
        /// Its index.
        index: usize,
        /// Distance from the position, m.
        distance_m: f64,
    },
}

/// Great-circle distance, m.
pub fn distance_m(a: (f64, f64), b: (f64, f64)) -> f64 {
    const R: f64 = 6_371_008.8;
    let (la1, la2) = (a.0.to_radians(), b.0.to_radians());
    let h = ((b.0 - a.0).to_radians() / 2.0).sin().powi(2)
        + la1.cos() * la2.cos() * ((b.1 - a.1).to_radians() / 2.0).sin().powi(2);
    2.0 * R * h.sqrt().asin()
}

/// Where RTL from `at` goes. `fence` (with its parameters) when one is enabled: rally points
/// outside it are skipped, as Copter does.
pub fn rtl_target(
    at: (f64, f64),
    home: (f64, f64),
    rally: &[(f64, f64)],
    params: RallyParams,
    fence: Option<(&Fence, &FenceParams)>,
) -> RtlTarget {
    let nearest = rally
        .iter()
        .enumerate()
        .filter(|(_, p)| fence.is_none_or(|(f, fp)| breach_at(home, f, fp, **p, 0.0).is_none()))
        .map(|(i, p)| (i, distance_m(at, *p)))
        .min_by(|a, b| a.1.total_cmp(&b.1));
    let Some((index, d)) = nearest else {
        return RtlTarget::Home {
            why: if rally.is_empty() {
                "no rally points"
            } else {
                "no rally point inside the fence"
            },
        };
    };
    if params.limit_km > 0.0 && d > params.limit_km * 1000.0 {
        return RtlTarget::Home {
            why: "the nearest rally point is beyond RALLY_LIMIT_KM",
        };
    }
    if params.incl_home && d >= distance_m(at, home) {
        return RtlTarget::Home {
            why: "home is nearer (RALLY_INCL_HOME)",
        };
    }
    RtlTarget::Rally {
        index,
        distance_m: d,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HOME: (f64, f64) = (-35.0, 149.0);
    fn north(m: f64) -> (f64, f64) {
        (HOME.0 + m / 111_195.0, HOME.1)
    }

    #[test]
    fn copter_rules() {
        let p = RallyParams::default();
        let rally = [north(150.0), north(1000.0)];
        // Near the first rally point, far from home: the rally point.
        assert!(matches!(
            rtl_target(north(200.0), HOME, &rally, p, None),
            RtlTarget::Rally { index: 0, .. }
        ));
        // Closer to home than to any rally point: home (RALLY_INCL_HOME).
        assert_eq!(
            rtl_target(north(50.0), HOME, &rally, p, None),
            RtlTarget::Home {
                why: "home is nearer (RALLY_INCL_HOME)"
            }
        );
        // Without RALLY_INCL_HOME the nearest rally point wins even then.
        let no_home = RallyParams {
            incl_home: false,
            ..p
        };
        assert!(matches!(
            rtl_target(north(50.0), HOME, &rally, no_home, None),
            RtlTarget::Rally { index: 0, .. }
        ));
        // 600 m from the nearest rally point: beyond the 0.3 km limit, home.
        assert_eq!(
            rtl_target(north(1600.0), HOME, &rally, no_home, None),
            RtlTarget::Home {
                why: "the nearest rally point is beyond RALLY_LIMIT_KM"
            }
        );
        // No rally points.
        assert_eq!(
            rtl_target(north(100.0), HOME, &[], p, None),
            RtlTarget::Home {
                why: "no rally points"
            }
        );
    }

    #[test]
    fn rally_points_outside_the_fence_are_skipped() {
        // A 500 m inclusion circle at home: the rally point at 1 km is outside it.
        let fence = Fence {
            inclusion_circles: vec![(HOME, 500.0)],
            ..Default::default()
        };
        let fp = FenceParams::default();
        let no_limit = RallyParams {
            limit_km: 0.0,
            incl_home: false,
        };
        assert!(matches!(
            rtl_target(
                north(900.0),
                HOME,
                &[north(1000.0), north(300.0)],
                no_limit,
                Some((&fence, &fp))
            ),
            RtlTarget::Rally { index: 1, .. }
        ));
        assert_eq!(
            rtl_target(
                north(900.0),
                HOME,
                &[north(1000.0)],
                no_limit,
                Some((&fence, &fp))
            ),
            RtlTarget::Home {
                why: "no rally point inside the fence"
            }
        );
    }
}
