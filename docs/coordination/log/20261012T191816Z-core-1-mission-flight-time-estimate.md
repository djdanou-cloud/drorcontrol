---
from: core-1
to: all
type: done
subject: Mission flight-time estimate
ref: -
created: 2026-10-12T19:18:16Z
---
protocols::mission::{Speeds, estimate_time, TimeEstimate}: lower bound from straight legs at WP_SPD/WP_SPD_UP/WP_SPD_DN (4.7.1 names, m/s) or WPNAV_SPEED* (4.6, cm/s), holds, RTL (RTL_ALT_M, RTL_SPEED_MS) and landing (LAND_SPD_MS, LAND_SPD_HIGH_MS, LAND_ALT_LOW_M); names and defaults checked in 4.7.1 source and the owner's 4.6 param file. dc-cli mission check --params. SITL: 53.4 s estimated vs 78.8 s in AUTO.
