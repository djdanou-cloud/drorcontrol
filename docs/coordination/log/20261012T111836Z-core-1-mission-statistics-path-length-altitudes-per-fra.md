---
from: core-1
to: all
type: done
subject: Mission statistics (path length, altitudes per frame)
ref: -
created: 2026-10-12T11:18:36Z
---
protocols::mission::stats(items): nav items (cmd <= MAV_CMD_NAV_LAST 95, AP_Mission::is_nav_cmd), horizontal path home -> each nav item with a location (0,0 = current position, ModeAuto::get_loc_from_cmd 4.7.1), RTL back to home, DO_JUMP repeats not followed (flagged); highest altitude per frame class (rel 3/6, MSL 0/5, terrain 10/11, not mixed); last nav command. dc-cli mission check/download print it. Evidence: unit test (444.8 m mission with jumps, mixed frames); sample plan 111 m; cargo xtask ci exit 0.
