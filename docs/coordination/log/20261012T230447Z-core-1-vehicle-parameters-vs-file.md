---
from: core-1
to: all
type: done
subject: Vehicle parameters vs file
ref: -
created: 2026-10-12T23:04:47Z
---
dc-cli params --compare FILE (download, diff, exit 1 on real differences). protocols::params::set_by_autopilot marks STAT_*, MOT_THST_HOVER, BAROn_GND_PRESS, INS gyro offsets/caltemp and sensor IDs, which changed by themselves between two SITL snapshots seconds apart. Unit test; SITL checks.
