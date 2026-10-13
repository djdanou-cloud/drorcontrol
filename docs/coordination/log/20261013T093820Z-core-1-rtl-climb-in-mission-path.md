---
from: core-1
to: all
type: done
subject: RTL climb in mission path
ref: -
created: 2026-10-13T09:38:20Z
---
mission::path: RTL climbs to 15 m above home when lower (ModeRTL::compute_return_target; RTL_ALT_M 4.7.1 / RTL_ALT 4.6.3), so fence and terrain checks see the real return.
