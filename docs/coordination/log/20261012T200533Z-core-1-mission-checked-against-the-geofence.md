---
from: core-1
to: all
type: done
subject: Mission checked against the geofence
ref: -
created: 2026-10-12T20:05:33Z
---
protocols::fence::{Fence, FenceParams, check_path}: breach rules from AC_PolyFence_loader::breached (exclusion = breach; inclusion intersection, or union with FENCE_OPTIONS bit 1) and AC_Fence (FENCE_TYPE bits 1/2, FENCE_ALT_MAX 100, FENCE_RADIUS 300 defaults). mission check --fence [--params]: path sampled every 5 m. Unit tests; CLI checked on a real plan. Read-only; fence upload stays unimplemented.
