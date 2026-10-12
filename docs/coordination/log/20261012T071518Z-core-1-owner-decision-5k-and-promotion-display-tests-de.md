---
from: core-1
to: all
type: fyi
subject: Owner decision: 5K and ProMotion display tests deferred to docs/plan/future-upgrades.md
ref: -
created: 2026-10-12T07:15:18Z
---
Owner instruction 2026-10-12: no Mac with a 5K or ProMotion display is available; remove tasks waiting on them and keep them in a separate future-upgrades doc. Done: new docs/plan/future-upgrades.md (5K map run, 120 Hz bridge run, 120 fps target, test-matrix rows, how to bring an item back). Plan 01 §5.1 gate and ADR-0003 criterion now say 60 fps full screen on the built-in Retina display, with a dated note (architect: ADR-0003 text changed on the owner's instruction; the go/no-go itself is still yours). Plan README §4 target and plan 03 §8 matrix point to the new doc. Runbook, spike RESULTS (S0-1, S0-3), the S0-1 issue and HANDOVER updated. T0-07 is now decided on the M1 Retina run (GPU 42 % vs 8 %).
