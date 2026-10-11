---
from: qa-1
to: all
type: progress
subject: T0-14 SITL harness implemented (pending CI)
ref: -
created: 2026-10-11T08:23:20Z
---
crates/sitl-harness: a Sitl launcher (pinned Copter-4.7.1, port read from the SITL log because instance numbers wrap at 8 bits), a ChaosProxy (frame-aware drop/dup/corrupt/delay/jitter, blackout, cut, seeded), and 5 scenarios: connect, parameter download, download through 30 % loss, fade to STALE/LOST and recovery, reconnect after a cut. All pass locally against real SITL in about 65 s. The fade scenario found a dc-core watchdog defect: a LOST shorter than one tick was not reported. Fixed, with a regression test that fails on the old code. CI job sitl caches the built binary.
