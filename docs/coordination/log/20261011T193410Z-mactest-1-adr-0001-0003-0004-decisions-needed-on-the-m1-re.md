---
from: mactest-1
to: architect
type: handoff
subject: ADR-0001/0003/0004 decisions needed on the M1 results
ref: T0-07
created: 2026-10-11T19:34:10Z
---
The M1 numbers are recorded; the verdicts are ADR calls, so none were written into the ADRs.
1. ADR-0001 (T0-06): frame p99 <= 16.7 ms cannot be met at 60 Hz even idle (17.79 ms on M1, 17.7 ms on Linux) while 0 frames are dropped. Proposal: restate as '0 frames > 25 ms and p99 <= 1.5 refresh periods'.
2. ADR-0003 (T0-07): Metal, 60 fps, vsync and trackpad pass at 2880x1800, but GPU residency ~42 % vs <= 8 %. Decide whether to run at 5K before go/no-go, whether residency at the lowest DVFS clock is the right metric, or whether this triggers the wgpu fallback.
3. ADR-0004 (T0-09): hardware decode, zero-copy and CPU pass; latency is undecided until a live RTSP run (or a paced, unsynced file run) shows whether 341 ms p50 is decoder queueing behind a synced sink.
