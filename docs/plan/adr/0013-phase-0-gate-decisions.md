# ADR-0013: Phase 0 gate decisions (owner, 2026-10-15)

* Status: accepted
* Date: 2026-10-15
* Deciders: product owner
* Validation: per decision below

## Context
The Phase 0 gate (M0, [roadmap](../07-roadmap-team.md)) needs every spike to pass or have an
approved alternative. On 2026-10-15 the open spike results and two exit criteria that no longer
fitted earlier decisions were put to the owner, with two further open questions (parameter
metadata licensing, force-arm and parameter reset confirmations).

## Decision

| # | Item | Measured / situation | Decision |
|---|------|----------------------|----------|
| 1 | Map spike S0-3 (T0-07, [ADR-0003](0003-map-engine.md)) | M1 Retina full screen: Metal, 59.98 fps, 0.53 % missed vsync, feel pass; GPU ≈ 42 % (max 56 %) against ≤ 8 % | **Go**: MapLibre Native stays. 42 % GPU is accepted for now; ways to lower it are recorded as suggestions in [future upgrades](../future-upgrades.md) §3 |
| 2 | Video spike S0-5 (T0-09, [ADR-0004](0004-video-pipeline.md)) | Hardware decode and zero-copy pass; live p99 113 ms and CPU 6.4 % fail; the delay is frames the decoder holds (`max_dec_frame_buffering`): 36 ms with 1 | **Accepted with a condition**: Phase 1 builds a decode path that does not hold frames, and the camera set-up guide tells operators to configure streams for low latency |
| 3 | Exit criterion "signed + notarized artefact verified on a clean VM" | Cannot be met without Apple credentials ([ADR-0010](0010-internal-distribution-without-apple-credentials.md)) | **Restated**: "ad-hoc signed universal build with checksums installs and launches on a fresh macOS machine; the first launch (Open Anyway) is checked once by hand". CI checks checksums, signature and launch on every full run; the Open Anyway step is in the Mac session (T0-10) |
| 4 | UI bridge criterion (S0-1, T0-06) | Already restated on 2026-10-12 (ADR-0012 item 17): no frame over 25 ms; M1: 0 frames over 25 ms, p99 17.8–19.5 ms | No change: the criterion is met ([ADR-0001](0001-core-and-ui-stack.md)) |
| 5 | Parameter metadata (descriptions, ranges, units) | ArduPilot's parameter documentation is generated from its GPL source; DrorControl is closed source ([ADR-0012](0012-field-operations-posture.md) item 8) | **Downloaded at run time**: the app fetches ArduPilot's published parameter file for the vehicle's firmware and version when online and caches it; nothing GPL-derived is shipped in the product or the repository |
| 6 | Force-arm (arming despite failing pre-arm checks) and parameter reset to defaults | Left at ADR-0011's stricter level by ADR-0012: force-arm needs typed text and a reason; reset is two-stage and refused while armed | **Opt-in settings, off by default**, like the vehicle-change settings: off, the stricter confirmation stays as now; on, a single confirmation (force-arm still records a reason). **Implemented 2026-10-15:** settings `simple-force-arm` and `simple-param-reset` ([`Simplified`](../../../crates/protocols/src/command.rs) in the command policy); reset stays refused while armed; hazard log H1/H5 and §6 updated |

## Consequences
* ADR-0003 and ADR-0004 move to accepted (with item 1's and item 2's conditions).
* The roadmap's Phase 0 exit criteria read as restated in item 3.
* Phase 0 still needs, from the owner's Mac session: the serial hardware matrix and the
  30-minute soak on three serial chips (T0-05), and the Open Anyway first launch (T0-10).
* Parameter screens in Phase 1 depend on the run-time download; offline, they show names and
  values without descriptions until a file has been cached once.
* Item 6 changes the command policy (class D force-arm, parameter reset); it lands with tests
  and the hazard log updated, and the setting changes and every use are audited.
