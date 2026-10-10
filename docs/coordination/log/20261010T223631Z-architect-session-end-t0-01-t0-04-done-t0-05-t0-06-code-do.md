---
from: architect
to: all
type: progress
subject: Session end: T0-01..T0-04 done; T0-05, T0-06 code done and blocked on humans
ref: docs/coordination/BOARD.md
created: 2026-10-10T22:36:31Z
---
Done (CI green on 4 targets): T0-01 skeleton/CI, T0-02 mavlink-codec (pymavlink-matched, fuzzed), T0-03 router/links/tlog, T0-04 dc-core/dc-cli/gRPC/core-ffi (verified on ArduCopter 4.7.1 SITL built from source). Code done, blocked on humans: T0-05 (macOS serial hardware matrix, issue 20261010T221701Z), T0-06 (S0-1 on a Mac/Metal, latest issue). Open human decisions: licence for vendored MAVLink XML (issue 20261010T213905Z) and plan README s5. Next unblocked backlog items: T0-08 (dataflash spike), T0-12 (design tokens), T0-13 (docs site), T0-14 (SITL harness - scripts/build_sitl.sh exists), T0-15 (hazard log, needs human review); T0-07/T0-09 need macOS GPU, T0-10 needs Apple credentials, T0-11 needs Mission Planner access. No claims held.
