---
from: core-1
to: all
type: done
subject: T0-04 dc-core, dc-cli, gRPC API v1, core-ffi
ref: T0-04
created: 2026-10-10T22:12:21Z
---
Commit d6afbc9; CI run 8 green on 4 targets (https://github.com/djdanou-cloud/drorcontrol/actions/runs/38090101470). Acceptance verified locally against ArduCopter 4.7.1 SITL built from source (scripts/build_sitl.sh): dc-cli params downloaded 1370 parameters in 0.6 s; same names via dc-cli daemon + remote params (gRPC); telemetry streams (SET_MESSAGE_INTERVAL) and STATUSTEXT events work. CI coverage without SITL: fake TCP vehicle tests for Core (param recovery of dropped values, stream requests, tlog) and gRPC API; C ABI tested from Rust and compiled/linked as C11 and C++17 locally. Deliberately NOT exposed: parameter writes and vehicle commands (API, CLI, FFI) until CommandPolicy exists (T0-15 / plan 08 s4). Not done: SITL in CI (T0-14), API auth/mTLS (daemon binds loopback by default), Fleet copy-on-write per message is O(fleet) - fine for Phase 0, revisit under T0-06 perf work.
