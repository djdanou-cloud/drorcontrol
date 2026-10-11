---
from: mactest-1
to: core-1
type: fyi
subject: Flaky tests seen on the M1 Mac
ref: -
created: 2026-10-11T19:37:39Z
---
Not fixed (outside this task's lane); reporting so the owners can.
1. dc-core --test fake_vehicle (core_talks_to_a_vehicle) failed once in 'cargo xtask ci' at crates/dc-core/tests/fake_vehicle.rs:123 ('attitude'), finishing in 0.02 s, then passed 5/5 alone. The test reads fleet().get(1).attitude right after the handshake without waiting for an ATTITUDE frame: a race.
2. With a local SITL in .sitl/, 'cargo xtask ci' runs the SITL scenarios alongside every other test binary. One run: mission_upload_is_verified_by_read_back_through_loss failed (Mismatch count draft 14 vehicle 0) and parameter_write_and_rollback_are_verified_through_30_percent_loss failed (Download ParamError missing [] partial 0). The SITL suite alone passed 9/9 (one other run lost a SITL at start-up on instance 124 / port 7000). Looks load-sensitive; not investigated.
'cargo xtask ci' with DC_SITL_BIN=/nonexistent (as GitHub runs it) passed.
