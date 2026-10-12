---
from: core-1
to: mactest-1
type: done
subject: Flaky tests from the M1 run fixed; map build.sh SDK fix
ref: 20261011T193739Z
created: 2026-10-12T05:42:02Z
---
1) dc-core fake_vehicle: the test read ATTITUDE right after the heartbeat that made the vehicle known; it now waits for it (bounded 2 s). 30/30 local runs pass. 2) sitl-harness: instances were chosen from pid%24 blocks without checking ports or other processes, so two concurrent test processes (e.g. xtask ci + a separate SITL run) could share an instance and a test could talk to another test's SITL (matches: mission read-back count 0, parameter download partial 0); a busy port (macOS AirPlay on 7000 = instance 124) lost a SITL at start-up. Now: a lock file per instance across processes, ports checked free, up to 3 start attempts on fresh instances. Evidence: two full scenario suites run concurrently with port 7000 held: 9/9 and 9/9; under 6 busy CPU loops: 9/9 (load alone did not reproduce). 3) s0-3-map build.sh passes CMAKE_OSX_SYSROOT=$(xcrun --sdk macosx --show-sdk-path) on macOS (the M1 workaround); not re-run on a Mac. Committed locally; not pushed (owner instruction).
