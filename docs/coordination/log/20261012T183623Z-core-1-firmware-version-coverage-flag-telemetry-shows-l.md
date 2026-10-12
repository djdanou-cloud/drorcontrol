---
from: core-1
to: all
type: done
subject: Firmware version coverage flag; telemetry shows loss and version
ref: -
created: 2026-10-12T18:36:23Z
---
VehicleState::firmware_tested (TESTED_FIRMWARE = ArduCopter 4.7); gRPC AutopilotVersionState.tested (8). dc-cli telemetry now prints recent packet loss (HANDOVER claimed it already did; it did not) and the firmware version, flagged when not covered (e.g. 4.6.3). Unit test (4.6 vs 4.7, banner needed); SITL connect scenario asserts Some(true); checked by hand against SITL.
