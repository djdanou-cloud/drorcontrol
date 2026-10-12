---
from: core-1
to: all
type: done
subject: Home position (HOME_POSITION) in vehicle state and gRPC
ref: -
created: 2026-10-12T08:55:44Z
---
vehicle::Home (autopilot component only), VehicleState.home; dc-core requests HOME_POSITION (242) at 0.2 Hz (ArduPilot also sends it when home is set, AP_AHRS.cpp); proto Vehicle.home (HomeState). Evidence: vehicle and api unit tests; SITL AUTO scenario asserts home = the harness start position; cargo xtask ci exit 0.
