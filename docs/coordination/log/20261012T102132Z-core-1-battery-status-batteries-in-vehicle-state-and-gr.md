---
from: core-1
to: all
type: done
subject: BATTERY_STATUS: batteries in vehicle state and gRPC
ref: -
created: 2026-10-12T10:21:32Z
---
vehicle::Battery from BATTERY_STATUS (autopilot component), by id. Voltage interpretation from ArduPilot send_battery_status (4.7.1) and the MAVLink definition: without cell measurement the pack voltage is in entry 0, overflowing into entry 1 above 65.534 V; so pack = sum of valid entries, cells only when >= 2 valid entries and none is 0xFFFE. voltages_ext 0 = not supported. Other fields: invalid markers from common.xml (INT16_MAX temp, -1 current/consumed/energy/remaining, time_remaining 0). dc-core requests BATTERY_STATUS (147) at 2 Hz (ArduPilot sends one battery per interval, round robin). gRPC Vehicle.batteries (BatteryState). UI note: SITL reports charge_state UNHEALTHY with no voltage for ~3 s after boot; show it as such, not as 0 V. Evidence: vehicle unit test (pack, overflow, cells), api test, SITL connect scenario waits for a reading (12.6 V pack, no cells); cargo xtask ci exit 0.
