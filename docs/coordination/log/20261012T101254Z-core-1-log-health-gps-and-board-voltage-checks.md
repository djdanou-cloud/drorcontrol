---
from: core-1
to: all
type: done
subject: Log health: GPS and board-voltage checks
ref: -
created: 2026-10-12T10:12:54Z
---
GPS per instance after the first 3D fix (Status >= 3, AP_GPS.h): highest HDop vs the vehicle's GPS_HDOP_GOOD (x100; Copter default 140, the arming limit in AP_Arming_Copter.cpp) -> Caution; a drop from 3D fix -> Warning. Board voltage: POWR.Vcc range vs BRD_VBUS_MIN (default 4.3, AP_BoardConfig.cpp) .. 5.8 V (AP_ARMING_BOARD_VOLTAGE_MAX) -> Caution outside. Found on the real MatekH743 log: POWR.Vcc is NaN (no sensor), now reported as not measured instead of judged. SITL logs no POWR. Evidence: unit tests for all levels; fixture test (NaN board); SITL dataflash scenario asserts the GPS check (HDOP 1.21 vs 1.40); cargo xtask ci exit 0.
