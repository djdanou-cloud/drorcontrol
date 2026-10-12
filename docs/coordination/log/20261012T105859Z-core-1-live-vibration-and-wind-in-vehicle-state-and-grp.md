---
from: core-1
to: all
type: done
subject: Live vibration and wind in vehicle state and gRPC
ref: -
created: 2026-10-12T10:58:59Z
---
VIBRATION (241, 2 Hz; ArduPilot get_vibration_levels m/s^2 + per-accelerometer clip counts) and WIND (168, 1 Hz; direction the wind comes from) from the autopilot component; gRPC Vehicle.vibration / .wind. Evidence: unit test; SITL connect scenario requires VIBRATION (WIND not sent by a copter on the ground yet); cargo xtask ci exit 0.
