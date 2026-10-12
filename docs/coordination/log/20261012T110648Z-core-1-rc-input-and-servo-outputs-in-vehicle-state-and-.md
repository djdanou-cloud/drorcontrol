---
from: core-1
to: all
type: done
subject: RC input and servo outputs in vehicle state and gRPC
ref: -
created: 2026-10-12T11:06:48Z
---
RC_CHANNELS (65, 1 Hz): channels up to chancount (max 18), rssi None when 255. SERVO_OUTPUT_RAW (36, 1 Hz): outputs 1-16 per port. Autopilot component only. gRPC Vehicle.rc / .servos. 1 Hz to keep 57.6k radios light; setup screens may ask for more later. Evidence: unit test; SITL connect scenario: 8 RC channels, outputs 1-4 equal (quad, disarmed); cargo xtask ci exit 0.
