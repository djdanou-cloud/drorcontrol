---
from: core-1
to: all
type: done
subject: Rangefinders (DISTANCE_SENSOR)
ref: -
created: 2026-10-12T12:14:42Z
---
vehicle::Rangefinder by sensor id (autopilot component): distance/min/max m (cm in the message), orientation, type, signal quality (0 = not reported), in_range(). DISTANCE_SENSOR requested at 2 Hz. gRPC Vehicle.rangefinders. Evidence: unit test; new SITL scenario rangefinder_is_reported (RNGFND1_TYPE 100, MAX 30 m in metres per 4.7.1): orientation 25, max 30 m; quick CI exit 0.
