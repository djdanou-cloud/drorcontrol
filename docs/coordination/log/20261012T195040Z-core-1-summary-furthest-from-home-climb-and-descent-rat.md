---
from: core-1
to: all
type: done
subject: Summary: furthest from home, climb and descent rates
ref: -
created: 2026-10-12T19:50:40Z
---
FlightSummary.max_from_home_m (POS vs latest ORGN type 1 home, AP_AHRS::LogOriginType) and max_climb_ms/max_descent_ms (flight::climb_rate: CTUN.CRt float m/s in 4.7, int16 cm/s in 4.6; shared with hover_throttle). Unit test; SITL AUTO 78.3 m (far corner), 2.6 / 1.57 m/s.
