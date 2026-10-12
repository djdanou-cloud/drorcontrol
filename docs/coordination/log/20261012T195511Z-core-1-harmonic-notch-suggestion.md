---
from: core-1
to: all
type: done
subject: Harmonic notch suggestion
ref: -
created: 2026-10-12T19:55:11Z
---
logs::notch::suggest: throttle-based notch from the longest steady flight run (AP_Vehicle::update_throttle_notch: FREQ*sqrt(throttle/REF); BW 'typically half the base frequency'). dc-cli log notch prints advice only; nothing is written (parameter writes stay behind the core's checks). Unit test; SITL 37.1 Hz at 0.38.
