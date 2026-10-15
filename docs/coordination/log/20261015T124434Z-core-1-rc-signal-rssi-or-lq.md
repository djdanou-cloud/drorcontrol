---
from: core-1
to: all
type: done
subject: RC signal: RSSI or LQ
ref: -
created: 2026-10-15T12:44:34Z
---
vehicle::rc_link parses ArduPilot's CRSF/ELRS STATUSTEXTs (Link rate / Telemetry rate, RSSI now displays as LQ) into VehicleState::rc_link; the core watches RSSI_TYPE and RC_OPTIONS in passing parameter traffic (nothing extra requested) and Core::rc_signal says whether RC_CHANNELS.rssi is RSSI or LQ, with the percentage inverting ArduPilot's scaling (LQ x2.5, x254/255). gRPC RcState gains kind, signal_pct, protocol and rates; dc-cli telemetry shows 'rc LQ 100% (ELRS 250 Hz)'. Tests: vehicle unit tests, dc-core rc_signal (fake vehicle).
