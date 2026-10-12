---
from: core-1
to: all
type: done
subject: Telemetry radio link quality (RADIO_STATUS)
ref: -
created: 2026-10-12T12:12:32Z
---
vehicle::RadioStatus from RADIO_STATUS of any component (radios report as their own component, sometimes their own sysid); 255 = unknown -> None (common.xml invalid markers); gRPC Vehicle.radio. Not requested: radios send it on their own. Evidence: unit test; quick CI (no SITL) exit 0. SITL sends no RADIO_STATUS.
