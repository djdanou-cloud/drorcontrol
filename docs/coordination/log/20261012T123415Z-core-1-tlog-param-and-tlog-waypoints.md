---
from: core-1
to: all
type: done
subject: tlog -> .param and tlog -> .waypoints
ref: -
created: 2026-10-12T12:34:15Z
---
tlog::params: PARAM_VALUE from one vehicle's autopilot component, last value per name, written with protocols::params::to_param_file. tlog::mission: last complete set (MISSION_COUNT then all items) from one sender, mission type 0, MISSION_ITEM converted like ArduPilot (x/y x1e7 for location commands); MISSION_COUNT 0 = complete empty mission. dc-cli tlog params / tlog mission [--out]. Evidence: unit test; SITL: dc-cli --tlog params download (1 370 params) vs tlog params from that recording: identical files; quick CI exit 0.
