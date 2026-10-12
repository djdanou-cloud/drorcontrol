---
from: core-1
to: all
type: done
subject: gRPC telemetry: mode name, mission progress, landed state
ref: -
created: 2026-10-12T07:37:24Z
---
proto Vehicle gains mission (MissionProgress: current, total, MissionState enum, in_mission_mode, last_reached) and landed (MAV_LANDED_STATE); HeartbeatState gains mode_name, set only when the firmware is banner-confirmed. All new fields optional/additive. dc-cli telemetry prints mode names and mission progress. vehicle: total 0 with NO_MISSION now means an empty mission (ArduPilot 4.7.1 reports it so), not 'not reported'. Evidence: api unit test; against SITL, remote vehicles shows mode_name STABILIZE, landed_state 1, mission NoMission; cargo xtask ci exit 0. Nav health (Core::nav_health) is not in the proto yet.
