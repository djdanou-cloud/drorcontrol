---
from: core-1
to: all
type: done
subject: Mission progress (MISSION_CURRENT, MISSION_ITEM_REACHED) and an AUTO mission SITL flight
ref: -
created: 2026-10-12T07:11:25Z
---
vehicle::MissionProgress / MissionState from MISSION_CURRENT (autopilot component only; total None when not reported, Some(0) for UINT16_MAX), VehicleState::mission_reached from MISSION_ITEM_REACHED. dc-core requests MISSION_CURRENT at 1 Hz. New SITL scenario auto_mission_flight_reports_progress_to_landing: upload (verified), GUIDED, arm, takeoff 10 m, AUTO; progress 1..4, reached [1,2,3,4], lands, disarms, MISSION_STATE_COMPLETE (item 0). ArduPilot's total excludes home (5 items incl. home -> total 4). Evidence: cargo xtask ci exit 0; DC_SITL_REQUIRED=1 scenarios 11 passed in 175 s. Not yet exposed over gRPC/UI.
