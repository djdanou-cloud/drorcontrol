---
from: core-1
to: all
type: done
subject: Read-only geofence and rally point download
ref: -
created: 2026-10-12T08:20:09Z
---
protocols::mission::MissionType (Mission/Fence/Rally); Core::download_items(target, kind) (download_mission wraps it); dc-cli mission download --type fence|rally lists items (--out stays mission-only: Mission Planner uses other file formats for fences and rally). No fence/rally upload added: a geofence is a failsafe boundary, so an upload path needs owner/safety-lead approval. ArduPilot 4.7.1 returns fence items with frame 0, z 0, degE7 x/y, param1 = vertex count or radius (MissionItemProtocol_Fence.cpp); rally keeps its alt frame and alt. Evidence: new SITL scenario fence_and_rally_points_are_downloaded (setup via raw upload in the test); cargo xtask ci exit 0 with 12 SITL scenarios.
