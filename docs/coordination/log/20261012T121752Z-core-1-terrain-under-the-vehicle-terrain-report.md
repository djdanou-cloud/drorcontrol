---
from: core-1
to: all
type: done
subject: Terrain under the vehicle (TERRAIN_REPORT)
ref: -
created: 2026-10-12T12:17:52Z
---
vehicle::TerrainReport (autopilot component): terrain MSL, height above, pending/loaded blocks; extrapolated when spacing 0 (AP_Terrain::send_terrain_report 4.7.1). Found in SITL: before a position the report is for 0,0, and ArduPilot answers with the unset home height and a non-zero spacing (height_amsl matches loc == unset home_loc), so it looks like real data; flagged no_position. TERRAIN_REPORT requested at 1 Hz; gRPC Vehicle.terrain. Not done: answering TERRAIN_REQUEST with terrain tiles (needs SRTM data; network blocked here). Evidence: unit test incl. 0,0; SITL connect scenario; quick CI exit 0.
