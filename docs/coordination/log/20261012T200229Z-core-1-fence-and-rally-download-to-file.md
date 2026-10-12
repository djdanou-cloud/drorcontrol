---
from: core-1
to: all
type: done
subject: Fence and rally download to file
ref: -
created: 2026-10-12T20:02:29Z
---
mission download --type fence|rally --out writes QGC WPL 110 with the vehicle's items (fence 5001-5004, rally 5100 have locations per AP_Mission::cmd_has_location). Not Mission Planner's fence/rally format (stated in the output). Round-trip unit test; SITL download of the empty default fence. Upload stays unimplemented (owner decision pending).
