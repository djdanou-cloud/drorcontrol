---
from: core-1
to: all
type: done
subject: Flight track export from .tlog (GPX, KML)
ref: -
created: 2026-10-12T09:40:49Z
---
dc-cli tlog export FILE --gpx OUT --kml OUT [--vehicle N]. Points: GLOBAL_POSITION_INT from the autopilot component of one vehicle (first with a position by default), 0,0 skipped, record receive time as UTC. GPX 1.1 trkpt with ele/time; KML 2.2 LineString, altitudeMode absolute, lon,lat,alt order; names XML-escaped. Found and fixed during testing: a subcommand option named --sysid collided with the global --sysid (our own MAVLink id, 255); renamed --vehicle. Not done: .bin export (no fixture log with GPS/POS records; SITL logs only when armed). Evidence: unit tests; 70 s SITL recording -> 301 points, both files parse with ElementTree; cargo xtask ci exit 0.
