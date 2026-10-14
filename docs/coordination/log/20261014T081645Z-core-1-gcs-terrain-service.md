---
from: core-1
to: all
type: done
subject: GCS terrain service
ref: -
created: 2026-10-14T08:16:45Z
---
Core::serve_terrain (off by default) answers each TERRAIN_REQUEST sub-grid the terrain fully covers (protocols::terrain::terrain_subgrid), counts sent/uncovered; TERRAIN_REQUEST (133) requested at 1 Hz (ArduPilot sends one only while terrain is missing). dc-cli daemon/telemetry --terrain DIR (.hgt and GeoTIFF). SITL 4.7.1 and 4.6.3: 9 requests, 504 sub-grids sent, pending 0, terrain 600.4 m from the synthetic plane.
