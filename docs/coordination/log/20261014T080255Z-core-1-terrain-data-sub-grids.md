---
from: core-1
to: all
type: done
subject: TERRAIN_DATA sub-grids
ref: -
created: 2026-10-14T08:02:55Z
---
protocols::terrain::offset_e7 (ArduPilot Location::offset_latlng, LATLON_TO_M_INV) and terrain_subgrid: 16 heights of sub-grid gridbit (gridbit/8 north, %8 east, 4x4) of a 28x32 block; None when any point lacks terrain. Unit test against ArduPilot's corner check.
