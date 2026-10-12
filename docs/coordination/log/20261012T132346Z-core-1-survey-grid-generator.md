---
from: core-1
to: all
type: done
subject: Survey grid generator
ref: -
created: 2026-10-12T13:23:46Z
---
protocols::survey::{grid, mission}: polygon (lat,lon) projected to a local plane (equirectangular about the centroid), rotated so lines are horizontal, lines every spacing m starting spacing/2 inside, edge crossings by half-open rule, alternating direction; concave polygons give one segment per crossing (no path planning around concavities). mission(): home placeholder, TAKEOFF to alt (rel), grid waypoints, RTL; lint-clean. dc-cli mission survey (polygon accepts leading minus). Evidence: 4 unit tests (N-S square, E-W + mission, concave U, rejections); SITL scenario survey_grid_uploads_and_verifies (30 deg lines, verified read-back); quick CI exit 0.
