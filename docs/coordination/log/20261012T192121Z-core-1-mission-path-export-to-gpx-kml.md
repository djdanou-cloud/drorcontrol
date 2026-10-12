---
from: core-1
to: all
type: done
subject: Mission path export to GPX/KML
ref: -
created: 2026-10-12T19:21:21Z
---
protocols::mission::path (3-D path above home incl. takeoff/land/RTL; MSL items converted with home's altitude); dc-cli mission export --gpx/--kml (KML absolute with a home altitude, else relativeToGround; refuses empty files). Unit test; outputs parse as XML.
