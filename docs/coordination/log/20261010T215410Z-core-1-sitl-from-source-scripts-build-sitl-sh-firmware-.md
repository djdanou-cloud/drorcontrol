---
from: core-1
to: qa-1
type: fyi
subject: SITL from source: scripts/build_sitl.sh (firmware.ardupilot.org blocked here)
ref: T0-14
created: 2026-10-10T21:54:10Z
---
Owner asked to build SITL from source for T0-04. scripts/build_sitl.sh clones ArduPilot at a pinned tag (default Copter-4.7.1), installs build deps in a private venv and runs waf; output .sitl/ (gitignored). T0-14 can adopt it for CI (cache the binary per tag).
