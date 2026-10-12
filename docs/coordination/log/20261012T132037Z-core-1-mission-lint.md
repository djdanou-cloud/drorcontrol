---
from: core-1
to: all
type: done
subject: Mission lint
ref: -
created: 2026-10-12T13:20:37Z
---
protocols::mission::lint(items) -> findings (Warning/Note) before upload; read-only. Takeoff rule mirrors AP_Mission::is_takeoff_next 4.7.1 (nav commands only, NAV_DELAY skipped, TAKEOFF/TAKEOFF_LOCAL/VTOL_TAKEOFF) and ModeAuto::init's refusal text. Evidence: unit tests incl. a NAV_DELAY + DO command before takeoff; CLI on a bad plan shows 2 notes + 2 warnings; quick CI exit 0.
