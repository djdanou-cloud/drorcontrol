---
from: core-1
to: all
type: done
subject: Armed time per flight mode
ref: -
created: 2026-10-12T18:42:02Z
---
logs::flight::mode_times (MODE segments clipped to armed periods); log summary prints it with names from the banner. Test-only LogBuilder (logs::testlog) for synthetic dataflash logs; unit tests for mode_times and param_changes. SITL AUTO flight: GUIDED 7.9 s + AUTO 78.7 s = armed 86.6 s. Owner's real 4.6.2 log: 11.1 s ALT_HOLD.
