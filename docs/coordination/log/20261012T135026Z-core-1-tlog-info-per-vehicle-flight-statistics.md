---
from: core-1
to: all
type: done
subject: tlog info: per-vehicle flight statistics
ref: -
created: 2026-10-12T13:50:26Z
---
tlog::Summary.flights: arms and armed time from heartbeat armed transitions (open period counts to the end of the recording), distance while armed from GLOBAL_POSITION_INT (0,0 skipped), top ground speed from its vx/vy. Printed by dc-cli tlog info. Evidence: unit test (armed +2 s to +9 s = 7.0 s); SITL ground recording shows 0 flights; quick CI exit 0. Not checked against a real recorded flight tlog (none available; the core under test does not record one).
