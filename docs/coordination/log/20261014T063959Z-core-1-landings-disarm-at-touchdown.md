---
from: core-1
to: all
type: done
subject: landings: disarm at touchdown
ref: -
created: 2026-10-14T06:39:59Z
---
Bench finding 4: 'not disarmed in this log' while DISARMED and LAND_COMPLETE shared 47.584 s (disarm first: disarming makes the land detector report landed). A disarm up to 1 s before touchdown now counts, as 0 s. Unit test.
