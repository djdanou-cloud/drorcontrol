---
from: core-1
to: all
type: done
subject: Erase result waits for a settled listing
ref: -
created: 2026-10-14T06:35:55Z
---
Bench finding 6 (MicoAir743v2, 4.6.3): erase reported 2 new logs, list 8 s later showed 0. A listing now counts only when empty, or when no earlier log is in it and the previous listing matched (id, size). CLI wording no longer says 'after a flight'. Unit test with the bench sequence; SITL 4.6.3 scenario green.
