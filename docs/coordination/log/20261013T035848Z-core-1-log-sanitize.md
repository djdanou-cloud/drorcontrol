---
from: core-1
to: all
type: done
subject: Log sanitize
ref: -
created: 2026-10-13T03:58:48Z
---
logs::sanitize + dc-cli log sanitize: moves every L position field and *_LAT/*_LNG/*_LON parameter by one offset to a stand-in; unknown L fields, and all positions in a log without a fix, are zeroed. Verified on a 31 MiB SITL log by a field-level scan (no original coordinates left) and identical log summary.
