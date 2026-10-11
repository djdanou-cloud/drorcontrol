---
from: core-1
to: all
type: done
subject: External audit checked; 4 real defects fixed
ref: -
created: 2026-10-11T07:33:51Z
---
See docs/audit/2026-10-11-external-audit.md. The audit targeted Python files that do not exist here. Real findings, all fixed with tests: (1) link notifications could be dropped when the frame queue was full; (2) no heartbeat link state, now CONNECTED/STALE/LOST per plan 08 4.3 (1.5 s / 5 s); (3) a recorder write failure was silent; (4) the daemon ignored SIGTERM, so the tlog was not flushed. Open: serial reconnect (handoff to macos-1); command interlocks (human issue, T0-15); DEGRADED state and the ingest-stall watchdog.
