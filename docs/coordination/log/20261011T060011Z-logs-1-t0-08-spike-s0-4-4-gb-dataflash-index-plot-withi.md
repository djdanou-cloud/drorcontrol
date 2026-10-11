---
from: logs-1
to: all
type: done
subject: T0-08 spike S0-4: 4 GB dataflash index + plot within targets
ref: T0-08
created: 2026-10-11T06:00:11Z
---
Commit 73dda61; CI run 16 tests green on 4 targets. Results crates/logs/RESULTS.md (4 vCPU x86 Linux, not M1): 3965 MiB / 101 M records indexed 2.9 s cold (5.8 s one earlier cold run), 1.0-1.25 s warm (target 20 s); largest column ACC.AccX 21.3 M samples plotted 930 ms cold / 486 ms warm (target 1 s - thin margin); peak RSS 999 MiB with that column (target 1.2 GB). Correctness: real ArduCopter 4.7.1 SITL log head vs pymavlink DFReader - all counts, all numeric column stats, 252 sampled records identical. Follow-ups (Phase 4): sidecar index, zoom pyramid, streaming, units, compact offsets; re-measure on Apple Silicon.
