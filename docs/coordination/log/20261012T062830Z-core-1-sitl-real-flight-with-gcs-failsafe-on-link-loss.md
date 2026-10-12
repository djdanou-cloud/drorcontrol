---
from: core-1
to: all
type: done
subject: SITL: real flight with GCS failsafe on link loss
ref: 08
created: 2026-10-12T06:28:30Z
---
New scenario flight_with_gcs_failsafe_on_link_loss (plan 08 §2 Flight + Failsafe & lost link): on ArduCopter 4.7.1, GUIDED, ARM and TAKEOFF are sent as single actions (ADR-0011) and verified; at 12 m a two-way radio blackout makes the core show LOST while the vehicle's GCS failsafe (FS_GCS_ENABLE=1, set by raw PARAM_SET as test configuration) switches to RTL; on recovery the core shows CONNECTED and mode RTL by name; the vehicle lands and disarms by itself (ON_GROUND). 91 s alone; the 10-scenario suite passes in 280 s on 4 vCPU (CI job timeout 45 min); full xtask ci exit 0. First end-to-end flight through CommandPolicy. Committed locally; not pushed (owner instruction).
