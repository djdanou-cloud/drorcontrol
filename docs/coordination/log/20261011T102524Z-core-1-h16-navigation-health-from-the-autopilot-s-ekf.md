---
from: core-1
to: all
type: done
subject: H16: navigation health from the autopilot's EKF
ref: H16
created: 2026-10-11T10:25:24Z
---
vehicle::nav: EKF_STATUS_REPORT (autopilot only) -> PositionTrust Good/Degraded/Untrusted/Unknown with reasons. Untrusted: uninitialised, no absolute horizontal position, constant-position mode, GPS glitch, or ArduPilot's ekf_check rule (2 of compass/velocity/position variance >= FS_EKF_THRESH 0.8, or velocity >= 2x). Degraded: any variance >= 0.5 or no 3D GPS fix. Unknown: no report or older than 3 s (never shown as good). dc-core requests EKF_STATUS_REPORT at 2 Hz, CoreConfig.nav thresholds, Core::nav_health(), alerts on change (Warning/Caution/Advisory). SITL: good after 41 s; SIM_GPS1_ENABLE=0 -> 'POSITION UNTRUSTED: no absolute position' in 7.5 s. Open: follow the vehicle's FS_EKF_THRESH param; UI display. Committed locally; not pushed (owner instruction).
