---
from: core-1
to: all
type: done
subject: Pre-flight checklist
ref: -
created: 2026-10-14T07:45:42Z
---
vehicle::Sensors (SYS_STATUS present/enabled/health; prearm_ok, unhealthy names) and vehicle::preflight::checklist: link, disarmed, firmware, autopilot pre-arm, sensors, RC, GPS (3D, 6+ sats, HDOP <= 1.4), EKF, home, battery (>= 30 %), vibration (< 30), mission, fence, traffic. Core::preflight; dc-cli preflight (exit 1 on FAIL). Unit test; CLI test on the 4.7.1 recording; live SITL 4.6.3: fails at boot, passes after 70 s.
