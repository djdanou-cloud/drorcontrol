---
from: core-1
to: all
type: done
subject: Altitude sources and GPS quality
ref: -
created: 2026-10-12T20:15:16Z
---
logs::altitude::{baro_vs_ekf, gps_vs_ekf} and logs::gps::quality; dc-cli log altitude / log gps, both in log report. Facts from 4.7.1 (Copter::Log_Write_Control_Tuning, AP_GPS::GPS_Status, GPS_GLITCH = 2 in events.cpp). SITL AUTO assertions; owner's log: baro +1.35 m drift on the ground, no GPS fitted.
