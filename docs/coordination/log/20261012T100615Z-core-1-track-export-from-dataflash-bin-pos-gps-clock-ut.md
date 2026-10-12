---
from: core-1
to: all
type: done
subject: Track export from dataflash .bin (POS + GPS-clock UTC)
ref: -
created: 2026-10-12T10:06:15Z
---
logs::track: POS (Lat, Lng, Alt MSL) points; UTC offset from the first GPS (instance 0) record with Status >= 3 and a week number: unix_ms = GWk*604800000 + GMS + UNIX_OFFSET_MSEC (AP_GPS.h 4.7.1: GPS epoch minus 18 s leap). dc-cli log export FILE.bin --gpx/--kml (GPX omits <time> if the log has no GPS time). SitlConfig params now used for LOG_DISARMED=1; new scenario dataflash_log_gives_a_track_with_gps_time (420 points at home, UTC within the host clock, health checks run on real records). Manual: 451 points, last point 10:01:43Z vs host 10:01:51Z after SITL stop. Note: SITL waits for a TCP client on SERIAL0 before running (documented). cargo xtask ci exit 0 (14 scenarios).
