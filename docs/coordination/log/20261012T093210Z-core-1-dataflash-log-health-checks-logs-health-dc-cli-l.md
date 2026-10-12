---
from: core-1
to: all
type: done
subject: Dataflash log health checks (logs::health, dc-cli log health)
ref: -
created: 2026-10-12T09:32:10Z
---
Checks with sourced thresholds only: VIBE max per IMU (wiki common-measuring-vibration.rst @ f4b21ca: <30 fine, >30 caution, >60 warning) and Clip increments; BAT min voltage per monitor vs the vehicle's own BATT_LOW_VOLT/BATT_CRT_VOLT (first monitor; BATT2_* not yet); XKF4 SV/SP/SH/SM max per core vs the vehicle's FS_EKF_THRESH (default 0.8; caution 0.5 as vehicle::nav); ERR records with LogErrorSubsystem names (AP_Logger.h 4.7.1), ECode 0 = resolved. Field layouts from 4.7.1 LogStructure.h. Not added: GPS and board-voltage (POWR.Vcc) checks, no sourced thresholds found (ardupilot.org blocked; wiki repo reachable on GitHub). Evidence: unit tests for every level; both fixture logs; cargo xtask ci exit 0.
