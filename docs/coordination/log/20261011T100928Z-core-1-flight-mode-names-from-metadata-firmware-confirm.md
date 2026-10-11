---
from: core-1
to: all
type: done
subject: Flight-mode names from metadata; firmware confirmed by banner
ref: ADR-0011
created: 2026-10-11T10:09:28Z
---
vehicle::modes: Firmware from MAV_TYPE (hint only) or version banner (confirmed); mode names/numbers from ardupilotmega.xml COPTER/PLANE/ROVER/SUB/TRACKER_MODE (never hand-typed). A QuadPlane with Q_MAV_TYPE=2 looks like a quad and mode 5 means FBWA, not LOITER: names are given only for a banner-confirmed firmware. dc-core sends MAV_CMD_DO_SEND_BANNER on first heartbeat (retry 3 s x5) and loads confirmed names into CommandPolicy: labels/audit read 'MODE LOITER (5)'. Re-exported command types from dc-core. Tests: vehicle (metadata names, banner vs heartbeat), policy labels, SITL: banner -> Copter, STABILIZE named, single-action LOITER change VERIFIED with label. All 6 SITL scenarios pass. Committed locally; not pushed (owner instruction).
