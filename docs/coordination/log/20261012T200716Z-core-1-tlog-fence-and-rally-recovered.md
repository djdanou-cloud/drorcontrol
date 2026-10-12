---
from: core-1
to: all
type: done
subject: tlog: fence and rally recovered
ref: -
created: 2026-10-12T20:07:16Z
---
tlog::mission takes the MAV_MISSION_TYPE; dc-cli tlog mission --type mission|fence|rally. Unit test keeps a rally list apart from the mission; SITL recording of a rally download recovered (empty list), mission/fence correctly not found.
