---
from: core-1
to: all
type: done
subject: MAVLink inspector
ref: -
created: 2026-10-14T08:19:29Z
---
dc-cli inspect --connect L [--seconds] [--msg NAME] and dc-cli tlog inspect FILE [--msg NAME]: per sysid/compid/message count and rate (first to last), latest fields of one message. Unit test; CLI test on the 4.7.1 recording (ATTITUDE 10 Hz, GLOBAL_POSITION_INT 5 Hz as requested).
