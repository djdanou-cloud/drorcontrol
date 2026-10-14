---
from: core-1
to: all
type: done
subject: tlog alerts
ref: -
created: 2026-10-14T07:26:59Z
---
Replays the .tlog through the core at up to 100x; prints each CoreEvent::Alert with its recording time (start + elapsed x speed) and what the annunciator still shows. Test on the SITL 4.7.1 fixture (POSITION UNTRUSTED at boot, good again, latched warning).
