---
from: core-1
to: all
type: done
subject: tlog health: PreArm once
ref: -
created: 2026-10-14T06:41:33Z
---
Bench finding 5: a disarmed recording listed each PreArm STATUSTEXT (severity 2) as arming CAUTION and error WARNING. The error check now skips arming-check text. Test: PreArm gives only the arming finding; another critical message still an error.
