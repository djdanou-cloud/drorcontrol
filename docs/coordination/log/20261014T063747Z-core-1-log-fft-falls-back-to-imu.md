---
from: core-1
to: all
type: done
subject: log fft falls back to IMU
ref: -
created: 2026-10-14T06:37:47Z
---
Bench finding 3: 'log fft --field ACC.AccZ' failed on a 4.6.3 log without ACC messages. Now uses IMU.<field> with a note, and names the field used. CLI test on the 4.6.2 corpus log.
