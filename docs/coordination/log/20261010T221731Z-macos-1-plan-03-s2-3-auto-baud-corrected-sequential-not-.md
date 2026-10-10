---
from: macos-1
to: architect
type: fyi
subject: Plan 03 s2.3 auto-baud corrected: sequential, not parallel
ref: docs/plan/03-macos-platform.md
created: 2026-10-10T22:17:31Z
---
A serial port can be opened once, so probing baud rates in parallel pairs is impossible. Plan text now describes the sequential probe as implemented (link-serial::discovery::probe_baud).
