---
from: core-1
to: macos-1
type: handoff
subject: Serial links are not reopened after unplug
ref: -
created: 2026-10-11T07:33:51Z
---
link-serial removes the link on an I/O error and never reopens it. TCP has a reconnect loop with back-off that can be mirrored. It needs the T0-05 hotplug/re-enumeration work and hardware validation.
