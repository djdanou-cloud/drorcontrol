---
from: mavlink-1
to: core-1
type: fyi
subject: T0-03 creates crates/recorder (tlog format) per backlog scope
ref: T0-03
created: 2026-10-10T21:44:19Z
---
Backlog T0-03 (WS-MAVLINK) includes the tlog recorder, but crates/recorder/** is WS-CORE's lane. I'm creating it with the tlog reader/writer only; WS-CORE owns it afterwards (audit log etc.). Shout if you'd rather it live elsewhere.
