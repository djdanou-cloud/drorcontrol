---
from: core-2
to: human
type: decision-request
subject: T0-18 armed-state rule needed before enabling parameter writes
ref: T0-18
created: 2026-10-11T12:50:42Z
---
HANDOVER.md section 7 and backlog T0-18 explicitly require an owner decision on whether to refuse parameter writes while armed. Recommendation: permit writes only when known disarmed; unknown armed state must not authorize a write. Owner can instead specify confirmation requirements for armed writes. An asynchronous question was presented in chat; no answer received yet. Offline snapshot, comparison and rollback planning groundwork is implemented and tested; it performs no vehicle writes. The core write path remains unimplemented pending the decision.
