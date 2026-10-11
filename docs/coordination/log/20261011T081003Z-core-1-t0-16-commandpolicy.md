---
from: core-1
to: all
type: done
subject: T0-16 CommandPolicy
ref: 447becd
created: 2026-10-11T08:10:03Z
---
CI run 31 green on all platforms. Acceptance met by tests: class C/D refused without a valid token; link state never refuses; retries stop on ACK, verified state, supersede or cancel; nothing is sent after the lifetime (tests in protocols/tests/command_policy.rs and dc-core/tests/commands.rs). Open items are listed in the earlier progress entry.
