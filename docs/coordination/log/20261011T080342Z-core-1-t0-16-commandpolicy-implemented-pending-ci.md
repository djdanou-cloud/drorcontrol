---
from: core-1
to: all
type: progress
subject: T0-16 CommandPolicy implemented (pending CI)
ref: -
created: 2026-10-11T08:03:42Z
---
Done:
- protocols::command::CommandPolicy (sans-I/O): plan 08 classes A-D; class C needs a single-use confirmation token valid 5 s and bound to the vehicle and exact command; class D also needs the typed vehicle name and a reason.
- Arguments validated; lockouts: motor test only when known disarmed and props-off attested; reboot and param reset not while armed.
- ADR-0009 delivery: link state never refuses; retransmit with the COMMAND_LONG confirmation counter; flood on every link when not CONNECTED (HubHandle::send_all); lifetimes 30 s RTL/Land, 10 s others; supersede within a group; cancel; state-based verification; NOT CONFIRMED / EXPIRED.
- Late ACK of a superseded command with the same MAV_CMD (ARM then DISARM) is ignored for 2 s instead of being misattributed. Found in self-review; a test reproduced it first.
- protocols::audit: SHA-256 hash-chained audit log with a verifier. dc-core writes it on a thread, resumes the chain on start, and refuses to start on a broken chain.
- dc-core: request_command, submit_command, cancel_command, command_status; CoreEvent::Command; gRPC CommandEvent (status only; no submit RPC yet).
- Tests: 18 policy unit tests, plus 2 fake-vehicle end-to-end tests (arm verified and audited; lost link retried, expired, nothing sent after the lifetime), plus a hub flood test.

Open:
- SetMode is always class C until mode metadata can classify Loiter etc. as class B.
- Param-write classification and metadata ranges need ArduPilot parameter metadata.
- Disarm cannot tell in-flight from on-ground (no EXTENDED_SYS_STATE yet).
- No gRPC or CLI submit path; adding one needs a session/identity design.
- DEGRADED link state is not computed yet.
- The reboot-while-armed lockout goes beyond plan 08 (conservative); flag it for the T0-15 review.
