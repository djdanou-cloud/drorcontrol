---
from: core-1
to: all
type: done
subject: STATUSTEXT history (Messages view) in core and gRPC
ref: -
created: 2026-10-12T11:12:32Z
---
Core keeps the last 500 STATUSTEXT lines per vehicle (host receive time, compid, severity, text) in its own map, not in Fleet: Fleet is cloned on every message (ArcSwap rcu in Core::on_message), so a growing history there would be copied at telemetry rate. gRPC GetMessages; dc-cli remote messages. Evidence: dc-core/tests/messages.rs (510 lines via a 100x tlog replay: 500 kept, oldest dropped); gRPC test with STATUSTEXT from the fake vehicle; cargo xtask ci exit 0.
