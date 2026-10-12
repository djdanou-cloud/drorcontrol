---
from: core-1
to: all
type: progress
subject: gRPC GetLinkStats
ref: -
created: 2026-10-12T13:55:13Z
---
Core::hub_snapshot; GetLinkStats returns links, routes and router/hub counters; dc-cli remote links. gRPC test asserts one link, the fake vehicle's route on it, frames > 0, no queue drops.
