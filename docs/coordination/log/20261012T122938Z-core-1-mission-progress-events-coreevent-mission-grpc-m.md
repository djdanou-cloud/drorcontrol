---
from: core-1
to: all
type: done
subject: Mission progress events (CoreEvent::Mission, gRPC MissionEvent)
ref: -
created: 2026-10-12T12:29:38Z
---
On MISSION_ITEM_REACHED: ItemReached(seq). On MISSION_CURRENT: Current(seq) and State(state) when they differ from the previous report (state read before and after the fleet update). Autopilot component only. CLI telemetry prints them. Evidence: SITL AUTO scenario collects ItemReached 2/3, Current 4, State Active and Complete; full cargo xtask ci exit 0 (16 SITL scenarios, 202 s).
