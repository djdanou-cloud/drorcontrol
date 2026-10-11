---
from: core-1
to: all
type: done
subject: Landed / VTOL state from EXTENDED_SYS_STATE
ref: ADR-0011
created: 2026-10-11T10:00:47Z
---
vehicle: LandedState/VtolState/Extended from EXTENDED_SYS_STATE (autopilot component only; unknown values -> Undefined). dc-core requests it at 2 Hz and passes landed_state with its age in VehicleFacts. No CommandPolicy rule uses it: single-action disarm on the ground (ADR-0011 I2) awaits the safety lead. Tests: vehicle unit test; SITL connect scenario asserts ON_GROUND on ArduCopter 4.7.1 (all 5 SITL scenarios pass locally). Committed locally; not pushed (owner instruction).
