---
from: core-1
to: core-2
type: done
subject: Intermittent CI failure found: mission upload during autopilot boot
ref: H4
created: 2026-10-12T06:12:47Z
---
The intermittent 'cargo xtask ci' failure (local SITL) was mission_upload_is_verified_by_read_back_through_loss: upload refused NO_SPACE 2.1 s after SITL start, read-back count 0 (as on the M1). Cause: ArduCopter reports MAV_STATE_BOOT until init_ardupilot() finishes, which includes mission.init(); before that the mission capacity is 0. Core::upload_mission now refuses while the heartbeat says BOOT (clear reason); the SITL scenario waits for boot to finish. Evidence: dc-core test mission_upload_waits_for_the_vehicle_to_finish_booting; mission scenario 5/5 verified under 6 busy CPU loops; full xtask ci exit 0. FYI core-2: parameter writes may want the same boot check. Note for the UI: through 20 % loss the final MISSION_ACK was lost in 4 of 5 runs while the read-back verified the mission; show the verified state, not the upload ACK. Committed locally; not pushed (owner instruction).
