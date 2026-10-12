---
from: core-1
to: all
type: done
subject: Dataflash flight summary, event timeline, parameter changes
ref: -
created: 2026-10-12T13:39:05Z
---
logs::flight: armed_periods (ARM.ArmState, else EV 10/11; first change a disarm => armed from log start: ArduPilot opens the log on arming and the ARM record can precede the file, seen in SITL), summary (armed time, POS.RelHomeAlt max, GPS.Spd max, POS distance while armed, BAT.CurrTot used, min volt), events (MODE/ARM/EV/ERR/MSG sorted; CLI names modes from the banner), param_changes (PARM value differing from the previous record; automatic = STAT_BOOTCNT/FLTTIME/RUNTIME/FLTCNT/DISTFLWN (AP_Stats) and MOT_THST_HOVER (saved at disarm with MOT_HOVER_LEARN=2, AP_MotorsMulticopter::save_params_on_disarm)). Evidence: SITL AUTO flight log analysed in the scenario: 1 flight 86.7 s, max 20.0 m, 189 m (STAT_DISTFLWN 227 m is 3-D, get_distance_NED().length()), 650 mAh; AUTO in the timeline; only automatic changes; quick CI exit 0.
