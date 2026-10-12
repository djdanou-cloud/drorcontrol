---
from: core-1
to: all
type: done
subject: Wind estimate from dataflash
ref: -
created: 2026-10-12T19:37:40Z
---
logs::wind::wind (XKF2 core 0 while flying; from-direction like SIM_WIND_DIR). Copter estimates wind only with drag fusion (setWindMagStateLearningMode), so all-zero is reported as not estimated. SITL AUTO flight now has SIM_WIND_SPD 4 / DIR 0 / T 1 and EK3_DRAG_BCOEF_X/Y 9.5, MCOEF 0.082 (ArduPilot autotest values): estimate 4.07 m/s from 2.7 deg; all other flight checks still pass.
