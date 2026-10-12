---
from: core-1
to: all
type: done
subject: Compass interference vs current from dataflash
ref: -
created: 2026-10-12T19:07:22Z
---
logs::compass::compass_interference: |MAG| vs BAT.Curr least squares while flying (rotation-invariant; lower bound of compassmot). dc-cli log compass. SITL AUTO flight now sets SIM_MAG_MOT_Z 1 (SIM_Aircraft.cpp adds it per amp) and recovers -0.80 mGauss/A (projection on the -66 deg inclined field), r -0.82.
