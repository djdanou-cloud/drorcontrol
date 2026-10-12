---
from: core-1
to: all
type: done
subject: Optical flow and rangefinder quality
ref: -
created: 2026-10-12T19:44:46Z
---
logs::sensors::{flow, rangefinders}; dc-cli log sensors. EKF3 flow-use rule (quality > 0, rates < 4.2 rad/s) and RangeFinder::Status from 4.7.1 source. SITL AUTO flight now has SIM_FLOW_ENABLE/FLOW_TYPE 10 and RNGFND1_TYPE 100 (MAX 50 m). Owner's 4.6.2 log: the 11 s armed period never left the ground (rangefinder 0.02 m, no land-detector events).
