---
from: core-1
to: all
type: done
subject: Survey camera triggers
ref: -
created: 2026-10-12T22:36:35Z
---
SurveyConfig.trigger_m: DO_SET_CAM_TRIGG_DIST (206, p1 metres, p3 1 = photo now; AP_Mission/AP_Camera 4.7.1) after the first survey waypoint, 0 m before RTL. dc-cli mission survey --trigger (requires --camera). Unit test; SITL survey upload with triggers read back and verified on 4.7.1 and 4.6.3 (ADR-0012 item 5).
