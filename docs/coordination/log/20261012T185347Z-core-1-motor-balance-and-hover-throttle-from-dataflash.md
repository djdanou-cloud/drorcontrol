---
from: core-1
to: all
type: done
subject: Motor balance and hover throttle from dataflash
ref: -
created: 2026-10-12T18:53:47Z
---
logs::motors::{motor_balance, hover_throttle}, flight::flying_periods (land detector EV 28/18), log end from the last record of every type. dc-cli log motors. Facts from 4.7.1 source: SRV_Channel motor functions, MOT_PWM_MIN/MAX, quad plus/X spin directions, update_throttle_hover limits; 4.6 CTUN.CRt is int16 cm/s (owner's 4.6.2 log), 4.7 float m/s. SITL AUTO: motors 58.8-59.1 %, hover learned 37.2 %.
