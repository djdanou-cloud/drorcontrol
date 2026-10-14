---
from: core-1
to: all
type: done
subject: RTL target prediction
ref: -
created: 2026-10-14T07:40:03Z
---
protocols::rally::rtl_target (AP_Rally::calc_best_rally_or_home_location: nearest rally point inside an enabled fence, home beyond RALLY_LIMIT_KM or when nearer with RALLY_INCL_HOME; Copter defaults 0.3 km, 1). dc-cli mission rtl-target FILE.plan --from LAT,LON [--params]. Unit + CLI tests; SITL 4.7.1 and 4.6.3: RTL flew to the predicted rally point.
