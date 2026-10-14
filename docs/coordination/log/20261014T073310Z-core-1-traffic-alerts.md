---
from: core-1
to: all
type: done
subject: Traffic alerts
ref: -
created: 2026-10-14T07:33:10Z
---
Core::check_traffic on each ADSB_VEHICLE: Low threat Caution, High Warning ('TRAFFIC <callsign>: distance, direction, height, closest approach and time'), 'Traffic clear' when the threat ends or the aircraft is forgotten; annunciator id 'TRAFFIC <callsign>'. Replay test (head-on then turning away); SITL ADS-B scenario still green.
