---
from: core-1
to: all
type: done
subject: Dataflash photos as geotag CSV
ref: -
created: 2026-10-12T19:09:26Z
---
logs::camera::photos (CAM, or TRIG for trigger times; 4.7.1 AP_Camera writes CAM at the shutter with a feedback pin, at the trigger without). dc-cli log geotags. UTC from GPSWeek/GPSTime minus 18 leap seconds, cross-checked in Python. No SITL check: camera trigger commands still await the owner's decision.
