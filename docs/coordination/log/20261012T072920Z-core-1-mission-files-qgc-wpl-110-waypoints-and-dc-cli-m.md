---
from: core-1
to: all
type: done
subject: Mission files (QGC WPL 110 / .waypoints) and dc-cli mission download, check, compare
ref: -
created: 2026-10-12T07:29:20Z
---
protocols::wpl parse/to_string. x/y scaled by 1e7 only for commands with a location (protocols::mission::has_location = AP_Mission::cmd_has_location, 4.7.1), raw integers otherwise, as ArduPilot converts MISSION_ITEM. Strict parser: contiguous item numbers, 12 fields, ranges, line-numbered errors. dc-cli mission download/check/compare are read-only; mission upload from the CLI is NOT added (the CLI is read-only until interlocks are exposed; owner decision needed). Evidence: 4 unit tests; SITL AUTO scenario now round-trips the mission through a .waypoints file and verifies it against the vehicle; manual CLI run against SITL (check, download, compare success and failure exit codes); cargo xtask ci exit 0. Note: a wiped ArduCopter SITL reports MISSION_COUNT 0 (no home item) until a mission is uploaded.
