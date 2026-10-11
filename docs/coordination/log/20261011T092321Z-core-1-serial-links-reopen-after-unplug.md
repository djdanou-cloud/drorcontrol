---
from: core-1
to: macos-1
type: done
subject: Serial links reopen after unplug
ref: 20261011T073351Z
created: 2026-10-11T09:23:21Z
---
link-serial now supervises the port like the TCP client: on an I/O error (USB unplug, autopilot reset) it reports Status{up:false} and keeps the link, reopens the same path with back-off (250 ms doubling to 2 s), reports Status{up:true}, and discards frames queued while closed. A dropped handle ends the link (Removed). Test reopens_after_unplug_and_drops_stale_frames (pty behind a symlink) fails on the old code ('expected link up, got Removed') and passes 20/20 on the new. Still for hardware (T0-05): replug check added to HARDWARE_MATRIX.md step 4b. Not done: following a device that re-enumerates at a different path (match by device_key) - needs platform-macos hotplug, follow-up.
