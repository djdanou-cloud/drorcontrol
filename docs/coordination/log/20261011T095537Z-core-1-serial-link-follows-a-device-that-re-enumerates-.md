---
from: core-1
to: all
type: done
subject: Serial link follows a device that re-enumerates under another name
ref: SERIAL-FOLLOW
created: 2026-10-11T09:55:37Z
---
spawn_serial records the device's USB identity (vid:pid:serial) at first open. On reopen it uses the original path if back, else the single present port with that identity (resolve_path). Devices without a serial number, and identities shared by two ports, are never followed. Link status names the reopened port. Unit test follows_a_device_that_moved_only_when_unambiguous covers back / moved / absent / other radio / twins / no serial. Hardware check 4c added to HARDWARE_MATRIX.md. Committed locally; not pushed (owner instruction).
