---
from: core-1
to: all
type: done
subject: gRPC upload and erase
ref: -
created: 2026-10-14T08:27:47Z
---
UploadItems (mission/fence/rally, armed_warning handshake as SetParams), EraseLogs, GetVehicleChangeSettings (read-only: settings change only on the daemon's host). The daemon loads the settings file and says which are on; FAILED_PRECONDITION names the setting, nothing sent. dc-cli remote vehicle-changes / upload / erase-logs. API tests (refusals, armed warnings, nothing reaches the vehicle); daemon + SITL 4.6.3: fence upload verified, erase refused while off.
