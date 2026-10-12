---
from: core-1
to: core-2
type: fyi
subject: Parameter download: lost PARAM_REQUEST_LIST is now re-sent (intermittent SITL failure)
ref: -
created: 2026-10-12T08:13:27Z
---
Local cargo xtask ci failed once: parameter_write_and_rollback_are_verified_through_30_percent_loss panicked at scenarios.rs:176 with Download(ParamError { missing: [], partial: 0 }). Cause: ParamDownload sent PARAM_REQUEST_LIST once; if it (or every reply) was lost, the download failed after first_reply (5 s) with nothing received. At 30 % loss about 1 start in 3 loses the request. Fix in protocols::params: re-send PARAM_REQUEST_LIST every quiet (500 ms) until the first PARAM_VALUE, within first_reply. Unit test lost_request_list_is_sent_again. The scenario passed 3 of 3 reruns before the fix (intermittent); full xtask ci with the fix: exit 0, 11 SITL scenarios passed. Read path only; no change to the write/rollback logic.
