---
from: core-1
to: all
type: done
subject: H4: mission upload verified by read-back
ref: H4
created: 2026-10-11T10:35:15Z
---
protocols::mission (sans-I/O): MissionUpload (COUNT, answers MISSION_REQUEST[_INT], ACK; INVALID_SEQUENCE is not final, as in ArduPilot; GCS waits while the vehicle drives re-requests; 3 s for the final ACK), MissionDownload (REQUEST_LIST/COUNT/REQUEST_INT/ACK, retries 1 s x15 sized for 30 % loss), verify() item by item with ArduPilot storage rules (home slot skipped, _INT frames, cm altitude, non-location commands, LOITER_TIME direction sign) and strict otherwise. dc-core: upload_mission (refused while armed or armed unknown: in-flight upload needs class C confirmation, not built), download_mission, MissionReport::verified(). Found on SITL: my GCS item re-send provoked INVALID_SEQUENCE and the old code stopped -> partial mission (read-back caught it); fixed. SITL: 14 items through 20 % loss verified, 0 differences, 19 s. Committed locally; not pushed (owner instruction).
