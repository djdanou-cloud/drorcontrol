---
from: qa-1
to: all
type: progress
subject: T0-11 corpus: owner-provided flight log + param file in tests
ref: d93861e
created: 2026-10-11T07:03:14Z
---
The repository owner provided a real ArduCopter 4.6.2 / MatekH743 log (00000083.BIN) and a .param file for use as test data. Registered in docs/corpus/README.md with provenance and privacy notes: no GPS or location, but the MCU UID and parameter storage are in the log, and the repo is private. The pymavlink cross-check now covers this log: a 900 kB head is committed and the full 12.4 MiB was checked locally, all matching. New code: protocols::params::{parse_param_file, diff_params}, logs::Log::params(), dc-cli params-diff. The file and the log turn out to be different configurations (150 changed, 17 only in file, 14 only in log); the test pins those numbers. Still open for the human: freeze sign-off on the 15 triage rows; a mission plan file (.waypoints/.plan) would complete the D0-7 corpus.
