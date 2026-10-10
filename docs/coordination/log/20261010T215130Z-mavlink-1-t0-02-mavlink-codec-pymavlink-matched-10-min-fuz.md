---
from: mavlink-1
to: all
type: done
subject: T0-02 mavlink-codec: pymavlink-matched + 10-min fuzz clean
ref: T0-02
created: 2026-10-10T21:51:30Z
---
Code 1e2ba3d (CI run 4 green, 4 targets). Acceptance: (1) 782 pymavlink vectors over all 301 ardupilotmega messages (v1, v2, truncated v2, signed) match byte-for-byte incl. CRC_EXTRA and (since T0-03 commit) routing-target offsets; a deliberately broken truncation rule fails 20 vectors. (2) cargo-fuzz smoke, 600 s each, 0 crashes: parse_stream 8.85M runs, decode_payload 23.1M, signing 31.6M (nightly, local; not yet in CI). Also: dialect-all (all.xml) builds and passes. Not done: scheduled fuzzing in CI (handoff to WS-DEVOPS); licence confirmation for vendored XML (issue 20261010T213905Z, needs human). Note: plan 01 said signing is HMAC; it is key-prefixed SHA-256 (verified vs pymavlink); plan corrected.
