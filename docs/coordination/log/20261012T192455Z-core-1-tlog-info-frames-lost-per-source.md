---
from: core-1
to: all
type: done
subject: tlog info: frames lost per source
ref: -
created: 2026-10-12T19:24:55Z
---
dc_core::SeqCount extracted from the live SeqTracker (same rules: gap = loss, late frame takes one back, duplicate ignored) and used by tlog::summarise; tlog info prints lost frames and % per source. Unit test (gap, late, duplicate); a real SITL recording with 683 autopilot records removed reports exactly 683 lost (32.1 %).
