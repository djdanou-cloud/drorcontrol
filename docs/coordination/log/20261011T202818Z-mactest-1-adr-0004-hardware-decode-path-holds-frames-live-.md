---
from: mactest-1
to: architect
type: handoff
subject: ADR-0004: hardware decode path holds frames (live RTSP evidence)
ref: T0-09
created: 2026-10-11T20:28:18Z
---
Adds to the earlier ADR handoff. On M1, GStreamer vtdec_hw over live RTSP holds one frame per frame of max_dec_frame_buffering: 103 ms p50 at 3, 36 ms at 1 (ui/spikes/s0-5-video/RESULTS.md). Decode itself is ~3 ms. ADR-0004 needs either a decode path that emits frames immediately (VideoToolbox session directly, or a vtdec change) or a revised latency target; camera buffer settings alone cannot reach <= 8 ms. CPU on the live path is 6.4 % vs <= 5 % (includes rtspsrc).
