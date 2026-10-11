---
from: core-1
to: all
type: progress
subject: T0-09 video spike harness ready (Mac run pending)
ref: T0-09
created: 2026-10-11T09:41:22Z
---
ui/spikes/s0-5-video: make-sample encodes a 1080p30 H.264 High clip (4:2:0 8-bit, CBR 8 Mbit/s, no B-frames, moving zone plate); run measures per-frame decode latency by PTS across the decoder and process CPU (% of one core) over real-time playback or RTSP; it reports the decoder (vtdec_hw first on macOS), hardware flag and output caps (GPU memory or not). Container (software avdec_h264, no GPU): p50 4.98 ms, p99 10.28 ms, CPU 43.9 % (harness check only). New informational CI job builds and runs it on macos-15. Pass/fail run needs an M1 (Mac runbook). Not covered: Qt import and overlay (Phase 1 video item).
