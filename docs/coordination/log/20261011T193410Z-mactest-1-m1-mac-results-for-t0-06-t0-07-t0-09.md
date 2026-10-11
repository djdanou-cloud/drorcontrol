---
from: mactest-1
to: all
type: progress
subject: M1 Mac results for T0-06, T0-07, T0-09
ref: T0-06
created: 2026-10-11T19:34:10Z
---
MacBook Air M1, macOS 27.0.1, Qt 6.11.2, GStreamer 1.28.8, 60 Hz Retina 2880x1800; run 2026-10-11 from the logged-in desktop session.

T0-06 (S0-1): 0 frames > 25 ms at 0/200k/400k msg/s; frame p99 17.79/19.49/18.94 ms (strict 16.7 ms bound missed, also at idle). tick() p50 3.8 ms at 200k (Linux 0.88 ms).
T0-07 (S0-3): graphics API Metal; warm 59.98 fps, 19/3598 missed vsync (0.53 %); pinch/rotate smooth; GPU HW active residency mean 42.2 % (max 56.4 %) vs <= 8 %. Not run at 5K.
T0-09 (S0-5): vtdec_hw hardware + zero-copy (GstCoreVideoMeta), CPU 2.2 % of one core; file-mode decode p99 404.9 ms (p50 341.2) vs <= 8 ms, very likely a file-mode timing artefact (unverified); software avdec_h264 p99 14.45 ms at 99.7 % CPU. Live RTSP run not done (no camera).

Also on this Mac: cargo xtask ci passed; SITL suite 9/9 on the second full run (first run lost one SITL at start-up on instance 124 / port 7000; that test passed 2/2 alone).
Details: ui/spikes/*/RESULTS.md. Not done: T0-05 hardware matrix (no devices; SiLabs CP210x driver extension is installed and must be removed first), T0-10 first launch, 5K, Intel, 120 Hz, Low Power Mode.
