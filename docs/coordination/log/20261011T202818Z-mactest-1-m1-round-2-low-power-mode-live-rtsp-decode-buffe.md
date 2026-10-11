---
from: mactest-1
to: all
type: progress
subject: M1 round 2: Low Power Mode, live RTSP, decode-buffer test, first launch
ref: T0-09
created: 2026-10-11T20:28:18Z
---
Same MacBook Air M1, 2026-10-11 22:57-23:30.

T0-06 (S0-1) in Low Power Mode: frame p99 18.76/20.36/19.14 ms at 0/200k/400k; 1 frame > 25 ms (33.7 ms) at 200k, 0 otherwise. ~1 ms worse than normal power.

T0-09 (S0-5) live RTSP (localhost mediamtx relay, ffmpeg -re publisher): vtdec_hw p50/p99 103.0/112.8 ms, CPU 6.38 %; avdec_h264 9.6/14.7 ms at 99.7 %. Same clip re-encoded with 1 reference frame (max_dec_frame_buffering 1 instead of 3, everything else equal; 3-ref control reproduced 103 ms): vtdec_hw 36.2/46.2 ms. Latency = held frames x 33 ms + ~3 ms decode. Fails <= 8 ms on this path whatever the camera; needs a decode path that emits each frame at once (VideoToolbox directly, or vtdec honouring max_num_reorder_frames=0). Not investigated which layer holds the frames.

T0-10 first launch (local build of bfcaee6 with packaging/macos/package.sh): universal arm64+x86_64, codesign --verify --deep --strict valid, dc-cli 0.0.0, zip+dmg+SHA256SUMS. Simulated browser download (com.apple.quarantine on the dmg): shasum -c OK x2; quarantine propagated to the copied app; spctl 'rejected' and dc-cli killed (exit 137) before approval, as ADR-0010 expects. 'Open Anyway' was clicked in a different macOS account from the app's owner, so the approval was not recorded (quarantine flags unchanged, still exit 137). Steps 4-6 unverified; this is a test-setup artefact, not a product finding. No FIRST-LAUNCH.txt wording issues reported.

Details: ui/spikes/s0-1-bridge/RESULTS.md, ui/spikes/s0-5-video/RESULTS.md.
