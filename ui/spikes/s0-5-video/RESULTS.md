# Spike S0-5 results — 1080p30 H.264 decode latency and CPU (T0-09)

**Question (plan 01 §6, §9):** does the platform's hardware decoder keep 1080p30 H.264 decode at
≤ 8 ms per frame and the process at ≤ 5 % CPU on an M1, so video never delays the operator?

**Status:** harness ready; **the pass/fail run needs a real Mac** (see the Mac test runbook).
CI's macOS runner records indicative numbers on every push (job "video spike S0-5").

## Method
* `make-sample` encodes a 1080p30 clip of a moving zone plate (detail and motion in every frame):
  H.264 High, 4:2:0 8-bit, no B-frames, constant 8 Mbit/s, GOP 60. These are the settings of a
  typical drone camera.
* `run` decodes the clip in real time (sink synced to the clock, as a live 30 fps stream) or an
  `rtsp://` stream (unsynced, as the GCS will run it). Decoder: the first available of
  `vtdec_hw`, `vtdec`, `avdec_h264` on macOS. `avdec_h264` (software) uses slice threading, as
  the GCS's software fallback would; its default frame threading adds about 3 frames (≈ 100 ms).
* **Decode latency** is measured per frame by PTS, from the buffer entering the decoder to the
  frame leaving it. It includes the decoder's own queueing. **CPU** is process user and system
  time over wall time, in % of one core (Activity Monitor's scale). The first 3 s are ignored.
* The JSON report also records the decoder's output caps: `memory:GLMemory`, `memory:VAMemory`
  and similar mean the frame stayed in GPU memory (zero-copy is possible); plain `video/x-raw`
  means a copy to system memory.
* Not measured here: import into the Qt scene graph and the telemetry overlay (the Qt video
  item, Phase 1).

## Results

| Where | Decoder | Decode p50 / p99 (ms) | CPU (% of one core) | GPU memory | Verdict |
|-------|---------|-----------------------|---------------------|------------|---------|
| Linux container, 4 shared vCPU, no GPU (2026-10-11) | `avdec_h264` (software) | 4.98 / 10.28 | 43.9 | no | Harness check only. Software decode misses both targets, as expected |
| macOS CI runner (VM) | see the CI job summary | — | — | — | indicative |
| **M1 Mac** | `vtdec_hw` | pending | pending | pending | **decides T0-09** |

## Run it
```bash
brew install gstreamer pkgconf
cargo build --release --manifest-path ui/spikes/s0-5-video/Cargo.toml
B=ui/spikes/s0-5-video/target/release/spike-s0-5-video
$B make-sample sample.mkv 90
$B run sample.mkv --seconds 60 --report s05-m1-file.json
$B run rtsp://<camera>/stream --seconds 60 --report s05-m1-rtsp.json   # if a camera is available
$B run sample.mkv --seconds 60 --decoder avdec_h264 --report s05-m1-sw.json  # software fallback
```
Pass: `"pass": {"decode_p99_le_8ms": true, "cpu_le_5pct": true}` with `"hardware": true`.
