# Spike S0-5 results — 1080p30 H.264 decode latency and CPU (T0-09)

**Question (plan 01 §6, §9):** does the platform's hardware decoder keep 1080p30 H.264 decode at
≤ 8 ms per frame and the process at ≤ 5 % CPU on an M1, so video never delays the operator?

**Status:** first real-Mac run done on an M1 (2026-10-11): **hardware decode, zero-copy handle
and CPU pass; decode p99 fails (405 ms) in file mode, very likely because of how file mode is
timed** (see below). The deciding live RTSP run is still to do. CI's macOS runner records
indicative numbers (job "video spike S0-5").

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
* The JSON report says whether a **zero-copy handle** comes with each frame: on macOS the
  IOSurface-backed `CVPixelBuffer` (`GstCoreVideoMeta`), which the GCS imports into Metal with
  `CVMetalTextureCache`; elsewhere GPU-memory caps (`memory:VAMemory`, `memory:DMABuf`, …).
  The default output is system-memory caps, so no OpenGL is involved; `--output any` lets
  `vtdec` pick `memory:GLMemory` instead (recorded in CI for comparison only).
* Not measured here: import into the Qt scene graph and the telemetry overlay (the Qt video
  item, Phase 1).

## Results

| Where | Decoder | Decode p50 / p99 (ms) | CPU (% of one core) | Zero-copy handle | Verdict |
|-------|---------|-----------------------|---------------------|------------|---------|
| Linux container, 4 shared vCPU, no GPU (2026-10-11) | `avdec_h264` (software) | 4.98 / 10.28 | 43.9 | no | Harness check only. Software decode misses both targets, as expected |
| macOS CI runner, 3-core arm64 VM, GStreamer 1.28.6, first harness version (2026-10-11) | `vtdec_hw`, output `memory:GLMemory` | 365 / 533 | 43.4 | yes (GL texture) | **Invalid measurement**: GStreamer warned that its GL path needs a Cocoa run loop; frames waited on the GL hand-off. Fixed: harness runs under `gst_macos_main`, default output is system-memory caps (the `CVPixelBuffer` handle is still attached) |
| same runner | `avdec_h264` (software) | 6.81 / 17.42 | 55.1 | no | Software misses both targets |
| macOS CI runner, fixed harness | see the CI job summary | — | — | — | indicative |
| M1 MacBook Air, file, synced (2026-10-11) | `vtdec_hw`, system-memory caps | 341.2 / 404.9 | **2.2** | **yes** (`GstCoreVideoMeta`) | Hardware, zero-copy and CPU pass; latency fails, very likely a file-mode artefact (below) |
| same Mac, file, synced | `avdec_h264` (software) | 9.85 / 14.45 | 99.7 | no | Software misses both targets |
| **M1 Mac, live RTSP (unsynced)** | `vtdec_hw` | pending | pending | pending | **decides T0-09** |

### M1 Mac run (2026-10-11, mactest-1)
* MacBook Air M1 (8 cores), macOS 27.0.1, AC power, GStreamer 1.28.8 (Homebrew). Clip from
  `make-sample` (x264enc, 90 s). 60 s measured per run, 1800 frames, 29.99 fps, 0 unmatched.
* `vtdec_hw` output NV12 with `GstCoreVideoMetaAPI` attached, so the `CVPixelBuffer` handle is
  there for a Metal import.
* **Why the hardware latency is probably not decode time.** The GL path is gone (system-memory
  caps), yet p50 is still in the range of the first CI run (365 ms), and on the M1 the software
  decoder took 10–14 ms on the same clip. In file mode the sink is synced to the clock, but
  the demuxer feeds the decoder as fast as queues allow, and latency is timed from decoder
  input. A decoder that holds more frames internally (VideoToolbox) then shows the wait for the
  clock: ≈ 10 frames × 33 ms ≈ 341 ms matches p50. **Not verified.** Two ways to settle it: the live RTSP run (unsynced,
  the case the GCS runs) or a file run with the sink unsynced and input paced at 30 fps.
* Run note: GStreamer's macOS start-up (`gst_macos_main`) waits for `NSApplication`, which
  never starts without a desktop session, so over SSH even `gst-inspect-1.0 --version` hangs.
  Run from a terminal in a logged-in desktop session.

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
Pass: `"pass": {"decode_p99_le_8ms": true, "cpu_le_5pct": true}` with `"hardware": true` and
`"zero_copy_handle": true`.
