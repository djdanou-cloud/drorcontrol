# Spike S0-5 results — 1080p30 H.264 decode latency and CPU (T0-09)

**Question (plan 01 §6, §9):** does the platform's hardware decoder keep 1080p30 H.264 decode at
≤ 8 ms per frame and the process at ≤ 5 % CPU on an M1, so video never delays the operator?

**Status:** real-Mac runs done on an M1 (2026-10-11), including live RTSP. **Hardware decode and
the zero-copy handle pass; decode latency fails (p99 113 ms live) and CPU is just over (6.4 %).**
The latency is frames held by the hardware decoder path, one per frame of decode buffer the
stream allows (`max_dec_frame_buffering`): 103 ms with 3, 36 ms with 1 (see below). Meeting
≤ 8 ms needs a decode path that outputs each frame at once. CI's macOS runner records
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
| M1 Mac, live RTSP (unsynced), localhost relay | `vtdec_hw` | **103.0 / 112.8** | 6.38 | yes | **Fails** latency and CPU; ≈ 3 frames held (below) |
| same, live RTSP | `avdec_h264` (software) | 9.6 / 14.7 | 99.7 | no | Software misses both targets |
| same, live RTSP, clip re-encoded with 1 reference frame | `vtdec_hw` | **36.2 / 46.2** | 6.59 | yes | ≈ 1 frame held: latency follows the decode buffer |

### M1 Mac run (2026-10-11, mactest-1)
* MacBook Air M1 (8 cores), macOS 27.0.1, AC power, GStreamer 1.28.8 (Homebrew). Clip from
  `make-sample` (x264enc, 90 s). 60 s measured per run, 1800 frames, 29.99 fps, 0 unmatched.
* `vtdec_hw` output NV12 with `GstCoreVideoMetaAPI` attached, so the `CVPixelBuffer` handle is
  there for a Metal import.
* **File mode overstates the latency.** With the sink synced to the clock, frames also wait for
  their display time inside the decoder's queue: 341 ms p50 in file mode vs 103 ms live.
* **Live RTSP setup:** mediamtx 1.21.2 relay on 127.0.0.1:8554 (RTSP over TCP only), `ffmpeg -re
  -c copy` publishing the clip in real time, spike `run rtsp://127.0.0.1:8554/cam`. 1800 frames,
  29.99 fps, 0 unmatched. CPU now includes `rtspsrc` and depayloading.
* **The hardware decoder path holds as many frames as the stream's decode buffer allows.** The
  `make-sample` clip (x264, default 3 reference frames) signals `max_num_reorder_frames = 0`
  but `max_dec_frame_buffering = 3`. Re-encoding the same clip with ffmpeg/libx264 and
  identical settings except the reference count gave:

  | Reference frames | `max_dec_frame_buffering` | `vtdec_hw` p50 / p99 (ms) |
  |--:|--:|--|
  | 3 (control; matches the `make-sample` clip) | 3 | 103.0 / 112.7 |
  | 1 | 1 | 36.2 / 46.2 |

  So latency ≈ buffer × 33 ms + ~3 ms: the decode itself takes a few ms, and the frames are
  held by GStreamer's `vtdec` or VideoToolbox even though the stream says no reordering is
  needed. Which of the two holds them was not determined. A real camera's buffer setting will
  decide its latency on this path; even with 1 frame it is 46 ms p99.
* **Next step (not done):** a decode path that emits each frame as soon as it is decoded, e.g.
  VideoToolbox directly, or `vtdec` told to trust `max_num_reorder_frames`. Then re-measure.

### Why the hardware path holds frames (source, 2026-10-12, core-1)
GStreamer's `vtdec` keeps its own reorder queue and pushes a decoded frame downstream only once
more than `dpb_size` frames are queued (`gst_vtdec_output_loop`, `sys/applemedia/vtdec.c` in
gst-plugins-bad, GStreamer `main` at e1edc09):

* `dpb_size = MAX(max_dec_frame_buffering, num_ref_frames)` when the SPS has
  `bitstream_restriction_flag`; otherwise it is the level's maximum DPB, which can be larger
  (`get_h264_dpb_size_from_sps`).
* `max_num_reorder_frames` is never read, so a stream without B-frames still waits.
* **Baseline profile (`profile_idc` 66) skips the queue:** `dpb_size = 0`
  (`compute_h264_decode_picture_buffer_size`).

That matches the M1 numbers: 3 frames held ≈ 103 ms, 1 frame ≈ 36 ms. It predicts that a
**Baseline** stream decodes in a few ms on the same path. `make-sample` now takes
`--profile high|main|baseline` and `--refs N` to test that. SPS fields checked with
`ffmpeg -bsf:v trace_headers`:

| Clip | `profile_idc` | `max_num_ref_frames` | `max_dec_frame_buffering` | `max_num_reorder_frames` | Frames `vtdec` holds (predicted) |
|---|--:|--:|--:|--:|--:|
| `make-sample s.mkv 90 --profile baseline` | 66 | 3 | 3 | 0 | **0** |
| `make-sample s.mkv 90 --refs 1` | 100 | 1 | 1 | 0 | 1 (M1: 36 ms p50) |
| `make-sample s.mkv 90` (default) | 100 | 3 | 3 | 0 | 3 (M1: 103 ms p50) |

The Baseline clip keeps 3 reference frames and a 3-frame buffer, so a low result on it
isolates the profile shortcut.

Options for ADR-0004, from least to most work:
1. **Run cameras in Baseline profile** where the camera allows it (or 1 reference frame:
   about 36 ms). This is a configuration choice; it costs some bitrate efficiency.
2. **Patch `vtdec`** to use `max_num_reorder_frames` when `bitstream_restriction_flag` is
   set. This is what decoders like FFmpeg do. Propose it upstream, and carry it in our bundled
   GStreamer meanwhile.
3. **Use VideoToolbox directly** (no `vtdec`): emit each frame as soon as it is decoded, and
   reorder only by `max_num_reorder_frames`. This is the most control and the most code.

To confirm on a Mac (live, as before, through the mediamtx relay):
`make-sample` each clip above, publish it, then
`run rtsp://127.0.0.1:8554/cam --seconds 60 --report s05-m1-<clip>.json`.
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
