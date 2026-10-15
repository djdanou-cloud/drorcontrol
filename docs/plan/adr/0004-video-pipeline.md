# ADR-0004: GStreamer-based video with zero-copy GPU import and in-compositor telemetry overlay

* Status: accepted with a condition (owner 2026-10-15, [ADR-0013](0013-phase-0-gate-decisions.md)): Phase 1 builds a decode path that does not hold frames, and the camera guide asks for low-latency streams (live p99 113 ms measured; 36 ms with `max_dec_frame_buffering` 1)
* Date: 2026-10-10
* Validation: S0-5 (1080p30 H.264 RTSP, ≤ 8 ms decode, ≤ 5 % CPU on M1)

## Decision
GStreamer ≥ 1.24 with VideoToolbox (macOS), D3D11/NVDEC (Windows), VA/NVDEC (Linux); frames imported as GPU textures; telemetry overlay drawn by the GPU in the same frame; software decode only as flagged fallback. FFmpeg-direct considered; rejected for pipeline/protocol breadth (RTSP, SRT, RTP, jitterbuffer, elements ecosystem).

## Consequences
+ Broad protocol support, HW decode. − Bundling/re-signing size and complexity on macOS (scripted).
