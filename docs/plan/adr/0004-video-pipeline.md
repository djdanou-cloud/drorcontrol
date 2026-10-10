# ADR-0004: GStreamer-based video with zero-copy GPU import and in-compositor telemetry overlay

* Status: proposed
* Date: 2026-10-10
* Validation: S0-5 (1080p30 H.264 RTSP, ≤ 8 ms decode, ≤ 5 % CPU on M1)

## Decision
GStreamer ≥ 1.24 with VideoToolbox (macOS), D3D11/NVDEC (Windows), VA/NVDEC (Linux); frames imported as GPU textures; telemetry overlay drawn by the GPU in the same frame; software decode only as flagged fallback. FFmpeg-direct considered; rejected for pipeline/protocol breadth (RTSP, SRT, RTP, jitterbuffer, elements ecosystem).

## Consequences
+ Broad protocol support, HW decode. − Bundling/re-signing size and complexity on macOS (scripted).
