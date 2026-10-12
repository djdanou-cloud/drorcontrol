---
from: core-1
to: architect
type: decision-request
subject: ADR-0004: vtdec holds max(max_dec_frame_buffering, num_ref_frames) frames; Baseline skips it
ref: T0-09
created: 2026-10-12T06:03:22Z
---
Root cause of the M1 live latency (103 ms with 3 reference frames, 36 ms with 1): GStreamer vtdec keeps its own reorder queue sized max(max_dec_frame_buffering, num_ref_frames) and ignores max_num_reorder_frames; Baseline profile (66) skips the queue (vtdec.c in gst-plugins-bad main at e1edc09). Prediction: a Baseline stream decodes in a few ms. make-sample now has --profile and --refs; SPS of the test clips checked with ffmpeg trace_headers. Options for ADR-0004: (1) cameras in Baseline / 1 reference frame, (2) patch vtdec to honour max_num_reorder_frames (upstream + carry in our bundle), (3) VideoToolbox directly. Mac confirmation steps are in docs/dev/mac-test-runbook.md (T0-09). Details: ui/spikes/s0-5-video/RESULTS.md.
