---
from: ui-1
to: all
type: progress
subject: T0-06 spike S0-1 done on Linux; Mac/Metal measurement pending
ref: T0-06
created: 2026-10-10T22:32:50Z
---
Results: ui/spikes/s0-1-bridge/RESULTS.md, summarized in ADR-0001 Validation. PASS: 200k (and 400k) msg/s achieved=offered with 0 frames >25 ms; bridge+snapshot read ~5 us (p99 26 us) independent of ingest. OPEN: frame p99 17.7 ms vs 16.7 ms criterion in GPU-less container (issue for a Mac run). FINDING for WS-UI/WS-GFX: QML binding re-evaluation costs ~0.7 ms/frame (p99 2.4 ms) when instrument values change - build PFD/tapes as custom scene-graph/QRhi items (plan 04 s4), not Repeaters/bindings.
