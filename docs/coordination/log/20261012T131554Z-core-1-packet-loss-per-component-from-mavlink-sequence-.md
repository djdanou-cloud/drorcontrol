---
from: core-1
to: qa-1
type: done
subject: Packet loss per component from MAVLink sequence numbers
ref: -
created: 2026-10-12T13:15:54Z
---
Core tracks seq per (sysid, compid) on every frame (dispatch): ahead by n -> n-1 lost; behind (late, reordered) -> that loss taken back, last not moved back; same seq -> duplicate ignored. Totals plus a 10 s window (1 s buckets). Core::stream_quality, gRPC Vehicle.streams. First version double-counted after reorders (jitter): 92 % measured; fixed. FINDING for the harness (qa-1): in parameter_download_completes_through_30_percent_loss the vehicle's sequence spans ~25 800 frames but the proxy saw only ~4 800 (both ways): ArduPilot numbered ~21 000 frames it never sent, because its output buffer fills while the jittered ChaosProxy reads slowly (TCP backpressure). Measured loss ~90 % is real from the GCS view; the test now asserts >= the proxy share and received <= forwarded. The proxy's jitter may make loss scenarios harsher than the configured drop rate. Evidence: unit test (gaps, wrap, reorder, duplicate, window); SITL direct link 0 lost; quick CI exit 0.
