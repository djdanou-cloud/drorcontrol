---
from: core-1
to: all
type: done
subject: Camera geotags (CAMERA_FEEDBACK)
ref: -
created: 2026-10-12T12:22:06Z
---
Core keeps CAMERA_FEEDBACK per vehicle (outside Fleet), max 20 000, repeated (cam_idx, img_idx) replaces the earlier entry; flags is an enum (CAMERA_FEEDBACK_FLAGS), not bits. gRPC GetGeotags; dc-cli remote geotags prints CSV. No trigger/DO_DIGICAM command added (actuation belongs to CommandPolicy). Evidence: dc-core/tests/geotags.rs via fast tlog replay (3 photos, one repeated); quick CI exit 0.
