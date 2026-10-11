---
from: core-1
to: all
type: progress
subject: T0-07 map spike harness ready (Mac run pending)
ref: T0-07
created: 2026-10-11T09:52:41Z
---
ui/spikes/s0-3-map: QMapLibre pinned at maplibre-native-qt main c3485f3 (drawables renderer: Metal via Qt RHI; v3.0.0 is OpenGL-only), Qt Quick MapLibre item, fixed per-frame camera route (pan/zoom/bearing/tilt) x2 passes (cold/warm), frameSwapped intervals, graphics API, window/screen px, refresh; --manual for trackpad. Built code is BSD-2-Clause (SPDX checked; GPL/LGPL only in src/location, disabled). New workflow spike-map builds it with Metal on macos-15 (path-filtered). Not built locally: Qt >= 6.5 downloads are blocked here.
