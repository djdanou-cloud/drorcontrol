# ADR-0003: `MapSurface` abstraction; MapLibre Native primary, `wgpu` fallback; `wgpu` for 3-D

* Status: proposed — **go/no-go at week 6 (spike S0-3)**
* Date: 2026-10-10

## Context
Need 60 fps Retina vector/raster map with offline tiles, plus 3-D terrain draping and calibration models. Interop of MapLibre Native's Metal backend with Qt RHI textures is unproven in our context.

## Decision
Define `MapSurface`. Implement with MapLibre Native (Metal on macOS) if it meets: 60 fps full screen on the built-in Retina display, ≤ 8 % GPU on M1, correct gestures. (Owner decision 2026-10-12: no 5K display is available; the original "60 fps at 5K" measurement moved to [future upgrades](../future-upgrades.md).) Otherwise implement the 2-D renderer in `wgpu`. The 3-D scene is `wgpu` regardless. Cesium is rejected for the desktop core.

## Consequences
+ No schedule dependency on one renderer; one 3-D path. − Possible extra ~10 person-weeks (reserved).
