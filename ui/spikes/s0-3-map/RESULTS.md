# Spike S0-3 results — MapLibre Native through Qt RHI (Metal) (T0-07)

**Question (plan 01 §5.1, ADR-0003):** can MapLibre Native draw the moving-map with Metal
inside the Qt Quick scene at 60 fps on a 5K Retina display, with ≤ 8 % GPU on an M1 and
correct trackpad pinch and rotate? If not, the fallback is a `wgpu` map renderer (+10
person-weeks, already budgeted).

**Status:** harness ready; **the go/no-go run needs a real Mac** (Mac test runbook). The
`spike-map` workflow builds it with Metal and flies the route on a macOS runner (indicative).

## What is used
* `maplibre-native-qt` at commit `c3485f3` (main). Its last release, v3.0.0 (Oct 2024),
  renders through OpenGL only. `main` replaced that with the "drawables" renderer, which draws
  with Metal, Vulkan or OpenGL 3 through Qt RHI (PR #216). The spike therefore pins a `main`
  commit. **Production would need a tagged release or our own pinned fork.**
* The `MapLibre` Qt Quick item (`import MapLibre 4.0`), Qt ≥ 6.5 (CI: 6.11.2), built with
  `-DMLN_WITH_METAL=ON`.
* Basemap: OpenFreeMap "liberty" vector style (`--style` to change). Network needed.

## Method
* The camera follows a fixed route, set every frame: a figure of eight over a coastline, zoom
  10–15, one full turn of bearing and tilt up to 45° per pass. This is the worst case for the
  renderer, like continuous pinch and rotate gestures.
* Two passes: **cold** (tiles loading) and **warm** (tiles cached). For each pass the report
  gives frames per second, frame interval p50 / p95 / p99 / max, frames that missed a vsync
  (interval > 1.5 refresh periods) and frames over 25 ms, measured with
  `QQuickWindow::frameSwapped`.
* The report includes the graphics API Qt actually used (it must say `Metal`), the window and
  screen size in pixels, and the refresh rate.
* GPU load cannot be read without root. Measure it in parallel with
  `sudo powermetrics --samplers gpu_power -i 1000` and note "GPU HW active residency".
* `--manual` skips the route for the trackpad check: pan, pinch and rotate by hand; the
  report covers the session and is written when the window closes.

## Results

| Where | API | Window (px) | Warm fps / p99 interval | Missed vsync | GPU | Verdict |
|-------|-----|-------------|-------------------------|--------------|-----|---------|
| macOS CI runner, first run (2026-10-11) | — | — | — | — | — | QMapLibre built with Metal (about 13 min) and the harness compiled; start-up failed: `module "MapLibre" is not installed` (QML import path not given to the engine). Fixed in the next commit; not yet re-run |
| macOS CI runner (VM) | see the `spike-map` run summary | — | — | — | — | indicative |
| **M1 Mac, 5K display, full screen** | pending | pending | pending | pending | pending | **decides T0-07** |

Pass (go): API `Metal`; warm pass ≥ 59 fps with < 1 % missed vsync at 5K full screen;
GPU ≤ 8 %; pinch and rotate feel right in `--manual`.

## Run it
```bash
brew install ninja ccache
export QT_ROOT_DIR=$(brew --prefix qt)    # Qt >= 6.5
ui/spikes/s0-3-map/build.sh               # first build: MapLibre from source, ~30-60 min
B=ui/spikes/s0-3-map/build/spike-s0-3-map
$B --fullscreen --report s03-m1-5k.json   # 2 × 60 s; run powermetrics alongside
$B --manual --report s03-m1-manual.json   # trackpad check; close the window to finish
```

## Licence (checked at the pinned commit)
Everything the spike builds is BSD-2-Clause: the core, `src/core/**`, and the Qt Quick item,
`src/quick/**`. That was checked file by file through SPDX headers. The only GPL/LGPL code
(`LGPL-3.0-only OR GPL-2.0-only OR GPL-3.0-only`) is in the Qt Location plugin,
`src/location`, which is turned off here (`MLN_QT_WITH_LOCATION=OFF`). MapLibre Native's
vendored third-party code still needs a licence inventory before shipping; that belongs with
the open licence issue in coordination.
