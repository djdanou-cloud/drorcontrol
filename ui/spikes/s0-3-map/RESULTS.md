# Spike S0-3 results — MapLibre Native through Qt RHI (Metal) (T0-07)

**Question (plan 01 §5.1, ADR-0003):** can MapLibre Native draw the moving-map with Metal
inside the Qt Quick scene at 60 fps on a 5K Retina display, with ≤ 8 % GPU on an M1 and
correct trackpad pinch and rotate? If not, the fallback is a `wgpu` map renderer (+10
person-weeks, already budgeted).

**Status:** first real-Mac run done on an M1 at 2880×1800 (2026-10-11): **Metal, frame rate,
vsync and feel pass; GPU load fails (≈ 42 % vs ≤ 8 %).** The 5K run that the criterion names is
still to do. Go/no-go is ADR-0003's call. The `spike-map` workflow builds it with Metal and
flies the route on a macOS runner (indicative).

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
| M1 MacBook Air, built-in Retina, full screen (2026-10-11) | `Metal` | 2880×1800 | 59.98 / 22.63 ms | 19 / 3598 (0.53 %) | ≈ 42 % (mean; max 56 %) | fps, vsync and feel pass; **GPU fails**. Not 5K |
| **M1 Mac, 5K display, full screen** | pending | pending | pending | pending | pending | **decides T0-07** |

Pass (go): API `Metal`; warm pass ≥ 59 fps with < 1 % missed vsync at 5K full screen;
GPU ≤ 8 %; pinch and rotate feel right in `--manual`.

### M1 Mac run (2026-10-11, mactest-1)
* MacBook Air M1, macOS 27.0.1, AC power, Low Power Mode off; built-in Retina display
  2880×1800 at 60 Hz, device pixel ratio 2 (no 5K display was available). Qt 6.11.2 (Homebrew),
  QMapLibre at `c3485f3` with `-DMLN_WITH_METAL=ON`, style OpenFreeMap "liberty".

| Pass | fps | Frames | Missed vsync | Frames > 25 ms | Interval p50 / p95 / p99 / max (ms) |
|--|--:|--:|--:|--:|--|
| cold | 59.69 | 3582 | 11 (0.31 %) | 11 | 16.66 / 19.21 / 22.38 / 164.9 |
| **warm** | **59.98** | 3598 | **19 (0.53 %)** | 19 | 16.67 / 19.11 / 22.63 / 36.0 |
| manual (262 s, idle most of the time) | — | 1136 | 15 | 15 | 16.67 / 18.65 / 26.14 / 2764.7 |

* **GPU:** `powermetrics --samplers gpu_power -i 1000` over both passes (125 samples): "GPU HW
  active residency" mean 42.2 % (cold half 40.5 %, warm half 43.9 %), max 56.4 %, all of it at
  the lowest DVFS state (396 MHz). Residency at the lowest clock overstates load compared with
  a GPU running faster for less time, but it is far from ≤ 8 %. A 5K window has about 2.8× the
  pixels, so the 5K figure is unlikely to be lower.
* **Trackpad:** pinch and rotate in `--manual` felt smooth (tester's verdict).
* Log line during both runs: `[ WARNING ] "{unknown}[General]: Invalid geometry in line layer"`
  (style data; no visible effect noted).
* Build note: Homebrew Qt's `qt.toolchain.cmake` sets `CMAKE_OSX_SYSROOT` to the Command Line
  Tools SDK (`MacOSX27.sdk` here), which Xcode's linker rejected ("unknown architecture",
  `.tbd` stubs), so `build.sh` failed at CMake's compiler check. The run used the script's
  steps with `-DCMAKE_OSX_SYSROOT=$(xcrun --sdk macosx --show-sdk-path)` added to both
  `qt-cmake` calls. `build.sh` now does this itself on macOS (not yet re-run on a Mac).

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
