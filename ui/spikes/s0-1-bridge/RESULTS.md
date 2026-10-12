# Spike S0-1 results — cxx-qt bridge under 200 k msg/s

**Question (ADR-0001):** can the Rust core ingest MAVLink at 200 k msg/s while a Qt Quick UI fed
through a cxx-qt bridge keeps rendering every frame, with UI-thread cost independent of ingest?

## Setup
* Path under test is the real one: synthetic ATTITUDE / VFR_HUD / GLOBAL_POSITION_INT / HEARTBEAT
  frames → UDP (20 frames/datagram) → `dc-core` UDP link → router hub → decode → `Fleet` snapshot
  (`arc-swap`). QML `FrameAnimation` calls `Telemetry.tick()` (Rust, cxx-qt 0.10) once per frame:
  one wait-free snapshot load, then one property write per field.
* Scene: attitude horizon (rotation + 19 ladder rectangles bound to pitch) + 8 formatted text readouts.
* Environment: Linux container, 4 vCPU (shared), **no GPU** — Xvfb + Mesa software OpenGL,
  Qt **6.4.2** (Ubuntu 24.04 packages), release build. 3 s warm-up, 20 s measured (~1250 frames).
* Generator paces by sleeping (an earlier spinning generator stole a full core; numbers were similar).

## Results

| Offered msg/s | Achieved (median) | Frame interval p50 / p99 / max (ms) | Frames > 25 ms | tick() p50 / p99 (µs) | Snapshot load p50 / p99 (µs) |
|--:|--:|--|--:|--|--|
| 0 (idle) | 0 | 16.21 / 16.52 / 20.14 | 0 | 7.0 / 25.6 | 6.3 / 24.9 |
| 0, values animated (`S01_FAKE=1`) | 0 | 15.84 / 18.05 / 38.27 | 1 | 734 / 2407 | 6.4 / 25.5 |
| 50 000 * | 50 002 | 15.85 / 18.22 / 22.70 | 0 | 845 / 2421 | — |
| **200 000** | **200 005** | **15.87 / 17.74 / 19.32** | **0** | 878 / 2582 | **5.4 / 25.6** |
| 400 000 * | 399 827 | 15.89 / 18.77 / 22.58 | 0 | 918 / 3100 | — |

\* earlier run with the spinning generator, before the snapshot-load timer was added.

## Findings
1. **Ingest keeps up to 400 k msg/s** with achieved = offered, and **no frame exceeded 25 ms** in any
   loaded run (1250 frames each). The UI thread is never starved by ingest.
2. **The bridge/snapshot read costs ~5 µs (p99 ~26 µs) and does not grow with ingest rate** — the
   four-plane design (ADR-0002) works as intended.
3. **Per-frame UI cost is dominated by QML binding re-evaluation** (~0.73 ms p50, 2.4 ms p99 when values
   change), identical with or without ingest. Ingest adds ~0.14 ms p50 (CPU contention on 4 shared
   vCPUs). ⇒ Instruments (PFD, tapes, ladders) must be custom scene-graph / QRhi items as plan 04 §4
   specifies, not QML property bindings; text readouts should update at a lower rate or via a single
   formatted model, and the PFD must not be built from Repeaters.
4. **Strict criterion "frame p99 ≤ 16.7 ms" is not met here** (17.7 ms at 200 k; 16.5 ms idle). On a
   60 Hz loop any jitter pushes p99 past 16.67 ms; here the jitter comes from software rendering on
   oversubscribed shared vCPUs. The criterion should be read as "no dropped frames" (met) plus a p99
   bound measured on target hardware.
5. cxx-qt 0.10 + Qt 6.4: `#[qobject] #[qml_element] #[qproperty] #[qinvokable]` built first time with a
   pure-Cargo build (`QMAKE=qmake6`); cold release build ~2.5 min.

## M1 Mac run (2026-10-11, mactest-1)
* MacBook Air M1 (8 cores), macOS 27.0.1, AC power, Low Power Mode off; built-in Retina display
  2880×1800 at 60 Hz. Qt 6.11.2 (Homebrew), release build, Qt's default scene-graph backend
  (Metal on macOS; the report does not record the graphics API). Run from the logged-in
  desktop session; 20 s measured per rate (1201 frames).

| Offered msg/s | Achieved (median) | Frame interval p50 / p99 / max (ms) | Frames > 25 ms | tick() p50 / p99 (µs) | Snapshot load p50 / p99 (µs) |
|--:|--:|--|--:|--|--|
| 0 (idle) | 0 | 16.67 / 17.79 / 18.75 | 0 | 9.5 / 14.1 | 6.8 / 11.1 |
| **200 000** | **199 968** | **16.69 / 19.49 / 22.04** | **0** | 3782 / 5404 | **4.1 / 19.5** |
| 400 000 | 399 865 | 16.67 / 18.94 / 19.40 | 0 | 2024 / 2890 | 1.5 / 5.5 |

* **No dropped frames at any rate** (0 frames > 25 ms), and ingest keeps up to 400 k msg/s.
* **The strict "p99 ≤ 16.7 ms" bound is missed again, including at idle** (17.79 ms). As in
  finding 4, a 60 Hz loop's ordinary jitter puts p99 just above one refresh period, so the
  criterion as written cannot be met by any app; it needs restating (e.g. "0 frames > 25 ms and
  p99 ≤ 1.5 refresh periods"). That is a decision for ADR-0001, not made here.
* **Per-frame UI cost is about 4× the Linux figure** under load (tick p50 3.8 ms at 200 k vs
  0.88 ms), while the snapshot read stays at a few µs. This strengthens finding 3: instruments
  must be scene-graph / QRhi items, not QML property bindings.
* tick() is lower at 400 k than at 200 k (2.0 vs 3.8 ms p50). Not investigated; one run each.

### Low Power Mode (same Mac, same day; LPM on for the runs only)

| Offered msg/s | Achieved | Frame interval p50 / p99 / max (ms) | Frames > 25 ms | tick() p50 / p99 (µs) |
|--:|--:|--|--:|--|
| 0 | 0 | 16.67 / 18.76 / 20.14 | 0 | 10.1 / 14.5 |
| 200 000 | 200 044 | 16.67 / 20.36 / 33.68 | **1** (of 1201) | 3864 / 5648 |
| 400 000 | 400 045 | 16.66 / 19.14 / 20.66 | 0 | 2026 / 2852 |

About 1 ms worse p99 than at normal power, and one 33.7 ms frame at 200 k. One run per rate.

## Not yet measured
Intel Mac, energy impact. ProMotion/120 Hz is not a current task (no ProMotion Mac available):
see [`docs/plan/future-upgrades.md`](../../../docs/plan/future-upgrades.md).
Run (from a terminal in a logged-in desktop session; over SSH Qt finds no screen):
```bash
export QMAKE=$(which qmake)   # Qt 6.5+ recommended on macOS
cargo build --release --manifest-path ui/spikes/s0-1-bridge/Cargo.toml
for r in 0 200000 400000; do S01_RATE=$r S01_REPORT=s01-$r.json ui/spikes/s0-1-bridge/target/release/spike-s0-1-bridge; done
```
