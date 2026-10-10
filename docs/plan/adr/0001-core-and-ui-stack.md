# ADR-0001: Rust core with Qt 6 / QML presentation shell

* Status: proposed (to be accepted at Phase 0 gate M0, pending spikes S0-1, S0-2, S0-3)
* Date: 2026-10-10
* Validation: spikes S0-1…S0-6 in `01-architecture.md` §9

## Context
Need a cross-platform GCS with first-class macOS (Metal, IOKit, GameController, notarization), deterministic high-rate telemetry handling, multi-GB log analysis, docking multi-window UI, and low battery draw.

## Decision drivers
macOS fidelity (20 %), GPU pipeline (15 %), throughput/latency determinism (15 %), packaging (10 %), UI velocity (10 %), GCS ecosystem (10 %), memory safety (10 %), legacy reuse (5 %), maintenance risk (5 %).

## Options considered
| Option | Weighted score /5 |
|--------|------------------:|
| A Rust + Tauri (webview) | 3.40 |
| B C++20 + Qt 6 | 3.85 |
| C .NET + Avalonia | 3.65 |
| **D Rust core + Qt 6/QML** | **4.15** |

## Decision
We will implement protocol, state, logs, geo and video-pipeline management in Rust (`dmp-core`), and the presentation layer in Qt 6/QML (C++20 glue), connected through our own stable `core-ffi` boundary (C ABI + schema'd payloads), with `cxx-qt` used as an implementation detail of the bridge, not as the architecture.

## Consequences
* + Memory-safe parsers/state machines; Qt RHI Metal rendering, native menus/gestures; core reusable headless in CI.
* − Two toolchains; Qt licence compliance; `cxx-qt` maturity risk (mitigated by hand-written C++ adapter fallback).
* Revisit if spike S0-1 fails its frame-time criterion or UI velocity blocks Phase 1 exit.
