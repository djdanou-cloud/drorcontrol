# DrorControl — Master Development Plan

> Product name: **DrorControl** (codename *Kestrel*; company brand: DrorAero). "Mission Planner" below always means the legacy ArduPilot Windows application.
>
> Status: **Plan v1.0 — for engineering/product review.** Owner: Principal Systems Architecture. Date: 2026-10-10.

## 1. What we are building

A cross-platform ground control station (GCS) for ArduPilot vehicles that:

1. Reaches **100 % functional parity** with Mission Planner, tracked as numbered, testable parity items (`PAR-xxx`, [02](02-parity-matrix.md)).
2. Is a **Tier-1 native macOS application** (Apple Silicon + Intel, Metal-accelerated, ad-hoc signed for internal distribution per ADR-0010, driverless serial) and equally first-class on Windows 10/11 and Linux ([03](03-macos-platform.md)).
3. Replaces WinForms/GDI+ with a **glass-cockpit UI** built on the DrorAero visual identity, with Daylight and Night-Ops (NVG-safe) themes ([04](04-ux-design-system.md)).
4. Is built around a **headless Rust core** so the same engine drives the desktop app, CI test pilots, scripts, and remote/web viewers ([01](01-architecture.md)).
5. Ships with a **publication-grade documentation system** treated as a release gate, not an afterthought ([06](06-documentation-system.md)).

## 2. Document map

| # | Document | Covers |
|---|----------|--------|
| 01 | [Architecture & stack](01-architecture.md) | Stack evaluation, decoupled core, IPC, link/router, map & terrain engine, video pipeline |
| 02 | [Parity matrix](02-parity-matrix.md) | Every Mission Planner tab/dialog → module, MAVLink messages, pain points, UX fix, phase |
| 03 | [macOS platform layer](03-macos-platform.md) | IOKit serial/hotplug, GameController, entitlements, sandbox, notarization CI, power budgets |
| 04 | [UI/UX & design system](04-ux-design-system.md) | Tokens, typography, color doctrine, PFD, mission canvas, 3D calibration wizards, parameter engine |
| 05 | [Telemetry & log analytics](05-telemetry-log-analytics.md) | `.bin` parser, LTTB/MinMaxLTTB, FFT, health rules, `.tlog` playback, exporters |
| 06 | [Documentation architecture](06-documentation-system.md) | ADR/ADD, API refs, onboarding, protocol guide, operator handbook, in-app help |
| 07 | [Roadmap, team, estimates](07-roadmap-team.md) | Phases 0–5, sprints, staffing, risks |
| 08 | [Safety, V&V](08-safety-vv.md) | SITL-in-CI, HITL bench, command interlocks, degraded-link UX |
| — | [ADRs](adr/README.md) | Architecture Decision Records 0001–0007 + template |

## 3. Decision register (summary)

| ID | Decision | Rationale (1 line) | Validated by |
|----|----------|--------------------|--------------|
| ADR-0001 | **Rust core + Qt 6 / QML shell** (hybrid) | Highest weighted score (4.15/5): Rust for protocol/log correctness & throughput; Qt for Metal RHI, native menus, gestures, docking, multimedia | Phase 0 spikes S0-1…S0-6 |
| ADR-0002 | Four-plane boundary: in-process FFI (default) + gRPC (remote/headless) + lock-free snapshot plane for telemetry | UI *pulls* state at vsync; ingest can never starve the UI thread | Soak test T-PERF-01 |
| ADR-0003 | 2-D map via `MapSurface` interface; MapLibre Native (Metal) primary, custom `wgpu` fallback; 3-D scene always `wgpu` | Avoid hard dependency on unproven Qt↔MapLibre-Metal interop | Spike S0-3 (go/no-go gate) |
| ADR-0004 | GStreamer ≥ 1.24 video with zero-copy IOSurface/D3D11/DMA-BUF import | Hardware decode on all OSes, RTSP/RTP/SRT/UDP, mature | Spike S0-5 |
| ADR-0005 | macOS: Developer ID + Hardened Runtime + notarization, **not sandboxed** in v1; MAS build is a later variant | SITL child processes, arbitrary log paths, plugin loading | Notarization dry-run Phase 0 |
| ADR-0006 | Out-of-process plugins/scripts over gRPC (Python/Lua clients); no in-process IronPython equivalent | Isolation, language-agnostic, crash-safe | Phase 4 |
| ADR-0007 | Docs-as-code: MkDocs Material + generated references; doc CI is a merge gate | Docs cannot rot | Phase 0 CI |

## 4. Headline numbers (targets, to be confirmed by Phase 0 measurement)

| Metric | Target |
|--------|--------|
| Sustained MAVLink ingest | ≥ 20 000 msg/s, ≥ 200 000 msg/s burst (SITL/Ethernet), zero UI frame drops at p99 |
| Telemetry → pixel latency | p95 ≤ 50 ms (link excluded) |
| PFD + map frame rate | 60 fps @ Retina (120 fps on ProMotion when plugged in); adaptive 30 fps on battery/Low Power Mode |
| CPU, flight view, M1 Air | ≤ 12 % total (one core ≈ 12.5 % of 8) at 50 Hz telemetry |
| RAM | ≤ 600 MB baseline; 4 GB `.bin` log open ≤ 1.2 GB RSS (mmap + lazy columns) |
| Parameter table (1 500+ params) | search-as-you-type ≤ 16 ms; full download over 57 600 baud has progress, resume, and cache |
| 4 GB `.bin` log | index in ≤ 20 s on M1 (NVMe), first plot ≤ 1 s after index |
| Cold start | ≤ 2.0 s to interactive window |
| Time to GA | ~22 months from kickoff (±30 %), private alpha at month 5 |

## 5. Open decisions (need owner sign-off before Phase 0 ends)

1. **Licence & IP posture.** Mission Planner is GPLv3. The plan assumes a *clean-room* implementation from public specs and behavioural observation (reading GPL source to learn protocol behaviour is fine; copying code is not). Qt (LGPL, dynamic link), GStreamer (LGPL) permit a proprietary build; ArduPilot-derived assets (parameter metadata, SITL) are GPLv3 but consumed as *data/separate processes*. **Legal must confirm** before any MP source is imported. If DrorAero prefers GPLv3 for the GCS itself, code reuse from MP becomes possible and schedule drops ~8 %.
2. **Trademark clearance** for "DrorControl" (name confirmed; search/registration pending). Avoid "Mission Planner" in product marketing — it is an ArduPilot community name.
3. **Urbane font licence** for redistribution inside the app bundle (fallback: Inter).
4. ~~Apple multicast networking entitlement~~ Not needed: no Apple credentials, unsandboxed app (ADR-0010).
5. **Hebrew/RTL** is specified here as P1 (DrorAero is Israel-based) — confirm.
6. **Sandboxed Mac App Store** distribution desired at all? (v1 plan: no.)

## 6. Conventions

* **Parity IDs** `PAR-<area>-<nnn>`; each maps to ≥ 1 automated test (`T-<area>-<nnn>`) and one handbook section.
* **Requirement language**: *shall* = mandatory for GA, *should* = target, *may* = optional.
* **Estimates** are person-weeks at ±30 % unless stated; velocity-based re-baselining occurs at the end of Phase 0.
* Numbers marked "target" are hypotheses to be falsified by benchmarks in the repo (`/bench`), not promises.
* Third-party facts (MAVLink IDs, ArduPilot log/param names) must be re-verified against the pinned MAVLink XML and ArduPilot release at implementation time; the generators in [01 §4](01-architecture.md) make the XML/metadata the single source of truth.
