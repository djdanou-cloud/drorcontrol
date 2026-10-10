# Backlog (Phase 0 seed)

Static task definitions. **Task state is not stored here** (that would cause merge conflicts); it lives in agent status files, claims, and `done` log entries, and is rolled up in [BOARD.md](BOARD.md). Add new tasks by appending rows (additive edits only) with the next free `T<phase>-<nn>` ID. Plan references point into [`../plan/`](../plan/README.md).

| ID | Workstream | Task | Plan ref | Depends on | Acceptance (verifiable) |
|----|-----------|------|----------|-----------|--------------------------|
| T0-01 | WS-DEVOPS | Monorepo skeleton, `just`/`cargo xtask`, pinned toolchains, CI on macOS arm64/x86_64, Windows, Linux | 07 D0-1 | — | CI green on all 4 targets from a clean clone |
| T0-02 | WS-MAVLINK | `mavlink-codec`: generated dialect (all + ardupilotmega), v1/v2 framing, CRC, signing, fuzz targets | 01 §4, 07 D0-2 | T0-01 | Round-trips vs pymavlink on corpus; fuzz smoke 10 min clean |
| T0-03 | WS-MAVLINK | `mav-router`, serial/UDP/TCP link crates, tlog recorder | 01 §3–4 | T0-02 | Router tests (loop prevention, forwarding); tlog byte-exact replay |
| T0-04 | WS-CORE | Core daemon, gRPC API v1, `core-ffi`, `dc-cli` (connect, params, telemetry dump) | 01 §3.3, 07 D0-3 | T0-02 | `dc-cli` connects to SITL and dumps params |
| T0-05 | WS-MACOS | macOS serial: IOKit enumeration, hotplug, baud (incl. `IOSSIOSPEED`), driverless matrix on macOS 13–26 | 03 §2, 07 D0-4, S0-2 | T0-01 | Matrix report committed; 30-min loopback 0 CRC errors on 3 chip families |
| T0-06 | WS-UI | Spike S0-1: `cxx-qt` bridge, coalesced 60 fps updates under 200 k msg/s | 01 §9 | T0-04 | p99 frame ≤ 16.7 ms, 0 dropped frames; result recorded in ADR-0001 |
| T0-07 | WS-GFX | Spike S0-3: MapLibre Native (Metal) ↔ Qt RHI texture import | 01 §5.1, ADR-0003 | T0-01 | Go/no-go memo by week 6 with numbers |
| T0-08 | WS-LOGS | Spike S0-4: 4 GB `.bin` index + LTTB/MinMaxLTTB plot | 05 §1, S0-4 | T0-01 | Index ≤ 20 s, plot ≤ 1 s, RSS ≤ 1.2 GB |
| T0-09 | WS-VIDEO | Spike S0-5: 1080p30 RTSP H.264 VideoToolbox zero-copy + overlay | 01 §6 | T0-01 | ≤ 8 ms decode, ≤ 5 % CPU on M1 |
| T0-10 | WS-MACOS | Notarized universal build in GitHub Actions (S0-6); Apple entitlements/multicast request | 03 §5, 07 D0-6 | T0-01 | `spctl`/`stapler` verified on clean VM; human provides Apple credentials |
| T0-11 | WS-QA | Parity audit → frozen `parity.yaml`; test corpus collection | 02, 07 D0-7 | — | `parity.yaml` merged; every row has ID/owner/test id placeholder |
| T0-12 | WS-UX | Design tokens v0 (JSON → QML/CSS), 4 themes, type scale, contrast unit test | 04 §1–2, 07 D0-8 | — | Contrast test passes (≥ 7:1 telemetry text) |
| T0-13 | WS-DOCS | MkDocs Material site, ADR tooling, generated MAVLink reference, CI doc gates | 06, 07 D0-9 | T0-01 | `mkdocs build --strict` in CI; ADR index generated |
| T0-14 | WS-QA | SITL harness: pinned ArduPilot SITL build + scenario runner + `link-chaos` proxy | 08 §2 | T0-01 | Connect + param-download scenarios pass in CI |
| T0-15 | WS-SAFETY | Hazard log v0 and `CommandPolicy` requirements (class A–D) | 08 §1, §4 | — | `docs/safety/hazards.md` reviewed by a human |
