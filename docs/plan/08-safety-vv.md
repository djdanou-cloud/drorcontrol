# 08 — Safety, Verification & Validation Protocol

Scope note: this GCS is not a certified avionics product (it is not DO-178C-certified). It **adopts DO-178C/ARP4754-inspired practices** (hazard analysis, requirements traceability, independent verification) for the *command path* and *safety-relevant displays*, because a GCS defect can cause or contribute to loss of a vehicle. Hazard analysis is maintained in `docs/safety/hazards.md` (FHA/FMEA-lite) and reviewed each phase gate.

## 1. Hazards driving requirements (excerpt)
| ID | Hazard | Causes | Controls |
|----|--------|--------|----------|
| H1 | Unintended command (arm, kill, mode, motor test, param reset) | Mis-click, wrong vehicle, key shortcut, joystick button, script | Two-stage confirm, vehicle identity echo, no single-key hazards, plugin permissions |
| H2 | Wrong vehicle commanded | Multi-vehicle, stale selection | Persistent colour+name banner; confirm dialog shows sysid/name/colour/position |
| H3 | Stale data shown as live | Link loss, freeze, parser stall | Per-field age, invalidation flags, link state machine, watchdog |
| H4 | Mission corrupt/partial upload | Packet loss, abort | Transactional transfer, readback verification, draft≠vehicle badge |
| H5 | Dangerous parameter change | Typo, imported file, batch | Risk labels, diff preview, typed confirmation, auto-snapshot/rollback |
| H6 | Calibration error giving bad sensor data | Wrong orientation | Guided 3-D wizard, fitness thresholds, post-cal sanity checks |
| H7 | Motor spins unexpectedly | Motor test with props | Prop-off attestation, armed-state lockouts, throttle cap, timeouts |
| H8 | GCS freeze / crash during flight | Bug, memory, GPU hang | Process separation (core daemon survives UI crash), watchdog, auto-reconnect, no loss of recorder |
| H9 | Alert fatigue / missed alert | Too many annunciations | Priority model, rate limiting, audio+visual redundancy |
| H10 | Compromised link/plugin | Spoofed MAVLink, malicious script | Signing, plugin sandbox & permissions, input validation |

## 2. Automated SITL testing pipeline (headless, in CI)

### 2.1 Architecture
```mermaid
flowchart LR
  subgraph CI["CI runner (Linux/macOS)"]
    SITL["ArduPilot SITL binary<br/>(pinned tag, cached build)"]
    CHAOS["link-chaos proxy<br/>drop/delay/dup/reorder/corrupt"]
    CORE["dc-core headless"]
    PILOT["Scenario runner (Rust/Python)<br/>asserts on state/events"]
    REC["tlog + dataflash artefacts"]
  end
  SITL <--> CHAOS <--> CORE
  PILOT -->|gRPC| CORE
  CORE --> REC
  SITL -->|dataflash .bin| REC
  REC --> RPT["Flight-health report + golden diff"]
```
* SITL built from pinned ArduPilot release tags (Copter, Plane, Rover, Sub, Tracker) plus *latest-stable* and *master* canaries (allowed to fail, tracked) — catches upstream changes early.
* **Scenario DSL** (YAML + Rust/Python hooks): `given vehicle=copter, location=…; when upload(mission.plan), arm, mode(AUTO); expect(reached(wp=3) within 120s); inject(link.loss for 15s); expect(event(FAILSAFE_GCS) ∧ mode==RTL)`.
* **Mandatory scenario suites**
  | Suite | Cases |
  |-------|-------|
  | Connect | all transports; v1/v2; signed link; wrong baud; multi-vehicle; reconnect after drop |
  | Parameters | full download with 30 % loss (must complete), set/readback, bitmask/enum, reboot-required, interrupted batch rollback, compare/diff correctness |
  | Mission | upload/download/clear round-trip for every supported `MAV_CMD`; fence + rally; resume after interruption; duplicate/out-of-order requests; `MISSION_ACK` error codes; terrain-frame missions with `TERRAIN_REQUEST` service |
  | Flight | arm → takeoff → AUTO survey → RTL → land → disarm for each vehicle; mode changes; guided goto; speed/alt change; pause/continue |
  | Failsafe & lost link | GCS-link loss (`FS_GCS_ENABLE`), RC loss, low battery, EKF, fence breach, terrain loss; verify GCS UI states (link degraded → lost → recovered) and no stale display |
  | Calibration | accel/compass flows against SITL’s simulated cal support; failure branches |
  | Logs | download over MAVLink and MAVFTP, resume, erase; tlog record/replay equality |
  | Robustness | 200 k msg/s flood, malformed frames fuzz replay, 72 h soak (weekly), memory/FD leak detection |
* **Determinism:** SITL speedup ×N with time-aware assertions; seeds recorded; flakes quarantined only with a ticket and an owner (never silently retried to green).
* **Per-PR smoke (≤ 12 min), nightly full (≤ 3 h), weekly soak + latest-ArduPilot canary.**

### 2.2 Other automated layers
| Layer | Technique |
|-------|-----------|
| Unit | `cargo test`, property tests (`proptest`) on codec, mission/param state machines, geodesy, survey math |
| Fuzz | `cargo-fuzz` (MAVLink frames, dataflash, tlog, plan/KML/param parsers), run continuously (OSS-Fuzz-style) |
| Differential | vs. `pymavlink` for parsing and CRC; vs. reference tools for survey/footprint calculations |
| UI | QtQuickTest, accessibility tree assertions, **golden-image screenshots** (PFD, map, wizards) × 4 themes × 3 DPI with tolerance; layout under RTL |
| Performance | `/bench` regression gates (ingest, parse, plot, frame time, power) ±10 % |
| Static/security | clippy `-D warnings`, `cargo-deny/audit`, clang-tidy, CodeQL, SBOM (CycloneDX), secret scanning |
| Packaging | install/upgrade/uninstall on clean VMs; Gatekeeper first-run test; signature/notarization verification |

## 3. Hardware-in-the-loop (HITL) test bench

```mermaid
flowchart TB
  CI[Self-hosted runner: Mac mini M-series + Intel Mac + Windows + Linux] -->|USB hub w/ per-port power control - uhubctl| HUB[Powered USB hub]
  HUB --> FC1[Cube Orange]
  HUB --> FC2[Pixhawk 6X]
  HUB --> FC3[Matek H743]
  HUB --> FC4[Kakute H7 / others]
  HUB --> RAD[RFD900x pair / SiK / ELRS TX]
  HUB --> FTDI[FTDI, CP210x, CH340 loopback adapters]
  PSU[Programmable PSU + current meter] --> FC1 & FC2 & FC3 & FC4
  RIG[Motor-test rig: props OFF, load cells/current sense, ESC] --> FC1
  SENS[GPS sim / RTK base / ADS-B generator / camera + RTSP encoder]
  SENS --> FC1
  SWITCH[Network switch + Wi-Fi AP + impairment box] --> CI
```
* **What runs on hardware nightly:** USB hotplug/replug cycles (1 000×), baud sweep, bootloader entry + firmware flash + verify + boot (each board, each OS), parameter full-sync timing, motor-test lockouts (props-off rig), calibration flows with a servo-driven orientation jig (or IMU-injection), RC calibration with a PPM/SBUS generator, telemetry-radio range-free configuration, video latency measurement (LED-to-screen test), RTK/NTRIP with recorded caster stream, controller input injection (HID emulator).
* **Safety rules for the lab:** props removed always; interlocked enclosure for motor rig; battery fire protocol; written test-pilot sign-off for any tethered/outdoor test.
* **Field qualification:** each release candidate flown by test pilots across ≥ 4 vehicle types (copter, plane, rover, sub/tracker where available) following the handbook procedures; flight logs auto-analysed and attached to the release record.
* **Verified-hardware table** in docs is generated from bench results.

## 4. UI safety interlocks

### 4.1 Command classification
| Class | Examples | Pattern |
|-------|----------|---------|
| **A – Benign / reversible** | change map layer, select vehicle, open dialogs | none |
| **B – Operational** | change speed/alt, guided go-to, set current WP, switch to Loiter, camera trigger | Single deliberate action + visible pending→ACK; undo hint |
| **C – Consequential (two-stage)** | **Arm**, **Disarm in flight**, **RTL**, **Land**, mode change to Auto/Guided/Acro, **mission upload** (when armed), **Motor test**, **Parameter reset / critical param write**, firmware flash, reboot autopilot | Stage 1: arm the control (slide-to-confirm or press-and-hold 0.8 s). Stage 2: confirmation card with vehicle identity (name/sysid/colour/mode/armed state), exact action, consequence, checks summary; explicit button (not default-focused); 5 s auto-cancel; Esc cancels |
| **D – Hazardous** | **Emergency motor kill / force-disarm**, disable arming checks, disable failsafes, calibration overwrite | C + typed confirmation (vehicle name or word), longer hold (1.5 s), reason logged; kill can also use a configured *physical* key/button combo (two-key chord) for speed in emergencies |

*Speed vs. safety:* RTL during a genuine emergency must be fast — so RTL stage 1 is a single prominent **press-and-hold (0.8 s) button**; stage 2 is an immediate ≤ 5 s “RTL — CONFIRM / CANCEL” card with default-armed **Confirm via Enter**. Total time-to-command ≤ 1.5 s, validated in usability tests.

### 4.2 Command lifecycle (single pipeline for UI, joystick, scripts)
```mermaid
stateDiagram-v2
  [*] --> Idle
  Idle --> Armed: stage 1 (hold/slide)
  Armed --> Confirming: show identity card
  Confirming --> Idle: cancel / timeout 5s / vehicle changed
  Confirming --> Sent: confirm
  Sent --> Pending: COMMAND_LONG/INT sent (retry per policy)
  Pending --> InProgress: COMMAND_ACK IN_PROGRESS
  InProgress --> Verified: expected state observed (e.g. HEARTBEAT armed flag)
  Pending --> Verified: ACK ACCEPTED + state observed
  Pending --> Failed: ACK denied/failed/unsupported
  Pending --> Timeout: no ACK after N retries
  Verified --> Idle
  Failed --> Idle
  Timeout --> Idle
```
* **Verification is state-based**, not ACK-based: “ARMED” is shown only when `HEARTBEAT.base_mode` confirms; otherwise show `ARM — NOT CONFIRMED`.
* Interlocks are enforced in the **core** (not just the UI): `CommandPolicy` rejects class-C/D commands lacking a valid, recent, vehicle-bound confirmation token → scripts/plugins/gRPC clients cannot bypass; headless CI uses a test policy with explicit tokens.
* Context lockouts: motor test only when disarmed and (copter) after prop-off attestation; param reset blocked while armed; mission upload while armed requires class-C and is blocked for non-Guided/Auto-compatible modes; firmware flash blocked if any link active to other vehicle.
* **Audit log:** append-only (hash-chained) record of every command, confirmation, parameter write, and outcome with time, user profile, vehicle, link — exportable and included in flight reports.
* **Joystick safety:** deadman button, neutral-on-start check, zero-stick on disconnect, controller-active indicator; joystick can never trigger class C/D without the same two-stage UI.

### 4.3 Degraded communications — explicit UI states
| State | Definition (defaults) | UI | Behaviour |
|-------|-----------------------|----|-----------|
| **CONNECTED** | HEARTBEAT < 1.5 s, loss < 5 % | Normal | All commands allowed |
| **DEGRADED** | loss 5–30 % or latency > 500 ms or RSSI/rem-RSSI low | Amber banner `LINK DEGRADED`, link quality widget pulses | Class C/D require extra confirm text “link degraded”; auto-retry tuned; param/mission transfers warn |
| **STALE** | No HEARTBEAT 1.5–5 s | Instruments dim; values show age; amber `NO DATA 2.1s` | Class B+ disabled except RTL/Land retry (sent repeatedly with explicit user intent) |
| **LOST** | No HEARTBEAT > 5 s | **Red** `LINK LOST` + audio + last-known position marker with age; instruments flagged invalid (X); map shows predicted position cone | Only “re-send RTL/Land” and reconnect assistance; what the vehicle will do (from `FS_GCS_*`) displayed |
| **REPLAY** | Source is log | Blue/cyan `REPLAY` watermark | Command path hard-disabled |
| **SIM** | SITL flag in vehicle/SYSID or user-set | `SIM` badge | Same code path, but clearly labelled |
Thresholds are user-configurable within safe bounds and stored per link profile; transitions are logged and announced. A **watchdog** on the UI frame loop and on the core’s ingest loop raises `GCS SOFTWARE FAULT` banner if either stalls > 500 ms (UI) / > 2 s (ingest).

### 4.4 Other UI safety rules
* No modal dialog may obscure the PFD’s critical annunciations; confirmation cards are anchored beside, not over, them.
* Prearm failures are always visible near the ARM control with direct remedy links.
* Defaults are safe: failsafe templates ship with conservative values; dangerous features (disable arming checks, kill) hidden behind an “Advanced” profile.
* Every irreversible action has a pre-action summary and, where possible, an automatic backup (params snapshot, mission download before overwrite).

## 5. Traceability & release qualification
* Requirements (parity rows + safety requirements `SR-nnn`) ↔ tests ↔ docs matrix generated in CI.
* Release checklist: all SITL suites green on pinned + latest firmware; HITL matrix green; field flights with logs; security scan; licence/SBOM; docs gates; independent V&V reviewer sign-off; rollback plan and staged rollout (5 % → 25 % → 100 %) with kill-switch to halt updates.
* Post-release: incident process (defect → hazard log update → regression test added before fix is merged).
