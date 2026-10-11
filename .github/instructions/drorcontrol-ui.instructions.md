---
applyTo: "ui/**,**/*.qml,docs/design/**,docs/plan/04-ux-design-system.md"
---
<!-- GENERATED from .claude/skills/drorcontrol-ui/SKILL.md by scripts/sync_skill.py - do not edit; edit the source and re-run the script. -->

# DrorControl UI/UX & Frontend Architecture Specification

You are an expert Aerospace Human-Machine Interface (HMI) architect and frontend systems engineer contributing to **DrorControl** — a modern, high-performance, tactical ground control station (GCS) designed to replace and elevate legacy ArduPilot Mission Planner.

When generating UI components, design tokens, layouts, or state logic for DrorControl, strictly enforce the following domain-specific rules. Never fall back to generic SaaS dashboard patterns (such as soft pastel colors, bloated card padding, or low-contrast gray text).

**This skill is aligned with the project plan and is subordinate to it.** Authoritative sources, in order: product-owner instruction → accepted ADRs → [`docs/plan/04-ux-design-system.md`](../../docs/plan/04-ux-design-system.md) (tokens, themes, typography, PFD, mission canvas, calibration, parameter UX) → [`docs/plan/08-safety-vv.md`](../../docs/plan/08-safety-vv.md) §4 (command classes, interlocks, link states) → this skill. If this file and the plan ever disagree, the plan wins; log a `conflict` entry (see `docs/coordination/README.md` §6) and fix whichever is stale.

**Implementation stack (ADR-0001):** Rust core (`dc-core`) + **Qt 6 / QML** shell, GPU instruments via Qt RHI / `wgpu`. The rules below are stack-neutral design requirements; "web" idioms (CSS, DOM, WebSocket, Zustand) appear only for the optional remote/web viewer. Never hard-code colors, sizes, fonts or durations — use design tokens (JSON → generated QML singleton / CSS variables).

---

## 1. VIEWPORT HIERARCHY & CANVAS ARCHITECTURE

- **Layer 0 (Base Canvas — full window):**
  - Full-bleed GIS map (via the `MapSurface` abstraction: MapLibre Native primary, `wgpu` fallback — ADR-0003) or real-time EO/IR video stream (GStreamer pipeline, GPU-composited — ADR-0004).
  - Central visual cone must remain clear for spatial orientation; never block the vehicle marker or immediate flight trajectory with heavy static panels.
- **Layer 1 (Tactical Overlays & HUD):**
  - Floating, dockable panels with multi-display support and saved workspaces (*Fly, Plan, Setup, Tune, Analyze, Sim*).
  - **Standard theme:** semi-translucent glass is permitted (token `surfaceGlass`: panel grey `#333333` at 85 % alpha, 1 px hairline border at ~8 % white; blur is optional and **off by default on battery / Low Power Mode**, since it is GPU-costly).
  - **Daylight theme:** matte and opaque only — no translucency or blur. **Night-Ops theme:** opaque, red-monochrome, no blur.
  - Edge-anchored or collapsible to maximize situational awareness.
- **Layer 2 (Critical Status Ribbon — pinned top center):**
  - Vehicle ID/name/color, flight-mode badge, arm state, GPS fix/HDOP, link quality (RSSI/SNR, loss, latency), battery (voltage, current, remaining %). Never obscured by modals; confirmation cards anchor beside it, not over it (plan 08 §4.4).
- **Layer 3 (Drawers, Sheets & Focused Overlays):**
  - Mission planning, parameter tree, calibration and setup slide in as contextual side drawers or focused overlays rather than navigating away from the active flight canvas.

---

## 2. AEROSPACE COLOR & ALERT TAXONOMY

Status colors carry safety-critical meaning (MIL-STD-1472 / ARP4102-style). They are **reserved**: never use them for decoration, brand accents or chart series. **Never color-only** — every state also changes shape/icon, text label or position (verified under deuteranopia/protanopia simulation).

| Level | Token | Standard / Night-Ops-aware hex | Daylight hex | Semantic meaning | Example triggers |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Warning (immediate action)** | `state.warning` | `#FF3B30` | `#C4001A` | Imminent danger or loss of control authority | Failsafe active, terrain alert, EKF fail, link LOST, battery critical, fence breach |
| **Caution (awareness / future action)** | `state.caution` | `#FF9400` (= brand Pizazz) | `#A85900` | Degraded performance or redundancy | Battery low (stage 1), link DEGRADED, GPS 3D non-RTK, HDOP above threshold, high vibration |
| **Advisory (information)** | `state.advisory` | `#22D3EE` | `#006C85` | Informational, not requiring action | Pre-arm info, mode-change notice, SIM/REPLAY badges |
| **Normal (healthy / safe)** | `state.normal` | `#2ECC71` | `#0B7A3B` | System nominal, ready | RTK Fixed, pre-arm checks pass, link CONNECTED |
| **Guidance (commanded)** | `state.guidance` | `#E040FB` | `#8E24AA` | Active target / commanded path | Active waypoint vector, target altitude/airspeed bugs |
| **Standby / Neutral** | `state.standby` | `#757575` (brand Sonic Silver) | `#5A5A5A` | Inactive, disarmed, offline | Disarmed, offline sensor, inactive parameter |

Notes:
- **Armed is a state, not an alarm.** Show ARMED/DISARMED as a high-visibility state badge (shape + label); do not color it as a Warning unless a genuine hazard exists (e.g. armed with a failed pre-arm check bypass).
- Thresholds are **defaults that live in configuration**, not constants: link states follow plan 08 §4.3 (CONNECTED / DEGRADED: loss 5–30 % or latency > 500 ms / STALE: no heartbeat 1.5–5 s / LOST: > 5 s / REPLAY / SIM). HDOP, battery and vibration thresholds come from firmware parameters and the diagnostic rule packs.
- **Night-Ops** is red-monochrome (no blue/green/white emitters): differentiate states by shape, flash pattern and audio, not hue.
- **Stale data:** value > 1 s old dims, > 3 s shows amber dashes `---`, > 10 s red `X` over the instrument. Never show a stale number as live.

**Base surfaces (Standard theme — DrorAero brand):** canvas Carbon `#212121`, panels `#333333` (darker than brand Matte Graphite `#424242` so state colours keep ≥ 3:1), text White Smoke `#FAFAFA`, primary accent/selection Bright Azure `#009CFF`, interactive micro-states Vaporwave `#22D3EE`, separators Sonic Silver `#757575`. Daylight canvas `#FAFAFA` with `#101010` text (≥ 7:1). Night-Ops canvas `#000000` with red text `#FF4D4D` (6.4:1, the accepted exception to 7:1). Pure black otherwise appears only in raw video letterboxing. Telemetry text ≥ 7:1 (WCAG AAA); other text ≥ 4.5:1; meaningful graphics ≥ 3:1 — asserted by an automated token-contrast test (`crates/design-tokens`). Use the generated tokens (`ui/tokens/`), never these hex values directly.

Typography: Urbane (headlines/UI; Inter fallback), **Space Grotesk Bold ALL CAPS** for section titles, and a tabular monospaced face (JetBrains Mono or IBM Plex Mono) for all telemetry numerals.

---

## 3. HIGH-FREQUENCY TELEMETRY (MAVLink 10–50 Hz)

1. **Zero Layout Shifts:** Numeric telemetry elements (`ATTITUDE`, `VFR_HUD`, `GLOBAL_POSITION_INT`, `SYS_STATUS`) update continuously.
   - Always use tabular / monospace numerals (`tabular-nums` in web; tabular-figure font in QML).
   - Enforce fixed bounding widths for telemetry readouts to prevent horizontal jitter from shifting neighboring widgets; shrink the label before truncating a critical value; units smaller, dimmed and baseline-aligned.
2. **Precision Budgets:**
   - Altitude / Ground Speed: 1 decimal place (e.g., `45.2 m`, `18.4 m/s`).
   - Battery Voltage: 2 decimal places (e.g., `22.45 V`).
   - Lat/Lon: 7 decimal places for survey/RTK precision (MGRS/UTM/DMS selectable).
   - Heading: whole integers (0°–359°).
3. **Unit Governance:** Global unit switching (Metric: m, m/s, km/h | Aviation: ft, kts | Imperial: ft, mph) via one centralized conversion/formatting layer (plan `XC-002`); no component formats units itself. Numerals stay LTR in RTL (Hebrew) layouts.
4. **Update path:** the UI **pulls** the latest immutable telemetry snapshot once per frame and coalesces property writes (change-detected per field). Never bind UI properties directly to per-message signals (ADR-0002).

---

## 4. ARDUPILOT SAFETY INTERACTION PATTERNS

Command classes and the lifecycle are defined in `docs/plan/08-safety-vv.md` §4; the core enforces them (`CommandPolicy`) — the UI cannot be the only guard.

- **Zero Accidental Triggers:** Class C/D commands must **never** be bare single-click buttons or single-key shortcuts.
  - **Arm:** slide-to-arm (stage 1) → confirmation card (stage 2) showing vehicle identity (name / sysid / color), mode, armed state and pre-arm summary; explicit button not default-focused; 5 s auto-cancel; Esc cancels.
  - **Disarm in flight, Land, mode change to Auto/Guided/Acro, mission upload while armed, motor test, critical parameter write/reset, firmware flash, autopilot reboot:** Class C two-stage (press-and-hold ≈ 0.8 s or slide, then confirmation card).
  - **RTL (emergency path):** stage 1 is a prominent **press-and-hold (0.8 s)** button; stage 2 is an immediate ≤ 5 s "RTL — CONFIRM / CANCEL" card. Total time-to-command ≤ 1.5 s (validated in usability tests).
  - **Emergency motor kill / force-disarm, disabling arming checks or failsafes, calibration overwrite:** Class D — hold **1.5 s** plus typed confirmation (vehicle name or word); reason logged. A configured physical key chord may serve as the fast path.
  - **Manual override / joystick:** deadman button, neutral-on-start check, zero-stick on disconnect, visible controller-active indicator; can never trigger Class C/D without the same two-stage UI.
- **Visible command lifecycle:** `PENDING → ACK → VERIFIED` (state-based verification, e.g. "ARMED" only once `HEARTBEAT` confirms; otherwise `ARM — NOT CONFIRMED`).
- **Pre-Arm Check Visualizer:**
  - Group pre-arm failures into clear, expandable badges (e.g., `Compass Inconsistent`, `GPS HDOP High`, `RC Not Calibrated`) placed beside the ARM control, each with a one-click deep link to the relevant setup drawer and help entry.
- **Flight Mode Switching:**
  - Mode lists come from ArduPilot metadata per vehicle/firmware — never hand-typed. Distinct visual badges for autonomy levels (UI classification only):
    - *Manual / Stabilized:* Loiter, AltHold, Stabilize, PosHold.
    - *Autonomous / Guided:* Auto (Mission), Guided, Follow.
    - *Emergency:* RTL, SmartRTL, Land.
  - Mode changes are verified against `HEARTBEAT`; hazardous modes need confirmation.
- **Degraded comms:** explicit CONNECTED / DEGRADED / STALE / LOST / REPLAY / SIM states with banner and instrument invalidation (plan 08 §4.3). **Link state never blocks a command** (ADR-0009): show the state, heartbeat age and the vehicle's own failsafe action in the confirmation card, add no extra step, and show `SENT — NOT CONFIRMED` (retries, time left) until telemetry verifies or the command expires. Only REPLAY has no command path (no vehicle).

---

## 5. REWRITING LEGACY MISSION PLANNER SCREENS

When refactoring classic Mission Planner components into DrorControl (parity IDs in `docs/plan/02-parity-matrix.md`):

### A. Flight Data & Primary Flight Display (PFD) — `FD-001…`
- Vector, GPU-rendered electronic artificial horizon (single shader), pitch ladder, roll pointer and bank arc, vertical speed tape (VSI), altitude tape (AMSL/Rel/AGL badge, baro/rangefinder source tag, terrain-clearance bar) and airspeed/groundspeed tape (trend vector, V-speeds from params).
- Compass tape / mini-compass rose at the base with heading bug, ground track and course-to-waypoint.
- Vehicle-aware (Copter, Plane, Rover, Sub), declutter levels 1–3, automatic declutter in unusual attitudes, priority-ordered annunciators with acknowledge, rate-limiting and help links.

### B. Flight Plan / Waypoint Editor — `FP-001…`
- Replace dense Excel-style tables with **interactive waypoint cards** linked to map markers, plus a command-aware typed inspector (units, validation, inline docs); a compact table view may remain as an option for power users.
- Visual altitude gradient profile underneath the map showing terrain clearance (AGL), fence ceilings, and climb-gradient limits; radius circles for loiter/acceptance.
- Batch actions: change altitude for all/selected waypoints, spline vs. linear toggle, automated survey/corridor/SAR generators with draggable polygon vertices and live stats.
- `DRAFT ≠ VEHICLE` badge until uploaded and verified; mission lint panel with fix buttons; undo/redo.

### C. Full Parameter Tree — `CT-001…`
- Virtualized table (QML `TableView` with delegate reuse) handling 1,500–3,000 parameters; search-as-you-type ≤ 16 ms over name, description, group and units (fuzzy).
- Visual diff: values modified from defaults, last snapshot, file, or another vehicle; unwritten dirty state highlights `Write Parameters`; write is verified by readback, with auto-snapshot and rollback.
- Metadata tooltips/side panel: range, units, increment, enums, **bitmask checkboxes**, reboot-required, firmware-since; curated hazard labels (Info / Caution / Critical, typed confirmation for the highest class).

### D. Sensor Calibration Wizards (Accel, Compass, Radio) — `IS-003…005`
- Replace static text prompts with interactive 3D scenes (`wgpu`) driven live by the vehicle's attitude:
  - *Accelerometer:* live model plus ghost target pose for each of the six positions (level, left, right, nose up, nose down, back); hold-still ring; completion checkmarks.
  - *Compass:* spherical coverage mesh painted from `MAG_CAL_PROGRESS`/samples with a coach arrow; report with fitness, offsets, field-magnitude vs. expected, per-compass consistency.
  - *Radio:* stick gimbals and real-time PWM/SBUS bars (1000–2000 µs), midpoints, deadbands, endpoint capture.
- Wizards are resumable state machines; all steps logged; testable with recorded attitude streams.

---

## 6. FRONTEND PERFORMANCE & CODE ARCHITECTURE

- **Render Loop Decoupling:**
  - Ingest/parse run on dedicated Rust threads; the UI thread never blocks (no locks on it) and reads wait-free snapshots at vsync (ADR-0002).
  - HUD instruments, plots, maps and 3-D scenes are isolated GPU items (Qt RHI / `wgpu`, SDF glyph atlases, instanced draws) with their own frame budget — kept separate from form-heavy QML trees so a large table cannot drop PFD frames. Targets: 60 fps at Retina (adaptive 30 fps on battery / Low Power Mode), render-on-dirty.
  - *Remote/web viewer only:* WebSocket/gRPC-web streams with canvas/WebGL HUD and selective-subscription stores (e.g., Zustand, signals) — same isolation rule.
- **Touch & Hardware Ergonomics:**
  - Minimum touch target **48×48 px** (≈ 44 pt) for all interactive controls; **56 px** in the Gloved/Daylight field profile. No hover-only affordances; long-press = context menu.
  - Respect OS conventions (macOS menu bar, trackpad gestures, system dark/light for app chrome); full keyboard operation; reduced-motion and increased-contrast honored.
- **Quality gates:** component gallery with golden-image screenshots across 4 themes × 3 DPI × LTR/RTL; contrast test from tokens; frame-time and power benchmarks in CI.
