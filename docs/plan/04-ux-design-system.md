# 04 — Modern UI/UX Architecture & Design System

Design thesis: **a glass cockpit, not a dashboard.** One glance answers *Is it safe? Where is it? What will it do next?* Everything else is available but quiet. The visual language extends the **DrorAero brand** (dark-first, targeting-bracket motif, azure/orange signal colours) with the avionics semantics a GCS needs.

## 1. Design tokens

All visual constants are tokens (JSON → generated QML singleton + CSS vars for docs). No literal colours in components. Source and contrast checks: [`ui/tokens/`](../../ui/tokens/README.md).

### 1.1 Brand palette (identity)
| Token | Hex | Use in GCS |
|-------|-----|-----------|
| `brand.carbon` | `#212121` | Night/Standard canvas |
| `brand.graphite` | `#424242` | Brand material colour. GCS panels use the darker `#333333` (decision 2026-10-11): warning red on graphite is only 2.83:1, on `#333333` it is 3.56:1 |
| `brand.smoke` | `#FAFAFA` | Primary text on dark; Daylight canvas |
| `brand.azure` | `#009CFF` | Selection, primary action, focus, active vehicle, own-ship symbology |
| `brand.pizazz` | `#FF9400` | **Caution** (amber) — also CTA highlight |
| `brand.vapor` | `#22D3EE` | Hover/interactive micro-states, advisory data |
| `brand.silver` | `#757575` | Separators, inactive outlines |

### 1.2 Functional (safety) palette — *added*; brand has no red/green
Avionics colour semantics follow **MIL-STD-1472 / SAE ARP4102-style conventions**: red = warning (immediate action), amber = caution (awareness/future action), green = normal/safe state, cyan/white = advisory/information, magenta = active guidance/target. These colours are **reserved** — never used decoratively.

| Role | Standard/Night-day token | Daylight token | Rule |
|------|--------------------------|----------------|------|
| `state.warning` | `#FF3B30` | `#C4001A` | Immediate action; always paired with shape + text + (optional) audio |
| `state.caution` | `#FF9400` (= `brand.pizazz`) | `#A85900` | Brand orange doubles as caution amber |
| `state.normal` | `#2ECC71` | `#0B7A3B` | Healthy/engaged only — not used for "just because" decoration |
| `state.advisory` | `#22D3EE` | `#006C85` | Informational |
| `state.guidance` | `#E040FB` | `#8E24AA` | Active target/commanded path (flight-director style) |
| `sky` / `ground` | `#0B5FA5` → `#009CFF` gradient / `#6B4A2B` → `#3B2A18` | brighter high-contrast pair | PFD horizon (azure echoes brand) |

**Never colour-only.** Every state also has a shape/label/position change (colour-blind safe; verified with deuteranopia/protanopia simulation in CI screenshots). Contrast: text ≥ 7:1 (AAA) for telemetry, ≥ 4.5:1 elsewhere, meaningful graphics ≥ 3:1, computed automatically from tokens in a unit test. Exception (decision 2026-10-11): Night-Ops telemetry keeps `#FF4D4D` for dark adaptation, 6.4:1 on black.

### 1.3 Themes (ambient lighting palettes)
| Theme | Intent | Canvas | Text | Accent | Notes |
|-------|--------|--------|------|--------|-------|
| **Standard** | Indoor/dusk, brand look | `#212121` (panels `#333333`) | `#FAFAFA` | `#009CFF` | Default |
| **Daylight (Full Sunlight)** | Direct sun on a laptop/tablet | `#FAFAFA` panels, pure-white text boxes inverted to black text; maps desaturated high-contrast | `#101010` | `#0066B3` | ≥ 7:1 everywhere; matte (no translucency/blur); thicker strokes (+25 %) and larger minimum text (+2 pt) |
| **Night-Ops (Red/NVG-safe)** | Preserve dark adaptation and NVG compatibility | `#000000` | `#B30000`–`#FF4D4D` red monochrome | red tints only | **No blue/green/white emitters**; telemetry text `#FF4D4D` (6.4:1, accepted exception to 7:1); brightness slider down to 1 %; all warnings differentiated by shape/flash/audio (since colour is mono); map swaps to red-on-black style |
| **High-Contrast / A11y** | Accessibility | black/white | | | Maps to macOS "Increase Contrast" |

Themes switch at runtime without restart; “Auto” can follow OS appearance and (optionally) ambient light sensor / sunrise-sunset at the vehicle position.

## 2. Typography
| Role | Font | Notes |
|------|------|-------|
| Headlines (H1/H2), main UI | **Urbane** (Heavy for headlines, Medium for UI, Light for accents) | Brand mandated; licence check for bundling (README §5); fallback Inter |
| Section titles / subheads | **Space Grotesk Bold, ALL CAPS**, +1.5 px tracking | Brand mandated |
| Technical notes, small captions | Space Grotesk Medium | |
| **Telemetry numerals** | Monospaced **tabular** face — *JetBrains Mono* or *IBM Plex Mono* (OFL) with slashed zero | Prevents digit jitter in rolling values (tabular figures are non-negotiable); Space Grotesk used only if its `tnum` feature verifies in CI |
| Tape/ladder glyphs | SDF-rendered bitmap atlas of the telemetry font | GPU text at any DPI, no per-frame shaping |

Scale (pt at 1×): telemetry-critical 28/20/14, UI body 13, caption 11 (Daylight +2). Line height 1.2 for numerics, 1.5 prose. Numeric formatting is locale-aware but **digits in readouts stay LTR in RTL layouts**.

Hierarchy rule: ≤ 3 type sizes visible in any panel; units always smaller, dimmed, and baseline-aligned; never truncate a critical value (shrink the label instead).

## 3. Layout system & shell
* **Workspaces** (saved docking layouts): *Fly*, *Plan*, *Setup*, *Tune*, *Analyze*, *Sim*, plus user-defined. Dockable panels (tabs, splits, float, multi-display), implemented with Qt Quick docking (KDDockWidgets-QML or custom), layouts persisted per display configuration.
* **Command palette** (⌘K / Ctrl+K): fuzzy-find any action, parameter, setting, help page ("go to `FS_THR_ENABLE`").
* **Touch/tablet:** 44 pt minimum hit target (56 pt on Daylight/gloved profile), swipe-drawers, no hover-only affordances, long-press = context menu, large "slide to arm" control. Layout breakpoints: phone-sized (remote viewer), tablet, laptop, ultrawide/multi-display.
* **Targeting-bracket motif:** brand corner frames (`┌ ┐ └ ┘`) used for focus/selection of map objects, active waypoint, video lock, and critical readouts; applying the brand **1/3-overlap rule** (bracket overlaps 33 % of the content box at opposite corners, flush to edges).
* Motion: 120–200 ms ease-out; none for safety state changes (instant); reduced-motion honoured.

## 4. Cockpit / PFD interface

```text
┌──────────────────────────── PFD (vector, GPU) ─────────────────────────────┐
│ MODE: LOITER   ARMED   GPS 3D-RTK  ▮▮▮▮  LINK 98% 12ms     [annunciators]   │
│  ┌ IAS/GS tape ┐   ╱ pitch ladder, bank arc, FD/guidance ╲   ┌ ALT tape ┐   │
│  │ 24.1 m/s ▸  │   │   artificial horizon (sky/ground)   │   │ 120.4 m  │   │
│  │ (V-speeds,  │   │   flight-path marker, turn coord.   │   │ AGL/AMSL │   │
│  │  stall/Vne) │   ╲        heading bug / track         ╱   │ VSI ▲1.2 │   │
│  └─────────────┘     ═════ compass tape (HDG/TRK/CRS) ════   └──────────┘   │
│  BATT 22.4V 61%  ▪ remain 11:42   WP 5/12 ▸ 340 m  ETA 00:41   WIND ↗ 4 m/s │
└─────────────────────────────────────────────────────────────────────────────┘
```
* **Rendering:** custom Quick item with a `QRhi` pipeline (or `wgpu` surface) drawing SDF glyphs and analytic primitives; the horizon is a single shader (no bitmap textures), pitch ladder instanced; target 60 fps / <1 ms GPU.
* **Tapes:** speed (IAS vs groundspeed selectable, trend vector, plane V-speeds from params e.g. `ARSPD_*`, `TRIM_ARSPD_CM`, stall limits), altitude (AMSL/Rel/AGL badge, baro/rangefinder source tag, terrain clearance bar), vertical speed, compass tape with heading bug, track, and course-to-WP.
* **Vehicle-aware:** Copter shows climb rate/hover indicators; Plane shows airspeed/stall margin, flap/gear; Rover shows speed/steering/turn radius (PFD becomes a plan-view HUD); Sub shows depth tape and leak/pressure.
* **Declutter levels** (1–3), and decluttering automatically for unusual attitudes (pitch > ±30°/bank > 60°: remove non-essential symbology).
* **Annunciators (contextual alerts):** priority-ordered (warning > caution > advisory > status), fixed positions, latched with acknowledge, de-duplicated and rate-limited ("alert fatigue" policy), tied to optional TTS phrases and haptics; examples: `LINK LOST`, `LOW BATT`, `GPS GLITCH`, `EKF VARIANCE`, `FENCE BREACH`, `PREARM FAIL`, `TERRAIN`, `GEO RTL`, `VIBRATION`, `CLIPPING`. Each alert links to its help article and recommended action.
* **Stale-data treatment:** each field carries age; >1 s old dims, >3 s shows amber dashes `---`, >10 s red `X` over the instrument ("flagged invalid" like a failed ADC in a real PFD) — *never* display a stale number as live.

## 5. Mission Canvas workspace

* One canvas, three coordinated views: **Map (2-D)**, **Profile (altitude vs distance with terrain)**, **3-D (perspective with draped terrain)**; selection is shared; changes are instantly mirrored.
* **Direct manipulation:** click to add; drag waypoints; midpoint handles to insert; multi-select lasso; snap to angle/grid; spline control handles that show the resulting curve; inline altitude "stem" gizmo in 3-D; modifier keys shown in a HUD hint.
* **Survey polygon manipulators:** draggable vertices, edge insertion, rotate-scan-angle handle, live preview of flight lines, photo footprints (optional), estimated time/battery/GSD in a floating readout; warnings for sidelap < threshold, turn radius violations, battery legs, fence intersections.
* **Altitude profile slicing:** a movable cut-plane along any segment; shows terrain, waypoint heights, AGL clearance band, fence max-alt line, vehicle climb-gradient limit.
* **Mission lint panel:** ranked issues with "fix" buttons (e.g. "raise WP7 by 14 m to clear terrain by 30 m").
* **Sync model:** every edit sets the mission to *Draft*; header shows `DRAFT ≠ VEHICLE` badge until uploaded + verified; download/compare shows diff before overwriting.
* **Simulation scrubber:** replay the planned mission with the vehicle model (speed/turn limits) to preview timing without SITL.

## 6. 3-D calibration wizards

Principle: **show the user what the autopilot sees**.
* A `wgpu` 3-D scene renders a type-correct vehicle model (copter frame class/type, plane, rover, sub) driven live by `ATTITUDE`/`AHRS` quaternions; a **ghost model** displays the *target* pose for the current step.
* **Accelerometer (6-pos):** steps *Level → Left → Right → Nose-down → Nose-up → Back* from `ACCELCAL_VEHICLE_POS`; the ghost rotates into position, the live model turns green as attitude matches within tolerance and stability timer (hold-still ring) completes; the app sends the position-acknowledge command only when stable; failures are explained ("Moved during sample – hold still").
* **Compass:** a unit sphere mesh divided into ~300 patches is painted as `MAG_CAL_PROGRESS` direction masks/completion arrives (and from raw field samples for GCS-side fit); uncovered regions pulse; a "rotate nose up/down/etc." coach arrow shows where to turn next. Report page shows fitness, offsets/diagonals, **field magnitude vs. expected (WMM)**, per-compass consistency, and a pass/fail with remedies (move away from metal, orientation param check).
* **Radio:** 3-D/2-D stick gimbals showing min/max extents as the user moves them.
* **Motor test:** top-down frame diagram with numbered motors, direction arrows (CW/CCW) and live throttle; "spin one motor" requires prop-off attestation + two-stage confirm ([08](08-safety-vv.md)).
* Wizards are **resumable state machines** (can leave and return), log all steps, and are fully testable by feeding recorded `ATTITUDE` streams.

## 7. Parameter engine UX

* **Instant search:** in-memory index (trigram + prefix, fuzzy) over name, description, group, units, tags; 1 500–3 000 params filter in <16 ms; results highlight matches; keyboard-first (↑↓, ⏎ edit, ⌘Z undo).
* **Views:** Flat list, Tree (by prefix/group), *Changed* (vs defaults / vs last-saved), *Favourites*, *Related* (e.g. selecting `ATC_RAT_RLL_P` shows the rate-controller family), *Modified-this-session*.
* **Editors by type:** numeric with range slider + units; **bitmask** checkboxes with names; **enum** dropdown with descriptions; read-only/reboot-required badges.
* **Real-time diffing:** compare against file/defaults/another vehicle/previous snapshot; side-by-side with colour + glyph change markers; "apply selected" with preview.
* **Hazard labelling:** a curated `risk.yaml` classifies parameters (e.g. `FS_*`, `FENCE_*`, `BATT_FS_*`, `ARMING_CHECK`, `FRAME_*`, `MOT_*`, `ATC_*`, `EK3_*`, `SERIALn_*`, `SCHED_*`):
  * **Info** (none) · **Caution** (amber chip: affects handling) · **Critical** (red chip + confirm: can cause crash/flyaway/loss of control, disables safety).
  * Writing a Critical parameter shows what it changes, old → new, a plain-language consequence, link to documentation, and requires typed confirmation for the highest class (e.g. disabling arming checks).
* **Schema-driven help:** tooltips and side panel rendered from metadata (description, range, units, increment, values, user-level, reboot-required, firmware-since) + our curated notes ("Tuning guide →").
* **Write pipeline:** snapshot before change (auto-undo point) → `PARAM_SET` → verify by `PARAM_VALUE` readback → reboot banner if needed → audit log. Batch applies are atomic from the user's view (progress, failure list, one-click rollback).
* **Download UX:** progress with ETA, resume of missing indices, local cache keyed by (vehicle id, firmware git hash) so reconnect shows params instantly.

## 8. Interaction safety patterns (summary; full spec in [08](08-safety-vv.md))
Slide-to-arm, hold-to-kill, two-stage confirm with vehicle identity echo, 5-second auto-cancel, consequences text, no hazardous action on single keystroke, visible *COMMAND PENDING → ACK → VERIFIED* states.

## 9. Component library & UX process
* **Storybook-equivalent** (`QtQuick` gallery app) renders every component in all 4 themes × 3 DPI × LTR/RTL; screenshots are golden-image tested.
* **Research:** monthly usability sessions with test pilots; task benchmarks (time-to-arm, time-to-find-param, mission-edit completion) tracked per release; SUS target ≥ 80.
* **Heuristics:** Nielsen + aviation HF guidance (alert prioritisation, consistent location, minimal modes, error tolerance); design reviews include an "error-prone moment" analysis.
