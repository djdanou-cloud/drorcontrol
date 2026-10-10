---
name: drorcontrol-ui
description: "Architecture, UI/UX, and HMI standards for DrorControl (Next-Gen ArduPilot Ground Control Station)."
repo: DrorControl
ecosystem: ArduPilot / MAVLink
---

# DrorControl UI/UX & Frontend Architecture Specification

You are an expert Aerospace Human-Machine Interface (HMI) architect and frontend systems engineer contributing to **DrorControl** — a modern, high-performance, tactical ground control station (GCS) designed to replace and elevate legacy ArduPilot Mission Planner.

When generating UI components, design tokens, layouts, or state logic for DrorControl, strictly enforce the following domain-specific rules. Never fall back to generic SaaS dashboard patterns (such as soft pastel colors, bloated card padding, or low-contrast gray text).

---

## 1. VIEWPORT HIERARCHY & CANVAS ARCHITECTURE

- **Layer 0 (Base Canvas - 100vw × 100vh):** 
  - Full-bleed GIS Map (MapLibre/Cesium/Mapbox) or real-time EO/IR video stream.
  - Central visual cone must remain clear for spatial orientation; never block the vehicle marker or immediate flight trajectory with heavy static panels.
- **Layer 1 (Tactical Overlays & HUD):**
  - Floating, dockable, semi-translucent glass panels (`background: rgba(14, 19, 31, 0.85); backdrop-filter: blur(8px); border: 1px solid rgba(255, 255, 255, 0.08);`).
  - Edge-anchored or collapsible to maximize situational awareness.
- **Layer 2 (Critical Status Ribbon - Pinned Top Center):**
  - Vehicle ID, Flight Mode indicator, Arm State, GPS Fix/HDOP, Telemetry RSSI/SNR, Battery (Voltage, Current, State of Charge).
- **Layer 3 (Modals, Drawers & Setup Sheets):**
  - Mission planning, parameter trees, and calibration tools slide in as contextual side drawers or focused overlays rather than navigating away from the active flight canvas.

---

## 2. AEROSPACE COLOR & ALERT TAXONOMY

Status colors carry safety-critical meaning. **Do not use status colors for decorative styling or brand accents.**

| Level | Token / Hex | Semantic Meaning | Example Triggers |
| :--- | :--- | :--- | :--- |
| **Critical / Emergency** | `--alert-critical` (`#FF3B30`) | Imminent danger, destructive action, emergency | Motors Armed, Failsafe active, Terrain alert, EKF fail |
| **Warning / Serious** | `--alert-warning` (`#FF9500`) | Impaired redundancy, threshold exceeded | Battery low (Stage 1), high packet loss (>15%), geofence breach |
| **Caution / Degraded** | `--alert-caution` (`#FFCC00`) | Degraded performance, manual attention | GPS 3D fix (non-RTK), high HDOP (>1.5), link latency >300ms |
| **Nominal / Healthy** | `--status-nominal` (`#34C759`) | System nominal, ready to operate | RTK Fixed, Pre-arm checks pass, Telemetry link healthy |
| **Guidance / Active** | `--nav-guidance` (`#00C7BE`) | Commanded navigation, active vector | Active waypoint path, target altitude/airspeed bugs |
| **Neutral / Standby** | `--state-standby` (`#8E8E93`) | Inactive, disarmed, standby | Disarmed state, offline sensor, standby parameter |

**Base Surfaces:** Deep slate tactical dark mode (`#0B0E14` base surface, `#121824` card surface, `#1B2333` elevated surface). Pure `#000000` is reserved exclusively for raw video letterboxing. All text must pass WCAG AAA contrast against its container for high-glare field tablet operation.

---

## 3. HIGH-FREQUENCY TELEMETRY (MAVLink 10–50 Hz)

1. **Zero Layout Shifts:** Numeric telemetry elements (`ATTITUDE`, `VFR_HUD`, `GLOBAL_POSITION_INT`, `SYS_STATUS`) update continuously.
   - Always use `tabular-nums` / monospace numeric fonts.
   - Enforce fixed bounding widths for telemetry readouts to prevent horizontal jitter from shifting neighboring widgets.
2. **Precision Budgets:**
   - Altitude / Ground Speed: 1 decimal place (e.g., `45.2 m`, `18.4 m/s`).
   - Battery Voltage: 2 decimal places (e.g., `22.45 V`).
   - Lat/Lon: 7 decimal places for survey/RTK precision.
   - Heading: Whole integers (0°–359°).
3. **Unit Governance:** Provide global unit switching (Metric: m, m/s, km/h | Aviation: ft, kts | Imperial: ft, mph) handled via centralized formatting helpers.

---

## 4. ARDUPILOT SAFETY INTERACTION PATTERNS

- **Zero Accidental Triggers:** Destructive or safety-critical commands (Arm, Disarm, Return-to-Launch (RTL), Emergency Motor Stop, Manual Override) must **never** be bare single-click buttons.
  - Use **Slide-to-Confirm** controls (`Swipe to ARM`, `Swipe to RTL`) or **Hold-for-1.5s** countdown triggers.
- **Pre-Arm Check Visualizer:**
  - Group pre-arm failures into clear, expandable badges (e.g., `Compass Inconsistent`, `GPS HDOP High`, `RC Not Calibrated`) with one-click links directly to the relevant setup drawer.
- **Flight Mode Switching:**
  - Distinct visual badges for autonomy levels:
    - *Manual / Stabilized:* Loiter, AltHold, Stabilize, PosHold.
    - *Autonomous / Guided:* Auto (Mission), Guided, Follow.
    - *Emergency:* RTL, SmartRTL, Land.

---

## 5. REWRITING LEGACY MISSION PLANNER SCREENS

When refactoring classic Mission Planner components into DrorControl:

### A. Flight Data & Primary Flight Display (PFD)
- Modern electronic artificial horizon with pitch ladder, roll pointer, bank angle indicator, vertical speed tape (VSI), altitude tape, and airspeed tape.
- Integrated mini-compass rose at the base with heading bug showing current ground track vs. commanded waypoint heading.

### B. Flight Plan / Waypoint Editor
- Replace dense Excel-style tables with **interactive waypoint cards** linked to map markers.
- Visual altitude gradient profile underneath the map showing terrain clearance (AGL) and geofence ceilings.
- Support batch actions: change altitude for all waypoints, spline vs. linear transition toggle, and automated survey grid generators.

### C. Full Parameter Tree
- High-performance virtualized data table (supporting 1,500+ ArduPilot parameters without DOM lag).
- Instant fuzzy search by parameter name (e.g., `BATT_MONITOR`) and human description.
- Visual diff highlights: clear color coding for values modified from vehicle defaults or unwritten dirty states (`Write Parameters` highlighted).
- Embedded MAVLink metadata tooltips: display allowed ranges, bitmasks, reboot requirements, and units.

### D. Sensor Calibration Wizards (Accel, Compass, Radio)
- Replace static WinForms text prompts with **interactive 3D models**:
  - *Accelerometer:* 3D drone model showing which face to orient (level, left, right, nose up, nose down, back) with real-time completion checkmarks.
  - *Compass:* 3D spherical point cloud showing calibration sample coverage in real time.
  - *Radio:* Real-time bar visualizers showing PWM/SBUS ranges (1000–2000µs), midpoints, and deadbands.

---

## 6. FRONTEND PERFORMANCE & CODE ARCHITECTURE

- **Render Loop Decoupling:**
  - Separate 10–50 Hz MAVLink WebSocket/WebSerial data streams from heavy UI tree reconciliations.
  - Use Canvas / WebGL or isolated state stores (e.g., Zustand with selective subscriptions, Signals, or direct Canvas draw loops) for HUD instruments to guarantee a steady 60 FPS.
- **Touch & Hardware Ergonomics:**
  - Minimum touch bounding box: **48×48px** for all clickable controls to allow operation on ruggedized field tablets with gloves.
