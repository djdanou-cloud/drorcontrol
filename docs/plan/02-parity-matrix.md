# 02 — Feature Decomposition & Parity Matrix

**Method.** The matrix below is seeded from the Mission Planner 1.3.x UI (top-level tabs, context menus, and the `Ctrl+F` hidden-tools menu). Because "100 % parity" is only meaningful against a frozen list, **Phase 0 deliverable D0-7** is an automated *feature inventory audit*: walk every Mission Planner form/menu/context-menu in the running application and its public source (read-only, for behaviour), reconcile with this table, and freeze `parity.yaml`. Every row gets a stable ID, an owner, an automated test (`T-…`), a handbook page, and a status that CI renders as a public parity dashboard.

MAVLink IDs in parentheses are message/command numbers from the common & ardupilotmega dialects and **must be re-verified against the pinned XML** (generated code is the source of truth, never this table).

Legend — Phase: P1 HUD/map · P2 planning · P3 config/calibration · P4 analytics/SITL/video · P5 hardening · *later* = after 1.0, not required for GA (product-owner decision, recorded in `parity.yaml`).

---

## A. Connectivity & Vehicle Profiles (`PAR-CON`)

| ID | Feature | MAVLink / protocol | Legacy pain point | Modern solution | Ph |
|----|---------|--------------------|-------------------|-----------------|----|
| CON-001 | Serial connect (any baud, 57 600–2 000 000+) | `HEARTBEAT`(0), `AUTOPILOT_VERSION`(148) via `MAV_CMD_REQUEST_MESSAGE`(512) | Manual COM/baud picking; hangs on wrong baud | Auto-detect: probe common bauds in parallel on new device; remembers per-device; hotplug toasts | 1 |
| CON-002 | UDP (server/client/broadcast), TCP client/server | MAVLink v1/v2 framing | Firewall confusion; silent bind failures | Link wizard with live port probe, firewall/local-network permission guidance | 1 |
| CON-003 | Multiple simultaneous links + routing | `sysid/compid` routing | Vehicle selector hidden; stale data bleeds | First-class multi-vehicle switcher, per-vehicle colour, router graph view | 1 |
| CON-004 | MAVLink 1 & 2, signing | `SETUP_SIGNING`(256) | Signing UI buried | Key manager with keychain storage, status chip | 3 |
| CON-005 | Vehicle profiles: Copter, Plane (incl. QuadPlane), Rover, Sub, Antenna Tracker | `MAV_TYPE`, `custom_mode`, param metadata | Mode tables hand-maintained | Generated from ArduPilot metadata; version-aware | 1 |
| CON-006 | Link quality stats | `SYS_STATUS`(1) drop_rate, `RADIO_STATUS`(109), local counters | One cryptic label | Link HUD: rate, loss, latency, RSSI/noise, history sparkline | 1 |
| CON-007 | Bluetooth SPP/BLE telemetry | stream | Windows-only | CoreBluetooth (mac), WinRT (Win), BlueZ (Linux) | 4 |
| CON-008 | MAVLink Inspector | all | Slow, single table | Virtualised, filter/graph any field, rate histogram | 1 |
| CON-009 | MAVLink forwarding / serial-over-UDP bridge | router | Hidden setting | Router config UI | 4 |
| CON-010 | Cursor-on-Target (CoT) output to TAK/ATAK: vehicle position and track | CoT XML over UDP multicast (TAK SA 239.2.3.1:6969), TCP, UDP or serial | Hidden Ctrl-F tool | Output in link/router settings; per-vehicle UID, CoT type and rate; status chip | 4 |

## B. Flight Data (`PAR-FD`)

| ID | Feature | MAVLink | Legacy pain point | Modern solution | Ph |
|----|---------|---------|-------------------|-----------------|----|
| FD-001 | HUD / PFD: horizon, pitch ladder, bank, heading tape, speed & alt tapes, VSI, mode, battery, GPS, link | `ATTITUDE`(30), `VFR_HUD`(74), `GLOBAL_POSITION_INT`(33), `SYS_STATUS`, `GPS_RAW_INT`(24), `HEARTBEAT` | GDI+ repaint flicker, fixed layout | Vector GPU PFD, resolution-independent, configurable declutter ([04](04-ux-design-system.md)) | 1 |
| FD-002 | Baro vs rangefinder altitude, AGL | `DISTANCE_SENSOR`(132), `TERRAIN_REPORT`(136), `AMSL`/rel | Source ambiguous | Altitude tape shows source badge (BARO/RNG/TERR) | 1 |
| FD-003 | Moving map w/ vehicle, trail, home, waypoints, fence, rally, heading/wind vectors | `GLOBAL_POSITION_INT`, `MISSION_CURRENT`(42), `NAV_CONTROLLER_OUTPUT`(62), `WIND`(168) | Tile flicker, slow zoom | GPU map, gestures, follow/free modes, scale, MGRS | 1 |
| FD-004 | Quick tab: customisable numeric fields | any field | Limited, mouse-only | Drag-and-drop tile builder, units, alert thresholds | 1 |
| FD-005 | Gauges (speedo, alt, VSI, compass, battery) | `VFR_HUD`, `SYS_STATUS`, `BATTERY_STATUS`(147) | Skeuomorphic | Gauge cluster widgets from design system | 1 |
| FD-006 | Messages console | `STATUSTEXT`(253) w/ severity & chunking | Unfiltered scroll | Severity filter, search, pin, copy, link to help | 1 |
| FD-007 | Status tab (all values) | all | Raw list | Searchable grid + pin to dashboard | 1 |
| FD-008 | Live graphs ("tuning" plot, up to N series) | any | Laggy | GPU plot with LTTB ([05](05-telemetry-log-analytics.md)) | 1 |
| FD-009 | Audio announcements (TTS) | events | Windows SAPI only | `AVSpeechSynthesizer` (mac), SAPI/OneCore (Win), speech-dispatcher (Linux); rule editor | 1 |
| FD-010 | Arm / Disarm | `MAV_CMD_COMPONENT_ARM_DISARM`(400) | One click; no precheck | Two-stage with pre-arm checklist from `STATUSTEXT` / `SYS_STATUS` sensors | 1 |
| FD-011 | Emergency motor kill | `MAV_CMD_COMPONENT_ARM_DISARM` param2 = 21196 (force) | Easy mis-click | Hold-to-confirm + typed vehicle ID, physical-key option | 1 |
| FD-012 | Mode change | `MAV_CMD_DO_SET_MODE`(176) / `SET_MODE`(11) | Dropdown, no feedback | Mode ladder, confirm for hazardous modes, ACK + HEARTBEAT verification | 1 |
| FD-013 | RTL / Land / Loiter / Auto start / Takeoff / Set WP / Change speed / Change alt | `DO_SET_MODE`, `NAV_TAKEOFF`(22), `MISSION_SET_CURRENT`(41), `DO_CHANGE_SPEED`(178), `DO_REPOSITION`(192) | Scattered | Context-sensitive Action bar per vehicle + mode | 1 |
| FD-014 | Guided: fly-to-here / point-and-click, orbit, ROI | `SET_POSITION_TARGET_GLOBAL_INT`(86), `DO_REPOSITION`, `DO_SET_ROI`(201), `NAV_LOITER_TURNS`/`DO_ORBIT`(34) | Right-click menus | Click map → target card with alt/speed → confirm | 1 |
| FD-015 | Servo / relay overrides, aux functions, repeat | `DO_SET_SERVO`(183), `DO_REPEAT_SERVO`(184), `DO_SET_RELAY`(181), `DO_REPEAT_RELAY`(182), `DO_AUX_FUNCTION`(218) | Hidden grids | Safe "Payload & Aux" panel, per-channel labels from params | 1 |
| FD-016 | Pre-flight checklist (user-defined) | local | Basic | Versioned checklist templates, logged sign-off | 3 |
| FD-017 | Tlog/Dataflash buttons from flight view | — | — | Embedded log browser & live record indicators | 4 |
| FD-018 | Fly-by-wire / Follow-me (GCS position stream) | `FOLLOW_TARGET`(144) / `GLOBAL_POSITION_INT` | Windows geolocation | CoreLocation/OS location with permission flow | 4 |
| FD-019 | Payload / gimbal control | `GIMBAL_MANAGER_*`, `DO_MOUNT_CONTROL`(205), `DO_GIMBAL_MANAGER_PITCHYAW`(1000) | Separate dialog | Gimbal joystick overlay on video | 4 |
| FD-020 | ADS-B traffic, collision risk | `ADSB_VEHICLE`(246) | Plot only | TCAS-style TA/RA symbology with CPA/TCPA | 4 |
| FD-021 | Python/IronPython scripts panel | — | Embedded, unsafe | Out-of-process scripts ([01 §7](01-architecture.md)) | 4 |
| FD-022 | Distance-to-home, time-in-air, ETA, wind, battery remaining estimation | derived | Naïve | Computed in core with documented models | 1 |
| FD-023 | Heading/track-up map, vehicle follow, multiple-vehicle overlays | — | — | Map orientation modes | 1 |
| FD-024 | Swarm control: formation, follow-the-leader path, sequenced multi-vehicle choreography (incl. swarm SITL launch) | per-vehicle guided targets | Hidden Ctrl-F tools; no multi-vehicle interlocks | Deferred: needs a multi-vehicle command-safety review (08 §4) first | later |

## C. Flight Plan (`PAR-FP`)

| ID | Feature | MAVLink | Legacy pain point | Modern solution | Ph |
|----|---------|---------|-------------------|-----------------|----|
| FP-001 | Waypoint table with all `MAV_CMD`s (NAV/DO/CONDITION), frames, params with per-command labels | `MISSION_COUNT`(44), `MISSION_REQUEST_INT`(51), `MISSION_ITEM_INT`(73), `MISSION_ACK`(47) | Raw param1–4 columns; easy to mis-set | Command-aware inspector with typed fields, units, validation, inline doc | 2 |
| FP-002 | Mission upload/download/clear/verify, set current | `MISSION_REQUEST_LIST`(43), `MISSION_CLEAR_ALL`(45), `MISSION_SET_CURRENT`(41), `MISSION_ITEM_REACHED`(46) | Silent partial failures | Transactional transfer state machine with per-item progress, CRC/readback verify, resume | 2 |
| FP-003 | Spline waypoints | `NAV_SPLINE_WAYPOINT`(82) | Rendering unclear | Smooth curve preview (Catmull-Rom/ArduPilot spline approx.) | 2 |
| FP-004 | Loiter/Circle/Takeoff/Land/Do-Jump/Delay/Condition commands | `NAV_LOITER_*`(17-19), `NAV_LAND`(21), `DO_JUMP`(177), `NAV_DELAY`(93), `CONDITION_*`(112-115) | Hard to see flow | Flow graph overlay for jumps/loops, run-time guards (jump loop detector) | 2 |
| FP-005 | Terrain-following (terrain frame, SRTM 1″), altitude profile | `MAV_FRAME_GLOBAL_TERRAIN_ALT`(10), `TERRAIN_REQUEST`(133)/`TERRAIN_DATA`(134)/`TERRAIN_REPORT`(136) | Online DEM; separate graph | Offline DEM, profile docked under map, clearance rules | 2 |
| FP-006 | Geofence: inclusion/exclusion polygons, circles, return point, max alt, breach action | `mission_type=FENCE`; `NAV_FENCE_POLYGON_VERTEX_INCLUSION`(5001)/`_EXCLUSION`(5002), `NAV_FENCE_CIRCLE_INCLUSION`(5003)/`_EXCLUSION`(5004), `NAV_FENCE_RETURN_POINT`(5000); `FENCE_*` params; `FENCE_STATUS`(162) | Separate tab, easy to forget upload | Same canvas, layer toggle, "unsynced changes" badge, breach simulation | 2 |
| FP-007 | Rally points | `mission_type=RALLY`; `NAV_RALLY_POINT`(5100) | Hidden | Rally layer with RTL preview path | 2 |
| FP-008 | Survey / photogrammetry grid (overlap, sidelap, GSD, camera DB, trigger distance, gimbal trigger, lead-in, turnarounds, altitude per camera) | `DO_SET_CAM_TRIGG_DIST`(206), `DO_DIGICAM_CONTROL`(203), `IMAGE_START_CAPTURE`(2000), `DO_SET_SERVO` | Dialog-heavy, can't preview | Live-editable polygon with draggable vertices, instant stats (photos, time, battery legs, GSD) | 2 |
| FP-009 | Corridor mapping | as above | Plan-level extension (listed for survey parity with newer tools) | Polyline + width → offset passes | 2 |
| FP-010 | SAR patterns (expanding square, sector, parallel track, creeping line) | `NAV_WAYPOINT`, `LOITER` | Limited | Pattern generators with reproducible params | 2 |
| FP-011 | Circle/orbit auto-WP, text/shape auto-WP, polygon from file | — | Fiddly | Shape tools | 2 |
| FP-012 | File I/O: `.waypoints`, `.plan` (QGC), KML/KMZ, CSV, Poly, shapefile overlay | — | — | Import/export with validation report | 2 |
| FP-013 | Home position set/relocate, default alt/speed/frame | `HOME_POSITION`(242), `DO_SET_HOME`(179) | Confusing home vs. WP0 | Explicit home marker and frame warnings | 2 |
| FP-014 | Mission statistics (distance, time, battery) | derived | Rough | Model-based energy estimate with wind input | 2 |
| FP-015 | Altitude Angel/airspace overlays | external | Vendor-specific | Pluggable airspace providers | 4 |
| FP-016 | Undo/redo, multi-select, copy/paste, bulk edit | — | Absent | Full command-history editing | 2 |
| FP-017 | Mission verify (pre-upload lint): altitude below terrain, crossing fence, unreachable speed, missing takeoff | — | None | Lint rules shown inline | 2 |

## D. Initial Setup (`PAR-IS`)

| ID | Feature | MAVLink / protocol | Legacy pain point | Modern solution | Ph |
|----|---------|--------------------|-------------------|-----------------|----|
| IS-001 | Firmware install (manifest, APJ, custom file, bootloader) over USB; STM32 DFU; ArduPilot/PX4 bootloader serial protocol; DroneCAN peripheral update | Bootloader proto (not MAVLink); `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`(246) to enter bootloader; `firmware.ardupilot.org` manifest | Easy to brick; Windows drivers | Board auto-ID, verified checksums, progress/rollback guidance, driverless (libusb/IOKit) | 3 |
| IS-002 | Frame type / class selection | `FRAME_CLASS`, `FRAME_TYPE` | Image-based only | Interactive 3-D frame diagram with motor numbering | 3 |
| IS-003 | Accelerometer calibration (6-pos), level, simple | `MAV_CMD_PREFLIGHT_CALIBRATION`(241), `MAV_CMD_ACCELCAL_VEHICLE_POS`(42429), `COMMAND_LONG` | Text prompts; users miss orientation | 3-D model mirrors real attitude; auto-advance on stable detect | 3 |
| IS-004 | Compass calibration: onboard (background) & GCS sphere fit, motor-compass interference | `DO_START_MAG_CAL`(42424), `DO_ACCEPT_MAG_CAL`(42425), `DO_CANCEL_MAG_CAL`(42426), `MAG_CAL_PROGRESS`(191), `MAG_CAL_REPORT`(192), `RAW_IMU`(27)/`SCALED_IMU2` | Progress bars with no coverage feedback | Live sphere coverage mesh, fitness score, per-compass offsets, priority/orientation checks | 3 |
| IS-005 | Radio (RC) calibration, trim, reversing | `RC_CHANNELS`(65), `RC_CHANNELS_RAW`, params `RCn_MIN/MAX/TRIM/REVERSED` | Bar graphs | Stick-gimbal visual, endpoint capture, deadzone | 3 |
| IS-006 | Flight modes assignment (6 positions / channel) | `FLTMODE_CH`, `FLTMODE1-6` | Raw numbers | Live switch position indicator and mode names per vehicle | 3 |
| IS-007 | Failsafe matrix (RC/throttle, battery low/critical, GCS link, EKF, fence, terrain, ADS-B, crash/vibration) | params `FS_*`, `BATT_FS_*`, `FENCE_*`, `FS_EKF_*`, `TERRAIN_*` | Spread across tabs; no consolidated view | **Single matrix**: trigger × action grid, simulated trigger preview, "what happens when…" narrative | 3 |
| IS-008 | Servo output / function mapping | `SERVOn_FUNCTION`, `SERVO_OUTPUT_RAW`(36) | Opaque | Output rail diagram | 3 |
| IS-009 | Motor test (all/individual, throttle %, duration), ESC calibration | `MAV_CMD_DO_MOTOR_TEST`(209), `DO_SET_MOTOR_*` | Dangerous | Prop-off interlock, two-stage confirm, order/direction verification wizard | 3 |
| IS-010 | Battery monitor setup & calibration (voltage/current multipliers, capacity) | `BATT_*` params, `BATTERY_STATUS` | Manual math | Guided measure-and-compute | 3 |
| IS-011 | Rangefinder / optical flow / airspeed / compass priority / GPS config (incl. RTK/dual) | `DISTANCE_SENSOR`, `OPTICAL_FLOW`(100), `GPS2_RAW`(124), `GPS_RTK`(127) | Scattered | Sensor health board with live values | 3 |
| IS-012 | RTK/NTRIP GPS correction injection | `GPS_RTCM_DATA`(233) fragmented, `GPS_INJECT_DATA`(123) legacy; NTRIP 1.0/2.0 TCP | Windows-only form, no reconnect | Robust NTRIP client: caster browse, mountpoint filter, auto-reconnect, GGA feedback, fix status | 4 |
| IS-013 | SiK radio config (local/remote AT) | AT commands over serial; `RADIO_STATUS` | Fragile | Guided RFD900x/Holybro config with presets | 3 |
| IS-014 | Antenna tracker setup & passthrough | Tracker `MAV_TYPE_ANTENNA_TRACKER`(5), forwarded `GLOBAL_POSITION_INT` | Second GCS window | Router forwarding profile | 4 |
| IS-015 | Camera gimbal config, OSD config (MSP/analog), parachute, bluetooth, DroneCAN/UAVCAN GUI | params, DroneCAN GUI tool integration | Many forms | Unified Peripherals hub | 3–4 |
| IS-016 | Joystick / gamepad mapping | `MANUAL_CONTROL`(69), `RC_CHANNELS_OVERRIDE`(70) | DirectInput only | `GameController.framework` / XInput / SDL; button→action mapper; deadman | 4 |
| IS-017 | Terminal / MAVLink shell (NSH, console) | `SERIAL_CONTROL`(126) | Raw | Real terminal widget (xterm.js-class) with history | 3 |
| IS-018 | HW ID / board info, bootloader version, CPU/USB identification | `AUTOPILOT_VERSION`, `BOARD_VERSION` | — | Hardware passport page | 3 |
| IS-019 | Pre-arm check dashboard | `STATUSTEXT`, `SYS_STATUS`(sensors present/enabled/health) | Message hunt | Check list with direct links to fix | 3 |

## E. Config / Tuning (`PAR-CT`)

| ID | Feature | MAVLink | Legacy pain point | Modern solution | Ph |
|----|---------|---------|-------------------|-----------------|----|
| CT-001 | Full Parameter List (search, edit, write, refresh, load/save, compare, defaults, favourites) | `PARAM_REQUEST_LIST`(21), `PARAM_VALUE`(22), `PARAM_SET`(23), `PARAM_REQUEST_READ`(20) | Slow search, unsafe edits | Instant index (ms), changed-only view, diff, risk labels ([04](04-ux-design-system.md)) | 3 |
| CT-002 | Full Parameter Tree (grouped by prefix/metadata) | metadata | Sparse | Tree + breadcrumbs + related-params graph | 3 |
| CT-003 | Metadata-driven help, ranges, units, bitmasks, enumerations | ArduPilot `apm.pdef.json/xml` | Stale offline XML | Versioned cache per firmware, auto-update, bundled offline | 3 |
| CT-004 | Param file import/export (`.param`, `.parm`), batch apply with preview | — | No dry-run | Preview diff + undo checkpoint (auto snapshot before write) | 3 |
| CT-005 | Param compare (vehicle vs file vs defaults vs other vehicle) | — | Basic | 3-way diff engine, merge selections | 3 |
| CT-006 | Basic tuning (sliders for roll/pitch rate, stab, throttle, RC feel) | `ATC_*`, `PSC_*` | Opaque effect | Sliders with live preview curves and revert | 3 |
| CT-007 | Extended tuning (full PID axes, filters, harmonic notch) | `ATC_RAT_*`, `INS_HNTCH_*`, `INS_GYRO_FILTER` | Dense grid | Per-axis panels with step-response plots from logs | 3 |
| CT-008 | In-flight tuning (CH6 tuning knob), AutoTune review | `TUNE`, `AUTOTUNE_*` | Manual | Review tool: before/after overlay from logs | 3–4 |
| CT-009 | On-screen display (OSD) layout | `OSDn_*` params | Fiddly | WYSIWYG canvas with drag-and-drop elements | 3 |
| CT-010 | User params / custom views | local | — | User dashboards | 3 |
| CT-011 | MAVFTP file browser (params, scripts, logs, terrain) | `FILE_TRANSFER_PROTOCOL`(110) | Slow | Resumable, burst-read, drag-drop | 4 |
| CT-012 | Lua script upload / SD card management | MAVFTP | Hidden | Script manager | 4 |
| CT-013 | Planner settings (units, layout, theme, maps cache) | local | Scattered | Settings with search | 1 |
| CT-014 | Param write safety (reboot-required, persisted vs RAM, `PARAM_VALUE` verify) | — | Silent failure | Every write verified by readback; reboot banner | 3 |

## F. Simulation & Analytics (`PAR-SA`)

| ID | Feature | MAVLink / data | Legacy pain point | Modern solution | Ph |
|----|---------|----------------|-------------------|-----------------|----|
| SA-001 | SITL one-click (Copter, Plane, Rover, Sub, Tracker; frame, location, speedup, wipe, wind) | ArduPilot SITL binary over TCP 5760 / UDP | Needs downloaded exe, console windows | Managed SITL (download by firmware version, process supervisor, headless) | 4 |
| SA-002 | Dataflash log download over MAVLink | `LOG_REQUEST_LIST`(117), `LOG_ENTRY`(118), `LOG_REQUEST_DATA`(119), `LOG_DATA`(120), `LOG_ERASE`(121), `LOG_REQUEST_END`(122) | Slow, no resume | Resumable windowed download, MAVFTP alternative | 4 |
| SA-003 | `.bin` review / multi-axis plot / map / modes timeline | file | Freeze on big files | [05](05-telemetry-log-analytics.md) | 4 |
| SA-004 | `.tlog` record/replay with scrubbing & sync HUD | file | Basic | Timeline, speed 0.1×–32×, bookmarks | 1–4 |
| SA-005 | Auto-analysis (vibration, EKF, compass, power, events) | file | Text report | Rule-pack health dashboard | 4 |
| SA-006 | FFT analysis (onboard batch sampler + offline) | `ISBH/ISBD`, `FTN1/FTN2` log msgs | Separate tool | Integrated spectrogram with notch calculator | 4 |
| SA-007 | Geotagging images from log/camera timestamps | `CAM`, `CAMERA_FEEDBACK`(180) | Fragile time-offset | EXIF write, offset auto-estimate, preview map | 4 |
| SA-008 | Export CSV/KML/KMZ/GPX/MAT | — | — | Streaming exporters | 4 |
| SA-009 | Video streaming + overlays + recording | see [01 §6](01-architecture.md) | Plugin-fragile | Native pipeline | 4 |
| SA-010 | Camera/gimbal tools (Do Mount, ROI) | — | — | In video overlay | 4 |

## G. Cross-cutting (`PAR-XC`)

| ID | Feature | Notes | Ph |
|----|---------|-------|----|
| XC-001 | Localisation incl. RTL (Hebrew) | Qt Linguist, ICU, bidi-safe numeric readouts | 3 |
| XC-002 | Units (metric/imperial/knots/ft) + coordinate formats | Single conversion layer | 1 |
| XC-003 | Auto-update with signature check | Sparkle (mac), WinSparkle/MSIX (Win), AppImage/Flatpak (Linux) | 5 |
| XC-004 | Crash reporting (opt-in) + safe-mode start | Sentry-style, offline queue | 5 |
| XC-005 | Accessibility (VoiceOver labels, keyboard operation, colour-blind-safe) | Qt accessibility | 5 |
| XC-006 | Audit log of all commands/parameter writes | SQLite, exportable | 1 |
| XC-007 | Multi-window/multi-display, saved workspaces | Qt docking | 1 |
| XC-008 | In-app help and about: handbook, offline search, version/build info, update check entry | Offline help bundle ([06](06-documentation-system.md)) | 3 |

## Coverage accounting
Total rows: ~125 (grows as D0-7 audit completes). CI computes `% implemented`, `% tested`, `% documented` per ID and blocks GA unless all three = 100 % for every *must* row; rows marked *won't* require a signed waiver recorded in an ADR.
