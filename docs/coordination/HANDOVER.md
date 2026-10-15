# Handover — state of the project for the next agent

Written 2026-10-11 by `core-1`, the agent that did most of Phase 0, so another agent can take
over cold. Read this first, then [`README.md`](README.md) (the protocol) and
[`BOARD.md`](BOARD.md) (generated live state). Where this file and the log disagree, the log
and the code are newer. Where either disagrees with an owner instruction, the owner wins.

## 1. Read order (30 minutes)

1. This file.
2. [`../../AGENTS.md`](../../AGENTS.md) and [`README.md`](README.md): the non-negotiables and
   `scripts/coord.py`.
3. The plan: [`../plan/README.md`](../plan/README.md), above all
   [`01-architecture.md`](../plan/01-architecture.md) and
   [`08-safety-vv.md`](../plan/08-safety-vv.md).
4. Accepted decisions: [ADR-0009](../plan/adr/0009-link-state-never-blocks-commands.md)
   (link state never blocks commands),
   [ADR-0010](../plan/adr/0010-internal-distribution-without-apple-credentials.md)
   (no Apple credentials),
   [ADR-0011](../plan/adr/0011-operational-profile-and-command-classes.md) (operational
   profile, single-action flight controls).
5. The hazard log: [`../safety/hazards.md`](../safety/hazards.md). Every safety task traces
   to a hazard ID (H1–H18) and a safety requirement (SR-01…SR-10).
6. `git log --oneline | head -60`. The commit messages are written to be read: what changed,
   why, and the evidence.

## 2. How the owner works (standing instructions)

* **Pushing.** The owner switches between two modes:
  * "push after CI is green, then continue";
  * "**commit locally only; do not push until I tell you**". This was the last standing
    instruction. It was lifted once ("push everything") and everything was pushed.

  If unsure, commit locally and ask. A stop hook nags about unpushed commits; the owner's
  instruction overrides it.
* **Reporting.** After every finished task, report:
  * what was done and the evidence;
  * how many commits are waiting on a push, listed.

  Report failures plainly.
* **"When CI is green, continue with the next task."** Check CI after each push. Fix red CI
  before anything else.
* **Stop and ask** for credentials, licensing, scope, safety-path changes (command pipeline,
  interlocks, failsafes, firmware flashing) and anything irreversible (AGENTS.md).
* **Owner decisions already made** (do not re-ask):
  * Link state never blocks or delays a command (ADR-0009).
  * CommandPolicy design approved (T0-16).
  * No Apple developer credentials: ad-hoc signed, internal distribution (ADR-0010).
  * No 5K or ProMotion (120 Hz) Mac display is available (2026-10-12): those tests and targets
    are deferred to `docs/plan/future-upgrades.md`; Mac display tests use the M1 MacBook Air's
    built-in Retina screen.
  * Parity: the recommendations were accepted; Cursor-on-Target (CoT) is in scope; swarm
    comes later.
  * Design tokens: Night-Ops telemetry `#FF4D4D` (6.4:1, accepted residual risk); panels
    `#333333`; Daylight caution `#A85900`.
  * The owner-provided flight log and param file are test fixtures (`docs/corpus/`).
  * Operational directive, Flight Operations & Systems Safety Lead, 2026-10-11 (ADR-0011):
    * MIL-STD-882E applies;
    * all mode changes are single action;
    * two-stage confirmation only for in-flight disarm, reboot and emergency motor
      termination;
    * command lifetimes: RTL/Land 30 s, mode change 2–3 s, others 10 s;
    * reboot is blocked while armed;
    * props-off attestation is required for every vehicle type.
  * **Field-operations posture, owner, 2026-10-12 (ADR-0012)**: a lean, permissive tool for
    expert operators; no typed confirmations, no blocking on certification paperwork.
    * Parameter writes for all parameters (failsafe, arming, fence, board safety included):
      a single-click confirmation, no typed text.
    * In-flight mission upload: one confirmation ("Update mission on active vehicle?").
    * CLI mission upload with no state lock.
    * Fence and rally upload on the ground and in flight, with read-back verification.
    * Camera trigger commands in generated missions.
    * Log erase without a prior download.
    * Hazard log: I1–I3 signed off (I3 amended by in-flight upload), H14 accepted, review
      approved.
    * **Amended 2026-10-13 (owner's go-ahead for items 3–5):** in-flight mission upload,
      fence/rally upload and log erase each sit behind an **opt-in setting, off by default,
      with a short warning about its danger** (`armed-mission-upload`, `fence-rally-upload`,
      `log-erase`; ADR-0012 amendment). With a setting on the operator can make that change.
    * **Phase 0 gate decisions, 2026-10-15 (ADR-0013):** map spike go (42 % GPU accepted,
      reductions in future-upgrades §3); video spike accepted on condition of a no-hold
      decode path in Phase 1; packaging exit criterion restated for ad-hoc signing;
      parameter metadata downloaded at run time (nothing GPL-derived shipped); force-arm
      and parameter reset behind opt-in settings, off by default.
    * Unchanged (not decided): ADR-0011 D2–D5 (two-stage in-flight disarm, reboot,
      termination; reboot blocked while armed; props-off for motor test), force-arm typed.
    * Closed source, clean-room; MAVLink XML/generated code and pymavlink approved;
      permissive licences (BSD/MIT/Apache-2.0) approved; prefer mature crates.
    * Trademark deferred; font Inter (OFL); Hebrew/RTL P3; no Mac App Store build.
    * The fleet runs ArduPilot 4.6.x: 4.6 SITL scenarios in CI.
    * **Never commit field logs with real-world positions**; only synthetic, sanitised or SITL
      logs. (The 4.6.2 corpus fixture has no GPS or position records.)
    * UI frame criterion: no frame over 25 ms (ADR-0001).
    * gRPC: configurable bind address (all interfaces allowed), with a pre-shared bearer key.
    * CI: full 4-OS SITL matrices on releases and schedules; lean checks on pushes and PRs.

## 3. Environment facts and traps

| Fact | Consequence |
|------|-------------|
| The git remote `github.com/djdanou-cloud/droraero-mission-planner` redirects to `djdanou-cloud/drorcontrol` | `git push` prints "This repository moved"; harmless. **GitHub MCP tools only accept `repo: droraero-mission-planner`**; the name `drorcontrol` is denied. |
| The session may name a `claude/…` branch | The owner's rule wins: **work on `main`**. |
| GitHub Actions minutes ran out once | Every job "fails" in about 2 s with no runner (`runner_id 0`) and no logs. That is billing, not code. Mac minutes cost 10×. Re-run once the limit resets (`mcp__github__actions_run_trigger rerun_workflow_run`). |
| No Mac and no GPU in the cloud container | Mac work is verified on CI's macOS runners (VMs, indicative only). Real numbers come from the [Mac test runbook](../dev/mac-test-runbook.md). |
| apt has only Qt 6.4; downloads from `download.qt.io` are blocked by the proxy | The map spike (Qt ≥ 6.5) cannot be built locally; it builds in the `spike-map` workflow. Spike S0-1 builds with Qt 6.4 (`QMAKE=qmake6`). |
| GStreamer installs from apt (`libgstreamer1.0-dev` + plugins) | The video spike builds and runs locally (software decode only). |
| Local ArduPilot SITL: `scripts/build_sitl.sh` → `.sitl/ardupilot-Copter-4.7.1/` (gitignored, about 5 min) | `DC_SITL_REQUIRED=1 cargo test -p sitl-harness --test scenarios` runs all scenarios (17, listed in `docs/dev/sitl-harness.md`; about 3 min on 4 vCPU). Stop docs serving on port 8000 or use port 18000: SITL instance 224 uses port 8000. The pinned ArduPilot source in `.sitl/` is the reference for ArduPilot behaviour: read it instead of guessing. |
| macOS pseudo-terminals cannot be opened by path through `serialport` (it sets the baud rate with `IOSSIOSPEED`) | Serial tests use `TTYPort::pair()` and the injectable `Opener` (`link-serial`). |
| Mac SDK mismatch (seen on macOS 27 with Xcode 26.x, 2026-10-11) | Homebrew Qt's `qt.toolchain.cmake` and the default `xcrun --show-sdk-path` pick the Command Line Tools SDK, whose `.tbd` stubs Xcode's linker rejects ("unknown architecture"). CMake: pass `-DCMAKE_OSX_SYSROOT=$(xcrun --sdk macosx --show-sdk-path)` (`ui/spikes/s0-3-map/build.sh` cannot pass it yet). Plain `cc`/waf (SITL): `export SDKROOT=$(xcrun --sdk macosx --show-sdk-path)`. |
| GUI spikes need a desktop session | Over SSH Qt prints "Cannot create window: no screens available" and GStreamer's `gst_macos_main` hangs. Run them from a terminal in a logged-in desktop session. Running one user's app in another user's desktop (`launchctl asuser`) crashed in AppKit. |
| SITL instance 124 → TCP 7000 | One full-suite run on a Mac lost a SITL at start-up on instance 124 (port 7000, used by macOS AirPlay Receiver); the rerun passed 9/9. Suspected, not confirmed. |
| `coord.py` | Quote glob paths (`'crates/x/**'`). Log types are only `progress handoff question decision-request blocker conflict fyi done`. |

## 4. Phase 0 status (backlog T0-01…T0-16)

| Task | State | Evidence / what is left |
|------|-------|-------------------------|
| T0-01 repo, CI on 4 targets | done | `ci.yml`; `cargo xtask ci` |
| T0-02 MAVLink codec | done | `crates/mavlink-codec`; XML and generated code approved (ADR-0012 item 9) |
| T0-03 router, links, tlog | done | `mav-router`, `link-*`, `recorder` |
| T0-04 core daemon, gRPC, FFI, CLI | done | `dc-core`, `api`, `core-ffi`, `dc-cli` |
| T0-05 macOS serial | **code done; hardware matrix pending** | Mac runbook T0-05; `crates/platform-macos/HARDWARE_MATRIX.md` |
| T0-06 spike S0-1 bridge | **M1 run done; strict p99 bound missed** | M1: 0 dropped frames, p99 19.5 ms at 200 k (17.8 ms idle). Criterion needs restating in ADR-0001; Intel not run (120 Hz deferred: `docs/plan/future-upgrades.md`) |
| T0-07 spike S0-3 map | **Go (owner 2026-10-15, ADR-0013)**: Metal, fps, vsync, feel pass; GPU ≈ 42 % accepted for now | `ui/spikes/s0-3-map/RESULTS.md`; reductions suggested in `docs/plan/future-upgrades.md` §3 |
| T0-08 spike S0-4 logs | done | `crates/logs` |
| T0-09 spike S0-5 video | **Accepted with a condition (owner 2026-10-15, ADR-0013)**: hardware decode and zero-copy pass; live p99 113 ms is frames the decoder holds (36 ms with `max_dec_frame_buffering` 1): Phase 1 builds a no-hold decode path | `ui/spikes/s0-5-video/RESULTS.md` |
| T0-10 macOS package | done on CI; local M1 package + first-launch steps 1–3 pass | Exit criterion restated for ad-hoc signing (ADR-0013); "Open Anyway" not yet verified on the M1 (approved in another account than the app's owner); see log |
| T0-11 parity freeze | done | `parity.yaml`, `docs/parity/` |
| T0-12 design tokens | done | `ui/tokens/` (owner decisions applied) |
| T0-13 docs site | done | `mkdocs build --strict` in CI |
| T0-14 SITL harness | done | `crates/sitl-harness`, `docs/dev/sitl-harness.md` |
| T0-15 hazard log | done: review approved by the owner 2026-10-12 | `docs/safety/hazards.md` §6 |
| T0-16 CommandPolicy | done | `crates/protocols/src/command.rs` |

## 5. Done beyond the backlog (all pushed, CI green on 130cae8)

| Work | Where | Evidence |
|------|-------|----------|
| Operational profile: command classes, 3 s mode lifetime (ADR-0011) | `protocols::command`, `dc-core` config bounds | `command_policy.rs`, `liveness.rs` |
| Serial link reopens after unplug, and follows a device to a new name only when unambiguous | `crates/link-serial/src/lib.rs` (`spawn_reopening`, `resolve_path`) | Unit tests (pty); hardware check in the runbook |
| Landed / VTOL state (`EXTENDED_SYS_STATE`) in vehicle facts; no rule uses it yet (ADR-0011 I2) | `vehicle`, `dc-core::facts` | SITL: ON_GROUND |
| Flight-mode names from `ardupilotmega.xml`; firmware confirmed only by its version banner (a QuadPlane can report a multirotor `MAV_TYPE`) | `vehicle::modes`, `dc-core` (`DO_SEND_BANNER`) | SITL: "MODE LOITER (5): VERIFIED" |
| SR-05 alerts: Warning when a flight command ends unconfirmed, Caution for a ground command, Advisory when a mode is verified | `CommandStatus::alert`, `CoreEvent::Alert`, gRPC `AlertEvent` | `unconfirmed_mode_change_alerts_the_pilot` |
| H16 navigation health mirrored from the autopilot's EKF (ArduPilot `FS_EKF_THRESH` rule) | `vehicle::nav`, `Core::nav_health` | SITL: GPS failed → UNTRUSTED in 7.5 s |
| H4 mission upload verified by read-back; ArduPilot storage rules; refused while armed | `protocols::mission`, `Core::upload_mission` | 30 % loss × 20 seeds; SITL: 20 % loss, 14 items, 0 differences |
| Mission progress (`MISSION_CURRENT`, `MISSION_ITEM_REACHED`) in `VehicleState` | `vehicle::MissionProgress` | SITL: AUTO mission flight followed item by item to COMPLETE |
| gRPC `Vehicle` carries `mode_name` (banner-confirmed only), `mission` progress, `landed` state, `nav` health (H16 trust + reasons), `link` state, `home` (HOME_POSITION, requested at 0.2 Hz) ADS-B `traffic` (ADSB_VEHICLE requested at 5 Hz; valid-flag fields only; forgotten after 60 s; distance, bearing and height relative to the vehicle once it has a position) `batteries` (BATTERY_STATUS at 2 Hz; pack voltage = sum of valid entries, cells only when measured), `vibration` (VIBRATION, 2 Hz), `wind` (WIND, 1 Hz), `rc` (RC_CHANNELS, 1 Hz), `servos` (SERVO_OUTPUT_RAW by port, 1 Hz) `radio` (RADIO_STATUS from a telemetry radio, under the system id it reports with) `rangefinders` (DISTANCE_SENSOR, 2 Hz, with an in-range flag) `terrain` (TERRAIN_REPORT, 1 Hz; flags extrapolated heights and the boot-time 0,0 report) and `autopilot_version` (AUTOPILOT_VERSION, requested once on connect like the banner; `tested` says whether the automated tests cover that firmware and major.minor, ArduCopter 4.7 today); `streams`: frames received / lost per component from MAVLink sequence numbers (total and last 10 s); `dc-cli telemetry` shows the loss and the firmware version (flagged when not covered by the tests) | `proto/dc/v1/core.proto`, `api::vehicle_to_pb` | unit test; checked against SITL through `dc-cli daemon` + `remote vehicles` |
| Read-only download of the geofence and rally points (`Core::download_items`, `MissionType`; `dc-cli mission download --type fence|rally`). Upload since 2026-10-13 behind the opt-in `fence-rally-upload` setting (§7) | `protocols::mission`, `dc-core`, `dc-cli` | SITL: fence and rally read back as set |
| gRPC `DownloadItems` (mission / fence / rally, read-only); `dc-cli remote items --type …` | `proto/dc/v1/core.proto`, `api` | gRPC integration test (fake vehicle) |
| `.tlog` replay link `tlog:PATH[@SPEED]` (1–100×): original timing, receive-only, allowed only as the core's sole link (no relaying of recorded traffic to a vehicle) | `dc-core/src/replay.rs`, `Core::connect` | `dc-core/tests/replay.rs`; daemon + `remote vehicles` on a SITL recording |
| Dataflash log health checks (`logs::health`, `dc-cli log health`): vibration/clipping (ArduPilot wiki guidance, commit f4b21ca), battery vs the vehicle's BATT_LOW_VOLT/BATT_CRT_VOLT, EKF3 variances vs its FS_EKF_THRESH, ERR records with subsystem names, GPS (HDOP vs the vehicle's GPS_HDOP_GOOD, 3D-fix losses) and board voltage (BRD_VBUS_MIN to 5.8 V, the arming range; NaN = not measured) | `crates/logs/src/health.rs` | unit tests; both fixture logs |
| `dc-cli tlog export FILE --gpx/--kml [--vehicle N]`: flight track from GLOBAL_POSITION_INT (GPX 1.1, KML 2.2 absolute altitude). `dc-cli log export FILE.bin` does the same from the EKF position (POS) with UTC from the GPS clock (`logs::track`) | `crates/dc-cli/src/track.rs` | unit tests; 70 s SITL recording, both files parse |
| STATUSTEXT history: last 500 lines per vehicle in the core (outside the per-message fleet clone), gRPC `GetMessages`, `dc-cli remote messages` | `Core::messages`, `api` | `dc-core/tests/messages.rs` (cap via fast replay), gRPC test |
| Mission statistics (`protocols::mission::stats`): navigation items, path length (great circle; 0,0 = current position as in `ModeAuto::get_loc_from_cmd`; RTL back home; DO_JUMP not expanded), highest altitude per frame, last nav command; printed by `dc-cli mission check/download` | `crates/protocols/src/mission.rs` | unit test |
| Dataflash log download over MAVLink (`protocols::log_transfer`, `Core::list_logs/download_log`, `dc-cli log list/download`): refused unless disarmed (ArduPilot ignores log requests while armed); gaps re-requested as one span after the stream goes quiet; written to `.partial` then renamed. LOG_ERASE since 2026-10-13 behind the opt-in `log-erase` setting (§7) | `crates/protocols/src/log_transfer.rs` | unit tests; SITL: byte-identical to SITL's file through 10 % loss (46 kB/s) |
| Camera geotags (CAMERA_FEEDBACK): up to 20 000 per vehicle in the core, repeated image reports replace the earlier one; gRPC `GetGeotags`, `dc-cli remote geotags` (CSV). No camera trigger command (actuation goes through CommandPolicy) | `Core::geotags`, `api` | `dc-core/tests/geotags.rs` |
| Mission events: `CoreEvent::Mission` / gRPC `MissionEvent` (item reached, current item, mission state) | `dc-core`, `api` | SITL AUTO flight: reached 2 and 3, current 4, Active, Complete |
| gRPC `ListLogs` / `DownloadLog` (server stream: progress, then 1 MiB chunks, last has `done`); `dc-cli remote logs [--download ID --out FILE]` | `api` | gRPC test with a fake vehicle log; SITL via daemon (360 kB) |
| gRPC `GetLinkStats`: router links, learned routes (system/component → link) and counters (frames, flooded, loops dropped, core-queue and link-event drops); `dc-cli remote links` | `api`, `Core::hub_snapshot` | gRPC test with a fake vehicle |
| CI uploads `dc-cli-<platform>` release builds (14 days, unsigned) for hardware bench tests; [`docs/dev/windows-bench-test.md`](../dev/windows-bench-test.md): read-only bench session with a real flight controller on Windows (e.g. Copter 4.6.3) | `.github/workflows/ci.yml` | actionlint; every command line in the sheet checked to parse; first real-hardware run 2026-10-13 (MicoAir743v2, Copter 4.6.3): steps 1–6, 8, 9, 11–16 pass with 7 findings, see `docs/dev/bench/results-2026-10-13-windows-micoair743v2.md` |
| `dc-cli tlog params` (PARAM_VALUE, last value per name -> .param) and `tlog changes` (later PARAM_VALUE reports with a different value) and `tlog mission [--type mission|fence|rally]` (last complete MISSION_COUNT + items of that mission type from one sender, `_INT` or not; empty counts -> .waypoints) | `crates/dc-cli/src/tlog.rs` | unit test; SITL: 1 370 params from a recording identical to a direct download |
| `.bin` export: `dc-cli log csv FILE MSG` (all fields, text quoted; `logs::export::csv`) and `log params` (PARM -> .param) | `crates/logs/src/export.rs` | ATT row matches pymavlink; exported .param vs the log: 0 differences in 1 548 |
| Mission lint (`protocols::mission::lint`, printed by `dc-cli mission check/download`): DO_JUMP to a missing item, takeoff/waypoints at or below 0 m above home, invalid positions (warnings); no TAKEOFF first per `AP_Mission::is_takeoff_next` (Copter refuses AUTO landed+armed: "Auto: Missing Takeoff Cmd"), no RTL/LAND at the end, mixed altitude references, terrain frames (notes) | `crates/protocols/src/mission.rs` | unit tests |
| Survey grid (`protocols::survey`, `dc-cli mission survey --polygon ... | --polygon-kml FILE --spacing --angle --alt`): lawnmower lines over a polygon (local plane, concave = one segment per crossing), takeoff + grid + RTL; `--crosshatch` (second set of lines at 90 degrees); `--overshoot M` (lines extended past both edges); `--camera W,H,F,PXW,PXH --sidelap --frontlap`: GSD, photo footprint, spacing from side overlap, photo distance and count (the mission does not trigger the camera) | `crates/protocols/src/survey.rs` | unit tests; SITL: generated survey uploads and verifies |
| Mission flight-time lower bound (`protocols::mission::{Speeds, estimate_time}`, `dc-cli mission check --params FILE`): straight legs at WP speeds (3-D, slower axis wins), holds, LOITER_TURNS, DELAY, RTL climb/return, landing above/below LAND_ALT_LOW; speeds from 4.7 names (m) or 4.6 names (cm), else 4.7.1 defaults | `crates/protocols/src/mission.rs` | unit tests; SITL AUTO flight: estimate 53 s, flown 79 s (asserted 1-2x) |
| Mission path export (`protocols::mission::path`, `dc-cli mission export FILE --gpx --kml`): home, takeoff up, navigation items, landing down, RTL home and down; absolute (MSL) when the file's home has an altitude, else relative to ground | `crates/protocols/src/mission.rs`, `crates/dc-cli/src/track.rs` | unit test; checked on a planned and a generated file (XML parses) |
| Mission editing offline (`protocols::mission::{reverse, shift}`, `dc-cli mission reverse|shift`): route backwards keeping home/TAKEOFF/final RTL-LAND (refused with DO_JUMP); move located items by metres (home optional, 0,0 items kept) | `crates/protocols/src/mission.rs` | unit tests; CLI checked on a planned file |
| `dc-cli mission diff A B [--with-home]`: two mission files compared item by item with the verify rules used after upload (count, then fields); exits non-zero on any difference | `crates/dc-cli/src/main.rs` | checked: identical, one altitude changed, different lengths |
| `dc-cli mission download --type fence|rally --out FILE`: the items as the vehicle holds them in QGC WPL 110 (read back by DrorControl; Mission Planner's own fence/rally file formats are not written) | `crates/dc-cli/src/main.rs`, `crates/protocols/src/wpl.rs` | WPL round-trip test of polygon, circle and rally items; SITL download (empty fence) |
| Mission vs geofence (`protocols::fence`, `dc-cli mission check FILE --fence FENCE [--params P]`): inclusion/exclusion polygons and circles from fence items, FENCE_TYPE circle (FENCE_RADIUS) and altitude (FENCE_ALT_MAX), intersection or union (FENCE_OPTIONS bit 1) as `AC_PolyFence_loader::breached`; the path is sampled every 5 m, so a leg clipping a corner is caught; read-only | `crates/protocols/src/fence.rs` | unit tests; checked on a planned mission with a containing and a too-small fence |
| `.bin` flight views (`logs::flight`, `dc-cli log summary|events|changes`): armed periods (ARM, else EV 10/11; a log that starts armed counts from its start), armed time, max rel alt, top speed, distance while armed, furthest from home (POS vs latest ORGN home), fastest climb/descent (CTUN.CRt, 4.6 cm/s or 4.7 m/s), mAh used; timeline of MODE/ARM/EV/ERR/MSG (modes named from the banner); in-flight parameter changes with autopilot-made ones marked (STAT_* counters, learned MOT_THST_HOVER); armed time per flight mode (`flight::mode_times`) | `crates/logs/src/flight.rs` | SITL AUTO flight log: 1 flight, 86.7 s, 20.0 m, 189 m, 650 mAh; only automatic changes |
| `.bin` motors (`logs::motors`, `dc-cli log motors`): flying periods from the land detector (EV 28/18, else armed); mean output per motor from RCOU mapped by SERVOn_FUNCTION, worst deviation, quad plus/X clockwise-vs-counter-clockwise difference; hover throttle: learned CTUN.ThH, mean ThO in level flight (|CRt| < 0.6 m/s; 4.6 int16 cm/s and 4.7 float m/s both read), flag at the 0.125/0.6875 learning limits | `crates/logs/src/motors.rs` | unit tests on synthetic logs; SITL AUTO flight (4 motors within 0.5 %, hover 37 %); owner's real 4.6.2 log |
| `.bin` attitude tracking (`logs::tuning`, `dc-cli log attitude`): RMS and max of desired minus achieved roll/pitch/yaw (yaw wrapped) while flying; numbers for comparison, no pass/fail | `crates/logs/src/tuning.rs` | unit test; SITL AUTO flight (roll 0.12°, pitch 0.21°, yaw 0.77° rms); owner's real 4.6.2 log |
| `.bin` battery under load (`logs::battery`, `dc-cli log battery [--instance]`): max/mean current and power, lowest voltage, largest sag below the resting estimate, mAh/Wh used while flying and mAh/min, logged `Res` and a least-squares fitted resistance | `crates/logs/src/battery.rs` | unit test; SITL AUTO flight (646 mAh, 28 A mean); owner's real 4.6.2 log |
| `.bin` compass interference (`logs::compass`, `dc-cli log compass [--instance]`): field strength (rotation-invariant) regressed on BAT current while flying: mGauss/A, correlation, change at the highest current as % of the field; a lower bound of the on-ground compassmot figure | `crates/logs/src/compass.rs` | unit test (rotating field); SITL AUTO flight with SIM_MAG_MOT_Z 1 mGauss/A: -0.80 mGauss/A, r -0.82 |
| `.bin` photos (`logs::camera`, `dc-cli log geotags [--trig] [--out]`): CAM (or TRIG) records as geotag CSV with UTC from the record's GPS week/time (18 leap s) | `crates/logs/src/camera.rs` | unit test (UTC cross-checked in Python); no SITL check: triggering the camera needs the pending camera-command decision |
| `.bin` wind (`logs::wind`, `dc-cli log wind`): EKF wind (XKF2 VWN/VWE, core 0) while flying, mean (vector) speed and from-direction, strongest; all-zero = not estimated (Copter needs drag fusion, EK3_DRAG_BCOEF_X/Y) | `crates/logs/src/wind.rs` | unit tests; SITL AUTO flight now flies in 4 m/s wind from the north with drag fusion: estimated 4.07 m/s from 3 deg |
| `.bin` EKF solution status (`log health`/`report`, check `ekf status`; plan 05 §2.1): XKF4.SS (`nav_filter_status`) on the primary core while flying — attitude/velocity/vertical-position invalid, dead reckoning, absolute position lost after having it, constant-position mode after having a position (Warning); GPS glitching (Caution) | `crates/logs/src/health.rs` (`status_problems`) | unit test; SITL AUTO scenario asserts OK; owner's no-GPS whoop: OK (constant-position mode without a position source is normal) |
| `.bin` compass offsets and consistency (`log health`/`report`, check `compass`; plan 05 §2.3): offsets above 600 mG Caution, above `COMPASS_OFFS_MAX` (default 1800) Warning; every used compass (`COMPASS_USE*`) against compass 0 while armed with `Compass::consistent`'s limits (90° 3-D, 60° horizontal, 200 mG; Warning) and 30° horizontal (Caution) | `crates/logs/src/health.rs` (`offset_level`, `disagreement`) | unit test; SITL AUTO scenario asserts all OK (3 compasses, 23 mG offsets, 0° apart) |
| `.bin` arming-check failures (`log health`/`report`, check `arming`; plan 05 §2.4): distinct `PreArm: …`/`Arm: …` MSG texts with first time and count (Caution); only in logs with `LOG_DISARMED` set | `crates/logs/src/health.rs` (`arming_messages`) | unit test; SITL 4.7.1 with LOG_DISARMED 1, BATT_ARM_VOLT 13: "PreArm: Battery 1 below minimum arming voltage (2 times) at 30.4 s" |
| `.bin` power status (`log health`/`report`, check `power`; plan 05 §2.4): POWR Flags/AccFlags as `AP_HAL::AnalogIn::PowerStatusFlag` — peripheral over-current ever (Warning), USB connected while armed and power status changing while armed (Caution); OK line gives the servo-rail range when measured | `crates/logs/src/health.rs` (`power_problems`) | unit test; owner's 4.6.2 log OK (SITL logs no POWR) |
| `.bin` EKF resets (`log health`/`report`, check `ekf resets`; plan 05 §2.1): position resets = changes of XKF4.OFN/OFE (primary core) while flying, with the largest size; yaw resets = EV 62 while flying, those within 2 s of EKF3's own planned-alignment message counted as expected ("in-flight yaw alignment complete", "yaw aligned…"; "ground mag anomaly" is not); Caution for unexpected ones | `crates/logs/src/health.rs` (`position_resets`, `classify_yaw_resets`) | unit tests; SITL AUTO 4.7.1 and 4.6.3: the one yaw reset 2.5 s after takeoff is the planned in-flight alignment → OK |
| `.bin` sensor agreement (`log health`/`report`, checks `baro` and `gps`): barometers vs barometer 0 while armed — largest altitude gap (information; no arming limit), unhealthy samples while armed Warning (`H` 4.7 / `Health` 4.6); GPS 1 vs GPS 0 while armed with both on 3D fix — 3-D distance, Warning at ArduPilot's 50 m arming limit (`AP_GPS::all_consistent`) | `crates/logs/src/health.rs` (`baro_agreement`, `gps_agreement`, `fix_distance_m`) | unit test (3-D distance); SITL AUTO 4.7.1/4.6.3: baros 0.20/0.34 m apart, OK; owner's whoop: baros 2.35 m apart while armed on the ground. Two-GPS path tested by unit test only (SITL scenario has one GPS) |
| Rate-controller oscillation (`logs::tuning::rate_oscillation`, in `log attitude`; plan 05 §2.4): RATE desired−actual per axis over the longest flight, Welch spectrum, strongest peak ≥ 1 Hz and its share of the error power (one narrow peak with most of the error = oscillation); notes when RATE is logged slowly (10 Hz default; ATTITUDE_FAST for loop rate) | `crates/logs/src/tuning.rs` | unit test (12 Hz oscillation 80 %+ share vs noise < 20 %); SITL AUTO: 3 axes, shares 0.12–0.17 |
| Spectrogram and throttle map (`logs::spectrogram`, `dc-cli log spectrogram [--csv F]`; plan 05 §5): 1 s Welch slices while armed; strongest frequency per throttle band (CTUN.ThO) — rising with throttle = motor harmonic, fixed = resonance | `crates/logs/src/spectrogram.rs` | unit test (tone 60→120 Hz with throttle 0.2→0.6); SITL AUTO: injected 37 Hz found in every band (35–39 Hz, 3.9 Hz bins) |
| Notch designer (`logs::notch_design::NotchConfig`, in `log notch [--response F.csv]`; plan 05 §5): ArduPilot's harmonic notch as built (biquad per HMNCS harmonic, A=10^(−ATT/40), Q from FREQ/BW, harmonics past Nyquist dropped; single notches only) — gain at the measured peak for suggested and current settings, response CSV | `crates/logs/src/notch_design.rs` | unit test (−40 dB at centres, flat far away); SITL AUTO: suggested notch −37.5 dB at the injected 37 Hz |
| `.bin` IMU consistency (`logs::imu::imu_consistency`; `log health`/`report`, check `imu`; plan 05 §2.4): each IMU against IMU 0 while armed, 5 s means (vibration averaged out, bias kept), with the arming limits — accel `ARMING_ACCTHRESH` (0.75 m/s², ×3 from the third IMU, Z halved), gyro 5°/s; Warning above, Caution above half | `crates/logs/src/imu.rs`, `health.rs` | unit test; SITL AUTO asserts OK (IMU 1 with SIM_ACC1_RND 2: 0.02 m/s², 0.18°/s); owner's log 0.05 m/s², 0.05°/s |
| `.bin` altitude-hold error (`logs::altitude::hold_error`, in `log altitude`/`report`; plan 05 §2.4): CTUN.Alt − DAlt while flying in altitude-controlled modes (DAlt is logged as 0 in manual-throttle modes, skipped): mean, largest and when | `crates/logs/src/altitude.rs` | unit test; SITL AUTO asserts it — 843 samples, mean +0.01 m, largest 0.76 m (at touchdown) |
| Machine-readable health (`dc-cli log health LOG --json [--fail-on caution|warning]`; plan 05 §2 "JSON usable in CI"): schema `dc.health/1` — `log`, `overall` (worst level or null), `findings[]` {check, level ok/caution/warning, text, at_s}; `--fail-on` exits non-zero when any finding reaches the level (log index line goes to stderr) | `crates/dc-cli/src/main.rs` (`health_json`, `log_health`) | unit test; SITL log with a forced PreArm failure: overall caution, `--fail-on caution` exit 1, `--fail-on warning` exit 0 |
| Machine-readable report (`dc-cli log report LOG --json`): schema `dc.report/1` — log, firmware banner, summary (flights, armed_s, distance_m, maxima or null), modes [{mode, armed_s}], health {overall, findings}, takeoffs, landings; times in s since boot | `crates/dc-cli/src/main.rs` (`report_json`) | owner's 4.6.2 log: ALT_HOLD 11.1 s, 13 findings, no takeoff/landing (never left the ground) |
| `log events --csv F`: the event timeline as CSV (time_s, kind mode/arm/event/error/message, detail quoted) | `crates/dc-cli/src/main.rs` | owner's log: 18 events, parses as CSV |
| Audit log check (`dc-cli audit-verify [FILE] [--print]`): verifies the hash chain of the command/parameter audit log and prints records with UTC; names the first broken line | `crates/dc-cli/src/main.rs` (`audit_verify`) | SITL param-set/rollback audit log: 6 records intact; one edited value → "chain broken at line 3", exit 1 |
| `dc-cli about`: version (+ commit for CI builds via GITHUB_SHA at compile time), OS/arch, debug/release, serial ports seen, firmware tested in CI — for test reports | `crates/dc-cli/src/main.rs`; `dc_core::TESTED_FIRMWARE` re-export | local: `0.0.0 (local build)`, linux/x86_64, Copter 4.7/4.6 |
| Single parameter reads (`Core::read_param`, `dc-cli param-get --connect L NAME…`): PARAM_REQUEST_READ by name, retried each second, 5 s limit; no full download | `crates/dc-core/src/parameters.rs`, `main.rs` | core test (mock vehicle); SITL: WP_SPD and FS_EKF_THRESH read in < 1 s, unknown name reported after 5 s |
| `params-review --connect LINK`: the safety-net review on the vehicle's live parameters (downloaded, read-only) instead of a file | `crates/dc-cli/src/main.rs` | SITL 4.7.1 defaults: BATT_FS_LOW_ACT OFF; GCS failsafe, fence and notch off noted |
| Aliasing note in `log fft` and `log notch` (plan 05 §2.2): the logged IMU rate's Nyquist limit, and where the motor rotation (ESC mean RPM/60) and the enabled `INS_HNTCH_FREQ` would fold to if above it (`logs::spectrum::alias`), with advice to log faster before trusting peaks | `crates/logs/src/spectrum.rs`, `notch.rs` (`sample_rate_hz`), `crates/dc-cli/src/main.rs` (`print_aliasing`) | unit test (fold-back); owner's log IMU.AccZ at 765 Hz: notch 150 Hz and motors ~103 Hz visible, no warning |
| `.bin` failsafe timeline (`logs::failsafe`, `dc-cli log failsafes`, in `log report`): ERR of FAILSAFE_*/CRASH_CHECK/EKFCHECK/THRUST_LOSS_CHECK (triggered, cleared) and mode changes whose ModeReason is a failsafe (4.7.1 = 4.6.3 numbering), cause listed before effect at equal times | `crates/logs/src/failsafe.rs` | unit test; SITL GCS-failsafe scenario 4.7.1 and 4.6.3: FAILSAFE_GCS triggered → RTL (GCS failsafe) → cleared 8–9 s later |
| `.bin` takeoffs (`logs::takeoff`, `dc-cli log takeoffs`, in `log report`): per NOT_LANDED (EV 28) the time from arming, fastest climb and largest roll/pitch error (ATT desired − actual) in the first 5 s; over 10° flagged (motor order/direction, CoG, props) | `crates/logs/src/takeoff.rs` | unit test; SITL AUTO scenario asserts one clean takeoff — 4.7.1: 3.3 s after arming, 2.53 m/s, 0.18°/0.28°; 4.6.3: 3.3 s, 2.54 m/s, 0.15°/0.34° |
| `.bin` landings (`logs::landing`, `dc-cli log landings`, in `log report`): per LAND_COMPLETE (EV 18) the mean and fastest descent over the 3 s before and time to disarm | `crates/logs/src/landing.rs` | unit test; SITL AUTO landing 0.50 m/s max (LAND_SPD_MS 0.5), disarmed 0.5 s later |
| `.bin` sanitize for sharing (`logs::sanitize`, `dc-cli log sanitize LOG --out OUT --to LAT,LON`; ADR-0012 data rule): every `L` position field and every `PARM` named `*_LAT`/`*_LNG`/`*_LON` moves by one offset so the first POS/GPS fix lands on the stand-in (shapes and distances kept); `L` fields not named lat/lon, and everything when the log has no fix, are zeroed. Altitudes and GPS time are not changed | `crates/logs/src/sanitize.rs` | unit tests; 31 MiB SITL log: 14 124 values in 46 fields moved, a field-by-field scan finds no original coordinate (int32 ×1e-7/×1e-5, float, text), `log summary` identical before/after; owner's no-GPS 4.6.2 log: nothing to move |
| `.bin` trim (`logs::trim`, `dc-cli log trim LOG --out OUT [--from S --until S] [--margin 10]`): copies the records in a window (default first arm − margin to last disarm + margin) byte-for-byte in file order, plus FMT/FMTU/UNIT/MULT/PARM/VER/MSG/FILE, records without TimeUS, and the MODE in force at the window start | `crates/logs/src/trim.rs` | unit test; owner's 4.6.2 log 12.4 → 10.0 MiB: `log report` identical except whole-log totals (battery used 5.5 → 4.8 mAh: disarmed draw outside the window; sample counts) |
| `.tlog` sanitize for sharing (`dc-cli tlog sanitize FILE --out OUT --to LAT,LON`): positions found from the dialect's generated field layout (`mavlink_codec::message_fields`: every `int32 degE7` field, `float deg` lat/lon, global-frame x/y of MISSION_ITEM(_INT)/COMMAND_INT, param5/6 of 15 position MAV_CMDs in COMMAND_LONG, `*_LAT/_LNG/_LON` parameters) move by one offset; changed frames get a fresh CRC and lose any signature (reported) | `crates/dc-cli/src/tlog_sanitize.rs`, `crates/mavlink-gen/src/codegen.rs` (layout table) | unit tests (layout tiles every payload, offsets match the serializer; every kind of position moves, MISSION-frame x/y untouched); 35 s SITL 4.7.1 recording at a synthetic home: 408 values in 10 fields moved, frame-by-frame scan finds none left |
| `.tlog` health (`dc-cli tlog health FILE [--json] [--fail-on …]`): the `log health` thresholds from telemetry while armed — VIBRATION level and clipping, SYS_STATUS battery vs BATT_LOW/CRT_VOLT, EKF_STATUS_REPORT variances vs FS_EKF_THRESH, GPS_RAW_INT HDOP and fix losses, STATUSTEXT errors (severity ≤ 3) and arming failures; parameters from PARAM_VALUE in the recording | `crates/dc-cli/src/tlog_health.rs` | unit test; SITL AUTO flight recorded with `DC_SITL_TLOG=PATH` (new opt-in hook in the scenario): all OK — vibration 1.5 m/s², 0 clips, 12.60 V, compass variance 0.19, HDOP 1.21 |
| `.tlog` link timeline (`dc-cli tlog link FILE [--window 10] [--vehicle N]`): per window the vehicle autopilot's frames received/lost (sequence numbers, late frames taken back), lowest RADIO_STATUS rssi/remrssi, longest heartbeat gap — for range tests | `crates/dc-cli/src/tlog_link.rs` | unit test (loss, radio, 4 s gap); SITL AUTO recording: 0 loss, 1.0 s heartbeats, no radio |
| `.tlog` index for playback (`recorder::TlogIndex`, `dc-cli tlog index FILE [--at S]`; plan 05 §3): 100 ms time→offset checkpoints (`offset_at`, binary search), records per message id, key events (armed/disarmed, mode, STATUSTEXT, MISSION_ITEM_REACHED), 10 s keyframes and `snapshot(t)` = latest record of every stream at or before t (stateful seek) | `crates/recorder/src/index.rs` | unit test; SITL AUTO recording (5651 records, 136 s): indexed in 16 ms, events match the flight, seeks restore 3/35/35 streams at 0.5/60/1000 s in ≤ 0.8 ms, equal to a brute-force scan |
| KML export by flight mode (`log export --kml`, `tlog export --kml`; plan 05 §3): one extruded line per mode run (same mode, same style; DrorAero brand colours from the design tokens), a time-stamped `gx:Track` for Google Earth's time slider, and mission waypoints (`.bin`: CMD records; `.tlog`: the last complete mission in the recording) | `crates/dc-cli/src/track.rs` (`from_tlog_modes`, `split_by_mode`, `kml_rich`, `segment_colours`), `main.rs` (`bin_kml_parts`) | unit test; SITL AUTO recording → STABILIZE/GUIDED/AUTO segments, 630 timed points, waypoints 0/2/3; SITL .bin → 2119 timed points |
| QGroundControl `.plan` files (`protocols::plan`; every `mission` command reads `.plan` by extension, writes it for `--out *.plan`; `dc-cli mission convert IN OUT`): SimpleItems with params[7] (null = unset), plannedHomePosition as item 0, geoFence polygons/circles ↔ fence items 5001–5004, rallyPoints ↔ 5100; QGC ComplexItems (surveys drawn in QGC) refused by name. Format from QGC's developer guide (`file_formats/plan.md`) | `crates/protocols/src/plan.rs`, `main.rs` (`read_mission`, `mission_text`) | unit tests (documented example, round trip, complex item); survey .waypoints → .plan → .waypoints: `mission diff` same |
| Mission CSV (`dc-cli mission export FILE --csv OUT`): seq, command name, frame, p1–p4, lat/lon (located global items), alt, leg and running distance (m, between located items; takeoff climb and RTL leg not counted) | `crates/dc-cli/src/main.rs` (`mission_csv`) | survey .plan → 17 rows, 1875.6 m between waypoints |
| KML route → mission (`dc-cli mission from-kml KML --alt A [--out F]`; `protocols::kml::first_line`, `kml::placemarks`, `mission::route`): the first LineString drawn in Google Earth becomes home (first point), takeoff, one waypoint per point at A m above home and RTL; validated and summarised like any mission | `crates/protocols/src/kml.rs`, `mission.rs`, `main.rs` | unit tests; drawn.kml → .plan: 3 waypoints, 345 m |
| KML polygons → geofence (`dc-cli mission fence-from-kml KML [--out F]`; `Fence::to_items`): each polygon an inclusion zone, or exclusion when named "exclusion"/"no-fly"/"no fly"/"nofly"; written as 5001/5002 items (.waypoints) or a .plan geofence; `mission check --fence` now reads a .plan's geofence | `crates/protocols/src/fence.rs`, `main.rs` (`read_fence`) | unit test (items round trip); drawn.kml: route inside; no-fly polygon over the route: breach on leg 3 |
| KML points → rally points (`dc-cli mission rally-from-kml KML --alt A [--out F]`): each Point placemark a `NAV_RALLY_POINT` 5100 (frame 3, A m above home), .waypoints or a .plan's rallyPoints | `main.rs` | drawn.kml: 2 rally points; .plan round trip via `mission convert` |
| Corridor scan (`protocols::pattern::corridor`, `dc-cli mission corridor --path|--path-kml --width W --spacing S [--overshoot] --alt A`): ⌈W/S⌉ lines parallel to the path, centred, flown back and forth; mitred corners (capped at 3× the offset), overshoot at the ends; via `mission::route` | `crates/protocols/src/pattern.rs` | unit test (L-shaped path: offsets, mitre, direction, overshoot); drawn.kml route: 3 lines, 863 m |
| SAR expanding square (`protocols::pattern::expanding_square`, `dc-cli mission search --pattern square --datum LAT,LON --spacing S --extent E [--track T] --alt A`): IAMSAR legs 1,1,2,2,3,3… × S from the datum, first leg on T, right turns, until a leg would exceed E | `crates/protocols/src/pattern.rs` | unit test (leg geometry, track 90°); 30 m / 300 m: 20 legs |
| SAR sector search (`protocols::pattern::sector`, `mission search --pattern sector --radius R [--track T]`): IAMSAR single sector pattern, three equilateral triangles through the datum, 120° right turns, corners 60° apart, back to the datum | `crates/protocols/src/pattern.rs` | unit test (leg lengths, every turn 120° right, first leg on track); R = 200 m: 1800 m path |
| SRTM terrain (`protocols::terrain`: `HgtTile`, `Terrain`, `tile_name`/`tile_origin`; `dc-cli terrain elevation --dem DIR "lat,lon;…"`; plan 01 §5.3): offline `.hgt` tiles (1″ 3601² or 3″ 1201², big-endian i16, north→south, EGM96), bilinear sampling, voids left out and re-weighted; only the tiles needed are read, missing ones named. Not yet: GeoTIFF/Copernicus, tile pyramid, TERRAIN_DATA service | `crates/protocols/src/terrain.rs`, `main.rs` (`load_terrain`) | unit tests (names, bilinear, voids, edges); synthetic hill tile: peak 880 m |
| Terrain clearance (`terrain::clearance`, `sample_points`; `dc-cli mission check FILE --dem DIR [--min-clearance 20]`; plan 01 §5.3 "verify clearance", parity FP-017): every horizontal leg sampled each 10 m, heights above home from the ground at home, legs to terrain-frame items (10/11) at their height above ground; lowest per leg, failure below the limit; takeoff/landing verticals not legs | `crates/protocols/src/terrain.rs`, `main.rs` (`terrain_check`) | unit test (rising ramp: 10 m at the far end; terrain frame keeps 40 m; no tile at home); synthetic hill: 120 m search hits the 880 m summit (6 legs below ground) |
| Mission path RTL climb (`mission::path`, `RTL_ALT_DEFAULT_M`): RTL first climbs to the default RTL altitude (15 m above home: 4.7.1 `RTL_ALT_M`, 4.6.3 `RTL_ALT` 1500 cm) when lower, as `ModeRTL::compute_return_target` does; fence, clearance, export and the map path see it. RTL_CONE_SLOPE and a non-default RTL altitude not modelled | `crates/protocols/src/mission.rs` | protocols tests; terrain-follow over the synthetic hill no longer returns below home ground |
| Terrain following by planning (`terrain::follow`, `dc-cli mission terrain-follow FILE --dem DIR --agl A [--step S]`): every NAV_WAYPOINT above home (frame 3) set to A over the ground under it (relative to the ground at home), optional extra waypoints every S m between them (refused with DO_JUMP, which would shift); prints the resulting lowest clearance. Needs no TERRAIN_ENABLE; the terrain frame (10) stays as authored | `crates/protocols/src/terrain.rs`, `main.rs` | unit test (heights, densified ramp stays 40 m up, DO_JUMP refused); synthetic hill: 216 points added, survey legs ≥ 50 m, RTL arrival 15 m |
| Terrain profile (`terrain::profile`, `dc-cli mission profile FILE --dem DIR [--step 25] [--out F.csv]`; data for the plan 04 profile view): per sample distance flown, item, position, ground, altitude MSL and above ground, heights as the clearance check reckons them | `crates/protocols/src/terrain.rs`, `main.rs` | unit test (terrain-frame leg stays 40 m up, distance, landing); synthetic hill: lowest −62 m at item 21, same as `mission check --dem` |
| Mission lint additions (`mission::lint`): DO_JUMP repeating forever (repeat −1, `AP_MISSION_JUMP_REPEAT_FOREVER`), two consecutive waypoints at the same position and altitude; the no-RTL/LAND note now says what Copter does (`ModeAuto::exit_mission`: LOITER, LAND if it cannot) | `crates/protocols/src/mission.rs` | unit test `endless_jumps_and_double_clicks` |
| Fence lint (`fence::lint`, `fence::self_intersects`; in `mission check --fence`, rally points from a .plan fence file): what ArduPilot refuses (4.7.1 `validate_fence`: vertex counts, circle radius ≤ 0, two return points, bad positions) and what it does not check: self-crossing polygons, return point outside the fence (upstream TODO), home outside/in a no-fly zone (arming "Vehicle breaching"), rally points outside | `crates/protocols/src/fence.rs`, `main.rs` | unit test; bow-tie KML + no-fly over home: both flagged |
| Mission edit model (`protocols::mission_edit::MissionEditor`, for the planner UI; not yet bridged to QML): insert/delete/move/replace/set-all-altitudes with undo/redo (snapshots, 200 deep, labels), home fixed at item 0, DO_JUMP targets renumbered on every insert/delete/move; a jump whose target is deleted is reported and points at the next item | `crates/protocols/src/mission_edit.rs` | unit tests (jump renumbering through insert/move/delete, undo/redo exact, bounded history) |
| Units and coordinate formats (`protocols::units`, parity XC-002; `dc-cli coord LAT,LON`): altitude/distance/speed in metric, imperial, nautical (exact ft/mi/NM); positions as decimal, DMS, DDM, UTM (WGS-84 Krüger 4th order, Norway/Svalbard zones) and MGRS (AA lettering), 80°S–84°N (no UPS) | `crates/protocols/src/units.rs` | unit tests: 10 points against PROJ 9 (pyproj) and the `mgrs` library within 2 mm, DMS carry, unit strings |
| Annunciator state (`dc_core::annunciator::Annunciator`; plan 04 annunciators, plan 08 H9): warning > caution > advisory > status then oldest first; warnings/cautions latch until acknowledged; raise de-duplicates (count, latest text); a return within `repeat_quiet` (10 s) is shown but not re-sounded, escalation always sounds; master light = highest unacknowledged caution/warning. Display only (ADR-0009); not yet fed from vehicle state or bridged to QML | `crates/dc-core/src/annunciator.rs` | unit tests (priority order, latching/ack, dedupe, flapping rate limit, escalation) |
| KML export of fence and rally (`mission export FILE.plan --kml`, `track::kml_add_shapes`): a .plan's inclusion/exclusion polygons ("Inclusion N", "No-fly N"), circles as 36-sided polygons, rally points as placemarks next to the path, readable back by `fence-from-kml`/`rally-from-kml`; KML names now decode XML entities and CDATA | `crates/dc-cli/src/track.rs`, `main.rs` (`plan_shapes`), `crates/protocols/src/kml.rs` | unit test (shapes read back, escaped name); .plan → KML → fence and rally: same zones and points |
| dc-cli reference page (`docs/reference/cli/dc-cli.md`, in the site under Reference): common tasks plus every command's `--help`, generated by `scripts/gen_cli_docs.py`; `cargo xtask ci` (repo-check) builds dc-cli and fails on a stale page — regenerate after changing a command | `scripts/gen_cli_docs.py`, `mkdocs.yml` | `mkdocs build --strict` passes; `--check` passes |
| `.bin` IMU health (`logs::imu`, `dc-cli log imu`, in `log report`): per IMU temperature range, gyro/accel error counts during the log, unhealthy samples while armed, sample rates | `crates/logs/src/imu.rs` | unit test; owner's 4.6.2 log |
| `.bin` EKF lane switches (`logs::lanes`, `dc-cli log lanes`, in `log report`): changes of XKF4.PI (core 0 records), flagged when armed | `crates/logs/src/lanes.rs` | unit test; owner's 4.6.2 log (none) |
| `.bin` cell balance (`logs::cells`, `dc-cli log cells`, in `log report`): per pack while flying from BCL (mV + 1, 0 = no cell): cells, mean and largest spread, weakest cell | `crates/logs/src/cells.rs` | unit test; no real or SITL log with cell voltages yet |
| `.bin` autopilot performance (`logs::perf`, `dc-cli log perf`, in `log report`): loop rate, long loops (> 1.2 x period, PerfInfo) as % of loops, longest loop, highest load (PM.Load x 0.1), least free memory, internal errors (ErC/InE) | `crates/logs/src/perf.rs` | unit test; owner's 4.6.2 log (801 Hz, 0 % long loops after boot, load 50.8 %) |
| `.bin` ESC telemetry (`logs::esc`, `dc-cli log esc`, in `log report`): per ESC while armed: RPM mean/max, telemetry error rate mean/max, temperature and current where reported | `crates/logs/src/esc.rs` | unit test; owner's 4.6.2 log (bidirectional DShot: error rates 7-12 % mean, peaks 97.5 %; ESC 2 about 45 % faster than the others) |
| `.bin` altitude sources (`logs::altitude`, `dc-cli log altitude`): barometer vs EKF height (CTUN.BAlt - Alt) and GPS vs EKF altitude (GPS.Alt - POS.Alt) while flying: mean, largest, drift | `crates/logs/src/altitude.rs` | unit test; SITL AUTO (baro within 0.12 m, GPS 0.79 m); owner's 4.6.2 log (baro drifted 1.35 m on the ground with motors running) |
| `.bin` GPS quality (`logs::gps`, `dc-cli log gps`): satellites, HDOP, share of each fix type (AP_GPS::GPS_Status), GPS glitches (ERR 11/2, resolved by 11/0) while armed; GPS vs EKF horizontal position while flying (mean, largest) | `crates/logs/src/gps.rs` | unit test; SITL AUTO (RTK fixed 100 %, 10 sats, no glitches) |
| `.bin` flow and rangefinder (`logs::sensors`, `dc-cli log sensors`): OF quality mean/min and the share EKF3 can use (quality > 0, rates < 4.2 rad/s); per rangefinder the share of each status and the good-reading range | `crates/logs/src/sensors.rs` | unit test; SITL AUTO flight now with SITL flow and rangefinder (99.9 % usable, 97 % good up to 23 m); owner's 4.6.2 log (flow 100 % usable; rangefinder 0.02 m throughout: it never left the ground) |
| `.bin` RC link (`logs::rc`, `dc-cli log rc`): receiver RSSI and link quality while armed (mean, lowest; -1 = not reported), radio failsafes (ERR 5: start/resolved) and late frames (ERR 2/2) | `crates/logs/src/rc.rs` | unit tests; owner's 4.6.2 log (CRSF: RSSI 98 %, LQ 100 %); no SITL check (radio failsafe would need a mid-flight parameter change) |
| `.bin` harmonic notch advice (`logs::notch`, `dc-cli log notch [--field --imu --segment --min-hz]`): FFT peak during the longest steady flight (flying, |CRt| < 0.6 m/s) and the mean throttle there → INS_HNTCH_MODE 1, FREQ, BW = FREQ/2, REF; printed only, never written | `crates/logs/src/notch.rs` | unit test; SITL AUTO flight: 37.1 Hz (simulated 37) at throttle 0.38 |
| `dc-cli params --connect L --compare FILE` (read-only): the vehicle's parameters against a .param/.bin, exit 1 on a difference; values the autopilot sets itself (`protocols::params::set_by_autopilot`: STAT_*, MOT_THST_HOVER, BAROn_GND_PRESS, boot gyro offsets/caltemp, sensor IDs) are marked and do not count, also in `params-diff` | `crates/dc-cli/src/main.rs`, `crates/protocols/src/params.rs` | unit test; SITL: fresh file passes (13 autopilot-set), WP_SPD change fails |
| Configuration review (`protocols::params::review`, `dc-cli params-review FILE` for .param or .bin, and in `log report`): names switched-off safety nets (pre-arm checks by name from ARMING_SKIPCHK 4.7 / ARMING_CHECK 4.6, RC failsafe, battery failsafe/threshold/monitor, GCS failsafe, fence, notch); advisory, blocks nothing | `crates/protocols/src/params/review.rs` | unit test; owner's 4.6 param file (14 pre-arm checks not run, no low-battery action) |
| Cursor-on-Target (`protocols::cot`, `dc-cli cot --connect L [--udp ADDR] [--rate] [--seconds]`): CoT 2.0 position events per vehicle (a-f-A-M-H-Q rotary / a-f-A-M-F-Q fixed wing, MSL altitude noted, track course/speed, 10 s stale), printed or one UDP datagram each | `crates/protocols/src/cot.rs` | unit tests (ISO 8601 incl. leap day, escaping); SITL: events printed and received over UDP |
| `dc-cli log report FILE`: every `.bin` analysis in one run, each section saying why it does not apply when the log lacks the data (handlers are `print_log_*` functions shared with the single commands) | `crates/dc-cli/src/main.rs` | run on the owner's 4.6.2 log; each command smoke-tested |
| `dc-cli log compare A B`: two `.bin` flights side by side (armed time, altitude, distances, speeds, climb/descent, battery, consumption, hover, worst motor, tracking errors, vibration) | `crates/dc-cli/src/main.rs` (`compare_rows`) | composed of the tested analyses; run on the owner's log vs a SITL log |
| IMU vibration spectrum (`logs::spectrum`: radix-2 FFT, Welch, Hann, 50 % overlap; `dc-cli log fft --field ACC.AccZ --imu 0`), armed samples only. Raw ACC/GYR need LOG_BITMASK IMU_RAW (1<<19) | `crates/logs/src/spectrum.rs` | unit test (80/210 Hz); SITL AUTO flight with SIM_VIB_FREQ_X 37 + SIM_ACC1_RND 2: peak 37.1 Hz |
| `dc-cli tlog info`: time span, sources, message counts, highest altitude, per-vehicle flight stats (times armed, armed time, armed time per mode, distance while armed, top ground speed), frames lost in transit per source from MAVLink sequence numbers (`dc_core::SeqCount`, shared with the live counter); link drops: autopilot heartbeat gaps over the core's STALE (1.5 s) and LOST (5 s) limits with times; telemetry radios from RADIO_STATUS (signal local/remote mean and lowest, smallest margin over noise, receive errors and corrections during the recording with counter resets handled, lowest free tx buffer; radio units, 255 = unknown); `dc-cli tlog csv FILE MSG [--vehicle] [--out]`: one message type as CSV (receive time, sender, every field via `Message::fields`), timeline of firmware, modes, arming and STATUSTEXT (same `Fleet` folding as the live core) | `crates/dc-cli/src/tlog.rs` | unit tests; SITL boot recording |
| Mission files (`QGC WPL 110`, Mission Planner `.waypoints`); `dc-cli mission download / check / compare` (read-only: no upload from the CLI) | `protocols::wpl`, `dc-cli` | unit tests; SITL: file round trip verifies against the vehicle |
| Mac test runbook | [`../dev/mac-test-runbook.md`](../dev/mac-test-runbook.md) | — |

## 6. Waiting on humans

| Item | Where | Needed from |
|------|-------|-------------|
| Mac session, remaining: T0-05 hardware matrix, T0-10 "Open Anyway" step, T0-09 re-run once a no-hold decode path exists (M1 results of 2026-10-11 are in the spike RESULTS.md files) | Mac runbook | Owner (hardware) |
| Windows bench test with a 4.6.3 flight controller | `docs/dev/windows-bench-test.md` | Owner (hardware) |
| Force-arm and parameter reset: decided 2026-10-15 (ADR-0013 item 6: opt-in settings, off by default); implementation pending | ADR-0013 | — (agent) |

## 7. Next tasks, in priority order

From the owner's decisions of 2026-10-12 (ADR-0012). Each lands with tests, SITL evidence where a
vehicle is involved, and the audit log:

1. **CI — done 2026-10-12:** pushes/PRs run Linux only (checks, tests, docs, SITL); Windows,
   macOS builds and packaging run nightly (02:17 UTC), on releases and on dispatch. The `sitl`
   job runs all scenarios (17 then, 21 now) on ArduCopter 4.7.1 **and 4.6.3** (both green locally);
   `TESTED_FIRMWARE` covers Copter 4.6 and 4.7.
**Item 2 — done 2026-10-13** (the owner confirmed in-session, adding: also while armed and
flying, behind a warning pop-up): no dangerous-parameter refusal; `ArmedWarningRequired` /
`ArmedWrite::Acknowledged` in the core; SITL in-flight write verified on 4.7.1 and 4.6.3.
CLI done: `dc-cli param-set` (one confirmation, the in-flight warning as a second prompt,
`--yes`/`--armed-ok` for scripts); every write is in the audit log. gRPC done: `SetParams`
(warning back until acknowledged; served only with `dc-cli daemon --param-dir DIR`; audited)
and `dc-cli remote set-params`. Still to do for item 2: the Qt pop-up (Phase 1).

**Items 3–5 — done 2026-10-13 (local commit)** after the owner's go-ahead in the session, with
their condition: each behind an opt-in setting, **off by default**, with a warning about its
danger (ADR-0012 amendment; `dc_core::VehicleChange`, `VehicleChangeSettings`,
`Core::upload_items`, `Core::erase_logs`). CLI: `dc-cli settings [show|enable|disable]`
(settings file in the platform config directory, or `DC_SETTINGS`), `dc-cli mission upload
FILE [--type mission|fence|rally]`, `dc-cli log erase`. SITL scenario
`in_flight_uploads_and_log_erase_need_their_settings` green on 4.7.1 and 4.6.3. Still to do:
the settings screen and in-flight warning pop-ups in the Qt app (Phase 1); gRPC calls for
upload and erase (the API has none yet).

2. **Parameter writes for all parameters** (T0-18 follow-up): drop the dangerous-parameter
   refusal; a single confirmation, no typed text. Keep the disarmed-heartbeat freshness and
   read-back verification unless the owner says otherwise. Expose through gRPC and the CLI.
3. **Mission upload:** in flight with one confirmation (T0-19); from the CLI without a state lock.
4. **Fence and rally upload** (ground and in flight) with read-back verification.
5. **Log erase** (`LOG_ERASE`) as a command, audited.
6. **Camera trigger commands — done 2026-10-12:** `mission survey --camera … --trigger` adds DO_SET_CAM_TRIGG_DIST (206) at the first survey waypoint and 0 m before RTL; SITL upload verified on 4.7.1 and 4.6.3.
7. **gRPC — done 2026-10-12:** `dc-cli daemon --listen ADDR [--token-file F]` (or `DC_API_TOKEN`); a key is required beyond loopback; every call needs `authorization: Bearer <key>` (constant-time check); `dc-cli remote --token-file`/`DC_API_TOKEN`. A shared secret, not encryption: trusted networks or a tunnel.
8. **FFT crate and Inter — done 2026-10-12:** `logs::spectrum` uses `rustfft` 6.4.1; the UI
   font token is Inter (`ui/tokens/tokens.json`, skill and mirrors synced).
9. Phase 1 (Qt shell) once the above are in.
10. **Hands-on tests (prepared 2026-10-13):** [`docs/dev/test-kit.md`](../dev/test-kit.md) —
    which full-CI build to download, the Windows bench test (now with parameter write and
    rollback, the optional in-flight warning, the new analyses and sanitizing before
    sharing) and the Mac session (Phase 0 tasks plus the same flight-controller checks).
    Every runbook command was walked against SITL on 2026-10-13.

11. **Mission planning batch (2026-10-13, local commits, not yet pushed):** QGC `.plan`
    files, mission CSV, routes/fences/rally points from KML and back, corridor scan, SAR
    expanding square and sector search, SRTM terrain (elevation, clearance check, profile,
    terrain-following by planning), RTL climb in the mission path, mission and fence lint
    additions, the mission edit model (undo/redo, DO_JUMP renumbering), units and
    coordinate formats (UTM/MGRS checked against PROJ), and the annunciator state model.
    Rows in §5. Next for these: bridge `MissionEditor`, `units` and `Annunciator` to QML in
    Phase 1.

12. **Batch 9 (2026-10-14, local commits, not yet pushed):** the bench findings of
    2026-10-13 fixed (erase result, fft fallback, landings, tlog health, parameter text,
    notch caveats, takeoff note, the 4.6.3 build labelled beta); then: FENCE_STATUS and
    breach alerts; the annunciator fed by every core alert (per vehicle, sound events,
    acknowledge; `tlog alerts`); ADS-B closest approach and traffic alerts (Copter AVD_*
    defaults); where RTL goes (`mission rtl-target`, Copter's rally rules, SITL-checked);
    pre-flight checklist (`dc-cli preflight`); `tlog signing`; fence margin in telemetry;
    elevation GeoTIFFs (Copernicus GLO-30) next to `.hgt`; the TERRAIN_DATA service
    (`--terrain DIR`, off by default, SITL-checked); the MAVLink inspector (`inspect`,
    `tlog inspect`); `mission edit`; `--units`; gRPC UploadItems/EraseLogs/
    GetVehicleChangeSettings behind the daemon's settings. SITL now 21 scenarios. Next:
    Phase 1 bridges (annunciator, preflight, settings screen, warning pop-ups); MAVLink 2
    signing on links needs an owner decision (SETUP_SIGNING writes the vehicle's key).

## 8. Where things are (code map)

| Area | Path | Notes |
|------|------|-------|
| MAVLink codec (generated from vendored XML) | `crates/mavlink-codec`, `crates/mavlink-gen` | Types under `mavlink_codec::{messages, enums}` |
| Routing and links | `crates/mav-router`, `crates/link-{core,net,serial}` | TCP client and serial reconnect; frames queued during an outage are discarded |
| Vehicle state | `crates/vehicle` | `Fleet::apply`; `modes`, `nav` modules |
| Protocols (sans-I/O) | `crates/protocols` | `command` (CommandPolicy, audit), `params`, `mission`, `audit` |
| Core | `crates/dc-core` | `Core`, `CoreEvent`, link watchdog, command / mission / parameter drivers |
| API / CLI / FFI | `crates/api` (`proto/dc/v1/core.proto`), `crates/dc-cli`, `crates/core-ffi` | |
| SITL tests | `crates/sitl-harness` (`Sitl`, `ChaosProxy`, `tests/scenarios.rs`) | Skipped without a SITL binary unless `DC_SITL_REQUIRED=1` |
| Spikes | `ui/spikes/s0-1-bridge`, `s0-3-map`, `s0-5-video` | Each has its own workspace and `RESULTS.md` |
| Packaging | `packaging/macos/` | ADR-0010 |
| Safety | `docs/safety/hazards.md`, ADR-0009 / 0011 | |

## 9. Before you finish a session

Follow protocol §9:
1. Run `cargo xtask ci`, plus the SITL suite if you touched core or protocols.
2. `coord.py log <you> done|progress …`
3. `coord.py release <task>`
4. `coord.py status <you> idle|blocked`
5. `coord.py board`
6. Commit, then push only if the owner's current instruction allows it.
7. Update this file if the state above changed.
