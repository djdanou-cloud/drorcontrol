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
| T0-02 MAVLink codec | done | `crates/mavlink-codec`; licence question open (§6) |
| T0-03 router, links, tlog | done | `mav-router`, `link-*`, `recorder` |
| T0-04 core daemon, gRPC, FFI, CLI | done | `dc-core`, `api`, `core-ffi`, `dc-cli` |
| T0-05 macOS serial | **code done; hardware matrix pending** | Mac runbook T0-05; `crates/platform-macos/HARDWARE_MATRIX.md` |
| T0-06 spike S0-1 bridge | **M1 run done; strict p99 bound missed** | M1: 0 dropped frames, p99 19.5 ms at 200 k (17.8 ms idle). Criterion needs restating in ADR-0001; Intel not run (120 Hz deferred: `docs/plan/future-upgrades.md`) |
| T0-07 spike S0-3 map | **M1 run done at 2880×1800: Metal, fps, vsync, feel pass; GPU ≈ 42 % fails** | `ui/spikes/s0-3-map/RESULTS.md`; ADR-0003 go/no-go pending (decided on the built-in Retina run; 5K deferred to `docs/plan/future-upgrades.md`, owner 2026-10-12) |
| T0-08 spike S0-4 logs | done | `crates/logs` |
| T0-09 spike S0-5 video | **M1 runs done incl. live RTSP: hardware + zero-copy pass; live p99 113 ms and CPU 6.4 % fail. Latency = frames held per `max_dec_frame_buffering` (36 ms with 1)** | `ui/spikes/s0-5-video/RESULTS.md`; separate `spike-video` workflow gives indicative numbers when the spike or its workflow changes, or on manual dispatch |
| T0-10 macOS package | done on CI; local M1 package + first-launch steps 1–3 pass | "Open Anyway" not verified on the M1 (approved in another account than the app's owner); see log |
| T0-11 parity freeze | done | `parity.yaml`, `docs/parity/` |
| T0-12 design tokens | done | `ui/tokens/` (owner decisions applied) |
| T0-13 docs site | done | `mkdocs build --strict` in CI |
| T0-14 SITL harness | done | `crates/sitl-harness`, `docs/dev/sitl-harness.md` |
| T0-15 hazard log | **v0 baselined; human review pending** | `docs/safety/hazards.md` §6–7 |
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
| gRPC `Vehicle` carries `mode_name` (banner-confirmed only), `mission` progress, `landed` state, `nav` health (H16 trust + reasons), `link` state, `home` (HOME_POSITION, requested at 0.2 Hz) ADS-B `traffic` (ADSB_VEHICLE requested at 5 Hz; valid-flag fields only; forgotten after 60 s) `batteries` (BATTERY_STATUS at 2 Hz; pack voltage = sum of valid entries, cells only when measured), `vibration` (VIBRATION, 2 Hz), `wind` (WIND, 1 Hz), `rc` (RC_CHANNELS, 1 Hz), `servos` (SERVO_OUTPUT_RAW by port, 1 Hz) `radio` (RADIO_STATUS from a telemetry radio, under the system id it reports with) `rangefinders` (DISTANCE_SENSOR, 2 Hz, with an in-range flag) `terrain` (TERRAIN_REPORT, 1 Hz; flags extrapolated heights and the boot-time 0,0 report) and `autopilot_version` (AUTOPILOT_VERSION, requested once on connect like the banner); `streams`: frames received / lost per component from MAVLink sequence numbers (total and last 10 s); `dc-cli telemetry` shows them | `proto/dc/v1/core.proto`, `api::vehicle_to_pb` | unit test; checked against SITL through `dc-cli daemon` + `remote vehicles` |
| Read-only download of the geofence and rally points (`Core::download_items`, `MissionType`; `dc-cli mission download --type fence|rally`). No fence/rally upload: a geofence is a failsafe boundary, so an upload path needs owner / safety-lead approval | `protocols::mission`, `dc-core`, `dc-cli` | SITL: fence and rally read back as set |
| gRPC `DownloadItems` (mission / fence / rally, read-only); `dc-cli remote items --type …` | `proto/dc/v1/core.proto`, `api` | gRPC integration test (fake vehicle) |
| `.tlog` replay link `tlog:PATH[@SPEED]` (1–100×): original timing, receive-only, allowed only as the core's sole link (no relaying of recorded traffic to a vehicle) | `dc-core/src/replay.rs`, `Core::connect` | `dc-core/tests/replay.rs`; daemon + `remote vehicles` on a SITL recording |
| Dataflash log health checks (`logs::health`, `dc-cli log health`): vibration/clipping (ArduPilot wiki guidance, commit f4b21ca), battery vs the vehicle's BATT_LOW_VOLT/BATT_CRT_VOLT, EKF3 variances vs its FS_EKF_THRESH, ERR records with subsystem names, GPS (HDOP vs the vehicle's GPS_HDOP_GOOD, 3D-fix losses) and board voltage (BRD_VBUS_MIN to 5.8 V, the arming range; NaN = not measured) | `crates/logs/src/health.rs` | unit tests; both fixture logs |
| `dc-cli tlog export FILE --gpx/--kml [--vehicle N]`: flight track from GLOBAL_POSITION_INT (GPX 1.1, KML 2.2 absolute altitude). `dc-cli log export FILE.bin` does the same from the EKF position (POS) with UTC from the GPS clock (`logs::track`) | `crates/dc-cli/src/track.rs` | unit tests; 70 s SITL recording, both files parse |
| STATUSTEXT history: last 500 lines per vehicle in the core (outside the per-message fleet clone), gRPC `GetMessages`, `dc-cli remote messages` | `Core::messages`, `api` | `dc-core/tests/messages.rs` (cap via fast replay), gRPC test |
| Mission statistics (`protocols::mission::stats`): navigation items, path length (great circle; 0,0 = current position as in `ModeAuto::get_loc_from_cmd`; RTL back home; DO_JUMP not expanded), highest altitude per frame, last nav command; printed by `dc-cli mission check/download` | `crates/protocols/src/mission.rs` | unit test |
| Dataflash log download over MAVLink (`protocols::log_transfer`, `Core::list_logs/download_log`, `dc-cli log list/download`): refused unless disarmed (ArduPilot ignores log requests while armed); gaps re-requested as one span after the stream goes quiet; written to `.partial` then renamed. LOG_ERASE deliberately not implemented | `crates/protocols/src/log_transfer.rs` | unit tests; SITL: byte-identical to SITL's file through 10 % loss (46 kB/s) |
| Camera geotags (CAMERA_FEEDBACK): up to 20 000 per vehicle in the core, repeated image reports replace the earlier one; gRPC `GetGeotags`, `dc-cli remote geotags` (CSV). No camera trigger command (actuation goes through CommandPolicy) | `Core::geotags`, `api` | `dc-core/tests/geotags.rs` |
| Mission events: `CoreEvent::Mission` / gRPC `MissionEvent` (item reached, current item, mission state) | `dc-core`, `api` | SITL AUTO flight: reached 2 and 3, current 4, Active, Complete |
| gRPC `ListLogs` / `DownloadLog` (server stream: progress, then 1 MiB chunks, last has `done`); `dc-cli remote logs [--download ID --out FILE]` | `api` | gRPC test with a fake vehicle log; SITL via daemon (360 kB) |
| `dc-cli tlog params` (PARAM_VALUE, last value per name -> .param) and `tlog mission` (last complete MISSION_COUNT + items from one sender, `_INT` or not; empty counts -> .waypoints) | `crates/dc-cli/src/tlog.rs` | unit test; SITL: 1 370 params from a recording identical to a direct download |
| `.bin` export: `dc-cli log csv FILE MSG` (all fields, text quoted; `logs::export::csv`) and `log params` (PARM -> .param) | `crates/logs/src/export.rs` | ATT row matches pymavlink; exported .param vs the log: 0 differences in 1 548 |
| Mission lint (`protocols::mission::lint`, printed by `dc-cli mission check/download`): DO_JUMP to a missing item, takeoff/waypoints at or below 0 m above home, invalid positions (warnings); no TAKEOFF first per `AP_Mission::is_takeoff_next` (Copter refuses AUTO landed+armed: "Auto: Missing Takeoff Cmd"), no RTL/LAND at the end, mixed altitude references, terrain frames (notes) | `crates/protocols/src/mission.rs` | unit tests |
| Survey grid (`protocols::survey`, `dc-cli mission survey --polygon ... --spacing --angle --alt`): lawnmower lines over a polygon (local plane, concave = one segment per crossing), takeoff + grid + RTL | `crates/protocols/src/survey.rs` | unit tests; SITL: generated survey uploads and verifies |
| `.bin` flight views (`logs::flight`, `dc-cli log summary|events|changes`): armed periods (ARM, else EV 10/11; a log that starts armed counts from its start), armed time, max rel alt, top speed, distance while armed, mAh used; timeline of MODE/ARM/EV/ERR/MSG (modes named from the banner); in-flight parameter changes with autopilot-made ones marked (STAT_* counters, learned MOT_THST_HOVER) | `crates/logs/src/flight.rs` | SITL AUTO flight log: 1 flight, 86.7 s, 20.0 m, 189 m, 650 mAh; only automatic changes |
| `dc-cli tlog info`: time span, sources, message counts, highest altitude, timeline of firmware, modes, arming and STATUSTEXT (same `Fleet` folding as the live core) | `crates/dc-cli/src/tlog.rs` | unit tests; SITL boot recording |
| Mission files (`QGC WPL 110`, Mission Planner `.waypoints`); `dc-cli mission download / check / compare` (read-only: no upload from the CLI) | `protocols::wpl`, `dc-cli` | unit tests; SITL: file round trip verifies against the vehicle |
| Mac test runbook | [`../dev/mac-test-runbook.md`](../dev/mac-test-runbook.md) | — |

## 6. Waiting on humans

| Item | Where | Needed from |
|------|-------|-------------|
| Mac session, remaining: T0-05 hardware matrix, T0-10 "Open Anyway" step, T0-09 re-run once a no-hold decode path exists (M1 results of 2026-10-11 are in the spike RESULTS.md files) | Mac runbook | Owner (hardware) |
| Per-hazard rating review; interpretations I1–I3; acceptance of the H14 residual risk | `docs/safety/hazards.md` §3, §7 | Flight Operations & Systems Safety Lead |
| Licence of the vendored MAVLink XML and the generated code | `issues/20261010T213905Z-…` | Owner / legal |
| MapLibre Native third-party licence inventory before shipping | `ui/spikes/s0-3-map/RESULTS.md` | Owner / legal |
| GitHub Actions minutes budget | — | Owner |

## 7. Next tasks, in priority order

The backlog rows were added in [`backlog.md`](backlog.md), T0-17 onwards.

1. **T0-17 — done 2026-10-12:** `spike-map` green on 19bb613 (run 38214160233): Metal on the
   macOS VM, warm 59.24 fps, p99 27.9 ms; recorded in `ui/spikes/s0-3-map/RESULTS.md`.
2. **T0-18 — core implementation local (core-2):**
   * The owner approved requiring a recent, explicitly disarmed autopilot heartbeat.
     The core uses a fixed 1.5 s freshness limit before starting writes, and stops further
     writes on arming or heartbeat gaps/staleness; a later disarmed report never revives
     an interrupted write transaction automatically. Read-only preparation can see gaps.
   * `Core::write_params` validates a complete baseline and batch, persists a new snapshot
     before any set, then verifies values/types by an explicit read request with a bounded
     10 s lifetime per parameter. Mismatches/timeouts stop the remaining batch.
   * `Core::rollback_params` restores only selected parameters to their original values,
     with a new recovery baseline and the same state, risk and read-back checks. It refuses
     parameters that disappeared or changed type; it never starts automatic rollback.
   * Failsafe/fence/arming/board-safety parameters are flagged dangerous and refused until
     dedicated typed confirmation exists. These conservative labels are not a full metadata
     catalog. Dangerous writes remain a follow-up, not implicitly authorized by disarming.
   * Each vehicle's last valid autopilot `FS_EKF_THRESH` updates `NavThresholds.ekf_fail`.
   * Ten offline tests, ten scripted-vehicle integration tests and the navigation-threshold
     source/value test cover the implementation. A real SITL `WP_SPD` write and restoration
     pass through 30% bidirectional loss (2,952 frames dropped in the initial run). The SITL
     test requests 5 Hz heartbeats; the production freshness rule is not weakened.
   * See [`../dev/parameter-changes.md`](../dev/parameter-changes.md). Changes are local only;
     the owner's instruction remains **do not push**. Safety review is required before merge.
3. **T0-19 — class C confirmation for an in-flight mission upload** (plan 08 §4.1, ADR-0011
   I3). It is refused today. This changes the command pipeline, so it needs owner/safety-lead
   approval first.
4. **T0-20 — implemented locally (devops-2):** the video-spike job moved from `ci.yml` to
   `.github/workflows/spike-video.yml`. Pushes to `main` and pull requests run it only when
   `ui/spikes/s0-5-video/**` or the new workflow changes; manual dispatch remains available.
   Actionlint 1.7.12 and 12 local trigger scenarios passed; the job steps and other CI jobs
   are unchanged. GitHub execution remains unverified until the owner authorizes a push.
5. Phase 1, once the Mac results are in: plan 07 roadmap; the Qt shell can start after the
   T0-06 / T0-07 verdicts (ADR-0001 / ADR-0003).

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
