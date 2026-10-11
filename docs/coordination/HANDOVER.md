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
| Local ArduPilot SITL: `scripts/build_sitl.sh` → `.sitl/ardupilot-Copter-4.7.1/` (gitignored, about 5 min) | `DC_SITL_REQUIRED=1 cargo test -p sitl-harness --test scenarios` runs all scenarios (8, about 65 s). The pinned ArduPilot source in `.sitl/` is the reference for ArduPilot behaviour: read it instead of guessing. |
| macOS pseudo-terminals cannot be opened by path through `serialport` (it sets the baud rate with `IOSSIOSPEED`) | Serial tests use `TTYPort::pair()` and the injectable `Opener` (`link-serial`). |
| `coord.py` | Quote glob paths (`'crates/x/**'`). Log types are only `progress handoff question decision-request blocker conflict fyi done`. |

## 4. Phase 0 status (backlog T0-01…T0-16)

| Task | State | Evidence / what is left |
|------|-------|-------------------------|
| T0-01 repo, CI on 4 targets | done | `ci.yml`; `cargo xtask ci` |
| T0-02 MAVLink codec | done | `crates/mavlink-codec`; licence question open (§6) |
| T0-03 router, links, tlog | done | `mav-router`, `link-*`, `recorder` |
| T0-04 core daemon, gRPC, FFI, CLI | done | `dc-core`, `api`, `core-ffi`, `dc-cli` |
| T0-05 macOS serial | **code done; hardware matrix pending** | Mac runbook T0-05; `crates/platform-macos/HARDWARE_MATRIX.md` |
| T0-06 spike S0-1 bridge | **Linux done; Metal run pending** | `ui/spikes/s0-1-bridge/RESULTS.md` (p99 17.7 ms without a GPU) |
| T0-07 spike S0-3 map | **harness ready; CI start-up fix unverified; Mac run pending** | `ui/spikes/s0-3-map/`; re-run workflow `spike-map` |
| T0-08 spike S0-4 logs | done | `crates/logs` |
| T0-09 spike S0-5 video | **harness ready; M1 run pending** | `ui/spikes/s0-5-video/RESULTS.md`; separate `spike-video` workflow gives indicative numbers when the spike or its workflow changes, or on manual dispatch |
| T0-10 macOS package | done on CI | First launch on a team Mac is a manual check (runbook) |
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
| Mac test runbook | [`../dev/mac-test-runbook.md`](../dev/mac-test-runbook.md) | — |

## 6. Waiting on humans

| Item | Where | Needed from |
|------|-------|-------------|
| Mac session: T0-05, T0-06, T0-07, T0-09, T0-10 first launch | Mac runbook | Owner (hardware) |
| Per-hazard rating review; interpretations I1–I3; acceptance of the H14 residual risk | `docs/safety/hazards.md` §3, §7 | Flight Operations & Systems Safety Lead |
| Licence of the vendored MAVLink XML and the generated code | `issues/20261010T213905Z-…` | Owner / legal |
| MapLibre Native third-party licence inventory before shipping | `ui/spikes/s0-3-map/RESULTS.md` | Owner / legal |
| GitHub Actions minutes budget | — | Owner |

## 7. Next tasks, in priority order

The backlog rows were added in [`backlog.md`](backlog.md), T0-17 onwards.

1. **T0-17 — re-run `spike-map`** on the macOS runner. This verifies 0d5834a (QML import path)
   and records the indicative Metal frame timing in `ui/spikes/s0-3-map/RESULTS.md`. If it
   still fails, read the log and fix it.
2. **T0-18 — H5 safe parameter changes:**
   * `ParamWrite` with read-back, exposed through `dc-core`;
   * a snapshot before any write, so the change can be rolled back;
   * failsafe and fence parameters marked dangerous;
   * a decision on whether to refuse while armed (safety path: ask the owner);
   * use the vehicle's own `FS_EKF_THRESH` in `NavThresholds`.
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
