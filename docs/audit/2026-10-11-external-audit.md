# External audit check, 2026-10-11

An externally supplied "Repository Audit & Remediation Plan" listed five tasks. Its target
files (`src/mavlink/connection.py`, `src/core/dispatcher.py`, `src/main.py`, …) and checks
(`pytest tests/…`) do not exist: DrorControl's core is Rust (`crates/`), and the only Python is
build tooling under `scripts/`. The audit does not describe this repository. Each concern was
still checked against the real code at `9de43e7`.

| # | Audit claim | Verdict | Action |
|---|-------------|---------|--------|
| 1 | Reader threads mutate shared state without synchronisation; races, segfaults | **Not applicable as stated; one related defect** | Fixed: link notifications could be dropped |
| 2 | No heartbeat watchdog; GCS stays "connected" forever; no reconnection | **Partly real** | Fixed: link-state watchdog. Open: serial reconnect |
| 3 | Commands (arm, mode, param upload) unvalidated | **Not applicable today** (no such path exists) | Proposal for human review (safety path) |
| 4 | Telemetry history in unbounded lists | **Not applicable; one related defect** | Fixed: recorder failure/backlog now reported |
| 5 | No clean shutdown; dangling sockets/threads | **Partly real** | Fixed: SIGTERM handling, flush verified |

## 1. Cross-thread state and dispatch

**Evidence.** The affected crates contain no `unsafe` (`unsafe_code = "deny"` workspace-wide;
the exceptions are FFI, mmap and IOKit), so data races are rejected at compile time. The router
state is owned by one task (`mav-router/src/hub.rs`) and fed through channels. Vehicle state is
an immutable snapshot swapped atomically (`arc-swap`, `dc-core/src/lib.rs`), so readers never
lock. A segfault from a race is not possible in this code.

**Defect found.** Link notifications (`LinkAdded`, `LinkRemoved`, `LinkStatus`) shared the
4096-frame application queue and were sent with `try_send`. During a telemetry burst that
filled the queue, a **"link down" was silently dropped**.

**Fix.** Link notifications have their own queue (`HubOutputs::links`, 1024 entries, with a
drop counter `links_dropped`). `dc-core` reads them with priority (`select! { biased; … }`).
Test: `mav-router/tests/hub_end_to_end.rs::link_down_survives_a_full_frame_queue`.

## 2. Heartbeat watchdog and reconnection

**Evidence.** The core recorded each heartbeat's time, and consumers saw its age (`age_ms` in
gRPC, `heartbeat_age_ms` over FFI). Nothing classified a vehicle as stale or lost, and no event
fired when heartbeats stopped. TCP client links already reconnect with exponential back-off
(250 ms → 5 s) and drop frames queued during the outage (`link-net`, tested). UDP is
connectionless. Serial links are removed on an I/O error, for example a USB unplug, and are
**not reopened**.

**Fix.** A watchdog in `dc-core` classifies every vehicle as `CONNECTED`, `STALE` or `LOST`
from heartbeat age and raises `CoreEvent::VehicleLink` on every transition, including recovery.
It is also available as `Core::vehicle_link()` and as a gRPC `VehicleLinkEvent`. The thresholds
follow **plan 08 §4.3 (1.5 s stale, 5 s lost)**, not the audit's 3.0 s, because the plan is
authoritative. They are configurable within bounds (stale 0.2–5 s; lost > stale and ≤ 30 s),
and `Core::start` refuses a configuration outside those bounds. Tests:
`dc-core/tests/liveness.rs` (silent radio → STALE → LOST → recovery; bounds).

**Open.**
* Serial reconnect after unplug or re-enumeration. Owner WS-MACOS: it needs the T0-05 hotplug
  work and hardware validation.
* The other plan 08 §4.3 parts: `DEGRADED` (loss/latency/RSSI), REPLAY/SIM states, and the
  ingest-stall watchdog (`GCS SOFTWARE FAULT` after 2 s).

## 3. Command validation and interlocks

**Evidence.** No path sends arm, mode, mission or parameter-write commands. `dc-cli` is
read-only by design (module docs). The gRPC API exposes `GetInfo`, `OpenLink`, `ListVehicles`,
`GetParams` and two streams. FFI exposes connect and telemetry only. The parameter *set* state
machine (`protocols::params`) verifies by read-back but is not exposed. The only command the
core sends is `SET_MESSAGE_INTERVAL`, from configuration. Its rates are now validated: they
must be finite and > 0.

**Proposal (needs human approval: command pipeline/interlocks).** Before any command RPC
exists, implement plan 08 §4 `CommandPolicy` in `dc-core`, so the UI is never the only guard:
* Classes A–D.
* Preconditions per command, such as disarmed for calibration and motor test.
* Ranges from ArduPilot parameter metadata.
* Gating by the new link state (STALE/LOST allow only RTL/Land retry).
* Single-use confirmation tokens for class C/D.
* Audit log.

This is T0-15 (hazard log, human review) plus a follow-up task.

## 4. Telemetry memory

**Evidence.** There is no telemetry history in memory. `Fleet` holds only the latest value per
message type for at most 255 vehicles, plus a counter per message id. Queues are bounded:
256 per link outbound, 1024 link events, 4096 to the application, 4096 broadcast. History goes
to disk through the `.tlog` recorder on a blocking thread. The one unbounded queue is the
recorder tap, which is lossless by design. Its comment claimed the recorder "reports backlog";
it did not. A failed write, for example a full disk, **stopped recording silently**.

**Fix.** `CoreEvent::Recorder { stopped: true }` is raised when a write or flush fails, and a
warning when more than 50 000 frames (~15 MB) are waiting. Both are also gRPC
`RecorderEvent`s. Test: `liveness.rs::recorder_failure_is_reported` (writes to `/dev/full`).

## 5. Shutdown

**Evidence.** Dropping the runtime cancels the tasks. That closes the hub, so the recorder
drains, flushes and exits, and the runtime waits for it. Serial threads exit when the hub
closes. FFI shuts down with a 500 ms timeout. But `dc-cli daemon` stopped only on Ctrl-C:
**SIGTERM**, which is what service managers and `kill` send, ended the process without
flushing the `.tlog`.

**Fix.** The daemon also stops cleanly on SIGTERM (Unix). Verified by hand: SIGTERM → exit 0,
`.tlog` ends on a complete record. Test: `liveness.rs::shutdown_flushes_the_tlog`.

## How to re-check

```sh
cargo test -p mav-router --test hub_end_to_end
cargo test -p dc-core --test liveness
cargo xtask ci
```
