# Hazard log — DrorControl

* Status: **v0 baseline** (T0-15). The operational profile, the risk-acceptance targets and the
  command decisions below were set by the Flight Operations & Systems Safety Lead on 2026-10-11.
  The per-hazard severity and probability ratings are **engineering proposals**: they have not
  yet been reviewed hazard by hazard. That review is still open (see §7).
* Method: MIL-STD-882E, Task 202 (preliminary hazard analysis) style, limited to hazards that the
  ground control station causes or contributes to. Hazards that belong only to the airframe,
  the autopilot or the radio are out of scope, except where the GCS is a control for them.
* Linked decisions: [ADR-0009](../plan/adr/0009-link-state-never-blocks-commands.md) (link
  state never blocks commands), [ADR-0011](../plan/adr/0011-operational-profile-and-command-classes.md)
  (operational profile and command classes); [plan 08](../plan/08-safety-vv.md) §1 and §4.

## 1. Operational profile (baseline)

| Item | Baseline |
|------|----------|
| Regulatory basis | No civil aviation authority oversight. **MIL-STD-882E** applies. |
| Fleet | Multirotor, hybrid and VTOL; **MTOW ≤ 50 kg** |
| Environment | BVLOS; maritime and remote areas; **GPS-degraded** or **RF-contested** |
| Overflight | No flight over uninvolved civil assemblies |
| Crew | Professional operators under SOPs; **1:1** (one operator, one vehicle) |
| Datalink | Encrypted long-range datalinks, provided by the operating organisation, not by the GCS |

## 2. Severity, probability and risk (MIL-STD-882E)

**Severity**

| Category | Definition in this profile |
|----------|----------------------------|
| **Catastrophic (1)** | **Uncontrolled kinetic impact outside the designated operational boundary**, **catastrophic vehicle loss in a critical flight phase** (take-off, transition, recovery, landing), or death |
| **Critical (2)** | **Unresponsiveness or delayed control** (directive: "Hazardous/Severe"); serious injury; vehicle loss outside a critical phase; mission failure that leaves the vehicle uncontrolled inside the boundary |
| **Marginal (3)** | Minor injury; recoverable vehicle damage; degraded mission; loss of records |
| **Negligible (4)** | Inconvenience; no injury or damage |

**Probability** (over the life of one system in this profile)

| Level | Definition |
|-------|------------|
| A Frequent | Likely to occur often |
| B Probable | Will occur several times |
| C Occasional | Likely to occur sometime |
| D Remote | Unlikely, but possible |
| **E Improbable** | So unlikely that it can be assumed it will not occur (≈ "Extremely Remote") |
| F Eliminated | Cannot occur: the hazard is removed by design |

**Risk matrix** (MIL-STD-882E Table III)

| | Catastrophic (1) | Critical (2) | Marginal (3) | Negligible (4) |
|---|---|---|---|---|
| **A Frequent** | High | High | Serious | Medium |
| **B Probable** | High | High | Serious | Medium |
| **C Occasional** | High | Serious | Medium | Low |
| **D Remote** | Serious | Medium | Medium | Low |
| **E Improbable** | Medium | Medium | Medium | Low |
| **F Eliminated** | Eliminated | Eliminated | Eliminated | Eliminated |

**Acceptance targets (safety lead, 2026-10-11)**
* Catastrophic hazards must reach **E Improbable** before fielding.
* Critical hazards (this includes delayed or unresponsive control) must reach **E Improbable**,
  or D Remote with a residual risk that the safety lead accepts explicitly.
* Risk-acceptance authority for this log: the **Flight Operations & Systems Safety Lead**.

## 3. Command decisions (baseline, ADR-0011)

| # | Decision (safety lead, 2026-10-11) | Implemented | Evidence |
|---|------------------------------------|-------------|----------|
| D1 | **Two-stage confirmation for mode changes is rejected.** Every mode transition is a single action: one click, one hotkey or one hardware switch. | `Command::class()` → class B for `SetMode` (any mode), RTL, Land; UI skill §4, plan 08 §4.1 | `command_policy.rs`: `classes_follow_the_operational_profile`, `mode_change_is_single_action_and_expires_after_three_seconds`, `class_b_needs_no_token` |
| D2 | **Two-stage (slide) is reserved for in-flight motor disarm, autopilot reboot and emergency motor termination.** | Class C for `Disarm`, `Reboot`, `ForceDisarm` (no typed text) | `class_c_is_refused_without_a_valid_token`, `emergency_termination_is_two_stage_without_typed_text`; `dc-core/tests/commands.rs`: `arm_is_one_action_disarm_is_confirmed_and_both_are_audited` |
| D3 | **Command lifetimes:** RTL/Land 30 s; general 10 s; **mode change 2–3 s at most**, then dropped and the pilot alerted. | `PolicyConfig::lifetime_mode` = 3 s; `CoreConfig::validate` refuses values outside 2–3 s; expiry is reported as `NOT CONFIRMED — EXPIRED` (`CoreEvent::Command`) | `mode_change_is_single_action_and_expires_after_three_seconds`, `nothing_is_sent_after_the_lifetime`; `liveness.rs`: `link_thresholds_are_bounded`; `commands.rs`: `lost_link_still_sends_retries_and_expires` |
| D4 | **Reboot while armed stays blocked.** | Precondition in the core, checked again at submit | `vehicle_state_lockouts` |
| D5 | **Motor-test props-off confirmation is kept for all vehicle types.** | Precondition: `props_removed` attested and known disarmed, for every vehicle type | `vehicle_state_lockouts` |
| D6 | **Night-Ops telemetry red #FF4D4D at 6.4:1 is accepted as residual risk** (below the 7:1 target, for dark adaptation and NVG). | Token exception recorded in plan 04 | `crates/design-tokens` contrast rules (`ui/tokens/tokens.json` → `contrast.rules`) |

**Interpretations awaiting confirmation** (from ADR-0011):
* (I1) Arm, Takeoff and motor test are single action, because the directive keeps two-stage
  confirmation "exclusively" for D2.
* (I2) `Disarm` stays two-stage even on the ground until landed state is known. The core now
  reads the landed state (`EXTENDED_SYS_STATE`, requested at 2 Hz) and passes it with its age
  to the command policy (SITL check: a copter on the ground reports ON_GROUND). No rule uses
  it yet: making a disarm on the ground single action is the safety lead's decision.
* (I3) Force-arm (class D, typed), and parameter reset, mission upload while armed and
  firmware flash (class C), keep their confirmation. None of them is a flight control.

## 4. Hazards

Columns:
* **Sev**: severity.
* **Init**: probability and risk with no GCS control.
* **Target**: residual probability and risk with the listed controls in place.
* **Status**:
  * *Verified (core)*: the controls are in the core and covered by passing tests.
  * *Partial*: some controls exist.
  * *Open*: no control is implemented yet.
* Ratings are proposals until the review in §7.

| ID | Hazard | Worst credible outcome | Sev | Init | GCS controls | Verification | Target | Status |
|----|--------|------------------------|-----|------|--------------|--------------|--------|--------|
| H1 | Unintended **terminal** command (in-flight disarm, emergency termination, reboot, force-arm) | Motors stop in flight → uncontrolled impact | 1 | C / High | Two-stage (slide + identity card), single-use token bound to vehicle and exact command, 5 s TTL; typed phrase and reason for class D; enforced in the core, so API clients and scripts cannot bypass it; reboot blocked while armed; audit | `class_c_is_refused_without_a_valid_token`, `class_d_needs_the_typed_phrase_and_a_reason`, `emergency_termination_is_two_stage_without_typed_text`, `vehicle_state_lockouts` | E / Medium | Verified (core); UI pending |
| H2 | Wrong vehicle commanded | Command meant for another vehicle → loss or boundary excursion | 1 | D / Serious | 1:1 operation; token bound to sysid; persistent vehicle identity banner; identity on cards | `class_c_is_refused_without_a_valid_token` (token for another vehicle refused) | E / Medium | Partial (core); UI pending |
| H3 | Stale data shown as live | Operator acts on an old position or attitude → boundary excursion | 1 | C / High | Link watchdog CONNECTED/STALE/LOST (1.5 s / 5 s, bounded config) with the heartbeat-gap check (a brief LOST is never skipped); per-field age; instrument invalidation (UI) | `liveness.rs`: `vehicle_goes_stale_then_lost_then_recovers`, `a_brief_lost_is_never_skipped`; SITL `radio_fade_goes_stale_then_lost_then_recovers` | E / Medium | Partial (core); UI pending |
| H4 | Corrupt or partial mission upload | Vehicle flies an unintended path outside the boundary | 1 | C / High | Upload then read back and compare item by item (ArduPilot storage rules applied, unknown differences reported); only a verified read-back counts as "on vehicle"; retries sized for 30 % loss; refused while armed until the class C confirmation exists for missions; "draft ≠ vehicle" badge (UI) | `protocols/tests/mission.rs` (30 % loss × 20 seeds, vehicle-changed value, refusal, dead link); `dc-core`: `mission_upload_is_refused_while_armed`; SITL `mission_upload_is_verified_by_read_back_through_loss` (20 % loss, 14 items, 0 differences) | E / Medium | Partial (core); in-flight upload confirmation and UI pending |
| H5 | Dangerous parameter change (including failsafe or fence parameters) | Failsafe or fence disabled → uncontrolled flight-away | 1 | C / High | Risk labels, diff preview, typed confirmation, snapshot and rollback; parameter reset class C and not while armed | `vehicle_state_lockouts` (reset); parameter download complete under 30 % loss (SITL) | E / Medium | Partial |
| H6 | Calibration error gives bad sensor data | Loss of control in a critical phase | 1 | D / Serious | Guided wizard, fitness thresholds, post-calibration sanity checks | — | E / Medium | Open |
| H7 | Motor spins unexpectedly during a bench motor test | Injury to personnel | 2 | C / Serious | Props-off attestation for **all vehicle types**; only when **known** disarmed (unknown counts as armed); throttle and duration bounds; checked again at submit | `vehicle_state_lockouts`, `arguments_are_validated` | E / Medium | Verified (core) |
| H8 | GCS freeze or crash in flight | **Unresponsive control** | 2 | C / Serious | Core survives a UI crash (separate process, plan 01); recorder failure reported; clean shutdown flushes the recorder; UI and ingest watchdogs (plan 08 §4.3) | `recorder_failure_is_reported`, `shutdown_flushes_the_tlog` | E / Medium | Partial |
| H9 | Alert fatigue or missed alert (including a missed mode-change expiry) | Operator misses a failsafe or an expired command → delayed control | 2 | C / Serious | Priority model, rate limiting, audio and visual redundancy; a mode-change expiry is a pilot alert (D3) | — | E / Medium | Open (UI) |
| H10 | Spoofed, injected or jammed link (RF-contested environment) | Hostile or false command; false telemetry → loss or boundary excursion | 1 | C / High | Encrypted datalinks (operator equipment); MAVLink 2 signing (planned); strict frame parsing and input validation; fuzzing (planned) | Codec tests; `arguments_are_validated` | E / Medium | Partial; signing open |
| H11 | Operator locked out of a recovery command while the link is degraded | **Delayed or no control** when it is needed most | 2 | C / Serious | ADR-0009: link state never refuses a command; retransmit on every link; bounded lifetime; TCP and serial links reopen by themselves after a drop or USB unplug (stale frames discarded) | `link_state_never_refuses_a_command`, `retransmits_with_backoff_and_counts_confirmation`; `commands.rs`: `lost_link_still_sends_retries_and_expires`; `link-serial`: `reopens_after_unplug_and_drops_stale_frames`; SITL `reconnects_after_the_connection_is_cut` | E / Medium | Verified (core) |
| H12 | Late command execution (a queued command arrives after the flight state has changed) | Wrong manoeuvre in a critical phase → vehicle loss | 1 | C / High | Lifetimes: RTL/Land 30 s, **mode 2–3 s (bounded in config)**, others 10 s; supersede within a group; cancel; frames from before a reconnect discarded; expiry reported, never success | `nothing_is_sent_after_the_lifetime`, `newer_command_supersedes_and_cancel_stops`, `mode_change_is_single_action_and_expires_after_three_seconds`, `link_thresholds_are_bounded` | E / Medium | Verified (core) |
| H13 | **Confirmation latency delays a flight control** (ADR-0011 driver) | **Delayed control** during arrest of an unsafe trajectory | 2 | B / High (plan 08 v1 two-stage flow) | Mode changes, RTL, Land, Arm, Takeoff and reposition are single action, with no hold timer, card or animation gate; the core needs no token for class B | `classes_follow_the_operational_profile`, `class_b_needs_no_token`; UI time-to-command test (input event → frame on the wire) pending | E / Medium | Verified (core); UI test pending |
| H14 | **Accidental single-action command** (mode change, Arm, Takeoff) introduced by ADR-0011 | Unintended mode or arm → boundary excursion or injury on the ground | 2 | C / Serious | Persistent identity and mode banner; hotkeys only while the flight view has focus; only the SOP-configured bindings; state-based verification; immediate correction by the next single action (supersede); vehicle pre-arm checks remain the arming gate; audit | `newer_command_supersedes_and_cancel_stops`, `retries_stop_on_ack_and_command_is_verified_by_state`, `audit_chain_records_everything_and_detects_tampering` | D / Medium (**acceptance by the safety lead required**) | Partial; UI rules open |
| H15 | False indication that a command succeeded | Operator believes the vehicle is in RTL or disarmed when it is not → delayed control | 2 | C / Serious | Verification is state-based, not ACK-based; an ACK after superseding the same `MAV_CMD` is treated as ambiguous; `SENT — NOT CONFIRMED` until verified | `state_verifies_even_if_the_ack_was_lost`, `late_ack_of_a_superseded_command_is_not_misattributed`, `in_progress_and_rejection` | E / Medium | Verified (core) |
| H16 | Misleading position display in GPS-degraded operation | Operator steers on a false position → boundary excursion | 1 | C / High | The core mirrors the autopilot's EKF (`EKF_STATUS_REPORT` flags and variances, ArduPilot's `FS_EKF_THRESH` rule) and GPS fix: position Good / Degraded / Untrusted / Unknown with reasons; Warning when untrusted, Caution when degraded. UI: show beside the position, invalidate when untrusted, predicted-position cone when LOST | `vehicle`: `mirrors_the_autopilot`; SITL `gps_loss_makes_the_position_untrusted` (GPS failed → UNTRUSTED in 7.5 s) | E / Medium | Partial (core); UI pending |
| H17 | Night-Ops telemetry legibility (6.4:1 instead of 7:1) | Misread value at night → delayed action | 3 | D / Medium | Red monochrome for dark adaptation; warnings by shape, flash and audio, not only colour; brightness control | Token contrast checks (`crates/design-tokens`) | D / Medium — **accepted residual risk (D6)** | Accepted |
| H18 | Command record lost or altered | No post-incident evidence (Marginal) | 3 | D / Medium | SHA-256 hash-chained audit log; a broken chain refuses to start; recorder failure is reported, never silent | `audit_chain_records_everything_and_detects_tampering`; `commands.rs` tamper check; `recorder_failure_is_reported` | E / Medium | Verified (core) |

## 5. Safety requirements derived from this log

| SR | Requirement | Hazards | Where |
|----|-------------|---------|-------|
| SR-01 | Class B commands are sent when the input event arrives, with no confirmation step, on every input path | H13 | `CommandPolicy::request` → `Ready`; UI skill §4 |
| SR-02 | Class C/D commands need a single-use token bound to the vehicle and the exact command; the core enforces it | H1, H2 | `CommandPolicy::submit` |
| SR-03 | Link state never refuses a command | H11 | ADR-0009 |
| SR-04 | No command is sent after its lifetime; a mode change lives 2–3 s | H12 | `PolicyConfig`, `CoreConfig::validate` |
| SR-05 | An expired or unconfirmed command raises an operator alert | H9, H12 | `CommandStatus::alert` → `CoreEvent::Alert` / gRPC `AlertEvent`: Warning for unconfirmed flight commands (incl. mode change after 3 s), Caution for ground commands, Advisory for a verified mode change (tests `command_outcomes_raise_the_right_alerts`, `unconfirmed_mode_change_alerts_the_pilot`); UI display and audio pending |
| SR-06 | Success is shown only from observed vehicle state | H15 | `CommandPolicy::on_state` |
| SR-07 | Motor test needs props-off attestation and a known disarmed state, for every vehicle type | H7 | `Command::preconditions` |
| SR-08 | Reboot and parameter reset are refused while armed | H1, H5 | `Command::preconditions` |
| SR-09 | Link staleness is detected within one watchdog tick, and no state is skipped | H3 | `dc-core` link watchdog |
| SR-10 | Every command, confirmation and outcome is in a tamper-evident log | H18 | `protocols::audit` |

## 6. Review record

| Date | Reviewer | Scope | Result |
|------|----------|-------|--------|
| 2026-10-11 | Flight Operations & Systems Safety Lead | Operational profile (§1); risk targets (§2); command decisions D1–D6 (§3) | Directive issued; baselined in this log and ADR-0011 |

## 7. Open items

* Review of every hazard's severity and probability rating by the safety lead (the T0-15 exit
  criterion).
* Confirm or overturn interpretations I1–I3 (§3).
* Accept or reduce the residual risk of H14 (target D / Medium).
* Open hazards still need controls built and tested: H4, H6, H9, H16, and signing for H10.
* UI-phase evidence: time-to-command for each class B path; the alert on mode-change expiry;
  hotkey focus rules.
