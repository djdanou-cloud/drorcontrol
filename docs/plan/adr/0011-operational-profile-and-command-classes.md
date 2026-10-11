# ADR-0011: Operational profile and command classes (single-action flight controls)

* Status: accepted
* Date: 2026-10-11
* Deciders: Flight Operations & Systems Safety Lead (operational directive "High-Integrity
  Remote & Maritime UAS Profile")
* Validation: `crates/protocols/tests/command_policy.rs`
  (`classes_follow_the_operational_profile`,
  `mode_change_is_single_action_and_expires_after_three_seconds`,
  `emergency_termination_is_two_stage_without_typed_text`, `vehicle_state_lockouts`);
  `crates/dc-core/tests/commands.rs`; `crates/dc-core/tests/liveness.rs`
  (`link_thresholds_are_bounded`); hazard log [`docs/safety/hazards.md`](../../safety/hazards.md)

## Context
Plan 08 §4.1 (v1) assumed a general-purpose civil GCS. It put Arm, RTL, Land, Takeoff, motor
test and mode changes to Auto/Guided/Acro behind a two-stage confirmation (hold or slide, then a
card). The operating organisation then set the profile this product serves:

* **Regulatory basis:** no civil aviation authority oversight; **MIL-STD-882E** system safety.
* **Fleet:** multirotor, hybrid and VTOL vehicles up to 50 kg MTOW.
* **Environment:** BVLOS, maritime and remote areas, GPS-degraded or RF-contested. No flight
  over uninvolved civil assemblies.
* **Operators:** professional, working under SOPs; one operator commands one vehicle (1:1);
  encrypted long-range datalinks.
* **Risk targets:** a Catastrophic outcome must be mitigated to Improbable (E), which is the
  same band as "Extremely Remote". Catastrophic here means an uncontrolled kinetic impact
  outside the designated operational boundary, or loss of the vehicle in a critical flight
  phase. **Unresponsive or delayed control is itself a Hazardous/Severe outcome**, which is
  Critical (2) in MIL-STD-882E.

With delayed control rated Critical, every confirmation step on a flight control adds to a
hazard. Mode changes are the operator's main way to arrest an unsafe trajectory, for example
Loiter, Brake, RTL or Land. Making them two-stage trades a rare mis-click for a slower response
on every emergency.

## Decision drivers
* Time-to-command for arresting manoeuvres must be as short as the input device allows.
* Two-stage confirmation must stay where the action is irreversible and ends the flight.
* A command that was not acknowledged must not stay pending and execute later, after the
  aircraft has moved into a different flight state.
* Interlocks must stay in the core (`CommandPolicy`) so that no client can bypass them.
  ADR-0009 still applies: link state never blocks a command.

## Options considered
| Option | Pros | Cons | Score |
|--------|------|------|-------|
| Keep plan 08 v1 (two-stage for mode, Arm, RTL, Land, Takeoff) | Fewer accidental commands | Confirmation latency on every flight-critical action; delay is rated Critical | rejected by the safety lead |
| Two-stage only for "hazardous" modes (Auto, Guided, Acro) | Middle ground | Still delays some arrest manoeuvres; mode hazard depends on context, not name | rejected (directive: all mode transitions single action) |
| **Single action for flight controls; two-stage only for terminal actions (chosen)** | Fast arrest; confirmation where it matters | Accidental single action is possible; mitigated by identity, focus and SOP controls (hazard H14) | chosen |

## Decision
We will classify commands by their consequence in flight, not by how dramatic they look:

| Class | Commands | Interaction (every input path: click, hotkey, hardware switch, joystick, API) |
|-------|----------|------------------------------------------------------------------------|
| **B — single action** | **All mode changes** (`SetMode`, any mode), **RTL**, **Land**, **Arm**, **Takeoff**, **Reposition / guided go-to**, **Motor test** (bench) | One action fires the command. No hold timer, no card, no animation gate: it is sent when the input event arrives. The vehicle state lockouts still apply in the core. Motor test still needs the props-off attestation and a known disarmed state. |
| **C — two-stage** | **In-flight motor disarm** (`Disarm`), **emergency motor termination** (`ForceDisarm`), **autopilot reboot**; parameter reset, mission upload while armed and firmware flash (interpretation 3) | Stage 1 is a slide (or press-and-hold). Stage 2 is the identity card with an explicit confirm. There is no typed text. The single-use token is bound to the vehicle and the exact command and lasts 5 s. |
| **D — typed** | Force-arm (bypasses pre-arm checks), disabling arming checks or failsafes | Class C plus a typed vehicle name and a logged reason. |

**Command lifetimes** (ADR-0009 delivery, now per class of command):

| Command | Lifetime | Bound |
|---------|----------|-------|
| RTL, Land | 30 s | — |
| Mode change | **3 s** | **2–3 s, enforced** (`CoreConfig::validate` refuses other values) |
| Everything else | 10 s | — |

If the vehicle does not confirm a mode change within its lifetime, the core drops it: it stops
retransmitting and marks it `NOT CONFIRMED — EXPIRED`. The UI must alert the operator (visual
and audio) the moment that state arrives. A mode change is never left to take effect after the
aircraft is in a different flight state.

**Lockouts kept** (safety lead, 2026-10-11): reboot is blocked while armed. The motor-test
props-off attestation is kept for **all** vehicle types, not only copters.

**Interpretations recorded for review.** The directive reserves two-stage confirmation
"exclusively" for in-flight disarm, reboot and emergency termination. We read that as follows:
1. Arm, RTL, Land, Takeoff and motor test become single action too, with the lockouts above.
2. `Disarm` stays two-stage even on the ground. The GCS cannot yet tell landed from flying
   reliably (`EXTENDED_SYS_STATE.landed_state` is not consumed yet). Once it is, a landed
   disarm can become single action.
3. The directive does not mention force-arm (class D), parameter reset, mission upload while
   armed or firmware flash (class C). None of these is a flight control, so they keep their
   confirmation until the safety lead decides.

The two-stage interaction itself (hold or slide length, card layout) and the rule that hotkeys
fire only while the flight view has keyboard focus belong to the UI phase. The rules are in the
UI skill §4 and plan 08 §4.1 (both updated by this change).

## Consequences
* Positive:
  * Arresting manoeuvres take one input.
  * Unacknowledged mode changes cannot execute late.
  * The confirmation burden now sits on terminal actions, which are rare and irreversible.
* Negative / risks (with mitigations):
  * **Accidental mode change or arm** (hazard H14). Mitigations:
    * persistent vehicle identity and current-mode banner;
    * hotkeys only while the flight view has focus;
    * no default single-key binding outside the SOP-configured set;
    * verification against `HEARTBEAT` with visible pending, verified and expired states;
    * the audit log;
    * the operator's next single action corrects the mode immediately (supersede).
  * **Arm without confirmation.** The vehicle's own pre-arm checks (`ARMING_CHECK`) remain the
    gate. Force-arm stays class D.
  * **A 3 s mode lifetime can expire on a slow but working link.** Mitigations: an immediate
    alert and a one-action re-issue. The expiry is the intended behaviour: the operator decides
    again with current information.
* Supersedes plan 08 §4.1 v1 for the commands listed. ADR-0009 is unchanged.

## Follow-ups
* Hazard log [`docs/safety/hazards.md`](../../safety/hazards.md) baselined for this profile
  (T0-15). A full hazard-by-hazard review by the safety lead is still pending.
* Consume `EXTENDED_SYS_STATE.landed_state` and then revisit interpretation 2.
* UI phase:
  * usability test of time-to-command, from input event to `COMMAND_LONG` on the wire, for
    every class B path;
  * the expiry alert;
  * the focus rules for hotkeys.
