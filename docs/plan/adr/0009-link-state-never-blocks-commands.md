# ADR-0009: Link state never blocks commands

* Status: accepted
* Date: 2026-10-11
* Deciders: product owner
* Validation: `CommandPolicy` tests (when built, see the command-interlocks issue); SITL link-chaos
  scenarios (T0-14)

## Context
Plan 08 §4.3 originally *restricted* commands by link state. STALE disabled class B and above
except an RTL/Land retry, and LOST allowed only "re-send RTL/Land". The 2026-10-11 audit check
proposed the same gating for `CommandPolicy`. The product owner rejected it: the link state is
measured from heartbeats the GCS *receives*, and that is not proof that the uplink is down.
Heartbeats can be lost, filtered or slowed on a radio while commands still reach the vehicle.
Blocking the operator at exactly the moment the link is bad removes their ability to recover the
vehicle. That is a worse hazard than a command that may not arrive.

## Decision drivers
* The operator must always be able to act; the GCS must not be the reason a recovery failed.
* A command must never fire long after the operator meant it (no stale queued commands).
* The operator must always know whether a command arrived (no false success).

## Options considered
| Option | Pros | Cons | Score |
|--------|------|------|-------|
| Block by link state (plan 08 v1) | Prevents commands that "probably won't arrive" | Locks the operator out when they most need control; heartbeat loss ≠ uplink loss | rejected |
| Allow, but add confirmation steps on bad links | Draws attention | Slows emergency actions (RTL time-to-command ≤ 1.5 s) | rejected |
| **Always send; adapt delivery; report honestly (chosen)** | Operator keeps control; delivery improves; outcome visible | More retransmission logic in the core | chosen |

## Decision
We will never block, delay or silently drop a confirmed command because of the link state
(CONNECTED, DEGRADED, STALE or LOST). Instead:

1. **Inform.** The confirmation card and status ribbon show the link state, heartbeat age and
   what the vehicle will do on its own (its `FS_GCS_*` failsafe). Link state adds no extra
   confirmation step: class C/D confirmation stays as in §4.1.
2. **Adapt delivery.** In DEGRADED, STALE and LOST, the core retransmits the command
   (MAVLink `confirmation` counter for `COMMAND_LONG`) with back-off. It does so on every link
   that reaches the vehicle, until the vehicle sends `COMMAND_ACK` or telemetry shows the
   expected state.
3. **Bound the intent.** Every command has a lifetime: by default 30 s for RTL and Land, 10 s
   for everything else. Retries stop when the lifetime expires, when a newer conflicting command
   goes to the same vehicle, or when the operator cancels. Frames queued before a reconnect are
   still discarded (`link-net`), and retries are always fresh frames from the command pipeline.
4. **Report honestly.** Success is state-based (§4.2). Until the result is verified the status
   reads `SENT — NOT CONFIRMED (link LOST 7.2 s)`, with a retry count and the time left. When
   the lifetime expires it becomes `NOT CONFIRMED — EXPIRED`, never success.

Vehicle-state preconditions (§4.2 context lockouts, such as motor test only when disarmed) are
unaffected. They use the last-known state and show its age. REPLAY keeps no command path,
because no vehicle is connected.

## Consequences
* Positive: the operator is never locked out; commands get more delivery attempts on bad links;
  nothing fires late; outcomes are explicit.
* Negative / risks (with mitigations):
  * Retransmissions add uplink load on a weak link. Mitigation: back-off, one pending
    command per conflict group, and a later option to lower telemetry rates while commands are
    pending.
  * A command can arrive late but within its lifetime. Mitigation: a short default lifetime,
    a countdown shown, and supersede/cancel.
  * Preconditions can rely on stale state. Mitigation: the age is shown; hazard review in
    T0-15.

## Follow-ups
* Plan 08 §4.3 and the UI skill updated (this change).
* `CommandPolicy` implementation follows this ADR (human-approved task after T0-15).
* T0-15 hazard log: add "operator locked out of recovery command during link degradation" and
  "late command execution" with these mitigations.
