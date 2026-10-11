# ADR-0008: Mission Planner items outside the parity scope

* Status: accepted
* Date: 2026-10-11
* Deciders: product owner (on the T0-11 triage list)
* Validation: `parity.yaml` (rows marked `status: wont`)

## Context
Parity is measured against the frozen Mission Planner inventory in `parity.yaml` (plan 02,
roadmap D0-7). A row marked *won't* needs a waiver recorded in an ADR (plan 02, coverage
accounting). The inventory of Mission Planner f1ac8f8 (2026-10-09) left 15 items that matched no
plan row. The product owner reviewed them. Most were mapped to plan rows, CoT output became new
row CON-010, and swarm control was deferred to after 1.0 (FD-024). Two items are not product
features.

## Decision drivers
* Parity means user-facing capability, not every developer or vendor tool in the reference.
* Each waived item needs a reason a reviewer can check.

## Options considered
| Option | Pros | Cons | Score |
|--------|------|------|-------|
| Port both tools | Literal 100 % | Effort on a vendor format and a debug benchmark nobody uses in the field | low |
| Waive both tools (chosen) | Effort goes to user-facing rows | Two hidden-menu items absent | high |

## Decision
We will not reproduce these Mission Planner Ctrl-F tools:

| Row | Item | What it does | Reason |
|-----|------|--------------|--------|
| `PAR-MP-TOOL-MicroDrone` | MicroDrone | Streams vehicle position/attitude over serial in a vendor-specific text format (`SerialOutputMD`) | Vendor-specific output. A plugin (ADR-0006) can provide it if a customer needs it |
| `PAR-MP-TOOL-structtest` | structtest | Developer benchmark of byte/struct conversion | Debug tool, not a user feature |

## Consequences
* Positive: these rows are closed (`status: wont`) and do not block GA.
* Negative / risks: a user relying on the MicroDrone output loses it. Mitigation: plugin API
  (ADR-0006), and this ADR is revisited if a customer asks.

## Follow-ups
* Re-run the inventory against newer Mission Planner releases. New hidden tools come back as
  triage rows for a decision.
