---
title: T0-11: parity freeze sign-off (15 triage rows) and test-corpus consent/licensing
severity: S3
state: resolved
owner: -
reporter: qa-1
needs_human: yes
created: 2026-10-11T06:08:51Z
updated: 2026-10-11T07:40:00Z
---

## Description

The parity list (D0-7) can only be frozen with a human decision on:

1. **15 triage rows** in `parity.yaml` (`triage: true`). No rule mapped them:
   - Main menu DATA/PLAN/SETUP/CONFIG/HELP (suggested: XC-007 workspaces, or no row of their own).
   - Ctrl-F: CoT, Swarm, Follow Swarm, Sequence Swarm, MicroDrone, structtest, grab threads.txt.
     These are swarm features, a vendor tool and debug tools.
   - Simulation: Copter Swarm (single link / multilink), Wipe.
2. **Test-corpus collection**: real flight logs, mission plans and param files from users or
   partners need consent and a licence before they can enter the repo.

## Evidence

`parity.yaml` (commit 2db1006). Method in `docs/parity/README.md`.

## Proposed resolution

For each triage row, either map it to a plan row or mark it out of scope. Out of scope needs an
ADR, which could cover the debug-only Ctrl-F tools as one group. The swarm items may warrant a new
plan row under multi-vehicle. Decide where the corpus is stored and under what licence. Then mark
the list frozen.

## Updates

- 2026-10-11 (qa-1): Product owner decided the 15 triage rows. Recommendations accepted, CoT in
  scope (CON-010), swarm control later (FD-024, 8 rows deferred), MicroDrone and structtest
  waived (ADR-0008). parity.yaml is frozen with 0 triage rows. Corpus: the owner provided a real
  flight log and a .param file (docs/corpus/README.md). A mission plan file would still help.
