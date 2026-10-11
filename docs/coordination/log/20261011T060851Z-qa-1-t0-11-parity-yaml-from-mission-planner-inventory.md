---
from: qa-1
to: all
type: done
subject: T0-11 parity.yaml from Mission Planner inventory
ref: 2db1006
created: 2026-10-11T06:08:51Z
---
Acceptance met: parity.yaml is on main with 322 rows. Every row has an id, an owner workstream and a T-MP-* test id placeholder (checked with yaml.safe_load: ids unique, rows/triage counters match).

- Source: ArduPilot/MissionPlanner f1ac8f8 (2026-10-09). The parser reads designer files and .resx only; no code is copied.
- Coverage: main menu (6), Flight Data tabs, buttons and context menus (98), Flight Plan menus and buttons (95), Initial Setup pages (41), Config/Tuning pages (14), Ctrl-F tools (63), Simulation (5).
- Mapping: ordered rules map each row to docs/plan/02-parity-matrix.md. I spot-checked every mapping change across iterations.
- 15 rows have `triage: true`: the 5 top-level nav entries, 7 Ctrl-F swarm/debug tools (CoT, Swarm, Follow/Sequence Swarm, MicroDrone, structtest, grab threads.txt) and 3 swarm/wipe SITL options. A human decides these before the freeze (see the issue).
- These plan rows have no MP item: CON-001/002/003/005/006/007, CT-002/005/008/012/014, FD-008/011/017/022/023, FP-009/010/014/015/016/017, IS-019, SA-010, XC-001..006. Most are DrorControl additions or behaviours with no single control; a reviewer should confirm none were missed.

Not done (needs a human): the freeze sign-off itself, and the D0-7 test corpus (real logs, plans and params need consent and licensing). Re-run with `scripts/mp_inventory.py <mp> [--check]`; the method is in docs/parity/README.md.
