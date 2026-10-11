# Mission Planner parity inventory

[`parity.yaml`](../../parity.yaml) lists every user-visible Mission Planner feature that DrorControl
must match. It is the frozen list behind the plan's "100 % parity" goal
([02](../plan/02-parity-matrix.md), roadmap D0-7). It is generated. Do not edit it by hand.

## How it is built

`scripts/mp_inventory.py` reads an `ArduPilot/MissionPlanner` checkout: the WinForms
`*.Designer.cs` files, their `.resx` string tables and `Strings.resx`. It records only labels and
structure, as facts about the reference product. No Mission Planner code is copied.

| Surface | Source | What becomes a row |
|---|---|---|
| Main menu | `MainV2.Designer.cs` | DATA, PLAN, SETUP, CONFIG, SIMULATION, HELP |
| Flight Data | `GCSViews/FlightData.Designer.cs` | each tab under `tabControlactions`, every button/checkbox on it, every context-menu item (map, HUD, actions tab, quick view) |
| Flight Plan | `GCSViews/FlightPlanner.Designer.cs` | every context-menu item, nested submenus included, and every button |
| Initial Setup, Config/Tuning | `GCSViews/InitialSetup.cs`, `SoftwareConfig.cs` | every `AddBackstageViewPage` page (loading and placeholder pages are skipped) |
| Ctrl-F tools | `temp.Designer.cs` | every button |
| Simulation | `GCSViews/SITL.Designer.cs` | every button and vehicle tile |

Each row carries:

* `id`: stable, `PAR-MP-<area>-<control or label slug>`.
* `mp`: where the item lives in Mission Planner, e.g. `Flight Plan > Map menu > Rally Points > Upload`.
* `source`: the file and control or class name it came from.
* `plan`: the matching row of the [parity matrix](../plan/02-parity-matrix.md), e.g. `PAR-FP-007`.
* `owner`: the workstream that delivers it ([coordination](../coordination/README.md)).
* `basis`: `rule`, or the product-owner decision that set `plan`, `owner` and `status`
  (`DECISIONS` in the script).
* `triage`: `true` when no rule or decision matched. `plan` and `owner` are then the area default and need a
  human decision.
* `status`: `not-started`, `deferred` (after 1.0) or `wont` (waived in
  [ADR-0008](../plan/adr/0008-parity-out-of-scope.md)). Rows can only become `implemented` with a test id and a handbook page
  ([ADR-0007](../plan/adr/0007-docs-as-code.md)).
* `test`: the placeholder automated test id, `T-MP-…`.
* `handbook`: `TBD` until the handbook page exists.

Rows map to the plan through the ordered regex `RULES` in the script. A rule sees the item's own
labels and control or class name, never the surface title ("Flight Plan" must not match "Plane").
A submenu item is matched by its parent path first, so `Rally Points > Upload` maps to rally
points, not to mission upload. A button on a tab or in a flat menu is matched by its own label
first and falls back to the tab, so `Actions tab > Joystick` maps to joystick setup and
`Telemetry Logs tab > 2x` maps to tlog replay.

## Re-running

```sh
git clone https://github.com/ArduPilot/MissionPlanner /path/to/mp   # optionally git -C /path/to/mp checkout <commit>
python3 scripts/mp_inventory.py /path/to/mp            # rewrites parity.yaml, prints plan rows with no MP item
python3 scripts/mp_inventory.py /path/to/mp --check    # exit 1 if parity.yaml is out of date
```

The header of `parity.yaml` records the Mission Planner commit and date it was built from. To
re-map a row, change the rules and re-run. If the rules map to a plan id that is not in the
matrix, the script stops with an error.

## Freezing

**Frozen on 2026-10-11.** The product owner decided the 15 rows no rule could map:
* The DATA, PLAN, SETUP and CONFIG menu entries are workspaces (XC-007). HELP is the new row
  XC-008.
* CoT output to TAK/ATAK is in scope as the new row CON-010.
* "grab threads.txt" is served by the MAVFTP browser (CT-011). The SITL "Wipe" option belongs
  to SA-001.
* Swarm control is deferred to after 1.0 as the new row FD-024 (phase *later*). That covers 8
  rows: the three swarm tools, "Follow the leader" (Mission Planner's swarm follow-path) and the
  four swarm SITL launchers.
* MicroDrone (vendor serial output) and structtest (debug benchmark) are waived in ADR-0008.

These decisions live in `DECISIONS` in the script, so re-runs keep them. If a decided row
disappears upstream, the script stops. A re-run against a newer Mission Planner that finds new
items marks them `triage: true`, and they need a decision before the list is frozen again.
Progress on status, test and handbook will move to a hand-maintained overlay keyed by `id`, so the
generated inventory stays read-only.

Some plan rows have no Mission Planner item. Some are DrorControl additions (e.g. accessibility,
audit log, emergency kill). Others are behaviours that have no single control to point at
(e.g. link-quality stats, multi-vehicle). The script prints them so reviewers can check that
nothing was missed.
