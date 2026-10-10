<!-- GENERATED from .claude/skills/drorcontrol-ui/SKILL.md by scripts/sync_skill.py - do not edit; edit the source and re-run the script. -->
# DrorControl — Copilot repository instructions

DrorControl: cross-platform (macOS-first) ArduPilot ground control station. Rust core + Qt 6/QML UI. Plan: `docs/plan/README.md`. Coordination protocol for agents and humans: `docs/coordination/README.md` (read first; claim paths, log progress, work on `main`, never force-push).

Hard rules (full UI spec: `.github/instructions/drorcontrol-ui.instructions.md`):
- Never use bare single-click buttons for Arm, Disarm-in-flight, RTL, Land, Kill, Motor test, Param reset, firmware flash. Use slide/hold + confirmation card; kill = 1.5 s hold + typed confirm; RTL = 0.8 s hold + <=5 s confirm. The core enforces this too.
- Status colors are reserved (warning #FF3B30, caution #FF9400, advisory #22D3EE, normal #2ECC71, guidance #E040FB, standby #757575). Never color-only; never decorative. Use design tokens, no literal colors.
- Telemetry numerals: tabular/monospace, fixed-width boxes, no layout shift. Alt/speed 1 dp, voltage 2 dp, lat/lon 7 dp, heading integer. One central unit formatter.
- UI pulls one telemetry snapshot per frame; never bind UI properties per MAVLink message. Keep HUD/GPU items isolated from form-heavy views.
- Touch targets >= 48x48 px (56 px gloved). Stale data is dimmed/invalidated, never shown as live.
- Mode lists, MAVLink IDs and parameter metadata come from pinned upstream (ArduPilot/MAVLink XML), not hand-typed.
- Never commit secrets. Ask a human before changing the command path, interlocks, failsafes, licences, or anything irreversible.
