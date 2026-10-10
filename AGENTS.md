# AGENTS.md — DrorControl

DrorControl is a cross-platform (macOS-first) ArduPilot ground control station: Rust core + Qt 6/QML UI. The plan is in [`docs/plan/`](docs/plan/README.md).

**Before doing anything, read [`docs/coordination/README.md`](docs/coordination/README.md)** (the coordination protocol) and [`docs/coordination/BOARD.md`](docs/coordination/BOARD.md) (live state).

Non-negotiables:
- Work on `main`; small commits; `git pull --rebase origin main` before each push; **never force-push or rewrite history**.
- `python3 scripts/coord.py register|claim|status|log|release|check|board` — claim paths before editing; stay in your workstream's lane; log progress, handoffs and blockers as new files in `docs/coordination/log/`.
- Truth order: human instruction > accepted ADR > `docs/plan` > `backlog.md` > chatter. Verify third-party facts (MAVLink IDs, ArduPilot names) against pinned upstream.
- Never commit secrets. Stop and ask a human for credentials, licensing, scope changes, safety-path changes (command pipeline, interlocks, failsafes, firmware flashing), and irreversible actions.
- Report outcomes faithfully: evidence for "done", plain statements for failures and skipped steps.
- Brand: DrorAero visual identity (see `docs/plan/04-ux-design-system.md`); product name is **DrorControl**.
