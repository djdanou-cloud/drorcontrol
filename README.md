# DrorControl

Next-generation, cross-platform ArduPilot ground control station — native macOS and Windows, glass-cockpit UX, headless Rust core.

**Start here:** [`docs/plan/README.md`](docs/plan/README.md) — the master development plan (architecture, parity matrix, macOS layer, design system, analytics, documentation system, roadmap, V&V, ExpressLRS/Crossfire link support).

**Taking over the work (agents and people):** [`docs/coordination/HANDOVER.md`](docs/coordination/HANDOVER.md) — current state, open decisions, next tasks. Safety: [`docs/safety/hazards.md`](docs/safety/hazards.md). Mac-only tests: [`docs/dev/mac-test-runbook.md`](docs/dev/mac-test-runbook.md).

## Quick start (developers)

```bash
rustup toolchain install     # installs the pinned toolchain from rust-toolchain.toml
cargo xtask ci               # what CI runs: fmt-check, clippy -D warnings, tests, repo hygiene
cargo xtask help             # all developer tasks (`just` recipes wrap the same commands)
```

Talk to a simulated vehicle (ArduPilot SITL, built from source; ~5 min the first time):

```bash
scripts/build_sitl.sh                    # -> .sitl/ardupilot-Copter-4.7.1/build/sitl/bin/arducopter
(cd /tmp && <that binary> --model + --defaults <ardupilot>/Tools/autotest/default_params/copter.parm -I0 &)
cargo run -p dc-cli -- params --connect tcp:127.0.0.1:5760 --out copter.param
cargo run -p dc-cli -- telemetry --connect tcp:127.0.0.1:5760 --seconds 10
cargo run -p dc-cli -- daemon --connect tcp:127.0.0.1:5760      # gRPC API (proto/dc/v1/core.proto)
cargo run -p dc-cli -- remote vehicles                           # gRPC client
```

CI runs `cargo xtask ci` on Linux x86_64, Windows x86_64, macOS arm64 and macOS x86_64 (`.github/workflows/ci.yml`).

Docs site (MkDocs Material: plan, ADRs, generated MAVLink reference). CI builds it with
`mkdocs build --strict`, so any warning fails:

```bash
python3 -m venv .venv-docs && .venv-docs/bin/pip install -r docs/requirements.txt
.venv-docs/bin/mkdocs serve              # http://127.0.0.1:8000
python3 scripts/gen_adr_index.py         # after adding an ADR (CI checks the index is current)
```

## Repository layout

| Path | Contents | Owner (see `docs/coordination/README.md` §4) |
|------|----------|------------------------------------------|
| `crates/` | Rust core crates (`dc-core`, codec, router, protocols, logs, platform …) | per workstream |
| `ui/` | Qt 6 / QML shell; `ui/tokens/` design tokens; `ui/spikes/` Phase 0 spikes (S0-1 bridge, S0-3 map, S0-5 video) | WS-UI / WS-GFX / WS-UX |
| `packaging/macos/` | Ad-hoc signed universal app, zip + dmg (ADR-0010) | WS-MACOS |
| `docs/safety/` | Hazard log (MIL-STD-882E) | WS-SAFETY |
| `xtask/` | Developer task runner (`cargo xtask`) | WS-DEVOPS |
| `scripts/` | Coordination and generator scripts | WS-DEVOPS |
| `docs/plan/` | Master development plan and seed ADRs | WS-ARCH |
| `docs/coordination/` | Agent/human coordination protocol, board, backlog | all |
| `.claude/skills/` | Agent skills (source of truth for the Cursor/Copilot/Gemini mirrors) | WS-UX |

