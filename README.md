# DrorControl

Next-generation, cross-platform ArduPilot ground control station — native macOS and Windows, glass-cockpit UX, headless Rust core.

**Start here:** [`docs/plan/README.md`](docs/plan/README.md) — the master development plan (architecture, parity matrix, macOS layer, design system, analytics, documentation system, roadmap, V&V).

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

## Repository layout

| Path | Contents | Owner (see `docs/coordination/README.md` §4) |
|------|----------|------------------------------------------|
| `crates/` | Rust core crates (`dc-core`, codec, router, protocols, logs, platform …) | per workstream |
| `ui/` | Qt 6 / QML shell | WS-UI / WS-GFX / WS-UX |
| `xtask/` | Developer task runner (`cargo xtask`) | WS-DEVOPS |
| `scripts/` | Coordination and generator scripts | WS-DEVOPS |
| `docs/plan/` | Master development plan and seed ADRs | WS-ARCH |
| `docs/coordination/` | Agent/human coordination protocol, board, backlog | all |
| `.claude/skills/` | Agent skills (source of truth for the Cursor/Copilot/Gemini mirrors) | WS-UX |

