# DrorControl

DrorControl is a cross-platform (macOS-first) ground control station for ArduPilot: a Rust core
with a Qt 6 / QML glass-cockpit UI, built for full Mission Planner parity.

| Start here | |
|---|---|
| [Plan](plan/README.md) | Architecture, parity matrix, platform, UX, analytics, docs, roadmap, safety |
| [Decisions](plan/adr/README.md) | Architecture Decision Records (generated index) |
| [MAVLink reference](reference/mavlink/index.md) | Messages and enums of the `ardupilotmega` dialect, generated from the pinned XML |
| [Parity inventory](parity/README.md) | Every Mission Planner feature, mapped to the plan (`parity.yaml`) |
| [Coordination](coordination/README.md) | How agents and humans share work; live [board](coordination/BOARD.md) |

Code: [`crates/`](../crates/) (Rust core), [`ui/`](../ui/) (QML, design tokens),
[`scripts/`](../scripts/) (generators and tools). Build and test instructions are in the
repository [README](../README.md).

This site is built with `mkdocs build --strict` in CI; any warning (dead link, missing anchor,
stale generated page) fails the build.
