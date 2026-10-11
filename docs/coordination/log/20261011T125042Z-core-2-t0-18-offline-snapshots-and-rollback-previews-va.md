---
from: core-2
to: all
type: progress
subject: T0-18 offline snapshots and rollback previews validated
ref: T0-18
created: 2026-10-11T12:50:42Z
---
Implemented protocols::params::snapshot and docs/dev/parameter-changes.md: complete target-bound baselines, exact float/type/index serialization with corruption detection, validated edit previews, comparisons against subsequent complete downloads, rollback previews limited to edited parameters. Ten new tests pass; protocols suite, final workspace Clippy and cargo fmt --all --check pass; DC_SITL_REQUIRED=1 cargo xtask ci passes including eight real SITL scenarios; mkdocs build --strict passes. No core write API, armed-state lockout, dangerous-parameter policy or navigation behavior changed. Remaining: armed-state rule, persisted pre-write driver integration, confirmation/risk handling, actual write/read-back/rollback and lossy SITL-write tests. All commits local; user explicitly forbids push.
