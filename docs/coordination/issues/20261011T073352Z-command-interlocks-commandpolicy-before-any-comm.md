---
title: Command interlocks (CommandPolicy) before any command RPC is exposed
severity: S2
state: resolved
owner: -
reporter: core-1
needs_human: yes
created: 2026-10-11T07:33:52Z
updated: 2026-10-11T07:33:52Z
---

## Description

The external audit (docs/audit/2026-10-11-external-audit.md §3) asked for validation and safety
interlocks on arm, mode and parameter commands. Today no such command path exists: the CLI,
gRPC and FFI are read-only. Plan 08 §4 puts `CommandPolicy` in the core, so the UI is never
the only guard. This is a safety-path change, so it needs approval before anyone builds it.

## Evidence

proto/dc/v1/core.proto exposes no command RPCs. crates/dc-cli/src/main.rs module docs say
"read-only by design". protocols::params::ParamSet exists but is not exposed.

## Proposed resolution

After T0-15 (hazard log) has been reviewed, add a task to implement `CommandPolicy` in
dc-core:
- classes A-D;
- per-command preconditions (e.g. disarmed for calibration and motor test);
- value ranges from ArduPilot parameter metadata;
- link state never blocks a command (ADR-0009): it is shown in the card and drives adaptive
  retransmission with a bounded lifetime;
- single-use confirmation tokens for class C/D;
- an audit log.

No command RPC is exposed before then.

## Updates

- 2026-10-11 (core-1): Product owner: link state must not block commands. Recorded as ADR-0009;
  plan 08 §4.3, the UI skill and this proposal updated.
- 2026-10-11 (core-1): Product owner approved CommandPolicy; build after T0-13. Tracked as backlog
  T0-16 (depends on T0-13). The T0-15 hazard log still needs human review before release.
