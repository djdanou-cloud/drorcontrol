---
title: Run spike S0-1 on macOS (Metal) to close T0-06 frame-time criterion
severity: S3
state: open
owner: -
reporter: ui-1
needs_human: yes
created: 2026-10-10T22:32:50Z
updated: 2026-10-10T22:32:50Z
---

## Description
Spike S0-1 (ui/spikes/s0-1-bridge) ran on Linux with software OpenGL: 200k msg/s ingest, 0 frames >25 ms, bridge read ~5 us p99 26 us independent of load; frame-interval p99 17.7 ms (>16.7 ms) attributed to software rendering on oversubscribed shared vCPUs. The ADR-0001 acceptance needs the same run on a Mac (Metal via Qt RHI), Apple Silicon and ideally Intel, Qt 6.5+.

## Evidence

## Proposed resolution
Someone with a Mac runs the commands in ui/spikes/s0-1-bridge/RESULTS.md ("Not yet measured") and appends the JSON numbers there. GitHub macOS runners may not expose a GPU, so a physical Mac is preferred.

## Updates
* 2026-10-11 (core-1): step-by-step procedure in `docs/dev/mac-test-runbook.md`.
