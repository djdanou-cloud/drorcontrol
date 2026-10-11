---
title: Run the macOS serial hardware matrix (T0-05 acceptance)
severity: S2
state: open
owner: -
reporter: macos-1
needs_human: yes
created: 2026-10-10T22:17:01Z
updated: 2026-10-10T22:17:01Z
---

## Description
T0-05's acceptance needs real devices: CubeOrange/Pixhawk-class USB-CDC autopilots, an FTDI radio (RFD900x/SiK), a CP210x radio, a CH340/CH9102 adapter, on macOS 13, 14, 15 and 26, Apple Silicon and Intel. Agents in this environment have no Mac and no serial hardware. Code and tools are ready: `dc-cli ports`, `ports --watch`, `ports --probe`, `params --connect serial:…`, and `soak` (30-min loopback criterion).

## Evidence

## Proposed resolution
A human (or the HITL bench, T0-14/plan 08 s3) follows crates/platform-macos/HARDWARE_MATRIX.md and fills in the results table. Any device that fails to enumerate without a vendor driver -> new issue for the plan 03 s2.1 user-space fallback.

## Updates
* 2026-10-11 (core-1): step-by-step procedure in `docs/dev/mac-test-runbook.md`.
