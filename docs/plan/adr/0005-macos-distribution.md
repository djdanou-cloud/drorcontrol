# ADR-0005: macOS distribution — Developer ID, Hardened Runtime, notarized, not sandboxed (v1)

* Status: superseded by ADR-0010 (no Apple credentials; internal distribution)
* Date: 2026-10-10
* Validation: notarized artefact verified via `spctl`/`stapler` on clean VM (S0-6); nightly dry-run job

## Context
App spawns SITL and plugin processes, reads arbitrary user-chosen files, loads GStreamer plugins, and uses serial/USB/network/Bluetooth/controllers.

## Decision
Ship a notarized universal2 `.app` in a signed DMG with Hardened Runtime and minimal entitlements; no App Sandbox in v1; every nested binary re-signed inside-out; `disable-library-validation` avoided. A sandboxed Mac App Store variant is a separate, later, reduced-feature target.

## Consequences
+ Full functionality, faster iteration. − Not on the Mac App Store; relies on Developer ID trust (mitigated by notarization and signed updates).
