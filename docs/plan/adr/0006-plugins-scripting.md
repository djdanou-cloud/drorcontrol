# ADR-0006: Out-of-process plugins and scripts over the public gRPC API

* Status: proposed
* Date: 2026-10-10

## Decision
No in-process scripting host (legacy IronPython model). Scripts/plugins are separate processes using generated clients (Python, Rust, TypeScript; Lua host), under a supervisor with a permission manifest. Command-sending plugins pass through the same `CommandPolicy` and confirmation tokens as the UI. UI extensions are declarative QML panels from signed bundles.

## Consequences
+ Crash isolation, language neutrality, auditable permissions, usable in CI. − Slightly higher latency for tight loops (acceptable; documented).
