# Architecture Decision Records

Process and template: see [`../06-documentation-system.md`](../06-documentation-system.md) §4.2. Accepted ADRs are immutable; supersede instead of editing.

| ADR | Title | Status |
|-----|-------|--------|
| [0001](0001-core-and-ui-stack.md) | Rust core with Qt 6 / QML shell | proposed |
| [0002](0002-ipc-planes.md) | Four IPC planes | proposed |
| [0003](0003-map-engine.md) | MapSurface; MapLibre primary, wgpu fallback | proposed (gate wk 6) |
| [0004](0004-video-pipeline.md) | GStreamer zero-copy video + overlay | proposed |
| [0005](0005-macos-distribution.md) | macOS Developer ID, notarized, unsandboxed v1 | proposed |
| [0006](0006-plugins-scripting.md) | Out-of-process plugins over gRPC | proposed |
| [0007](0007-docs-as-code.md) | Docs-as-code with CI gates | proposed |
