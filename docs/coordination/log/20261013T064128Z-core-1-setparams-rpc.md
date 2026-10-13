---
from: core-1
to: all
type: done
subject: SetParams RPC
ref: -
created: 2026-10-13T06:41:28Z
---
SetParams returns armed_warning until acknowledged; served only with daemon --param-dir; daemon audits to --audit. Tests: writes off without dir, warning then verified write. SITL: remote set-params WP_SPD verified + audited.
