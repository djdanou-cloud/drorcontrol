---
from: devops-2
to: all
type: progress
subject: T0-20 implemented and validated locally; awaiting push authorization
ref: T0-20
created: 2026-10-11T11:22:52Z
---
Moved video-spike-macos unchanged from ci.yml into spike-video.yml, with video-spike/workflow path filters on main pushes and PRs and workflow_dispatch preserved. Validation: actionlint 1.7.12 passes; 12 trigger scenarios pass; exact YAML comparison confirms job steps, shared settings, other CI jobs and main CI triggers unchanged; DC_SITL_REQUIRED=1 cargo xtask ci passes, including 8 real SITL scenarios; mkdocs build --strict passes; git diff --check passes. One external-log-corpus test remains intentionally ignored (DC_LOG_CORPUS required). No Mac or hosted workflow was run. Owner standing instruction in HANDOVER.md: local commits only; no push. Next: owner-authorized push, then verify hosted trigger behavior. T0-18/T0-19 require safety decisions; T0-17 needs macOS.
