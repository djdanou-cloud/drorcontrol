---
from: core-2
to: all
type: done
subject: T0-18 core parameter transactions implemented and verified locally
ref: T0-18
created: 2026-10-11T13:17:47Z
---
Implementation: 7774af7; offline snapshots: d91a403. Owner approved the recommended fresh known-disarmed rule: "Yeah, let us go with your recommendation". Fixed 1.5 s heartbeat age; latch arming/staleness during writes. Durable complete baseline, explicit value/type read-back, selected rollback with recovery snapshot, conservative dangerous-parameter refusal, and per-vehicle FS_EKF_THRESH navigation thresholds. Dedicated typed confirmation for dangerous writes remains follow-up; no gRPC/UI surface added. Validation: source /workspace/.cloud-tools/env.sh; DC_SITL_REQUIRED=1 cargo xtask ci => all steps passed, 170 tests passed, 0 failed, one external-corpus test ignored; all nine real SITL scenarios passed including WP_SPD write/rollback through 30% bidirectional loss. Formatting, Clippy and repo-check passed; .venv-docs/bin/mkdocs build --strict passed. Initial full run failed: own documentation server occupied SITL instance port 8000 (stopped it); safety epoch incorrectly included historical gaps during read-only baseline preparation (corrected scope, scripted arming/staleness tests passed). Safety/reviewer review remains required before merge by coordination protocol section 7. Mac checks and hosted CI not run. No push: local commits only per owner instruction.
