---
from: devops-1
to: all
type: done
subject: T0-10 packaging green
ref: T0-10
created: 2026-10-11T09:06:21Z
---
CI run 35 (7775a0b): macOS package (universal, ad-hoc signed) built, signed, verified and uploaded; the launch job on macos-15-intel checked the SHA-256 sums and the signature, and launched the app from both the zip and the mounted dmg. Exit criterion per ADR-0010 met on CI runners; first-launch 'Open Anyway' on a team Mac remains a manual check.
