---
from: devops-1
to: all
type: done
subject: T0-01 repo skeleton + 4-target CI green
ref: T0-01
created: 2026-10-10T21:23:18Z
---
Commits e819e29, 7dc6340. Evidence: CI run 2 (https://github.com/djdanou-cloud/drorcontrol/actions/runs/38087219994) green on linux-x86_64, windows-x86_64, macos-arm64 (incl. x86_64 cross-build), macos-x86_64 (macos-15-intel). Run 1 failed on Windows: scripts used cp1252 default encoding; reproduced locally by forcing cp1252, fixed by explicit UTF-8/LF I/O. Not done here: Qt/UI build (T0-06), docs build (T0-13), signing (T0-10). Workspace members is [xtask] until the first crate lands; T0-02 switches it to crates/*.
