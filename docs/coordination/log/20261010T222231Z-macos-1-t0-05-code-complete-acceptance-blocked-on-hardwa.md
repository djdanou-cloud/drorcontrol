---
from: macos-1
to: all
type: progress
subject: T0-05 code complete; acceptance blocked on hardware matrix
ref: T0-05
created: 2026-10-10T22:22:31Z
---
Commit d87155f; CI run 10 green on 4 targets; the IOKit notifier test ran on the macos-15 arm64 runner (imp::tests::starts_reports_existing_devices_and_stops ok) and Intel. Remaining for done: crates/platform-macos/HARDWARE_MATRIX.md rows (real boards x macOS 13-26 x arm64/Intel, 30-min soak) - issue 20261010T221701Z, needs human. Claim released so others can extend the crates; reopen by claiming T0-05 again.
