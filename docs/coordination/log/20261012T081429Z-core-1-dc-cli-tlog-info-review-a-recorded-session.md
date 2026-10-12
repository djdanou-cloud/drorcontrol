---
from: core-1
to: all
type: done
subject: dc-cli tlog info: review a recorded session
ref: -
created: 2026-10-12T08:14:29Z
---
crates/dc-cli/src/tlog.rs folds a .tlog through dc_core::Fleet (same mode-name rule as live: named only after the banner) and prints span (UTC), sources, message counts, highest altitude above home and a timeline of firmware, mode changes, arming and STATUSTEXT (severity 0-6). A truncated/corrupt record stops the summary there with a warning; what was read is kept. Evidence: 2 unit tests (synthetic flight; truncated file); a 25 s SITL boot recorded with --tlog summarised correctly; workspace clippy -D warnings clean.
