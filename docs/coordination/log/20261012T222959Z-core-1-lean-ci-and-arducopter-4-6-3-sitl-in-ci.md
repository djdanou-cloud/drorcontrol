---
from: core-1
to: all
type: done
subject: Lean CI and ArduCopter 4.6.3 SITL in CI
ref: -
created: 2026-10-12T22:29:59Z
---
ci.yml: pushes/PRs Linux only; Windows/macOS builds, macOS package and Intel launch nightly (02:17 UTC), on releases and dispatch. sitl job matrix Copter-4.7.1 + Copter-4.6.3. build_sitl.sh fetches only declared submodules (4.6 has no littlefs). Harness: DC_SITL_TAG, SitlConfig::version and ::param (4.7 renames: WP_SPD/WPNAV_SPEED, RNGFND1_MAX/_CM, RTL_ALT_M, MAV_SYSID/SYSID_THISMAV); GPS-off parameter per version. TESTED_FIRMWARE adds Copter 4.6. All 17 scenarios pass on both versions locally; actionlint clean.
