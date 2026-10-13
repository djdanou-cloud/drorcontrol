# Test kit: Windows and macOS

Everything needed for a hands-on test session with a real flight controller, in the order
to do it. Two runbooks hold the steps:

* [Windows bench test](windows-bench-test.md): about 1½ hours, one PC and a flight
  controller on USB.
* [Mac test session](mac-test-runbook.md): the Phase 0 platform tasks (serial, UI bridge,
  map, video, first launch) and the same flight-controller checks as on Windows.

## Builds

Pushes to `main` build and test on Linux only. The Windows and macOS builds come from a
**full** `ci` run: nightly (02:17 UTC), on a release, or started by hand (Actions → `ci` →
"Run workflow" on `main`). Use the newest green full run. The `dc-cli` artifacts are kept
for 14 days, the macOS package for 30.

| Artifact | For | Contents |
|---|---|---|
| `dc-cli-windows-x86_64` | Windows bench test | `dc-cli.exe` (unsigned: SmartScreen asks once) |
| `dc-cli-macos-arm64`, `dc-cli-macos-x86_64` | Mac flight-controller checks | `dc-cli` |
| `drorcontrol-macos` | Mac T0-10 first launch | DrorControl.app (ad-hoc signed) as `.dmg` and `.zip`, with `SHA256SUMS` |

Check that the build is the one you meant: `dc-cli about` prints the commit it was built
from.

## Before you start

* **Propellers off** for every step. Only the optional in-flight-warning step arms the
  vehicle, on the bench, held or strapped down.
* Close Mission Planner, QGroundControl and anything else that could hold the serial port.
* Have to hand: a real flight's `.bin` log (ideally with a hover), your usual `.param`
  file and mission, and a telemetry radio pair if you want the range step.

## What is new to try (since 2026-10-12)

* **Parameter writes** (ADR-0012): `param-set` (one confirmation; if the vehicle is armed a
  warning and a second confirmation), `param-set --file F.param` (only real differences),
  `param-rollback --baseline B --changed`, `param-get`, and the audit trail
  (`audit-verify --print`). Over the API: `daemon --param-dir DIR` and
  `remote set-params`.
* **Log analysis**: `log report` (also `--json`), `log health --json --fail-on warning`,
  `log failsafes`, `log takeoffs`, `log landings`, `log attitude` (rate oscillation),
  `log spectrogram`, `log notch --response F.csv`, `log events --csv F`, and new health
  checks (EKF status and resets, compass, IMU and barometer agreement, power flags,
  arming-check failures).
* **Telemetry recordings**: `tlog health`, `tlog link` (range tests), `tlog index --at S`,
  `tlog export --kml` (track by flight mode with the mission).
* **Sharing safely**: `log sanitize`, `tlog sanitize`, `log trim`.

## Also new: mission planning (offline, 2026-10-13)

None of these talk to the vehicle; they make and check files to load as usual.

* **QGroundControl `.plan` files** everywhere a mission is read or written, and
  `mission convert`; `mission export --csv`.
* **From Google Earth:** draw a path, polygons and points, save as KML, then
  `mission from-kml`, `mission fence-from-kml` (name a polygon "no-fly" for an exclusion
  zone) and `mission rally-from-kml`; `mission export X.plan --kml` goes the other way.
* **Patterns:** `mission corridor`, `mission search --pattern square|sector`.
* **Terrain:** put SRTM `.hgt` tiles for your field in a folder, then
  `terrain elevation`, `mission check --dem DIR`, `mission profile --dem DIR` and
  `mission terrain-follow --dem DIR --agl A`.
* **Positions:** `dc-cli coord LAT,LON` prints decimal, DMS, DDM, UTM and MGRS.

## What to send back

From each session: the console output of every step, `dc-cli about`, `report.txt`,
`health.json`, `dc-audit.log` and the parameter files the runbook names. Logs and
recordings **only as sanitized copies** (`log sanitize`, `tlog sanitize`): field logs with
real positions never go into the repository (ADR-0012).
