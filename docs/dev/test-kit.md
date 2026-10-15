# Test kit: Windows and macOS

Everything needed for a hands-on test session with a real flight controller, in the order
to do it. Two runbooks hold the steps:

* [Windows bench test](windows-bench-test.md): about 2 hours, one PC and a flight
  controller on USB, nothing flown. Copy [`bench/bench-test.plan`](bench/bench-test.plan)
  next to `dc-cli.exe` for the upload steps.
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

**Build for the next Windows bench test (2026-10-15):** full `ci` run 74 on `main`, commit
`bfbe442`, every job green (Windows checks and tests, both SITL suites). Actions → `ci` →
run #74 → artifact `dc-cli-windows-x86_64` (kept until 2026-10-29; after that, start a new
full run). `dc-cli.exe about` must name `bfbe442`. It has the fixes for the 2026-10-13
bench findings and everything under "Also new: 2026-10-14".

## Before you start

* **Nothing is flown.** **Propellers off** for every step. Only the optional armed steps
  arm the vehicle, on the bench, held or strapped down, and in them you answer "no".
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

## Also new: changing the vehicle behind settings (2026-10-13)

Five settings, all **off** until you turn them on; each shows its danger when you do:

```
dc-cli settings                                  # show them and where the file is
dc-cli settings enable fence-rally-upload        # armed-mission-upload, log-erase,
                                                 # simple-force-arm, simple-param-reset
dc-cli settings disable fence-rally-upload
```

Then, against the flight controller (props off; Windows bench test steps 11–14 and 16 walk
through all of it, with a backup and restore of your own mission, fence and rally points):

* `dc-cli mission upload X.plan --connect …` (disarmed: no setting needed), and
  `--type fence` / `--type rally` (needs `fence-rally-upload`). Each upload is read back
  and compared; it says `verified` or lists the differences.
* `dc-cli log erase --connect …` (needs `log-erase`, vehicle disarmed): erases every log.
  **Download any log you want to keep first.** Wait 15 s after landing (or a failed arming
  attempt): ArduPilot keeps logging that long and would start a new log straight away.
* With a setting off, each of these says which setting to turn on and sends nothing.
* Everything is recorded in `dc-audit.log` (`dc-cli audit-verify --print`).

## Also new: 2026-10-14

Nothing here changes the vehicle unless said so; the bench sheet's steps still apply.

* **Pre-flight:** `dc-cli preflight --connect …` lists link, firmware, the autopilot's own
  pre-arm result, unhealthy sensors, RC, GPS, EKF, home, battery, vibration, mission,
  fence and traffic, PASS/FAIL (advice only). Worth running on the bench: note any item
  that looks wrong.
* **Inspector:** `dc-cli inspect --connect … --seconds 20 [--msg SYS_STATUS]`: every
  message the board sends with its rate (compare with Mission Planner's MAVLink Inspector).
* **Recordings:** `tlog alerts`, `tlog signing [--key-file]`, `tlog inspect`.
* **Fence and RTL:** `telemetry --fence X.plan` shows the margin to the fence;
  `mission rtl-target X.plan --from LAT,LON` says where RTL would go.
* **Terrain:** GeoTIFF tiles (Copernicus GLO-30) work wherever `--dem` does; `telemetry
  --terrain DIR` answers the board's terrain requests (it then loads that terrain:
  only with `TERRAIN_ENABLE` 1 and tiles for your field).
* **Editing:** `mission edit X.plan --out Y.plan insert:… delete:… move:… alt:…`.
* **Units:** `--units imperial` or `nautical` on any command.
* **Over the API:** `daemon` reads the same settings; `remote vehicle-changes`,
  `remote upload`, `remote erase-logs`.

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
