# SITL harness

`crates/sitl-harness` runs `dc-core` against a real ArduPilot SITL (plan 08 §2, T0-14).

| Part | What it does |
|---|---|
| `Sitl` | Starts the pinned `arducopter` (Copter-4.7.1) headless in a temp dir with a wiped EEPROM; reads the TCP port from SITL's own log; kills it on drop |
| `chaos::ChaosProxy` | Frame-aware TCP proxy: drop, duplicate, corrupt, delay and jitter (reorders) per frame from a logged seed; `blackout` (radio fade) and `cut` (connection drop) |
| `tests/scenarios.rs` | 17 scenarios (incl. core-2's parameter write/rollback): connect + telemetry (+ landed state ON_GROUND); full parameter download; download through 30 % frame loss both ways (must complete); fade → STALE → LOST → CONNECTED; reconnect after a cut; firmware banner + named single-action mode change verified (ADR-0011); GPS failure → position UNTRUSTED warning (H16); mission upload through 20 % loss verified by read-back (H4); a real flight: GUIDED, arm and take off as single actions, a radio blackout during which the GCS shows LOST and the vehicle's own GCS failsafe flies RTL, then CONNECTED with the mode shown as RTL, landing and auto-disarm; an AUTO mission flight (take off, two waypoints, RTL) whose progress (`MISSION_CURRENT`, `MISSION_ITEM_REACHED`) the GCS follows item by item to MISSION_STATE_COMPLETE after landing; geofence (polygon, return point, circle) and a rally point read back through the read-only download (set up with a raw upload: the product has no fence/rally upload); ADS-B traffic from SITL's simulated receiver (`SitlConfig::params`/`args`: `--serial4=sim:adsb`) reported with callsigns and the simulated flag; the vehicle's own dataflash log (LOG_DISARMED=1, in the instance's `logs/`) gives a track with GPS-clock UTC and runs the log health checks; log download over MAVLink through 10 % loss, byte-identical to SITL's file; SITL's simulated rangefinder reported with its configured range; a generated survey grid uploads and verifies |

## Running

```sh
scripts/build_sitl.sh                      # once; builds .sitl/ardupilot-Copter-4.7.1
cargo test -p sitl-harness -- --nocapture  # ~65 s; prints timings and the chaos seed

# The owner's fleet version (ADR-0012); CI runs both:
scripts/build_sitl.sh .sitl Copter-4.6.3 copter
DC_SITL_TAG=Copter-4.6.3 cargo test -p sitl-harness -- --nocapture
```

Without a SITL binary the scenarios skip. `DC_SITL_TAG` picks the build under `.sitl/` and tells
the scenarios which version they test (`SitlConfig::version`; `SitlConfig::param` maps the
parameters 4.7 renamed, e.g. `WP_SPD` to 4.6's `WPNAV_SPEED` in cm/s). Set `DC_SITL_BIN` /
`DC_SITL_DEFAULTS` to use another build, and `DC_SITL_REQUIRED=1` to make a missing binary a
failure (CI does). The `sitl` CI job runs every scenario on ArduCopter 4.7.1 and 4.6.3 and
caches each built binary keyed on `scripts/build_sitl.sh` and the tag.

## Notes

* SITL keeps the `-I` instance in 8 bits: ports wrap above instance 255. The harness uses
  instances 1–240 and trusts the port SITL logs.
* The 30 % loss download takes ~60 s: the parameter protocol re-requests missing indices in
  batches of 10 after 0.5 s of silence. Tuning the batch for lossy links is a follow-up.
* The fade scenario found a watchdog defect (a LOST shorter than one 100 ms tick was not
  reported); fixed in dc-core with a regression test.
* The navigation scenario waits for the EKF to converge (~40 s) and fails the GPS with
  `SIM_GPS1_ENABLE=0` (`PARAM_SET`).
* The flight scenario sets `FS_GCS_ENABLE=1` with a raw `PARAM_SET` (test configuration;
  the product's guarded write path refuses failsafe parameters by design) and takes ~90 s.
* SITL waits for a client on its SERIAL0 TCP port before it starts; a manual run needs one
  connected (e.g. `dc-cli telemetry`).
* The AUTO scenario climbs to 10 m in GUIDED before switching to AUTO, so it does not depend
  on how Copter starts an AUTO take-off from the ground (pilot throttle, `AUTO_OPTIONS`); the
  harness has no RC input. At completion
  ArduPilot reports item 0 with MISSION_STATE_COMPLETE.
* Not yet covered from plan 08 §2: other failsafes (RC, battery, EKF,
  fence), calibration, logs and robustness suites; fence/rally upload (download is covered); other vehicles; the YAML scenario DSL; nightly/weekly runs.
