# SITL harness

`crates/sitl-harness` runs `dc-core` against a real ArduPilot SITL (plan 08 §2, T0-14).

| Part | What it does |
|---|---|
| `Sitl` | Starts the pinned `arducopter` (Copter-4.7.1) headless in a temp dir with a wiped EEPROM; reads the TCP port from SITL's own log; kills it on drop |
| `chaos::ChaosProxy` | Frame-aware TCP proxy: drop, duplicate, corrupt, delay and jitter (reorders) per frame from a logged seed; `blackout` (radio fade) and `cut` (connection drop) |
| `tests/scenarios.rs` | Connect + telemetry; full parameter download; download through 30 % frame loss both ways (must complete); fade → STALE → LOST → CONNECTED; reconnect after a cut |

## Running

```sh
scripts/build_sitl.sh                      # once; builds .sitl/ardupilot-Copter-4.7.1
cargo test -p sitl-harness -- --nocapture  # ~65 s; prints timings and the chaos seed
```

Without a SITL binary the scenarios skip. Set `DC_SITL_BIN` / `DC_SITL_DEFAULTS` to use another
build, and `DC_SITL_REQUIRED=1` to make a missing binary a failure (CI does). The `sitl` CI job
caches the built binary keyed on `scripts/build_sitl.sh` and the tag.

## Notes

* SITL keeps the `-I` instance in 8 bits: ports wrap above instance 255. The harness uses
  instances 1–240 and trusts the port SITL logs.
* The 30 % loss download takes ~60 s: the parameter protocol re-requests missing indices in
  batches of 10 after 0.5 s of silence. Tuning the batch for lossy links is a follow-up.
* The fade scenario found a watchdog defect (a LOST shorter than one 100 ms tick was not
  reported); fixed in dc-core with a regression test.
* Not yet covered from plan 08 §2: mission, flight, failsafe, calibration, logs, robustness
  suites; other vehicles; the YAML scenario DSL; nightly/weekly runs.
