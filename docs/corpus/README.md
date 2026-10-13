# Test corpus

Real-world inputs that tests run against (roadmap D0-7). Every entry records where it came
from, who agreed to its use, and what it reveals. Add a row here before committing a new
fixture. Anything larger than the repository's 1 MiB file limit stays out of git: commit a head
cut with `scripts/dataflash_vectors.py` and check the whole file locally with the opt-in test
(see below).

| Fixture | What | Source and consent | Used by |
|---|---|---|---|
| `crates/logs/tests/data/sitl-copter-4.7.1-head.{bin,json}` | First 900 kB of an ArduCopter 4.7.1 SITL log, with pymavlink reference values | Generated here from SITL (`scripts/build_sitl.sh`). No real vehicle. | `logs` pymavlink cross-check, corruption test |
| `crates/logs/tests/data/copter-4.6.2-matekh743-head.{bin,json}` | First 900 kB of log `00000083.BIN`: ArduCopter V4.6.2 on a MatekH743-bdshot (quad X, DShot600, CRSF, optical flow, rangefinder), with pymavlink reference values | Provided by the repository owner on 2026-10-11 for use as test data in this repository | `logs` pymavlink cross-check, `params_vs_file` |
| `crates/dc-cli/tests/data/sitl-copter-4.7.1-auto.tlog` | 136 s telemetry recording (245 kB) of the SITL AUTO scenario: GUIDED takeoff, 4-item mission, RTL, landing, recorded by the core with `DC_SITL_TLOG` | Generated here from SITL (`auto_mission_flight_reports_progress_to_landing`). No real vehicle; the position is ArduPilot's simulated home (Canberra). | `dc-cli` `tlog_fixture` tests (info, health, index, link, sanitize, KML) |
| `crates/protocols/tests/data/u3_whoop_260725.param` | Mission Planner `.param` file, 1551 parameters, same airframe saved at another time | Provided by the repository owner on 2026-10-11 for use as test data in this repository | `params_vs_file` |

## What the owner-provided files reveal

* No location: the vehicle has no GPS (`GPS1_TYPE` 0) and the log has no GPS or position
  records.
* Hardware identity: the log's boot messages include the flight controller's MCU unique ID
  (`MatekH743-bdshot 0033…`). It also embeds the board's `@SYS/storage.bin` (parameter
  storage), `@ROMFS/hwdef.dat`, and the `uarts`/`threads`/`memory` reports as `FILE` records.
* The repository is private. Review these files with the owner before making it public.

## Findings from the owner-provided files

* The whole log (12.4 MiB, 297 519 records) matches pymavlink `DFReader`: every message count,
  every numeric column's count, min, max, first and last value, and 3104 sampled records. It
  indexes in 0.02 s. The last record is cut 4 bytes short, as when power is removed while
  logging. Both parsers drop it.
* The `.param` file and the log's `PARM` records are different configurations: 150 values
  differ (e.g. `AHRS_ORIENTATION` 0 → 8, `SCHED_LOOP_RATE` 400 → 800, a different second
  barometer). 17 parameters are only in the file (`QUIK_*`, `RELAY1_*`) and 14 only in the log
  (`INS_HNTC2_*`, `SID_*`). The test pins these numbers, so it also exercises the
  vehicle-vs-file compare (parity CT-004/CT-005). Try it with
  `dc-cli params-diff <file.param> <log.bin>`.

## Checking a full log that is not committed

```sh
.venv/bin/python scripts/dataflash_vectors.py big.BIN --name big --max-bytes 0 --out ~/dc-corpus
DC_LOG_CORPUS=~/dc-corpus cargo test -p logs -- --ignored
```
