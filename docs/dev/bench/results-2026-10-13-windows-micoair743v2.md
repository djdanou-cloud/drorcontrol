# Windows bench test results, 2026-10-13

First run of [the Windows bench test](../windows-bench-test.md) on real hardware. Nothing was
flown; the vehicle stayed on the desk.

| | |
|---|---|
| Board | MicoAir743v2, USB `1209:5741`, COM20 |
| Firmware | ArduCopter V4.6.3 (`3fc7011a`), QUAD/X_REV, DShot600 |
| PC | Windows 11 Home 10.0.26200, x86_64 |
| dc-cli | local release build of `44fb5cc` (Rust 1.97.0, MSVC), not the CI artifact |
| Sensors | no GPS fix, no RC receiver, battery on for part of the earlier log only |

Commands that write were run with `--yes`: the session had no terminal, and dc-cli refuses to
write without one (`error: no terminal to confirm on: pass --yes`). The questions themselves
(steps 9, 12–14) are still to be checked by hand.

## Results by step

| Step | Result |
|---|---|
| 1 Ports | Pass. `ports` lists the board with its hint; `--probe` finds a HEARTBEAT at 115200 baud, sysid 1. `--watch` replug not done yet |
| 2 Telemetry | Pass. Banner, attitude, 0.0 % loss over 60 s; `bench.tlog` 104 kB. Tilt and 5 s unplug not done yet |
| 3 Parameters | Pass. 1491 parameters in 11.8 s; `param-get` answers at once; `params-review` flags ARMING_CHECK (partly off) and BATT_FS_LOW_ACT 0. No Mission Planner `.param` to diff against |
| 4 Mission, fence, rally | Pass. All three empty on the vehicle; `mission check` reads the empty file |
| 5 Logs | Pass. 2 logs (dates 1980-01-01: no GPS time); both downloaded complete at about 117 kB/s (21.6 MB in 184 s, 18.7 MB in 160 s) |
| 6 Log analysis | Mostly pass, see findings 3, 4 and 7. The log is a 22 s armed bench session, not a flight |
| 7 Radio | Not done (no radio) |
| 8 Recording | Ran; see finding 5 |
| 9 Parameter write and rollback | Pass. DISARM_DELAY 10 → 12 → 10, each verified by read-back; compares as expected; audit chain intact |
| 10 Armed warning | Not done (the vehicle cannot arm: no RC, battery below arming voltage) |
| 11–13 Mission, fence, rally upload and restore | Pass. Fence refused while the setting was off; mission 6, fence 7, rally 2 items verified; the empty backups restored with `0 items (clears it on the vehicle)`, verified |
| 13b Armed refusal | Not done (as step 10) |
| 14 Log erase | Pass with finding 6. Refused while the setting was off; erase verified; last `log list` shows 0 logs |
| 15 Sanitize | Pass. No position fix, so position fields are zeroed; `log summary` identical before and after |
| 16 Settings off, audit | Pass. All three settings off; `audit-verify` 22 records, chain intact |

## Findings

1. **No terminal, no confirmation.** Without a terminal every write needs `--yes`. That is
   the safe behaviour, but the sheet does not say so, and an agent or script running the
   sheet never sees the questions.
2. **Firmware shown as beta.** `telemetry` shows `fw 4.6.3-beta` while the banner says
   `ArduCopter V4.6.3`: AUTOPILOT_VERSION reports `FIRMWARE_VERSION_TYPE` 128. Probably the
   board's own build; worth checking against the official 4.6.3 binary for this board.
3. **`log fft --field ACC.AccZ` fails on 4.6.3** (`no ACC.AccZ for IMU 0`): this log has
   `IMU.AccZ` and no `ACC` messages. `--field IMU.AccZ` works (peak 39.8 Hz). The sheet's
   step 6 command should use `IMU.AccZ`, or the tool should fall back to it.
4. **`log landings` says `not disarmed in this log`** while `log events` shows DISARMED at
   47.584 s, the same timestamp as LAND_COMPLETE (touchdown 47.6 s). A disarm at the
   touchdown's own time seems to be missed.
5. **`tlog health` on a disarmed recording reports WARNING**, from two PreArm STATUSTEXTs
   (severity 2), listed both as `arming` CAUTION and as `error` WARNING. The sheet says a
   disarmed recording reports `no checks apply`.
6. **Erase result contradicts the next list.** `log erase` printed `the vehicle started 2 new
   log(s) after the erase (normal after a flight)` and the audit log recorded `2 new logs
   listed`; the `log list` 8 s later showed 0 logs. There was no flight.
7. Smaller:
   * `param-set` and `param-rollback` print Rust debug text:
     `DISARM_DELAY Verified(Param { value: 12.0, ty: 2, index: 65535 })`.
   * `log notch` suggests a 23 Hz throttle notch from 19 s of a bench session, against the
     configured 120 Hz; nothing says the log may be too short or not a flight.
   * `log takeoffs` reports a liftoff with a 0.00 m/s climb (the land detector on the bench).
   * `log export --kml` stops with `no POS records with a position`: expected without GPS.

## Still to do on this bench

* Step 1 `ports --watch` replug, step 2 tilt and 5 s unplug.
* The questions of steps 9 and 12–14, answered by hand in a terminal.
* Steps 3, 4 and 6 against Mission Planner's files, and step 6 on a real flight log with GPS.
* Optional steps 7, 10 and 13b.

The raw logs, recordings and console output stay with the owner: they are over the 1 MiB
limit and include the board's unique ID (see [the corpus rules](../../corpus/README.md)).
