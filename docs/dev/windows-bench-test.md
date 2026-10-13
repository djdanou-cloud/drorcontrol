# Windows bench test (real flight controller)

One session on a Windows PC with a flight controller on USB checks what the simulator
cannot:
* Windows serial ports;
* real sensors;
* real firmware on real hardware (CI covers ArduCopter 4.6.3 and 4.7.1 in simulation only).

Plan on about one and a half hours.

Steps 1–8 only **read** from the vehicle: nothing arms it, uploads to it, or erases anything.
The only thing they send besides read requests is `MAV_CMD_SET_MESSAGE_INTERVAL`
(telemetry rates for this connection; not saved to the vehicle). Step 9 **writes
parameters** (ADR-0012) and undoes the change again; step 10 is optional and needs the
vehicle armed on the bench with the propellers off.

## Safety

* **Remove the propellers.** Bench only: the vehicle stays on the desk, disarmed.
* Leave the battery disconnected unless a step asks for it (USB power is enough for most
  steps).
* Close Mission Planner, QGroundControl and anything else that could hold the COM port.

## What to bring

| Item | Needed for |
|------|-----------|
| Windows 10 or 11 PC (x86_64) | All steps |
| Flight controller with ArduPilot (e.g. Copter 4.6.3), data-capable USB cable | All steps |
| A dataflash log from a real flight (`.bin`), ideally including a hover | Steps 6 and 11 |
| Telemetry radio pair (SiK or similar) | Optional step 7 |
| The Mission Planner `.param` file and `.waypoints` mission you normally use | Steps 3 and 4 |

## Get dc-cli

Use one of these:
* **CI build (no compiler needed):** on GitHub, open the latest green `ci` run on `main`
  that was started by hand or by the nightly schedule (pushes build Linux only; see
  [the test kit](test-kit.md)). Download the `dc-cli-windows-x86_64` artifact and unzip
  `dc-cli.exe`. Run `.\dc-cli.exe about` and keep its output: it names the commit. It is unsigned, so
  Windows SmartScreen may ask once ("More info", then "Run anyway").
* **Build it yourself:** install Rust (rustup.rs), then run
  `cargo build --release -p dc-cli` in the repository. The tool is
  `target\release\dc-cli.exe`.

Run the commands below in PowerShell, in the folder containing `dc-cli.exe`. Replace `COM3`
with your port.

## Steps

1. **Find the port.** Plug in the flight controller.
   ```
   .\dc-cli.exe ports
   .\dc-cli.exe ports --probe COM3
   .\dc-cli.exe ports --watch 30
   ```
   Expect:
   * `ports` lists the board with a hint (vendor/product);
   * `--probe` reports a HEARTBEAT, the baud rate and sysid 1;
   * during `--watch`, unplugging and replugging prints remove and attach events.

2. **Live telemetry, recorded.**
   ```
   .\dc-cli.exe --tlog bench.tlog telemetry --connect serial:COM3:115200 --seconds 60
   ```
   Expect:
   * the firmware banner (version and board);
   * attitude changing when you tilt the board;
   * GPS (if fitted and outdoors), EKF health, battery (with the battery connected), RC inputs
     (with the transmitter on);
   * the link line showing no packet loss over USB.

   Unplug the cable for 5 s and plug it back in. The link should drop and recover by itself.

3. **Parameters.**
   ```
   .\dc-cli.exe params --connect serial:COM3:115200 --out vehicle.param
   .\dc-cli.exe params-diff vehicle.param your-mission-planner.param
   ```
   Expect the full count (typically 900–1 500 for Copter) and no differences other than ones
   you know about. Then the quicker checks:
   ```
   .\dc-cli.exe param-get --connect serial:COM3:115200 FS_THR_ENABLE BATT_FS_LOW_ACT
   .\dc-cli.exe params-review --connect serial:COM3:115200
   ```
   `param-get` answers in about a second without a full download; `params-review` lists
   the safety nets your configuration switches off (advice only).

4. **Mission, fence and rally (download only).**
   ```
   .\dc-cli.exe mission download --connect serial:COM3:115200 --out vehicle.waypoints
   .\dc-cli.exe mission download --connect serial:COM3:115200 --type fence
   .\dc-cli.exe mission download --connect serial:COM3:115200 --type rally
   .\dc-cli.exe mission compare your-mission.waypoints --connect serial:COM3:115200
   .\dc-cli.exe mission check vehicle.waypoints
   ```
   Expect `compare` to pass if the vehicle holds that mission. `check` lists the items and any
   lint findings.

5. **Logs on the vehicle (vehicle disarmed).**
   ```
   .\dc-cli.exe log list --connect serial:COM3:115200
   .\dc-cli.exe log download --connect serial:COM3:115200 --out newest.bin
   ```
   Expect the list to match what Mission Planner shows. The download should finish, with
   `newest.bin.partial` renamed at the end. Nothing is erased.

6. **Analyse a real flight log** (from step 5, or the SD card).
   ```
   .\dc-cli.exe log summary flight.bin
   .\dc-cli.exe log health flight.bin
   .\dc-cli.exe log events flight.bin
   .\dc-cli.exe log changes flight.bin
   .\dc-cli.exe log fft flight.bin --field ACC.AccZ
   .\dc-cli.exe log export flight.bin --kml flight.kml
   ```
   Compare the summary (armed time, altitude, distance) and the event timeline with your
   notes and Mission Planner. Note the FFT peak frequency; it should match what Mission
   Planner's FFT shows.

   Then the full set of analyses, and the machine-readable health result:
   ```
   .\dc-cli.exe log report flight.bin > report.txt
   .\dc-cli.exe log health flight.bin --json > health.json
   .\dc-cli.exe log takeoffs flight.bin
   .\dc-cli.exe log landings flight.bin
   .\dc-cli.exe log failsafes flight.bin
   .\dc-cli.exe log attitude flight.bin
   .\dc-cli.exe log spectrogram flight.bin
   .\dc-cli.exe log notch flight.bin --field IMU.AccZ
   ```
   Note any finding that looks wrong for that flight (a CAUTION or WARNING you do not
   believe, or a known problem that is not flagged).

7. **Optional: telemetry radio.** Repeat step 2 over the radio's COM port at its baud rate,
   recording to a new file (e.g.
   `.\dc-cli.exe --tlog radio.tlog telemetry --connect serial:COM5:57600 --seconds 600`), and
   walk away until the link drops. Then
   `.\dc-cli.exe tlog link radio.tlog --window 10` shows loss, radio signal and heartbeat gaps
   for every 10 s; note where loss starts.

8. **Recording.** Analyse the recording from step 2:
   ```
   .\dc-cli.exe tlog info bench.tlog
   .\dc-cli.exe tlog params bench.tlog --out from-tlog.param
   .\dc-cli.exe tlog health bench.tlog
   .\dc-cli.exe tlog index bench.tlog --at 30
   ```
   `tlog health` judges armed time only, so a disarmed bench recording reports
   `no checks apply`; run it on a recording of a real flight if you have one.

9. **Parameter write and rollback (vehicle disarmed, propellers off).** Change one harmless
   value, check it, then put it back from the baseline the tool saved:
   ```
   .\dc-cli.exe param-set --connect serial:COM3:115200 DISARM_DELAY=12
   .\dc-cli.exe params --connect serial:COM3:115200 --compare vehicle.param
   .\dc-cli.exe param-rollback --connect serial:COM3:115200 --baseline param-baseline-1-XXXX.param --changed
   .\dc-cli.exe params --connect serial:COM3:115200 --compare vehicle.param
   ```
   Expect:
   * `param-set` lists the change and asks once; answer `y`. It ends with
     `all 1 change(s) verified by read-back` and names the baseline file;
   * the first compare shows exactly `DISARM_DELAY` 10 → 12 (plus values the autopilot
     maintains itself, which are marked) and ends with `error: the vehicle differs from the
     file`: that is expected, compare fails whenever something differs;
   * `param-rollback --changed` restores it, verified; the second compare passes (only
     values marked "set by the autopilot", such as `STAT_RUNTIME`, may differ);
   * `.\dc-cli.exe audit-verify --print` lists one line per step and ends with
     `chain intact`.

10. **Optional: the in-flight warning (bench, propellers OFF).** Only with the propellers
    removed and the vehicle held or strapped down: arm it with your transmitter, then run
    `.\dc-cli.exe param-set --connect serial:COM3:115200 DISARM_DELAY=12`. Expect the
    `WARNING: The vehicle is ARMED…` text and a second question; answer `n` and check that
    nothing changed. Disarm.

11. **Make a log safe to share.** Before sending any real log, move its positions to a
    stand-in location (owner data rule, ADR-0012):
    ```
    .\dc-cli.exe log sanitize flight.bin --out flight-share.bin
    .\dc-cli.exe tlog sanitize bench.tlog --out bench-share.tlog
    .\dc-cli.exe log summary flight-share.bin
    ```
    Expect the summary to match the original (same flight time and distances; positions
    moved). Send the `-share` files, not the originals.

## What to send back

* The console output of every step, copied into a text file. Note anything unexpected
  next to it.
* The output of `.\dc-cli.exe about`.
* `report.txt`, `health.json`, `dc-audit.log`, `vehicle.param`, `vehicle.waypoints`.
* The **sanitized** recordings from step 11 (`bench-share.tlog`, `flight-share.bin`). Never
  send or commit a field log with real positions (ADR-0012); the sanitized copies can become
  test fixtures in `docs/corpus/`, so that every build is tested against your firmware.
* The firmware version and board (from the step 2 banner), the Windows version and the USB
  cable/port used.

## Known limits

* Tested in CI against SITL ArduCopter 4.7.1 and 4.6.3. On other versions, a parameter or log
  field that was renamed shows up as missing or as a difference. Report it rather than
  working around it.
* No UI yet: this is the command-line tool the app will be built on.
