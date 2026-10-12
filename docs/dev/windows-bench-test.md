# Windows bench test (real flight controller)

One session on a Windows PC with a flight controller on USB checks what the simulator
cannot:
* Windows serial ports;
* real sensors;
* real firmware on real hardware (CI covers ArduCopter 4.6.3 and 4.7.1 in simulation only).

Plan on about one hour.

Everything here **reads** from the vehicle. Nothing arms it, uploads to it, or erases
anything. The only thing sent besides read requests is `MAV_CMD_SET_MESSAGE_INTERVAL`
(telemetry rates for this connection; not saved to the vehicle).

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
| A dataflash log from a real flight (`.bin`), ideally including a hover | Step 6 |
| Telemetry radio pair (SiK or similar) | Optional step 7 |
| The Mission Planner `.param` file and `.waypoints` mission you normally use | Steps 3 and 4 |

## Get dc-cli

Use one of these:
* **CI build (no compiler needed):** on GitHub, open the latest green `ci` run on `main`.
  Download the `dc-cli-windows-x86_64` artifact and unzip `dc-cli.exe`. It is unsigned, so
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
   you know about.

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

7. **Optional: telemetry radio.** Repeat step 2 over the radio's COM port at its baud rate
   (e.g. `serial:COM5:57600`) for 5 minutes, then walk away until the link drops. Note the
   packet-loss figures as range increases.

8. **Recording.** Analyse the recording from step 2:
   ```
   .\dc-cli.exe tlog info bench.tlog
   .\dc-cli.exe tlog params bench.tlog --out from-tlog.param
   ```

## What to send back

* The console output of every step, copied into a text file. Note anything unexpected
  next to it.
* `bench.tlog`, `vehicle.param`, `vehicle.waypoints` and one real-flight `.bin`.

  Recordings can contain locations: share them only where you are comfortable doing so.
  With your agreement, they can become test fixtures in `docs/corpus/`, so that every
  build is then tested against your firmware version.
* The firmware version and board (from the step 2 banner), the Windows version and the USB
  cable/port used.

## Known limits

* Tested in CI against SITL ArduCopter 4.7.1 and 4.6.3. On other versions, a parameter or log
  field that was renamed shows up as missing or as a difference. Report it rather than
  working around it.
* No UI yet: this is the command-line tool the app will be built on.
