# Mac test session runbook

One session on a Mac closes the Phase 0 tasks that cannot be done on CI or in a cloud
container:
* T0-05 (serial);
* T0-06 (UI bridge frame timing);
* T0-10 (first launch);
* the go / no-go numbers for T0-07 (map) and T0-09 (video).

Plan on one working day with one Apple Silicon Mac; add an Intel Mac for the full matrix.
(A copy of this runbook was shared with the owner as a Claude Doc; this file is the source of
truth.)

## What to bring

| Item | Needed for | Minimum | Ideal |
|------|-----------|---------|-------|
| Apple Silicon Mac (M1 or later), admin rights | All tasks | 1, on macOS 15 | One each on macOS 13, 14, 15, 26 |
| Intel Mac | T0-05 x86_64 rows, T0-10 Intel launch | Optional | 1 |
| Flight controller with USB (STM32 USB-CDC) | T0-05 | 1 (e.g. CubeOrange or Pixhawk 6X) | 2 different boards |
| USB-serial adapters | T0-05 soak | 3 chip families: FTDI, SiLabs CP210x, WCH CH340/CH9102 | Plus a SiK/RFD900x radio pair |
| Jumper wires (TX↔RX loopback) | T0-05 soak | 1 per adapter | — |
| USB-C hub, data-capable cables | T0-05 | Cables | Powered hub |
| RTSP H.264 1080p30 camera | T0-09 | Optional: the harness makes its own clip | — |

Time budget:
* setup: 1 h;
* T0-05: about 45 min per adapter. 30 min of that is the soak, and soaks on different
  adapters can run in parallel;
* T0-06: 30 min;
* T0-07: about 1 h (the first MapLibre build takes 30–60 min);
* T0-09: 15 min;
* T0-10: 15 min.

## One-time setup

1. `xcode-select --install`
2. Install Homebrew, then `brew install qt cmake ninja ccache gstreamer pkgconf python@3.12`
3. `curl https://sh.rustup.rs -sSf | sh`. The repo pins Rust 1.97.0 and installs it on the
   first build.
4. `git clone https://github.com/djdanou-cloud/drorcontrol.git && cd drorcontrol`
5. `export QMAKE=$(brew --prefix qt)/bin/qmake QT_ROOT_DIR=$(brew --prefix qt)`
6. `cargo xtask ci` must end with `xtask ci: all steps passed`.
7. `cargo build --release -p dc-cli && export B=$PWD/target/release/dc-cli`
8. Record the machine:
   `sw_vers -productVersion; uname -m; sysctl -n machdep.cpu.brand_string`

Do **not** install vendor serial drivers (FTDI, SiLabs, WCH) before T0-05: the test is whether
macOS works without them.

## T0-05 — macOS serial, no vendor drivers

Pass:
* every device enumerates without a driver;
* hotplug is reported within 1 s;
* a 30-minute soak shows 0 CRC errors and 0 lost frames on 3 chip families.

First confirm that neither of these lists anything from FTDI, SiLabs or WCH:
* `systemextensionsctl list`
* `kmutil showloaded | grep -iv com.apple`

Per device, per Mac:
1. **Enumeration.** Run `$B ports`. Pass: a `/dev/cu.*` line with the right VID:PID, and a
   board hint for flight controllers.
2. **Hotplug.** Run `$B ports --watch 60`, then unplug and replug the device twice. Pass:
   * `+` within 1 s of plug-in;
   * `-` within 1 s of unplug;
   * the same `device_key` after replug.
3. **Probe** (flight controllers, and radios linked to a powered vehicle):
   `$B ports --probe /dev/cu.usbmodemXXXX`. Pass: HEARTBEAT found and the baud rate reported.
4. **Parameters:**
   `$B params --connect serial:/dev/cu.usbmodemXXXX:115200 --out ~/t005-params.param`.
   Pass: the file is written and the count matches the vehicle.
5. **Replug during a session:** run
   `$B telemetry --connect serial:/dev/cu.usbmodemXXXX:115200`, unplug for 5 s and plug back
   in. Pass: telemetry resumes by itself within 3 s. Repeat, replugging into a **different
   USB port**:
   * a device with a USB serial number is followed to its new name, and the link status
     names it;
   * one without a serial number is not followed.
6. **Soak** (USB-serial adapters only, with a TX↔RX jumper):
   `$B soak --tx /dev/cu.usbserial-XXXX --baud 921600 --seconds 1800`. Repeat at
   `--baud 2000000` if the chip supports it. Pass: 0 CRC errors and 0 lost frames.

| Device | Chip family | Steps | Soak baud |
|--------|-------------|-------|-----------|
| Flight controller (CubeOrange, Pixhawk 6X, Matek H743) | STM32 USB-CDC | 1–5 | none (no UART) |
| SiK / RFD900x radio | FTDI FT231X | 1–6 | 57600 and 115200 |
| Generic or Holybro adapter | SiLabs CP2102 | 1, 2, 6 | 921600 |
| Generic adapter | WCH CH340 / CH9102 | 1, 2, 6 | 921600, then 2000000 |

Results go in `crates/platform-macos/HARDWARE_MATRIX.md`, one row per device. If a CP210x or
CH34x adapter does not enumerate without a driver, that is a finding: record it and stop for
that device.

## T0-06 — UI bridge frame timing on Metal (spike S0-1)

Pass: at 200 000 MAVLink messages per second, p99 frame time ≤ 16.7 ms with 0 dropped frames.
The container run (no GPU) got 17.7 ms p99 and 0 dropped frames.

1. Build: `cargo build --release --manifest-path ui/spikes/s0-1-bridge/Cargo.toml`
2. Prepare the Mac: close other apps, plug in power, turn Low Power Mode off.
3. Run:
   `for r in 0 200000 400000; do S01_RATE=$r S01_REPORT=s01-$r.json ui/spikes/s0-1-bridge/target/release/spike-s0-1-bridge; done`
4. Repeat with Low Power Mode on (`s01-lpm-*.json`). (120 Hz ProMotion runs are not a current
   task: see [`../plan/future-upgrades.md`](../plan/future-upgrades.md).)
5. Record the numbers in `ui/spikes/s0-1-bridge/RESULTS.md` and in ADR-0001's validation.

## T0-07 — MapLibre through Qt RHI on Metal (spike S0-3)

Pass (go):
* the report says the graphics API is `Metal`;
* the warm pass runs at ≥ 59 fps with < 1 % missed vsync, full screen on the built-in Retina
  display (done on the M1 on 2026-10-11; 5K runs are in
  [`../plan/future-upgrades.md`](../plan/future-upgrades.md));
* GPU ≤ 8 %;
* pinch and rotate feel right.

Fail → the `wgpu` fallback (plan 01 §5.1).

```bash
ui/spikes/s0-3-map/build.sh                 # MapLibre from a pinned commit, Metal; ~30-60 min first time
B=ui/spikes/s0-3-map/build/spike-s0-3-map
sudo powermetrics --samplers gpu_power -i 1000 > gpu.txt &   # GPU load while the route runs
$B --fullscreen --report s03-m1-full.json   # 2 × 60 s route (cold, warm)
sudo pkill powermetrics
$B --manual --report s03-m1-manual.json     # trackpad pinch/rotate check; close the window to finish
```
Record in `ui/spikes/s0-3-map/RESULTS.md` and ADR-0003. Read the GPU load from the
"GPU HW active residency" line in `gpu.txt`.

**Status on CI:** the `spike-map` workflow builds QMapLibre with Metal and flies the route on
a macOS VM (green on 19bb613, 2026-10-12; numbers in `ui/spikes/s0-3-map/RESULTS.md`). VM
numbers are indicative only; the go / no-go uses the M1 run.

## T0-09 — 1080p30 H.264 decode on VideoToolbox (spike S0-5)

Pass: `"hardware": true`, `"zero_copy_handle": true`, decode p99 ≤ 8 ms, CPU ≤ 5 % of one core.

```bash
cargo build --release --manifest-path ui/spikes/s0-5-video/Cargo.toml
B=ui/spikes/s0-5-video/target/release/spike-s0-5-video
$B make-sample sample.mkv 90
$B run sample.mkv --seconds 60 --report s05-m1-file.json
$B run sample.mkv --seconds 60 --decoder avdec_h264 --report s05-m1-sw.json
$B run rtsp://<camera>/stream --seconds 60 --report s05-m1-rtsp.json   # optional
```
Then the decode-buffer check (why the hardware path held 100 ms; see RESULTS.md "Why the
hardware path holds frames"). Make the Baseline and 1-reference clips, and run each live as in
round 2 (mediamtx relay, `ffmpeg -re -c copy` publisher):
```bash
$B make-sample base.mkv 90 --profile baseline
$B make-sample ref1.mkv 90 --refs 1
# publish each to rtsp://127.0.0.1:8554/cam in turn, then:
$B run rtsp://127.0.0.1:8554/cam --seconds 60 --report s05-m1-<clip>.json
```
Prediction: Baseline ≈ decode time only (a few ms); `--refs 1` ≈ 36 ms; default ≈ 103 ms.

Record in `ui/spikes/s0-5-video/RESULTS.md` and ADR-0004.

## T0-10 — first launch of the downloaded app (ADR-0010)

1. In a browser, download the `drorcontrol-macos` artifact from the latest green `ci` run, so
   that macOS marks it as downloaded.
2. Run `shasum -a 256 -c SHA256SUMS`: every line must say `OK`.
3. Open the `.dmg`, copy DrorControl.app to Applications and double-click it. macOS should
   block it once.
4. System Settings → Privacy & Security → Open Anyway → confirm. Pass: it opens with no other
   warning.
5. Run `/Applications/DrorControl.app/Contents/MacOS/dc-cli --version`. Pass: it prints the
   version.
6. Run `/Applications/DrorControl.app/Contents/MacOS/dc-cli ports` with a flight controller
   plugged in, and allow Local Network if asked. Pass: the device is listed.
7. Repeat on the Intel Mac.

Note any confusing wording in `FIRST-LAUNCH.txt`.

## Where results go

| Task | Evidence | Recorded in |
|------|----------|-------------|
| T0-05 | Terminal output of steps 1–6 per device, `sw_vers`, `uname -m` | `crates/platform-macos/HARDWARE_MATRIX.md` |
| T0-06 | `s01-*.json` | `ui/spikes/s0-1-bridge/RESULTS.md`, ADR-0001 |
| T0-07 | `s03-*.json`, `gpu.txt`, pinch/rotate verdict | `ui/spikes/s0-3-map/RESULTS.md`, ADR-0003 (go / no-go) |
| T0-09 | `s05-*.json` | `ui/spikes/s0-5-video/RESULTS.md`, ADR-0004 |
| T0-10 | Pass/fail per step | Coordination log; `packaging/macos/FIRST-LAUNCH.txt` fixes |

The plugging and unplugging in T0-05 and the pinch gestures in T0-07 need hands on the
machine; everything else can be driven remotely if the Mac is connected to an agent session.
