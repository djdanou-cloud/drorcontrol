# macOS serial hardware matrix (T0-05 / spike S0-2)

Status: **procedure ready, hardware runs pending** — needs a person with the devices below
(see the open issue in `docs/coordination/issues/`). CI covers the code paths on GitHub's
macOS runners (arm64 + Intel) but those machines have no serial devices.

## Pass criteria (plan 03 §2, §9 S0-2)
* Device enumerates as `/dev/cu.*` with USB identity; no third-party driver or KEXT installed.
* Hotplug: `+` within 1 s of plug-in, `-` within 1 s of unplug; replug yields the same `device_key`.
* Replug during a session: the open link reopens the same path by itself and telemetry resumes within 3 s (link-serial reopen, tested on Linux pseudo-terminals).
* `--probe` finds the HEARTBEAT (autopilots) at the configured rate.
* Full parameter download succeeds.
* 30-minute soak at the highest supported rate: **0 CRC errors, 0 lost frames**.

## Procedure (per device × macOS version × architecture)
```bash
cargo build --release -p dc-cli && B=target/release/dc-cli
$B ports                                   # 1. enumeration + board hint
$B ports --watch 60                        # 2. unplug / replug twice while this runs
$B ports --probe /dev/cu.usbmodemXXXX      # 3. autopilots only
$B params --connect serial:/dev/cu.usbmodemXXXX:115200 --out /tmp/p.param   # 4.
# 4b. Replug during a session: the link must come back by itself within ~3 s.
$B telemetry --connect serial:/dev/cu.usbmodemXXXX:115200   # unplug 5 s, replug; telemetry resumes
# 4c. Same, but replug into a different USB port: a device with a USB serial number is
#     followed to its new /dev/cu.* name (link status says so); one without is not.
# 5. Soak: USB-UART adapters with a TX-RX jumper (loopback), or two adapters cross-wired.
$B soak --tx /dev/cu.usbserial-XXXX --baud 921600 --seconds 1800
$B soak --tx /dev/cu.usbserial-XXXX --baud 2000000 --seconds 1800   # if the chip supports it
```
Record `sw_vers -productVersion`, `uname -m`, and whether any vendor driver is installed
(`systemextensionsctl list`, `kmutil showloaded | grep -iv com.apple`).

## Results

| macOS | Arch | Device | Chip | VID:PID | Enumerates | Hotplug | Probe | Params | Soak (baud, result) | Tester | Date |
|-------|------|--------|------|---------|-----------|---------|-------|--------|---------------------|--------|------|
| 13 / 14 / 15 / 26 | arm64, x86_64 | CubeOrange (ArduPilot) | STM32 USB-CDC | 2dae:… | pending | pending | pending | pending | n/a (USB-CDC) | | |
| 13 / 14 / 15 / 26 | arm64, x86_64 | Pixhawk 6X / Matek H743 | STM32 USB-CDC | 1209:… / … | pending | pending | pending | pending | n/a | | |
| 13 / 14 / 15 / 26 | arm64, x86_64 | RFD900x / SiK radio | FTDI FT231X | 0403:6015 | pending | pending | via radio | via radio | 57600/115200 pending | | |
| 13 / 14 / 15 / 26 | arm64, x86_64 | Holybro telemetry radio | SiLabs CP2102 | 10c4:ea60 | pending | pending | via radio | via radio | pending | | |
| 13 / 14 / 15 / 26 | arm64, x86_64 | Generic adapter | WCH CH340 / CH9102 | 1a86:… | pending | pending | n/a | n/a | pending | | |

Driver note to verify per version: FTDI and USB-CDC are in-box on all supported macOS
versions; CP210x and CH34x in-box support varies by release — if a row fails to enumerate
without a vendor driver, record it and raise it (plan 03 §2.1 user-space fallback).

## Verified in this repository without hardware
* IOKit notifier starts, reports present devices, stops in < 1 s — CI test on macOS runners.
* Serial link, probe and parameter download against ArduCopter 4.7.1 SITL through a
  pseudo-terminal bridge (Linux): HEARTBEAT found at 115200; 1370 parameters in 0.7 s.
* Soak tool through a pseudo-terminal echo loopback: 440 578 frames, 0 lost, 0 CRC errors.
