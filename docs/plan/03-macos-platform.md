# 03 — macOS Native Engineering & Platform Layer

Goal: macOS is a **Tier-1** target — same feature set, same release cadence, same test gates as Windows. Support matrix: **macOS 13 Ventura → current (macOS 26)**, Apple Silicon (primary) and Intel (supported while Apple ships Intel-capable macOS releases; re-evaluate at macOS 27). Release artefact: a notarized **universal2** `.app` in a signed `.dmg` (+ `.zip` for Sparkle updates).

## 1. Build topology

| Component | arm64 | x86_64 | Universal strategy |
|-----------|-------|--------|--------------------|
| Rust core | `aarch64-apple-darwin` | `x86_64-apple-darwin` | build both, `lipo -create` static libs/dylibs |
| Qt 6 | official arm64 + x86_64 kits | same | use Qt universal build or `lipo` per framework |
| GStreamer | official universal framework | same | bundle needed plugins only; re-sign every dylib |
| App | link universal libs | | single universal Mach-O, verify with `lipo -archs`, test on both runners |

`cargo xtask package-macos` is the single entry point (local & CI). Deployment target `MACOSX_DEPLOYMENT_TARGET=13.0`.

## 2. Serial ports: enumeration, baud, hotplug (driverless)

### 2.1 Which chips work with no third-party driver
| Chip family | Typical boards/radios | macOS driver status | Plan |
|-------------|----------------------|---------------------|------|
| USB-CDC-ACM (STM32 native USB) | Pixhawk/Cube/Matek ArduPilot boards in app mode | In-box (`AppleUSBCDC`) | Primary path; always tested |
| FTDI FT232/FT231 | SiK radios, RFD900x, many FC UARTs | In-box since OS X 10.9 | Tested |
| SiLabs CP210x | Holybro/3DR telem, some FC | Varies by macOS version (in-box on recent releases; absent/legacy on older) | **Phase 0 matrix test per macOS version.** If absent: ship optional user-space fallback through USB host APIs (no KEXT, no DriverKit extension) for bulk-transfer CP210x |
| WCH CH340/CH9102 | Low-cost FCs/ESP radios | In-box on recent releases; verify | Same as CP210x |
| STM32 ROM DFU / ArduPilot bootloader | Firmware flashing | Class driver (DFU) / CDC | Direct `IOUSBHost` / libusb user-space transfer — no driver install |

**Rule:** the product must never instruct a macOS user to install a kernel extension. Any fallback runs in user space.

### 2.2 Enumeration & identity
* Use the **callout device** `/dev/cu.*`, never `/dev/tty.*` (the latter blocks on carrier-detect during open).
* Enumerate through `IOServiceMatching(kIOSerialBSDServiceValue)`; for each service walk the registry **parent chain** to read `idVendor`, `idProduct`, `USB Serial Number`, `USB Vendor Name`, `locationID`, and `kIOCalloutDeviceKey` → stable `DeviceId = vid:pid:serial|locationID`.
* A board database (ArduPilot `board_types`/USB IDs, SiK/FTDI, community boards) turns `(vid,pid)` into friendly names ("CubeOrange (ChibiOS)", "RFD900x"), expected baud, and "likely MAVLink / bootloader / console" role. ArduPilot boards often expose **multiple CDC interfaces** (MAVLink, GPS passthrough, console): rank them using interface number + product string.

### 2.3 Baud rates (57 600 → 921 600 → 2 000 000+)
* Standard `termios` rates via `cfsetspeed`; **non-standard/high rates** via `ioctl(fd, IOSSIOSPEED, &speed)` (from `<IOKit/serial/ioss.h>`) applied *after* `tcsetattr`.
* For USB-CDC boards the baud is nominal (full-speed USB bulk); we still set it so USB-UART bridges and flow control behave.
* Raw mode (`cfmakeraw`), `CLOCAL | CREAD`, hardware flow control (`CRTSCTS`) optional per profile (RFD900x), `VMIN=0/VTIME=0` with `kqueue`/`mio` readiness rather than blocking reads.
* Low latency: set `IOSSDATALAT` where supported; disable FTDI latency timer coalescing via vendor settings where possible; measure with S0-2.
* **Auto-baud probe:** a port can be opened only once, so rates are tried one after another — [115200, 57600, 921600, 460800, 230400, 500000, 1500000, 2000000, 38400] — accepting the first CRC-checked HEARTBEAT within ~1.5 s each (USB-CDC ignores the rate, so the first try succeeds); cache the result per `device_key`. Implemented in `link-serial::discovery::probe_baud` / `dc-cli ports --probe`.
* **CRSF TX modules** (ExpressLRS / Crossfire, [09](09-elrs-crsf-support.md)) use non-standard rates (400 000; ELRS also 921 600, 1 870 000, 2 250 000, 3 750 000, 5 250 000) on a single-wire half-duplex pin through a USB–UART adapter: set with `IOSSIOSPEED`; the adapter's real maximum is measured on the bench. The MAVLink auto-baud probe does not apply to them.

### 2.4 Hotplug (no polling)
```rust
// platform-macos/src/hotplug.rs  (sketch)
let port = IONotificationPortCreate(kIOMainPortDefault);
let rl_src = IONotificationPortGetRunLoopSource(port);
CFRunLoopAddSource(CFRunLoopGetCurrent(), rl_src, kCFRunLoopDefaultMode);

let matching = IOServiceMatching(kIOSerialBSDServiceValue);       // match all serial BSD clients
IOServiceAddMatchingNotification(port, kIOFirstMatchNotification, matching.retain(),
    on_attach, ctx, &mut iter_attach);                              // drain iterator to arm it
IOServiceAddMatchingNotification(port, kIOTerminatedNotification, matching,
    on_detach, ctx, &mut iter_detach);
// A dedicated CFRunLoop thread forwards events to the core's DeviceEvent channel.
```
* Same pattern with `kIOUSBHostDeviceClassName`/`IOUSBHostDevice` for **DFU/bootloader** devices (PID changes when a board re-enumerates after reboot — essential for the flasher).
* Attach events debounce 150 ms (CDC composite devices announce several nodes); detach during an active link ⇒ link state `LOST (device removed)` immediately, not after heartbeat timeout.
* Sleep/wake: observe `NSWorkspaceWillSleepNotification`/`DidWake`; close and reopen serial links on wake; show "link resumed" annunciation.

### 2.5 Cross-platform trait
```rust
trait SerialBackend { fn list() -> Vec<PortInfo>; fn open(&PortInfo, &Config) -> Result<Port>; fn events() -> Receiver<DeviceEvent>; }
// impls: macos (IOKit+termios+IOSSIOSPEED), windows (SetupAPI/CM_Register_Notification+overlapped), linux (udev+termios2/BOTHER)
```

## 3. Game controllers & joysticks — `GameController.framework`
* `GCController.controllers()` + `GCControllerDidConnect/DisconnectNotification`; use `extendedGamepad`/`GCPhysicalInputProfile` (and `GCRacingWheel`, `GCDualSenseGamepad`, generic HID via SDL3 fallback for flight sticks/HOTAS, which `GameController` does not cover).
* **Foreground rule:** by default controllers deliver input only to the frontmost app. A GCS must keep control when other windows are in front (e.g. video on a second display): set `GCController.shouldMonitorBackgroundEvents = true` (macOS 11.3+), and expose a visible "controller active" indicator + deadman requirement.
* Mapping layer is cross-platform (`input` crate): axes → `MANUAL_CONTROL` (69) x/y/z/r, buttons → actions (arm, mode, RTL …) **never** mapped by default to hazardous actions; calibration, deadzone, expo, inversion, per-vehicle profiles stored as JSON.
* Send rate 20–50 Hz, immediate zero-stick frame on disconnect, link loss, window-close; vehicle failsafe (`FS_GCS_ENABLE`) documented in handbook.
* Second output (safety-gated): 16 RC channels in µs to a CRSF TX module, so DrorControl acts as the transmitter ([09](09-elrs-crsf-support.md) Step 4b); same deadman and neutral-on-start rules; RC failsafe on the vehicle instead of the GCS failsafe.
* Windows uses XInput/RawInput/SDL; Linux evdev/SDL. A thin SDL3 backend unifies all if framework gaps appear.

## 4. Permissions, entitlements & sandboxing

### 4.1 Decision (ADR-0005)
> **Superseded in part by [ADR-0010](adr/0010-internal-distribution-without-apple-credentials.md):** no Apple credentials; the app is **ad-hoc signed**, not notarized, and shared as a file with a SHA-256 checksum. Developer ID and notarization below apply only if that decision is ever reversed.

Distribute **outside the Mac App Store** with **Developer ID + Hardened Runtime + notarization**. App Sandbox is **off** in v1 because: SITL supervisor spawns/downloads executables, plugins/scripts run as child processes, users pick arbitrary log/mission/DEM paths, and bundled GStreamer plugins are loaded dynamically. A **sandboxed MAS variant** is a separate later target (feasible subset: serial, network, user-selected file access via security-scoped bookmarks; SITL and plugin features reduced).

### 4.2 Hardened Runtime entitlements (`DrorControl.entitlements`)
```xml
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <!-- Only if Qt's QML JIT is enabled in the shipped build; prefer disabling JIT (QV4_FORCE_INTERPRETER-style build) to drop this -->
  <key>com.apple.security.cs.allow-jit</key><true/>
  <!-- Avoid: re-sign ALL bundled dylibs/plugins with our Team ID instead. Enable only if a 3rd-party plugin can't be re-signed. -->
  <!-- <key>com.apple.security.cs.disable-library-validation</key><true/> -->
  <key>com.apple.security.device.camera</key><true/>        <!-- UVC capture cards / webcams -->
  <key>com.apple.security.device.audio-input</key><false/>  <!-- not needed -->
  <key>com.apple.security.device.bluetooth</key><true/>     <!-- sandbox-only key; kept for MAS variant -->
  <key>com.apple.security.device.usb</key><true/>           <!-- same: sandbox variant -->
  <key>com.apple.security.device.serial</key><true/>        <!-- same: sandbox variant -->
  <key>com.apple.security.network.client</key><true/>
  <key>com.apple.security.network.server</key><true/>
</dict></plist>
```
(For the non-sandboxed Developer ID build, Hardened Runtime already permits serial/USB access; the `device.*` keys matter for the sandboxed variant. We keep one file per variant to avoid drift.)

### 4.3 Info.plist usage strings (TCC)
| Key | Purpose string (shown to user) |
|-----|--------------------------------|
| `NSBluetoothAlwaysUsageDescription` | "Connect to Bluetooth telemetry radios and controllers." |
| `NSLocalNetworkUsageDescription` + `NSBonjourServices` | "Discover vehicles, SITL, and cameras on your local network (MAVLink UDP, RTSP)." — required for local-network privacy prompts (macOS 15+) |
| `NSLocationWhenInUseUsageDescription` | "Use this computer's location for Follow-Me and to centre the map." (optional feature) |
| `NSCameraUsageDescription` | "Display video from capture devices." |
| `NSMicrophoneUsageDescription` | **omit** (not used) |
| `NSDesktopFolderUsageDescription`/`NSDocumentsFolderUsageDescription` | only if TCC prompts appear for log folders; prefer user-chosen locations via open panel |
| `GCSupportsControllerUserInteraction` / `GCSupportedGameControllers` | declare supported gamepad profiles |

* **UDP multicast/broadcast:** On recent macOS, local-network access prompts apply; apps using multicast may need the restricted `com.apple.developer.networking.multicast` entitlement from Apple (**request in Phase 0, lead time weeks**). Fallbacks: unicast UDP, directed-broadcast; user-visible diagnostics when the Local Network permission is denied (we can detect by failing send and show "Open System Settings → Privacy & Security → Local Network").
* **Permission UX rule:** never prompt at launch. Prompt contextually when the user first uses the feature, with a pre-prompt explainer card.

## 5. Signing, notarization & CI pipeline (GitHub Actions)

> **Current pipeline ([ADR-0010](adr/0010-internal-distribution-without-apple-credentials.md)):** universal build → inside-out **ad-hoc** `codesign --sign -` → `codesign --verify --deep --strict` + `lipo` checks → launch test on arm64 and x86_64 runners → `.zip` + `.dmg` + SHA-256. Steps below that need Apple credentials (Developer ID, `notarytool`, `stapler`, `spctl` acceptance) are not used.

### 5.1 Steps
1. Build arm64 + x86_64 → lipo → assemble `.app` (`macdeployqt` then our post-processor).
2. **Inside-out signing**: every `.dylib`, `.so`, framework, XPC, helper tool (`dc-core-daemon`, `gst-plugin-scanner`) individually: `codesign --force --timestamp --options runtime --sign "$IDENT" --entitlements …` — then the main bundle last. Never `--deep` for final signing.
3. Create `.dmg` (create-dmg), sign it.
4. `xcrun notarytool submit DrorControl.dmg --key AuthKey.p8 --key-id … --issuer … --wait` ; on failure fetch `notarytool log` and attach to the job.
5. `xcrun stapler staple` and validate: `spctl --assess --type open --context context:primary-signature -v`, `codesign --verify --deep --strict --verbose=2`.
6. Sparkle: generate EdDSA-signed appcast; publish to release bucket.

### 5.2 Workflow sketch
```yaml
name: macos-release
on: { push: { tags: ['v*'] } }
jobs:
  build:
    strategy: { matrix: { include: [ {arch: arm64, runner: macos-14}, {arch: x86_64, runner: macos-13} ] } }
    runs-on: ${{ matrix.runner }}
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@stable
      - run: brew bundle --file=packaging/macos/Brewfile
      - run: cargo xtask build --release --target ${{ matrix.arch }}
      - uses: actions/upload-artifact@v4
        with: { name: slice-${{ matrix.arch }}, path: target/slices/${{ matrix.arch }} }
  package-sign-notarize:
    needs: build
    runs-on: macos-14
    environment: apple-release            # protected environment, required reviewers
    steps:
      - uses: actions/checkout@v4
      - uses: actions/download-artifact@v4
      - name: Import certificate into ephemeral keychain
        run: ./packaging/macos/import-cert.sh   # decodes secrets.MACOS_CERT_P12 into a temp keychain, deleted in 'always()' step
        env: { P12: ${{ secrets.MACOS_CERT_P12 }}, P12_PASS: ${{ secrets.MACOS_CERT_PASS }} }
      - run: cargo xtask package-macos --universal --sign "$IDENT"
      - run: cargo xtask notarize --dmg dist/DrorControl.dmg
        env: { ASC_KEY: ${{ secrets.ASC_API_KEY_P8 }}, ASC_KEY_ID: ${{ secrets.ASC_KEY_ID }}, ASC_ISSUER: ${{ secrets.ASC_ISSUER }} }
      - run: ./packaging/macos/verify.sh dist/DrorControl.dmg   # spctl, stapler validate, lipo -archs
      - uses: actions/upload-artifact@v4
        with: { name: dmg, path: dist/*.dmg }
```
*Notes:* runner labels/Intel runner availability must be re-checked at implementation (Intel hosted runners are being phased out; fallback = cross-compile x86_64 on arm64 and test on a self-hosted Intel Mac mini). Secrets live in a protected environment; the keychain is ephemeral; the notarization API key is a team-scoped App Store Connect key with minimal role. A **nightly "dry-run" signing job** on `main` prevents certificate/entitlement rot.

## 6. Native macOS conventions (checklist)
* Global menu bar via `QMenuBar` (roles: About, Preferences ⌘,, Quit ⌘Q, Services, Hide); View menu with Enter Full Screen; Window menu lists open workspaces.
* **Trackpad:** pinch-to-zoom, two-finger pan, rotate, smart-zoom (double-tap two fingers), momentum scrolling; `Natural scrolling` respected; pixel vs line deltas handled.
* **Appearance:** follow `NSApp.effectiveAppearance` for the *app chrome*; flight themes (Daylight / Night-Ops) are independent of OS theme but default-linked ("Auto" = follow system).
* Retina/HiDPI: device-independent units everywhere; render targets at native scale; vector PFD means no @2x assets.
* Window management: native full-screen Spaces, Stage Manager friendly, tabs optional; multi-display with per-display workspace memory.
* Dock badge/bounce for critical annunciations while app is backgrounded; `UNUserNotificationCenter` for "low battery" etc.
* Drag & drop of `.waypoints/.plan/.param/.bin/.tlog` onto Dock icon/window; document types and UTIs registered; Quick Look generator for `.waypoints` / `.plan` (stretch).
* Shortcuts follow HIG (⌘ rather than Ctrl); hazardous commands never get single-key shortcuts.
* **App Nap / timers:** while any link is connected, hold `NSProcessInfo.beginActivity(options: [.userInitiated, .latencyCritical], reason: "Active vehicle link")` and disable automatic termination/sudden termination; otherwise macOS may coalesce timers and delay MAVLink heartbeats. Also prevent display/system sleep while armed (`IOPMAssertionCreateWithName`), with a user setting.
* Accessibility: VoiceOver labels on controls; Reduce Motion honoured; Increase Contrast maps to high-contrast token set.

## 7. Power & memory targets (MacBook field operations)

| Scenario (M1 MacBook Air, 13″, 50 % brightness) | CPU | GPU | Memory | Notes |
|---|---|---|---|---|
| Idle, connected, window visible | ≤ 3 % | ≤ 2 % | ≤ 450 MB | Render-on-dirty, 30 fps cap when nothing animates |
| Flight view (PFD + map, 50 Hz telem) | ≤ 12 % | ≤ 15 % | ≤ 600 MB | 60 fps when plugged in; 30–45 adaptive on battery |
| Flight view + 1080p30 video + overlay | ≤ 20 % | ≤ 25 % | ≤ 900 MB | VideoToolbox zero-copy; no CPU copies |
| 4 GB `.bin` review, 12 series | ≤ 25 % transient | ≤ 20 % | ≤ 1.2 GB RSS | mmap, lazy columns, pyramid levels |
| Background (window hidden/minimised) | ≤ 2 % | 0 | ≤ 400 MB | Rendering suspended, link + recorder keep running |

Techniques: render only on invalidation (no free-running timers), `CAMetalLayer.displayLinkEnabled`/vsync-driven frame loop with `maximumFrameLatency` tuning, coalesced property updates (§[01 3.4](01-architecture.md)), `NSProcessInfo.isLowPowerModeEnabled` → cap fps and drop shadows/blur, thermal-state observation (`thermalState`) → degrade gracefully, texture atlas for glyphs/symbols, SDF fonts for tapes, instanced draws for map overlays, avoid per-frame allocations (arena allocators), and a **battery-impact CI benchmark** using `powermetrics`/Instruments Energy Log on a dedicated Mac mini + MacBook in the bench lab (regression ≥ 10 % fails the nightly).

## 8. macOS-specific test matrix
| Dimension | Values |
|-----------|--------|
| Hardware | M1 Air, M3/M4 Pro MacBook, Intel Mac (2019+), Mac mini CI |
| OS | 13, 14, 15, 26 (+ beta track for next) |
| Displays | built-in Retina, external 4K, mixed-DPI dual screen (ProMotion and 5K: [future upgrades](future-upgrades.md)) |
| Serial devices | CubeOrange/Pixhawk 6X (CDC), Matek H743, FTDI & CP210x & CH340 radios, RFD900x, ELRS TX |
| Network | Wi-Fi + Ethernet adapter, VPN on/off, Local Network permission denied/allowed |
| Controllers | DualSense, Xbox, 8BitDo, generic HID joystick/HOTAS |
| Install | fresh VM from DMG (Gatekeeper first-launch path), upgrade via Sparkle, quarantine attribute set |
