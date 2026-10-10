//! Serial port discovery, board identification, hotplug and auto-baud.
//!
//! * [`list_ports`]: ports with USB identity. On macOS only callout devices (`/dev/cu.*`)
//!   are returned — `/dev/tty.*` blocks on open waiting for carrier detect.
//! * [`identify`]: best-effort hint (autopilot, bootloader, USB-serial adapter) from USB
//!   vendor id and product string. A hint only: a MAVLink `HEARTBEAT` is the proof.
//! * [`DeviceMonitor`]: attach/remove events — IOKit notifications on macOS, a 1 s polling
//!   diff elsewhere (native Windows/Linux notifications are WS-PLAT work).
//! * [`probe_baud`]: find the baud rate at which a `HEARTBEAT` arrives.

use std::collections::BTreeMap;
use std::io::{self, Read};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::{Duration, Instant};

use mavlink_codec::{FrameParser, Message};

/// USB identity of a port.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UsbInfo {
    /// Vendor id.
    pub vid: u16,
    /// Product id.
    pub pid: u16,
    /// Serial number string, if the device reports one.
    pub serial: Option<String>,
    /// Manufacturer string.
    pub manufacturer: Option<String>,
    /// Product string.
    pub product: Option<String>,
}

/// What a port probably is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    /// Autopilot running its application (MAVLink expected).
    Autopilot,
    /// Autopilot in its bootloader (firmware flashing, no MAVLink).
    Bootloader,
    /// USB-to-UART bridge (telemetry radio, FC UART cable, GPS …).
    UsbSerialAdapter,
    /// Unknown device.
    Unknown,
}

/// Identification hint.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoardHint {
    /// Vendor/family name for display.
    pub vendor: &'static str,
    /// Likely role.
    pub role: Role,
}

/// One serial port.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortInfo {
    /// Path to open (`/dev/cu.usbmodem1101`, `/dev/ttyACM0`, `COM3`).
    pub path: String,
    /// USB identity, if USB.
    pub usb: Option<UsbInfo>,
    /// Identification hint (USB ports only).
    pub hint: Option<BoardHint>,
}

impl PortInfo {
    /// Stable identity across re-enumeration: `vid:pid:serial` when the device has a
    /// serial number, else the path.
    pub fn device_key(&self) -> String {
        match &self.usb {
            Some(u) => match &u.serial {
                Some(s) => format!("{:04x}:{:04x}:{s}", u.vid, u.pid),
                None => format!("{:04x}:{:04x}@{}", u.vid, u.pid, self.path),
            },
            None => self.path.clone(),
        }
    }
}

/// Best-effort identification from USB vendor id and descriptor strings.
pub fn identify(vid: u16, product: Option<&str>) -> BoardHint {
    let p = product.unwrap_or("").to_ascii_lowercase();
    let bootloader = p.contains("-bl") || p.ends_with(" bl") || p.contains("bootloader");
    let autopilot = |vendor| BoardHint {
        vendor,
        role: if bootloader {
            Role::Bootloader
        } else {
            Role::Autopilot
        },
    };
    let adapter = |vendor| BoardHint {
        vendor,
        role: Role::UsbSerialAdapter,
    };
    match vid {
        0x1209 if p.contains("ardupilot") || bootloader || p.is_empty() => {
            autopilot("ArduPilot (pid.codes)")
        }
        0x2DAE => autopilot("CubePilot"),
        0x3162 => autopilot("Holybro"),
        0x26AC => autopilot("3D Robotics / PX4-compatible"),
        0x0483 => autopilot("STMicroelectronics (STM32 USB)"),
        0x0403 => adapter("FTDI"),
        0x10C4 => adapter("Silicon Labs CP210x"),
        0x1A86 => adapter("WCH CH34x/CH9102"),
        0x067B => adapter("Prolific"),
        _ => BoardHint {
            vendor: "unknown",
            role: Role::Unknown,
        },
    }
}

/// List serial ports.
pub fn list_ports() -> io::Result<Vec<PortInfo>> {
    let ports = serialport::available_ports().map_err(io::Error::other)?;
    let mut out: Vec<PortInfo> = ports
        .into_iter()
        .filter(|p| !cfg!(target_os = "macos") || p.port_name.starts_with("/dev/cu."))
        .map(|p| {
            let usb = match p.port_type {
                serialport::SerialPortType::UsbPort(u) => Some(UsbInfo {
                    vid: u.vid,
                    pid: u.pid,
                    serial: u.serial_number,
                    manufacturer: u.manufacturer,
                    product: u.product,
                }),
                _ => None,
            };
            let hint = usb.as_ref().map(|u| identify(u.vid, u.product.as_deref()));
            PortInfo {
                path: p.port_name,
                usb,
                hint,
            }
        })
        .collect();
    out.sort_by(|a, b| a.path.cmp(&b.path));
    out.dedup_by(|a, b| a.path == b.path);
    Ok(out)
}

/// Attach/remove notification.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DeviceEvent {
    /// A port is present (also reported once for ports present at start).
    Added(PortInfo),
    /// A port went away.
    Removed {
        /// Its path.
        path: String,
    },
}

/// Watches for serial ports coming and going until dropped.
pub struct DeviceMonitor {
    events: Receiver<DeviceEvent>,
    #[cfg(target_os = "macos")]
    _notifier: platform_macos::SerialNotifier,
    #[cfg(not(target_os = "macos"))]
    _stop: Sender<()>,
}

impl DeviceMonitor {
    /// Start watching. Ports already present are reported first as `Added`.
    pub fn start() -> io::Result<DeviceMonitor> {
        let (tx, rx) = channel();
        Self::start_impl(tx, rx)
    }

    /// Event receiver.
    pub fn events(&self) -> &Receiver<DeviceEvent> {
        &self.events
    }

    #[cfg(target_os = "macos")]
    fn start_impl(tx: Sender<DeviceEvent>, rx: Receiver<DeviceEvent>) -> io::Result<DeviceMonitor> {
        use platform_macos::{RawSerialEvent, SerialNotifier};
        let (raw_tx, raw_rx) = channel();
        let notifier = SerialNotifier::start(raw_tx)?;
        // Enrich raw paths with USB identity (IOKit gives the path; serialport the details).
        std::thread::Builder::new()
            .name("serial-hotplug".into())
            .spawn(move || {
                for ev in raw_rx {
                    let out = match ev {
                        RawSerialEvent::Added(path) => {
                            let info = list_ports()
                                .ok()
                                .and_then(|ps| ps.into_iter().find(|p| p.path == path))
                                .unwrap_or(PortInfo {
                                    path,
                                    usb: None,
                                    hint: None,
                                });
                            DeviceEvent::Added(info)
                        }
                        RawSerialEvent::Removed(path) => DeviceEvent::Removed { path },
                    };
                    if tx.send(out).is_err() {
                        break;
                    }
                }
            })?;
        Ok(DeviceMonitor {
            events: rx,
            _notifier: notifier,
        })
    }

    #[cfg(not(target_os = "macos"))]
    fn start_impl(tx: Sender<DeviceEvent>, rx: Receiver<DeviceEvent>) -> io::Result<DeviceMonitor> {
        let (stop_tx, stop_rx) = channel::<()>();
        std::thread::Builder::new()
            .name("serial-poll".into())
            .spawn(move || {
                let mut known: BTreeMap<String, PortInfo> = BTreeMap::new();
                loop {
                    let now: BTreeMap<String, PortInfo> = list_ports()
                        .unwrap_or_default()
                        .into_iter()
                        .map(|p| (p.path.clone(), p))
                        .collect();
                    for ev in diff(&known, &now) {
                        if tx.send(ev).is_err() {
                            return;
                        }
                    }
                    known = now;
                    match stop_rx.recv_timeout(Duration::from_secs(1)) {
                        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
                        _ => return,
                    }
                }
            })?;
        Ok(DeviceMonitor {
            events: rx,
            _stop: stop_tx,
        })
    }
}

/// Events turning `before` into `after` (removals first).
pub fn diff(
    before: &BTreeMap<String, PortInfo>,
    after: &BTreeMap<String, PortInfo>,
) -> Vec<DeviceEvent> {
    let removed = before
        .keys()
        .filter(|k| !after.contains_key(*k))
        .map(|k| DeviceEvent::Removed { path: k.clone() });
    let added = after
        .iter()
        .filter(|(k, _)| !before.contains_key(*k))
        .map(|(_, p)| DeviceEvent::Added(p.clone()));
    removed.chain(added).collect()
}

/// Baud rates tried by [`probe_baud`], most common for ArduPilot first. USB-CDC ports
/// ignore the rate, so the first attempt succeeds there.
pub const PROBE_BAUDS: [u32; 9] = [
    115_200, 57_600, 921_600, 460_800, 230_400, 500_000, 1_500_000, 2_000_000, 38_400,
];

/// Result of a successful probe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProbeResult {
    /// Baud rate that worked.
    pub baud: u32,
    /// System id of the heartbeat sender.
    pub sysid: u8,
    /// Component id of the heartbeat sender.
    pub compid: u8,
}

/// Wait up to `window` for a checksum-valid `HEARTBEAT` on an open port.
pub fn wait_for_heartbeat(port: &mut dyn Read, window: Duration) -> Option<(u8, u8)> {
    let deadline = Instant::now() + window;
    let mut parser = FrameParser::new();
    let mut buf = [0u8; 512];
    while Instant::now() < deadline {
        match port.read(&mut buf) {
            Ok(n) if n > 0 => {
                parser.push(&buf[..n]);
                while let Some(f) = parser.next_frame() {
                    if matches!(f.decode(), Ok(Message::Heartbeat(_))) {
                        return Some((f.sysid(), f.compid()));
                    }
                }
            }
            Ok(_) => {}
            Err(e)
                if e.kind() == io::ErrorKind::TimedOut
                    || e.kind() == io::ErrorKind::Interrupted => {}
            Err(_) => return None,
        }
    }
    None
}

/// Try each rate in `bauds` (one at a time: a port can only be opened once) and return
/// the first at which a `HEARTBEAT` arrives within `window`. ArduPilot sends heartbeats at
/// 1 Hz, so `window` should be at least 1.2 s.
pub fn probe_baud(path: &str, bauds: &[u32], window: Duration) -> io::Result<Option<ProbeResult>> {
    for &baud in bauds {
        let mut port = serialport::new(path, baud)
            .timeout(Duration::from_millis(50))
            .open()
            .map_err(io::Error::other)?;
        if let Some((sysid, compid)) = wait_for_heartbeat(&mut port, window) {
            return Ok(Some(ProbeResult {
                baud,
                sysid,
                compid,
            }));
        }
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identifies_common_hardware() {
        assert_eq!(identify(0x2DAE, Some("CubeOrange")).role, Role::Autopilot);
        assert_eq!(
            identify(0x2DAE, Some("CubeOrange-BL")).role,
            Role::Bootloader
        );
        assert_eq!(
            identify(0x1209, Some("ArduPilot MatekH743")).role,
            Role::Autopilot
        );
        assert_eq!(
            identify(0x0403, Some("FT231X USB UART")).role,
            Role::UsbSerialAdapter
        );
        assert_eq!(identify(0x10C4, None).vendor, "Silicon Labs CP210x");
        assert_eq!(identify(0x1234, Some("Thing")).role, Role::Unknown);
    }

    #[test]
    fn device_key_prefers_serial_number() {
        let usb = |serial: Option<&str>| UsbInfo {
            vid: 0x2DAE,
            pid: 0x1016,
            serial: serial.map(Into::into),
            manufacturer: None,
            product: None,
        };
        let p = PortInfo {
            path: "/dev/cu.usbmodem1".into(),
            usb: Some(usb(Some("ABC"))),
            hint: None,
        };
        assert_eq!(p.device_key(), "2dae:1016:ABC");
        let p = PortInfo {
            usb: Some(usb(None)),
            ..p
        };
        assert_eq!(p.device_key(), "2dae:1016@/dev/cu.usbmodem1");
    }

    #[test]
    fn diff_reports_removals_then_additions() {
        let port = |p: &str| {
            (
                p.to_string(),
                PortInfo {
                    path: p.into(),
                    usb: None,
                    hint: None,
                },
            )
        };
        let before: BTreeMap<_, _> = [port("a"), port("b")].into();
        let after: BTreeMap<_, _> = [port("b"), port("c")].into();
        assert_eq!(
            diff(&before, &after),
            vec![
                DeviceEvent::Removed { path: "a".into() },
                DeviceEvent::Added(port("c").1)
            ]
        );
    }

    #[test]
    fn listing_ports_works_here() {
        // Enumeration must not fail on any CI runner (it may legitimately find nothing).
        let ports = list_ports().expect("enumeration");
        if cfg!(target_os = "macos") {
            assert!(
                ports.iter().all(|p| p.path.starts_with("/dev/cu.")),
                "{ports:?}"
            );
        }
        eprintln!("ports on this machine: {ports:#?}");
    }

    #[test]
    fn monitor_reports_present_ports_then_stops() {
        let present: Vec<String> = list_ports().unwrap().into_iter().map(|p| p.path).collect();
        let m = DeviceMonitor::start().expect("monitor starts");
        let mut seen = Vec::new();
        while let Ok(DeviceEvent::Added(p)) = m.events().recv_timeout(Duration::from_millis(1500)) {
            seen.push(p.path);
        }
        seen.sort();
        assert_eq!(seen, present, "initial events match enumeration");
        let t = Instant::now();
        drop(m);
        assert!(
            t.elapsed() < Duration::from_secs(2),
            "monitor stops promptly"
        );
    }

    #[cfg(unix)]
    #[test]
    fn finds_heartbeat_on_a_pty() {
        use mavlink_codec::messages::Heartbeat;
        use mavlink_codec::{Header, Version, encode};
        use serialport::SerialPort;
        use std::io::Write;
        let (mut far, mut near) = serialport::TTYPort::pair().unwrap();
        near.set_timeout(Duration::from_millis(50)).unwrap();
        let hb = encode(
            Version::V2,
            Header {
                seq: 0,
                sysid: 9,
                compid: 1,
            },
            &Message::Heartbeat(Heartbeat::default()),
        )
        .unwrap();
        far.write_all(b"\x00\x01noise").unwrap();
        far.write_all(hb.as_bytes()).unwrap();
        assert_eq!(
            wait_for_heartbeat(&mut near, Duration::from_secs(2)),
            Some((9, 1))
        );
        assert_eq!(
            wait_for_heartbeat(&mut near, Duration::from_millis(200)),
            None,
            "silence"
        );
    }
}
