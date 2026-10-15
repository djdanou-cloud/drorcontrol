//! SITL test harness (plan 08 §2, T0-14).
//!
//! * [`Sitl`]: launch a pinned ArduPilot SITL binary headless in a temporary directory, wait
//!   for its MAVLink TCP port, kill it on drop.
//! * [`chaos::ChaosProxy`]: frame-aware TCP proxy between the GCS and SITL that drops,
//!   duplicates, corrupts, delays and reorders MAVLink frames, and can black out or cut the
//!   link, from a recorded seed.
//!
//! Scenario tests live in `tests/`. They need a SITL binary: set `DC_SITL_BIN` (and
//! `DC_SITL_DEFAULTS`), or build one with `scripts/build_sitl.sh` (found under `.sitl/`).
//! Without one they skip, unless `DC_SITL_REQUIRED=1` (CI), which makes a missing binary fail.

pub mod chaos;

use std::io;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU16, Ordering};
use std::time::{Duration, Instant};

/// Pinned ArduPilot tag the harness is validated against (`scripts/build_sitl.sh` default).
pub const PINNED_TAG: &str = "Copter-4.7.1";

/// Where SITL is and how to run it.
#[derive(Debug, Clone)]
pub struct SitlConfig {
    /// `arducopter` binary.
    pub bin: PathBuf,
    /// Default parameter file (`Tools/autotest/default_params/copter.parm`).
    pub defaults: PathBuf,
    /// Frame model (`+`, `quad`, …).
    pub model: String,
    /// Simulation speed-up.
    pub speedup: u32,
    /// Start location `lat,lon,alt,heading`.
    pub home: String,
    /// Extra parameter defaults applied at boot, after `defaults` (e.g. simulated devices
    /// that need a reboot to take effect).
    pub params: Vec<(String, f64)>,
    /// Extra command-line arguments, e.g. `--serial4=sim:adsb`.
    pub args: Vec<String>,
    /// ArduPilot version of the binary (major, minor, patch), from `DC_SITL_TAG`.
    pub version: (u8, u8, u8),
}

/// The ArduPilot tag under test: `DC_SITL_TAG` (CI runs 4.7.1 and the owner's fleet version,
/// 4.6.3), else [`PINNED_TAG`].
pub fn sitl_tag() -> String {
    std::env::var("DC_SITL_TAG").unwrap_or_else(|_| PINNED_TAG.to_string())
}

/// Version from a tag such as `Copter-4.6.3`.
pub fn tag_version(tag: &str) -> Option<(u8, u8, u8)> {
    let mut v = tag
        .rsplit('-')
        .next()?
        .split('.')
        .map(|p| p.parse::<u8>().ok());
    Some((v.next()??, v.next()??, v.next()??))
}

impl SitlConfig {
    /// Copter SITL from `DC_SITL_BIN` / `DC_SITL_DEFAULTS`, or the `scripts/build_sitl.sh`
    /// output under `.sitl/`. `None` if not found.
    pub fn copter() -> Option<SitlConfig> {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let tag = sitl_tag();
        let src = root.join(format!(".sitl/ardupilot-{tag}"));
        let bin = std::env::var_os("DC_SITL_BIN")
            .map(PathBuf::from)
            .unwrap_or_else(|| src.join("build/sitl/bin/arducopter"));
        let defaults = std::env::var_os("DC_SITL_DEFAULTS")
            .map(PathBuf::from)
            .unwrap_or_else(|| src.join("Tools/autotest/default_params/copter.parm"));
        (bin.is_file() && defaults.is_file()).then(|| SitlConfig {
            bin,
            defaults,
            model: "+".into(),
            speedup: 1,
            home: "-35.363261,149.165230,584,353".into(),
            params: Vec::new(),
            args: Vec::new(),
            version: tag_version(&tag).unwrap_or((4, 7, 1)),
        })
    }

    /// The parameter for this version: scenarios use 4.7 names; on 4.6 a renamed parameter
    /// gets its old name and unit (4.7.1 conversion tables: `AC_WPNav::convert_parameters`,
    /// `AP_RangeFinder_Params.cpp`, `MAV_SYSID` from `SYSID_THISMAV`).
    pub fn param(&self, name: &str, value: f64) -> (String, f64) {
        const RENAMED_IN_4_7: [(&str, &str, f64); 7] = [
            ("WP_SPD", "WPNAV_SPEED", 100.0),
            ("WP_SPD_UP", "WPNAV_SPEED_UP", 100.0),
            ("WP_SPD_DN", "WPNAV_SPEED_DN", 100.0),
            ("RNGFND1_MAX", "RNGFND1_MAX_CM", 100.0),
            ("RNGFND1_MIN", "RNGFND1_MIN_CM", 100.0),
            ("RTL_ALT_M", "RTL_ALT", 100.0),
            ("MAV_SYSID", "SYSID_THISMAV", 1.0),
        ];
        match RENAMED_IN_4_7.iter().find(|r| r.0 == name) {
            Some((_, old, scale)) if self.version < (4, 7, 0) => (old.to_string(), value * scale),
            _ => (name.to_string(), value),
        }
    }

    /// Put ArduPilot's ExpressLRS simulator (`SIM_ELRS`, a port of the ELRS receiver's
    /// MAVLink mode: 500 B/s each way, `RADIO_STATUS` flow control) on `SERIAL<serial>`, as
    /// MAVLink2 at 460 800 baud. It listens on TCP 5761 + `serial` for **every** instance
    /// (`SIM_ELRS.cpp` ignores `-I`), so the returned lock makes this machine-wide: hold it
    /// until the SITL is dropped. Waits up to 10 min for another holder.
    pub fn with_elrs(&mut self, serial: u8) -> io::Result<ElrsPort> {
        let port = 5761 + u16::from(serial);
        let deadline = Instant::now() + Duration::from_secs(600);
        let lock = loop {
            if let Some(l) = InstanceLock::take_named(&format!("elrs-{port}")) {
                break l;
            }
            if Instant::now() > deadline {
                return Err(io::Error::other(format!("ELRS port {port} stays locked")));
            }
            std::thread::sleep(Duration::from_millis(500));
        };
        if std::net::TcpListener::bind(("127.0.0.1", port)).is_err() {
            return Err(io::Error::other(format!(
                "TCP {port} is in use (a SITL with -I0?): SIM_ELRS cannot listen there"
            )));
        }
        self.args.push(format!("--serial{serial}=sim:ELRS"));
        self.params.push((format!("SERIAL{serial}_PROTOCOL"), 2.0));
        self.params.push((format!("SERIAL{serial}_BAUD"), 460.0));
        Ok(ElrsPort {
            addr: ([127, 0, 0, 1], port).into(),
            _lock: lock,
        })
    }

    /// [`SitlConfig::copter`], or why the caller should skip. Panics when
    /// `DC_SITL_REQUIRED=1` and no binary is found, so CI cannot silently skip.
    pub fn copter_or_skip() -> Option<SitlConfig> {
        let cfg = Self::copter();
        if cfg.is_none() {
            assert!(
                std::env::var("DC_SITL_REQUIRED").as_deref() != Ok("1"),
                "DC_SITL_REQUIRED=1 but no SITL binary: run scripts/build_sitl.sh or set DC_SITL_BIN"
            );
            eprintln!("skipping: no SITL binary (scripts/build_sitl.sh or DC_SITL_BIN)");
        }
        cfg
    }
}

/// The TCP side of ArduPilot's ELRS simulator ([`SitlConfig::with_elrs`]).
pub struct ElrsPort {
    /// Connect the GCS here.
    pub addr: SocketAddr,
    _lock: InstanceLock,
}

/// A running SITL instance. Killed on drop.
pub struct Sitl {
    child: Child,
    dir: PathBuf,
    addr: SocketAddr,
    /// `-I` instance number (ports are 5760 + 10 × instance).
    pub instance: u16,
    /// Released after `Drop::drop` has killed the process, so the next user of the instance
    /// never meets the old SITL on its ports.
    _lock: InstanceLock,
}

static NEXT: AtomicU16 = AtomicU16::new(0);

/// Highest instance used: SITL keeps the instance in 8 bits and ports wrap above 255.
const MAX_INSTANCE: u16 = 240;
/// Lock files older than this are from a crashed run and are taken over.
const STALE_LOCK: Duration = Duration::from_secs(30 * 60);
/// Start attempts (each on a fresh instance) before giving up.
const START_ATTEMPTS: u32 = 3;

/// Exclusive use of one SITL instance across all test processes on this machine: a lock
/// file in the temp directory, removed on drop. Two test binaries (or `cargo xtask ci` and a
/// separate SITL run) can otherwise pick the same instance; their SITLs then fight over the
/// ports and a test can talk to another test's vehicle (seen on an M1: an empty mission
/// read-back and a parameter download that never got its count).
struct InstanceLock(PathBuf);

impl InstanceLock {
    fn try_take(instance: u16) -> Option<InstanceLock> {
        Self::take_named(&format!("instance-{instance}"))
    }

    fn take_named(name: &str) -> Option<InstanceLock> {
        let path = std::env::temp_dir().join(format!("dc-sitl-{name}.lock"));
        let stale = std::fs::metadata(&path)
            .and_then(|m| m.modified())
            .ok()
            .and_then(|t| t.elapsed().ok())
            .is_some_and(|age| age > STALE_LOCK);
        if stale {
            let _ = std::fs::remove_file(&path);
        }
        std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .ok()?;
        Some(InstanceLock(path))
    }
}

impl Drop for InstanceLock {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}

/// SITL's TCP ports for `instance` (SERIAL0–2) are all free. A port held by something else
/// (macOS's AirPlay Receiver listens on 7000 = instance 124) makes SITL fail to start.
fn ports_free(instance: u16) -> bool {
    [5760, 5762, 5763]
        .iter()
        .all(|base| std::net::TcpListener::bind(("0.0.0.0", base + 10 * instance)).is_ok())
}

/// A free instance, locked for this caller.
fn take_instance() -> io::Result<(u16, InstanceLock)> {
    let start = (std::process::id() % 24) as u16 * 10 + NEXT.fetch_add(1, Ordering::Relaxed);
    for i in 0..MAX_INSTANCE {
        // Skip 0: a developer's own SITL.
        let instance = 1 + (start + i) % MAX_INSTANCE;
        if let Some(lock) = InstanceLock::try_take(instance)
            && ports_free(instance)
        {
            return Ok((instance, lock));
        }
    }
    Err(io::Error::other(
        "no free SITL instance (all locked or ports in use)",
    ))
}

impl Sitl {
    /// Start SITL with wiped EEPROM in a fresh directory; returns once its SERIAL0 TCP port
    /// accepts connections (and immediately closes that probe).
    pub fn start(cfg: &SitlConfig) -> io::Result<Sitl> {
        let mut last = None;
        for _ in 0..START_ATTEMPTS {
            let (instance, lock) = take_instance()?;
            match Self::start_on(cfg, instance, lock) {
                Ok(sitl) => return Ok(sitl),
                Err(e) => {
                    eprintln!("SITL instance {instance} did not start ({e}); trying another");
                    last = Some(e);
                }
            }
        }
        Err(last.expect("at least one attempt"))
    }

    fn start_on(cfg: &SitlConfig, instance: u16, lock: InstanceLock) -> io::Result<Sitl> {
        let dir = std::env::temp_dir().join(format!("dc-sitl-{}-{instance}", std::process::id()));
        std::fs::create_dir_all(&dir)?;
        let log = std::fs::File::create(dir.join("sitl.log"))?;
        let mut defaults = cfg.defaults.clone().into_os_string();
        if !cfg.params.is_empty() {
            let extra = dir.join("extra.parm");
            let text: String = cfg
                .params
                .iter()
                .map(|(k, v)| format!("{k} {v}\n"))
                .collect();
            std::fs::write(&extra, text)?;
            defaults.push(",");
            defaults.push(&extra);
        }
        let child = Command::new(&cfg.bin)
            .current_dir(&dir)
            .args([
                "--model",
                &cfg.model,
                "--speedup",
                &cfg.speedup.to_string(),
                "--wipe",
            ])
            .arg("--defaults")
            .arg(&defaults)
            .args(["--home", &cfg.home, "-I", &instance.to_string()])
            .args(&cfg.args)
            .stdin(Stdio::null())
            .stdout(log.try_clone()?)
            .stderr(log)
            .spawn()?;
        let mut sitl = Sitl {
            child,
            dir,
            addr: ([127, 0, 0, 1], 5760 + 10 * instance).into(),
            instance,
            _lock: lock,
        };
        // SITL listens on SERIAL0 and waits for its first client; wait until the port is
        // bound (log line), without connecting, so the test's own connection is the first.
        let deadline = Instant::now() + Duration::from_secs(60);
        loop {
            if let Some(status) = sitl.child.try_wait()? {
                return Err(io::Error::other(format!(
                    "SITL exited ({status}); see {}",
                    sitl.dir.join("sitl.log").display()
                )));
            }
            // Trust the port SITL reports, not our own arithmetic.
            let log = std::fs::read_to_string(sitl.dir.join("sitl.log")).unwrap_or_default();
            let port = log.lines().find_map(|l| {
                l.strip_prefix("bind port ")?
                    .strip_suffix(" for SERIAL0")?
                    .parse::<u16>()
                    .ok()
            });
            if let (Some(port), true) = (port, log.contains("Waiting for connection")) {
                sitl.addr = ([127, 0, 0, 1], port).into();
                return Ok(sitl);
            }
            if Instant::now() > deadline {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!(
                        "SITL never opened its port; see {}",
                        sitl.dir.join("sitl.log").display()
                    ),
                ));
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    /// SERIAL0 MAVLink TCP address.
    pub fn addr(&self) -> SocketAddr {
        self.addr
    }

    /// Working directory (EEPROM, logs, `sitl.log`).
    pub fn dir(&self) -> &Path {
        &self.dir
    }
}

impl Drop for Sitl {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

#[cfg(test)]
mod instance_tests {
    use super::*;

    #[test]
    fn instances_are_exclusive_and_skip_busy_ports() {
        let (a, lock_a) = take_instance().unwrap();
        let (b, _lock_b) = take_instance().unwrap();
        assert_ne!(a, b, "one instance per caller");
        assert!(
            InstanceLock::try_take(a).is_none(),
            "held by the first caller"
        );
        drop(lock_a);
        let again = InstanceLock::try_take(a).expect("free again after release");
        drop(again);
        // A port held by someone else (AirPlay on 7000 on a Mac) disqualifies the instance.
        let (c, lock_c) = take_instance().unwrap();
        let _busy = std::net::TcpListener::bind(("0.0.0.0", 5760 + 10 * c)).unwrap();
        assert!(!ports_free(c));
        drop(lock_c);
    }
}
