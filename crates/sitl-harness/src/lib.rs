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
}

impl SitlConfig {
    /// Copter SITL from `DC_SITL_BIN` / `DC_SITL_DEFAULTS`, or the `scripts/build_sitl.sh`
    /// output under `.sitl/`. `None` if not found.
    pub fn copter() -> Option<SitlConfig> {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let src = root.join(format!(".sitl/ardupilot-{PINNED_TAG}"));
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

/// A running SITL instance. Killed on drop.
pub struct Sitl {
    child: Child,
    dir: PathBuf,
    addr: SocketAddr,
    /// `-I` instance number (ports are 5760 + 10 × instance).
    pub instance: u16,
}

static NEXT: AtomicU16 = AtomicU16::new(0);

impl Sitl {
    /// Start SITL with wiped EEPROM in a fresh directory; returns once its SERIAL0 TCP port
    /// accepts connections (and immediately closes that probe).
    pub fn start(cfg: &SitlConfig) -> io::Result<Sitl> {
        // SITL keeps the instance in 8 bits (ports wrap above 255). Spread instances so
        // parallel tests and parallel test binaries do not collide; skip 0 (a developer's SITL).
        let instance =
            1 + ((std::process::id() % 24) as u16 * 10 + NEXT.fetch_add(1, Ordering::Relaxed) % 10);
        let dir = std::env::temp_dir().join(format!("dc-sitl-{}-{instance}", std::process::id()));
        std::fs::create_dir_all(&dir)?;
        let log = std::fs::File::create(dir.join("sitl.log"))?;
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
            .arg(&cfg.defaults)
            .args(["--home", &cfg.home, "-I", &instance.to_string()])
            .stdin(Stdio::null())
            .stdout(log.try_clone()?)
            .stderr(log)
            .spawn()?;
        let mut sitl = Sitl {
            child,
            dir,
            addr: ([127, 0, 0, 1], 5760 + 10 * instance).into(),
            instance,
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
