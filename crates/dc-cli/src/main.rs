//! `dc-cli`: DrorControl from the command line.
//!
//! ```text
//! dc-cli params    --connect tcp:127.0.0.1:5760 [--out copter.param]
//! dc-cli telemetry --connect udp:0.0.0.0:14550 [--seconds 10]
//! dc-cli daemon    --connect tcp:127.0.0.1:5760 [--listen 127.0.0.1:50051]
//! dc-cli daemon    --connect tlog:flight.tlog@4     # replay a recording (sole link)
//! dc-cli remote    --grpc http://127.0.0.1:50051 vehicles|params|events
//! dc-cli params-diff old.param flight.bin
//! dc-cli mission download --connect tcp:127.0.0.1:5760 [--out plan.waypoints]
//! dc-cli mission check plan.waypoints
//! dc-cli mission compare plan.waypoints --connect tcp:127.0.0.1:5760
//! dc-cli tlog info flight.tlog
//! dc-cli tlog export flight.tlog --gpx flight.gpx [--kml flight.kml]
//! dc-cli tlog params flight.tlog [--out flight.param]
//! dc-cli tlog mission flight.tlog [--out flight.waypoints]
//! dc-cli log export flight.bin --gpx flight.gpx [--kml flight.kml]
//! dc-cli log csv flight.bin ATT [--out att.csv]
//! dc-cli log params flight.bin [--out flight.param]
//! ```
//!
//! Read-only by design for now: parameter writes and vehicle commands will be exposed only
//! through the safety interlocks (`CommandPolicy`, plan 08 §4) once they exist.

mod tlog;
mod track;

use std::process::ExitCode;
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand};
use dc_core::{Core, CoreConfig, CoreEvent, LinkSpec};
use protocols::params::{diff_params, parse_param_file, to_param_file};

#[derive(Parser)]
#[command(name = "dc-cli", version, about = "DrorControl command-line tool")]
struct Cli {
    /// Our MAVLink system id (the vehicle's SYSID_MYGCS should match).
    #[arg(long, default_value_t = 255, global = true)]
    sysid: u8,
    /// Our MAVLink component id.
    #[arg(long, default_value_t = 190, global = true)]
    compid: u8,
    /// Record everything to this .tlog file.
    #[arg(long, global = true)]
    tlog: Option<std::path::PathBuf>,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Download all parameters and print them (or write a .param file).
    Params {
        /// Link, e.g. tcp:127.0.0.1:5760, udp:0.0.0.0:14550, serial:/dev/ttyACM0:115200, tlog:flight.tlog@4 (replay).
        #[arg(long)]
        connect: LinkSpec,
        /// Target system id (default: first vehicle heard).
        #[arg(long)]
        target: Option<u8>,
        /// Target component id.
        #[arg(long, default_value_t = 1)]
        component: u8,
        /// Write a Mission Planner-compatible .param file instead of printing.
        #[arg(long)]
        out: Option<std::path::PathBuf>,
        /// Seconds to wait for a vehicle heartbeat.
        #[arg(long, default_value_t = 15)]
        wait: u64,
    },
    /// Run the core with the gRPC API (proto/dc/v1/core.proto) until interrupted.
    Daemon {
        /// Links to open at start (repeatable).
        #[arg(long)]
        connect: Vec<LinkSpec>,
        /// gRPC listen address. Loopback by default: the API has no authentication yet.
        #[arg(long, default_value = "127.0.0.1:50051")]
        listen: std::net::SocketAddr,
    },
    /// Talk to a running daemon over gRPC.
    Remote {
        /// Daemon URL.
        #[arg(long, default_value = "http://127.0.0.1:50051")]
        grpc: String,
        #[command(subcommand)]
        what: RemoteCmd,
    },
    /// List serial ports with board hints; optionally watch hotplug or probe for MAVLink.
    Ports {
        /// Keep printing attach/remove events for this many seconds.
        #[arg(long)]
        watch: Option<u64>,
        /// Probe this port for a HEARTBEAT at common baud rates.
        #[arg(long)]
        probe: Option<String>,
    },
    /// Serial integrity soak: stream MAVLink frames out of `tx` and verify them on `rx`
    /// (same port with a TX-RX loopback jumper, or two adapters cross-wired).
    Soak {
        /// Sending port.
        #[arg(long)]
        tx: String,
        /// Receiving port (default: same as --tx, i.e. loopback).
        #[arg(long)]
        rx: Option<String>,
        /// Baud rate.
        #[arg(long, default_value_t = 921_600)]
        baud: u32,
        /// Duration in seconds.
        #[arg(long, default_value_t = 60)]
        seconds: u64,
    },
    /// Compare two parameter sets, each a .param file or the parameters recorded in a .bin log.
    ParamsDiff {
        /// Left side (.param or .bin).
        a: std::path::PathBuf,
        /// Right side (.param or .bin).
        b: std::path::PathBuf,
    },
    /// Dataflash (.bin) log tools: index statistics and downsampled plots, with timings.
    Log {
        #[command(subcommand)]
        what: LogCmd,
    },
    /// Mission files (QGC WPL 110, Mission Planner's .waypoints): download, check, compare.
    /// Read-only: uploads stay behind the core's interlocks and are not offered here yet.
    Mission {
        #[command(subcommand)]
        what: MissionCmd,
    },
    /// Telemetry logs (.tlog, as recorded with --tlog or by Mission Planner).
    Tlog {
        #[command(subcommand)]
        what: TlogCmd,
    },
    /// Print a telemetry summary once per second.
    Telemetry {
        /// Link to open.
        #[arg(long)]
        connect: LinkSpec,
        /// Stop after this many seconds.
        #[arg(long, default_value_t = 10)]
        seconds: u64,
    },
}

/// Which vehicle to talk to.
#[derive(clap::Args)]
struct VehicleArgs {
    /// Link, e.g. tcp:127.0.0.1:5760, udp:0.0.0.0:14550, serial:/dev/ttyACM0:115200, tlog:flight.tlog@4 (replay).
    #[arg(long)]
    connect: LinkSpec,
    /// Target system id (default: first vehicle heard).
    #[arg(long)]
    target: Option<u8>,
    /// Target component id.
    #[arg(long, default_value_t = 1)]
    component: u8,
    /// Seconds to wait for a vehicle heartbeat.
    #[arg(long, default_value_t = 15)]
    wait: u64,
}

/// `--type` of `mission download`.
#[derive(Clone, Copy, PartialEq, clap::ValueEnum)]
enum ListKind {
    Mission,
    Fence,
    Rally,
}

#[derive(Subcommand)]
enum TlogCmd {
    /// Time span, systems, message counts, maximum altitude, and a timeline of mode changes,
    /// arming and STATUSTEXT.
    Info {
        /// The .tlog file.
        file: std::path::PathBuf,
    },
    /// Parameters the recording saw (PARAM_VALUE, last value per name) as a .param file.
    Params {
        /// The .tlog file.
        file: std::path::PathBuf,
        /// Output .param file (default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
        /// Vehicle system id (default: the first that sent parameters).
        #[arg(long)]
        vehicle: Option<u8>,
    },
    /// The last complete mission in the recording (a download or an upload) as .waypoints.
    Mission {
        /// The .tlog file.
        file: std::path::PathBuf,
        /// Output .waypoints file (default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Export the flight track (GLOBAL_POSITION_INT) as GPX and/or KML.
    Export {
        /// The .tlog file.
        file: std::path::PathBuf,
        /// Write a GPX 1.1 track.
        #[arg(long)]
        gpx: Option<std::path::PathBuf>,
        /// Write a KML 2.2 line (Google Earth), altitude above MSL.
        #[arg(long)]
        kml: Option<std::path::PathBuf>,
        /// Vehicle system id (default: the first with a position). Not `--sysid`, which
        /// is our own MAVLink id.
        #[arg(long)]
        vehicle: Option<u8>,
    },
}

#[derive(Subcommand)]
enum MissionCmd {
    /// Download the vehicle's mission and print it (or write a .waypoints file).
    Download {
        #[command(flatten)]
        vehicle: VehicleArgs,
        /// Which list: the mission, the geofence or the rally points.
        #[arg(long = "type", value_enum, default_value_t = ListKind::Mission)]
        kind: ListKind,
        /// Write a .waypoints file instead of printing (mission only).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Read a .waypoints file and list its items (offline).
    Check {
        /// The file.
        file: std::path::PathBuf,
    },
    /// Does the vehicle hold exactly this file's mission? Downloads and compares (home, item
    /// 0, is not compared); exits with failure on any difference.
    Compare {
        /// The file.
        file: std::path::PathBuf,
        #[command(flatten)]
        vehicle: VehicleArgs,
    },
}

#[derive(Subcommand)]
enum LogCmd {
    /// Index a log and print message counts, timings and memory use.
    Info {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// List the dataflash logs on the vehicle (over MAVLink; vehicle must be disarmed).
    List {
        #[command(flatten)]
        vehicle: VehicleArgs,
    },
    /// Download a dataflash log from the vehicle over MAVLink (vehicle must be disarmed).
    /// Written to OUT.partial and renamed when complete. Logs are never erased.
    Download {
        #[command(flatten)]
        vehicle: VehicleArgs,
        /// Log number from `log list` (default: the newest).
        #[arg(long)]
        id: Option<u16>,
        /// Output file.
        #[arg(long)]
        out: std::path::PathBuf,
    },
    /// Every record of one message as CSV (all fields; text quoted).
    Csv {
        /// Path to the .bin log.
        path: std::path::PathBuf,
        /// Message name, e.g. ATT.
        msg: String,
        /// Output file (default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// The parameters recorded in the log as a .param file.
    Params {
        /// Path to the .bin log.
        path: std::path::PathBuf,
        /// Output file (default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Automatic checks: vibration and clipping, battery against the vehicle's failsafe
    /// voltages, EKF variances against FS_EKF_THRESH, and the autopilot's ERR records.
    Health {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Export the flight track (EKF position, UTC from the GPS clock) as GPX and/or KML.
    Export {
        /// Path to the .bin log.
        path: std::path::PathBuf,
        /// Write a GPX 1.1 track.
        #[arg(long)]
        gpx: Option<std::path::PathBuf>,
        /// Write a KML 2.2 line (Google Earth), altitude above MSL.
        #[arg(long)]
        kml: Option<std::path::PathBuf>,
    },
    /// Extract MSG.FIELD and downsample it (MinMaxLTTB) for plotting.
    Plot {
        /// Path to the .bin log.
        path: std::path::PathBuf,
        /// Message name, e.g. ATT.
        msg: String,
        /// Field name, e.g. Roll.
        field: String,
        /// Output points (plot width in pixels).
        #[arg(long, default_value_t = 2000)]
        width: usize,
        /// Keep only records whose field `I` equals this instance number.
        #[arg(long)]
        instance: Option<f64>,
        /// Write the downsampled points as CSV (time_s,value).
        #[arg(long)]
        csv: Option<std::path::PathBuf>,
    },
}

#[derive(Subcommand)]
enum RemoteCmd {
    /// Daemon identity.
    Info,
    /// Vehicles and their latest telemetry.
    Vehicles,
    /// Download parameters through the daemon.
    Params {
        /// Write a .param file instead of printing.
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Download the mission, geofence or rally points through the daemon and list them.
    Items {
        /// Which list.
        #[arg(long = "type", value_enum, default_value_t = ListKind::Mission)]
        kind: ListKind,
    },
    /// The vehicle's recent STATUSTEXT lines (the "Messages" view), oldest first.
    Messages,
    /// Photos the vehicle reported taking (CAMERA_FEEDBACK) as CSV, for geotagging.
    Geotags,
    /// List the vehicle's dataflash logs, or download one through the daemon.
    Logs {
        /// Log number to download.
        #[arg(long, requires = "out")]
        download: Option<u32>,
        /// Output file for --download (written to FILE.partial, renamed when complete).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Print events (STATUSTEXT, links) for a while.
    Events {
        /// Seconds to listen.
        #[arg(long, default_value_t = 10)]
        seconds: u64,
    },
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let rt = match tokio::runtime::Runtime::new() {
        Ok(rt) => rt,
        Err(e) => {
            eprintln!("error: cannot start runtime: {e}");
            return ExitCode::FAILURE;
        }
    };
    match rt.block_on(run(cli)) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

/// Ctrl-C, or SIGTERM (service managers, `kill`) on Unix.
async fn shutdown_signal() {
    #[cfg(unix)]
    {
        use tokio::signal::unix::{SignalKind, signal};
        match signal(SignalKind::terminate()) {
            Ok(mut term) => {
                tokio::select! {
                    _ = tokio::signal::ctrl_c() => {}
                    _ = term.recv() => {}
                }
            }
            Err(_) => {
                let _ = tokio::signal::ctrl_c().await;
            }
        }
    }
    #[cfg(not(unix))]
    {
        let _ = tokio::signal::ctrl_c().await;
    }
}

async fn start(cli: &Cli, link: &LinkSpec) -> Result<Core, String> {
    let cfg = CoreConfig {
        sysid: cli.sysid,
        compid: cli.compid,
        tlog: cli.tlog.clone(),
        ..Default::default()
    };
    let core = Core::start(cfg)
        .await
        .map_err(|e| format!("cannot start core: {e}"))?;
    core.connect(link)
        .await
        .map_err(|e| format!("cannot open {link}: {e}"))?;
    Ok(core)
}

async fn run(cli: Cli) -> Result<(), String> {
    match &cli.cmd {
        Cmd::Params {
            connect,
            target,
            component,
            out,
            wait,
        } => {
            let core = start(&cli, connect).await?;
            let sysid = match target {
                Some(t) => *t,
                None => core
                    .wait_for_vehicle(Duration::from_secs(*wait))
                    .await
                    .ok_or_else(|| format!("no vehicle heartbeat on {connect} within {wait} s"))?,
            };
            eprintln!("vehicle {sysid}: downloading parameters…");
            let t0 = Instant::now();
            let mut last_print = Instant::now();
            let params = core
                .download_params((sysid, *component), |r, t| {
                    if last_print.elapsed() > Duration::from_millis(500) {
                        eprintln!("  {r}/{t}");
                        last_print = Instant::now();
                    }
                })
                .await
                .map_err(|e| e.to_string())?;
            eprintln!(
                "{} parameters in {:.1} s",
                params.len(),
                t0.elapsed().as_secs_f32()
            );
            let text = to_param_file(&params);
            match out {
                Some(path) => {
                    std::fs::write(path, text).map_err(|e| format!("{}: {e}", path.display()))?;
                    eprintln!("wrote {}", path.display());
                }
                None => print!("{text}"),
            }
            Ok(())
        }
        Cmd::Daemon { connect, listen } => {
            let cfg = CoreConfig {
                sysid: cli.sysid,
                compid: cli.compid,
                tlog: cli.tlog.clone(),
                ..Default::default()
            };
            let core = Core::start(cfg)
                .await
                .map_err(|e| format!("cannot start core: {e}"))?;
            for link in connect {
                core.connect(link)
                    .await
                    .map_err(|e| format!("cannot open {link}: {e}"))?;
            }
            eprintln!("dc-core API on {listen} (Ctrl-C or SIGTERM to stop)");
            // Returning lets main drop the runtime, which flushes and closes the .tlog and
            // the links.
            tokio::select! {
                r = api::serve(core, *listen) => r.map_err(|e| format!("gRPC server: {e}")),
                _ = shutdown_signal() => Ok(()),
            }
        }
        Cmd::Remote { grpc, what } => remote(grpc, what).await,
        Cmd::Ports { watch, probe } => ports(*watch, probe.as_deref()),
        Cmd::Log {
            what: what @ (LogCmd::List { .. } | LogCmd::Download { .. }),
        } => log_transfer(&cli, what).await,
        Cmd::Log { what } => log_cmd(what),
        Cmd::Mission { what } => mission_cmd(&cli, what).await,
        Cmd::Tlog { what } => tlog_cmd(what),
        Cmd::ParamsDiff { a, b } => params_diff(a, b),
        Cmd::Soak {
            tx,
            rx,
            baud,
            seconds,
        } => soak(tx, rx.as_deref().unwrap_or(tx), *baud, *seconds),
        Cmd::Telemetry { connect, seconds } => {
            let core = start(&cli, connect).await?;
            let mut events = core.events();
            let end = Instant::now() + Duration::from_secs(*seconds);
            let mut tick = tokio::time::interval(Duration::from_secs(1));
            while Instant::now() < end {
                tokio::select! {
                    _ = tick.tick() => print_fleet(&core),
                    ev = events.recv() => match ev {
                        Ok(CoreEvent::StatusText { sysid, text }) => println!("[{sysid}] sev{} {}", text.severity, text.text),
                        Ok(CoreEvent::Link { link, up, detail }) => println!("{link} {}: {detail}", if up { "up" } else { "down" }),
                        Ok(CoreEvent::VehicleSeen { sysid }) => println!("vehicle {sysid} detected"),
                        Ok(CoreEvent::VehicleLink { sysid, state, heartbeat_age }) => {
                            println!("vehicle {sysid} link {state} (last heartbeat {:.1} s ago)", heartbeat_age.as_secs_f32())
                        }
                        Ok(CoreEvent::Command(st)) => println!("[{}] {}", st.target.sysid, st.summary()),
                        Ok(CoreEvent::Alert { sysid, level, text }) => {
                            eprintln!("[{sysid}] {} {text}", format!("{level:?}").to_uppercase())
                        }
                        Ok(CoreEvent::Recorder { stopped, detail }) => {
                            eprintln!("tlog {}: {detail}", if stopped { "STOPPED" } else { "warning" })
                        }
                        Ok(CoreEvent::Mission { sysid, change }) => println!("[{sysid}] mission: {change:?}"),
                        Err(_) => {}
                    },
                }
            }
            Ok(())
        }
    }
}

/// Resident memory (Linux): (current total, anonymous, file-backed, peak) in MiB.
fn rss_mib() -> Option<(f64, f64, f64, f64)> {
    let s = std::fs::read_to_string("/proc/self/status").ok()?;
    let kb = |key: &str| {
        s.lines()
            .find(|l| l.starts_with(key))?
            .split_whitespace()
            .nth(1)?
            .parse::<f64>()
            .ok()
            .map(|v| v / 1024.0)
    };
    Some((
        kb("VmRSS:")?,
        kb("RssAnon:")?,
        kb("RssFile:")?,
        kb("VmHWM:")?,
    ))
}

/// Parameters from a .param file, or from the `PARM` records of a .bin log.
fn load_params(path: &std::path::Path) -> Result<std::collections::BTreeMap<String, f64>, String> {
    let is_log = path
        .extension()
        .is_some_and(|e| e.eq_ignore_ascii_case("bin"));
    if is_log {
        let log = logs::Log::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let p = log.params();
        if p.is_empty() {
            return Err(format!("{}: no PARM records", path.display()));
        }
        Ok(p)
    } else {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        parse_param_file(&text).map_err(|e| format!("{}: {e}", path.display()))
    }
}

fn tlog_cmd(what: &TlogCmd) -> Result<(), String> {
    let file = match what {
        TlogCmd::Info { file } => file,
        TlogCmd::Export {
            file,
            gpx,
            kml,
            vehicle,
        } => return tlog_export(file, gpx.as_deref(), kml.as_deref(), *vehicle),
        TlogCmd::Params { file, out, vehicle } => {
            let f = std::fs::File::open(file).map_err(|e| format!("{}: {e}", file.display()))?;
            let (sys, p) = tlog::params(std::io::BufReader::new(f), *vehicle)
                .map_err(|e| format!("{}: {e}", file.display()))?
                .ok_or_else(|| format!("{}: no PARAM_VALUE in this recording", file.display()))?;
            let text = to_param_file(&p);
            return write_or_print(
                out.as_deref(),
                &text,
                &format!("{} parameters from system {sys}", p.len()),
            );
        }
        TlogCmd::Mission { file, out } => {
            let f = std::fs::File::open(file).map_err(|e| format!("{}: {e}", file.display()))?;
            let ((sys, comp), items) = tlog::mission(std::io::BufReader::new(f))
                .map_err(|e| format!("{}: {e}", file.display()))?
                .ok_or_else(|| {
                    format!("{}: no complete mission in this recording", file.display())
                })?;
            print_stats(&items);
            let text = protocols::wpl::to_string(&items);
            return write_or_print(
                out.as_deref(),
                &text,
                &format!("{} items sent by {sys}/{comp}", items.len()),
            );
        }
    };
    let f = std::fs::File::open(file).map_err(|e| format!("{}: {e}", file.display()))?;
    let s = tlog::summarise(std::io::BufReader::new(f))
        .map_err(|e| format!("{}: {e}", file.display()))?;
    let Some((first, last)) = s.span else {
        return Err(format!("{}: no records", file.display()));
    };
    println!(
        "{}: {} records, {} to {} UTC ({:.1} s)",
        file.display(),
        s.records,
        tlog::utc(first),
        tlog::utc(last),
        (last - first) as f64 / 1e6
    );
    for ((sys, comp), n) in &s.sources {
        println!("  source {sys}/{comp}: {n} frames");
    }
    let mut counts: Vec<_> = s.messages.iter().collect();
    counts.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
    println!("messages ({} types):", counts.len());
    for (name, n) in counts {
        println!("  {n:8}  {name}");
    }
    for (sys, alt) in &s.max_alt_rel_m {
        println!("system {sys}: highest {alt:.1} m above home");
    }
    println!("timeline:");
    for e in &s.events {
        println!(
            "  +{:8.1} s  [{}] {}",
            (e.unix_us - first) as f64 / 1e6,
            e.sysid,
            e.text
        );
    }
    if let Some(why) = &s.stopped {
        eprintln!("warning: stopped early: {why}");
    }
    Ok(())
}

/// Write `text` to `out`, or print it; `what` describes it on stderr.
fn write_or_print(out: Option<&std::path::Path>, text: &str, what: &str) -> Result<(), String> {
    match out {
        Some(p) => {
            std::fs::write(p, text).map_err(|e| format!("{}: {e}", p.display()))?;
            eprintln!("wrote {} ({what})", p.display());
        }
        None => {
            print!("{text}");
            eprintln!("{what}");
        }
    }
    Ok(())
}

fn tlog_export(
    file: &std::path::Path,
    gpx: Option<&std::path::Path>,
    kml: Option<&std::path::Path>,
    sysid: Option<u8>,
) -> Result<(), String> {
    if gpx.is_none() && kml.is_none() {
        return Err("nothing to write: give --gpx and/or --kml".into());
    }
    let f = std::fs::File::open(file).map_err(|e| format!("{}: {e}", file.display()))?;
    let (sys, points) = track::from_tlog(std::io::BufReader::new(f), sysid)
        .map_err(|e| format!("{}: {e}", file.display()))?
        .ok_or_else(|| format!("{}: no vehicle position in this recording", file.display()))?;
    let name = format!(
        "{} (system {sys})",
        file.file_name()
            .map_or_else(String::new, |n| n.to_string_lossy().into_owned())
    );
    let write = |path: &std::path::Path, text: String| {
        std::fs::write(path, text).map_err(|e| format!("{}: {e}", path.display()))?;
        eprintln!(
            "wrote {} ({} points, system {sys})",
            path.display(),
            points.len()
        );
        Ok::<_, String>(())
    };
    if let Some(p) = gpx {
        write(p, track::gpx(&name, &points))?;
    }
    if let Some(p) = kml {
        write(p, track::kml(&name, &points))?;
    }
    Ok(())
}

fn read_mission(path: &std::path::Path) -> Result<Vec<dc_core::MissionItem>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    protocols::wpl::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
}

fn print_mission(items: &[dc_core::MissionItem]) {
    for (seq, i) in items.iter().enumerate() {
        let name = mavlink_codec::enums::MavCmd(i.command)
            .name()
            .map_or_else(|| format!("command {}", i.command), str::to_string);
        if protocols::mission::has_location(i.command) {
            // Fence items carry the polygon's vertex count or the circle's radius in param1.
            let p1 = if i.params[0] != 0.0 {
                format!("  p1 {}", i.params[0])
            } else {
                String::new()
            };
            println!(
                "{seq:4}  {name:<28} frame {:2}  {:.7}, {:.7}  {} m{p1}",
                i.frame,
                f64::from(i.x) * 1e-7,
                f64::from(i.y) * 1e-7,
                i.z
            );
        } else {
            println!("{seq:4}  {name:<28} params {:?}", i.params);
        }
    }
}

/// Path length, altitudes and ending of a mission (an estimate).
fn print_stats(items: &[dc_core::MissionItem]) {
    let s = protocols::mission::stats(items);
    let alt = |name: &str, v: Option<f32>| v.map(|a| format!(", highest {a:.1} m {name}"));
    let alts: String = [
        alt("above home", s.max_alt_rel_m),
        alt("MSL", s.max_alt_msl_m),
        alt("above terrain", s.max_alt_terrain_m),
    ]
    .into_iter()
    .flatten()
    .collect();
    let ends = match s.last_nav {
        Some(20) => "ends with RTL",
        Some(21) => "ends with LAND",
        Some(_) => "does not end with RTL or LAND",
        None => "no navigation items",
    };
    eprintln!(
        "{} navigation items, path {:.0} m{alts}; {ends}{}",
        s.nav_items,
        s.path_m,
        if s.has_jumps {
            " (DO_JUMP repeats not counted)"
        } else {
            ""
        }
    );
}

async fn download_mission(
    cli: &Cli,
    v: &VehicleArgs,
    kind: dc_core::MissionType,
) -> Result<Vec<dc_core::MissionItem>, String> {
    let core = start(cli, &v.connect).await?;
    let sysid = match v.target {
        Some(t) => t,
        None => core
            .wait_for_vehicle(Duration::from_secs(v.wait))
            .await
            .ok_or_else(|| format!("no vehicle heartbeat on {} within {} s", v.connect, v.wait))?,
    };
    eprintln!("vehicle {sysid}: downloading {kind:?}…");
    let (status, items) = core.download_items((sysid, v.component), kind).await;
    match status {
        dc_core::TransferStatus::Done => Ok(items),
        s => Err(format!("{kind:?} download did not complete: {s:?}")),
    }
}

async fn mission_cmd(cli: &Cli, what: &MissionCmd) -> Result<(), String> {
    match what {
        MissionCmd::Download { vehicle, kind, out } => {
            let kind = match kind {
                ListKind::Mission => dc_core::MissionType::Mission,
                ListKind::Fence => dc_core::MissionType::Fence,
                ListKind::Rally => dc_core::MissionType::Rally,
            };
            if kind != dc_core::MissionType::Mission {
                // Mission Planner keeps fences and rally points in other file formats; until
                // those are implemented, only list them.
                if out.is_some() {
                    return Err("--out writes .waypoints files, for --type mission only".into());
                }
                let items = download_mission(cli, vehicle, kind).await?;
                eprintln!("{} {kind:?} items", items.len());
                print_mission(&items);
                return Ok(());
            }
            let items = download_mission(cli, vehicle, kind).await?;
            eprintln!("{} items (including home)", items.len());
            print_stats(&items);
            let text = protocols::wpl::to_string(&items);
            match out {
                Some(path) => {
                    std::fs::write(path, text).map_err(|e| format!("{}: {e}", path.display()))?;
                    eprintln!("wrote {}", path.display());
                }
                None => print!("{text}"),
            }
            Ok(())
        }
        MissionCmd::Check { file } => {
            let items = read_mission(file)?;
            print_mission(&items);
            eprintln!(
                "{}: {} items (including home), readable",
                file.display(),
                items.len()
            );
            print_stats(&items);
            Ok(())
        }
        MissionCmd::Compare { file, vehicle } => {
            let draft = read_mission(file)?;
            let on_vehicle = download_mission(cli, vehicle, dc_core::MissionType::Mission).await?;
            let diff = protocols::mission::verify(&draft, &on_vehicle, true);
            if diff.is_empty() {
                println!("the vehicle holds this mission ({} items)", draft.len());
                return Ok(());
            }
            for m in &diff {
                let at = m.seq.map_or_else(String::new, |s| format!("item {s} "));
                println!("{at}{}: file {}, vehicle {}", m.field, m.draft, m.vehicle);
            }
            Err(format!("{} differences", diff.len()))
        }
    }
}

fn params_diff(a: &std::path::Path, b: &std::path::Path) -> Result<(), String> {
    let (pa, pb) = (load_params(a)?, load_params(b)?);
    let d = diff_params(&pa, &pb);
    // Shortest text that round-trips the vehicle's f32.
    let f = |v: f64| v as f32;
    for (k, x, y) in &d.changed {
        println!("~ {k:<16} {} -> {}", f(*x), f(*y));
    }
    for k in &d.only_left {
        println!("- {k:<16} {}", f(pa[k]));
    }
    for k in &d.only_right {
        println!("+ {k:<16} {}", f(pb[k]));
    }
    println!(
        "{} vs {}: {} vs {} parameters; {} changed, {} only left, {} only right",
        a.display(),
        b.display(),
        pa.len(),
        pb.len(),
        d.changed.len(),
        d.only_left.len(),
        d.only_right.len()
    );
    Ok(())
}

async fn log_transfer(cli: &Cli, what: &LogCmd) -> Result<(), String> {
    let (LogCmd::List { vehicle } | LogCmd::Download { vehicle, .. }) = what else {
        return Ok(());
    };
    let core = start(cli, &vehicle.connect).await?;
    let sysid = match vehicle.target {
        Some(t) => t,
        None => core
            .wait_for_vehicle(Duration::from_secs(vehicle.wait))
            .await
            .ok_or_else(|| {
                format!(
                    "no vehicle heartbeat on {} within {} s",
                    vehicle.connect, vehicle.wait
                )
            })?,
    };
    // The armed state comes with the heartbeat; give it a moment after detection.
    let t0 = Instant::now();
    while core.facts(sysid).armed.is_none() && t0.elapsed() < Duration::from_secs(3) {
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    let target = (sysid, vehicle.component);
    let logs = core.list_logs(target).await?;
    if let LogCmd::List { .. } = what {
        for e in &logs {
            let when = if e.time_utc > 0 {
                tlog::utc(u64::from(e.time_utc) * 1_000_000)
            } else {
                "time unknown".into()
            };
            println!("{:5}  {:>12} bytes  {when}", e.id, e.size);
        }
        eprintln!("vehicle {sysid}: {} logs", logs.len());
        return Ok(());
    }
    let LogCmd::Download { id, out, .. } = what else {
        return Ok(());
    };
    let entry = match id {
        Some(id) => logs.iter().find(|e| e.id == *id).copied(),
        None => logs.last().copied(),
    }
    .ok_or_else(|| {
        format!(
            "no log {} on the vehicle",
            id.map_or("at all".into(), |i| i.to_string())
        )
    })?;
    eprintln!("downloading log {} ({} bytes)…", entry.id, entry.size);
    let t0 = Instant::now();
    let mut last = Instant::now();
    let data = core
        .download_log(target, entry, |r, s| {
            if last.elapsed() > Duration::from_secs(1) {
                eprintln!("  {r}/{s} bytes");
                last = Instant::now();
            }
        })
        .await?;
    let partial = out.with_extension("partial");
    std::fs::write(&partial, &data).map_err(|e| format!("{}: {e}", partial.display()))?;
    std::fs::rename(&partial, out).map_err(|e| format!("{}: {e}", out.display()))?;
    let secs = t0.elapsed().as_secs_f32();
    eprintln!(
        "wrote {} ({} bytes in {secs:.1} s, {:.1} kB/s)",
        out.display(),
        data.len(),
        data.len() as f32 / secs.max(0.001) / 1000.0
    );
    Ok(())
}

fn log_cmd(what: &LogCmd) -> Result<(), String> {
    let mem = || match rss_mib() {
        Some((t, a, f, peak)) => {
            format!("RSS {t:.0} MiB (anon {a:.0}, file {f:.0}; peak {peak:.0})")
        }
        None => "RSS n/a".into(),
    };
    let open = |path: &std::path::Path| {
        let t0 = Instant::now();
        let log = logs::Log::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let size = log.bytes().len() as f64 / (1 << 20) as f64;
        let dt = t0.elapsed().as_secs_f64();
        eprintln!(
            "indexed {size:.0} MiB in {dt:.2} s ({:.0} MiB/s): {} records, {} types, {} skipped bytes, {} FMT ({} conflicts); {}",
            size / dt,
            log.stats.records,
            log.formats.len(),
            log.stats.skipped_bytes,
            log.stats.fmt_records,
            log.stats.fmt_conflicts,
            mem()
        );
        Ok::<_, String>(log)
    };
    match what {
        // Handled by log_transfer (they need a link).
        LogCmd::List { .. } | LogCmd::Download { .. } => {}
        LogCmd::Info { path } => {
            let log = open(path)?;
            for (name, n) in log.counts() {
                println!("{name:<6} {n}");
            }
        }
        LogCmd::Csv { path, msg, out } => {
            let log = open(path)?;
            let text = logs::export::csv(&log, msg)
                .ok_or_else(|| format!("{}: no {msg} messages", path.display()))?;
            let rows = text.lines().count().saturating_sub(1);
            write_or_print(out.as_deref(), &text, &format!("{rows} {msg} records"))?;
        }
        LogCmd::Params { path, out } => {
            let log = open(path)?;
            let text = logs::export::param_file(&log);
            if text.is_empty() {
                return Err(format!("{}: no PARM records", path.display()));
            }
            let n = text.lines().count();
            write_or_print(out.as_deref(), &text, &format!("{n} parameters"))?;
        }
        LogCmd::Export { path, gpx, kml } => {
            if gpx.is_none() && kml.is_none() {
                return Err("nothing to write: give --gpx and/or --kml".into());
            }
            let log = open(path)?;
            let points = track::from_bin(&log);
            if points.is_empty() {
                return Err(format!(
                    "{}: no POS records with a position",
                    path.display()
                ));
            }
            if points[0].unix_us.is_none() {
                eprintln!("note: no GPS time in this log; the GPX has no timestamps");
            }
            let name = path
                .file_name()
                .map_or_else(String::new, |n| n.to_string_lossy().into_owned());
            for (out, text) in [
                (gpx, gpx.as_ref().map(|_| track::gpx(&name, &points))),
                (kml, kml.as_ref().map(|_| track::kml(&name, &points))),
            ] {
                if let (Some(p), Some(t)) = (out, text) {
                    std::fs::write(p, t).map_err(|e| format!("{}: {e}", p.display()))?;
                    eprintln!("wrote {} ({} points)", p.display(), points.len());
                }
            }
        }
        LogCmd::Health { path } => {
            let log = open(path)?;
            let findings = logs::health::assess(&log);
            if findings.is_empty() {
                println!("no checks apply: the log has no VIBE, BAT, XKF4 or ERR records");
            }
            for f in &findings {
                let at = f
                    .at_us
                    .map_or_else(String::new, |t| format!(" at {:.1} s", t as f64 / 1e6));
                println!(
                    "{:<8} {:<10} {}{at}",
                    format!("{:?}", f.level).to_uppercase(),
                    f.check,
                    f.text
                );
            }
            let worst = findings.iter().map(|f| f.level).max();
            println!(
                "overall: {}",
                worst.map_or("n/a".into(), |l| format!("{l:?}").to_uppercase())
            );
        }
        LogCmd::Plot {
            path,
            msg,
            field,
            width,
            instance,
            csv,
        } => {
            let log = open(path)?;
            let t0 = Instant::now();
            let col = log
                .column(msg, field, instance.map(|i| ("I", i)))
                .ok_or_else(|| format!("no column {msg}.{field}"))?;
            let t_extract = t0.elapsed().as_secs_f64();
            let x: Vec<f64> = if col.time_us.is_empty() {
                (0..col.values.len()).map(|i| i as f64).collect()
            } else {
                col.time_us.iter().map(|&t| t as f64 * 1e-6).collect()
            };
            let t1 = Instant::now();
            let keep = logs::downsample::minmax_lttb(&x, &col.values, *width, 4);
            let t_ds = t1.elapsed().as_secs_f64();
            eprintln!(
                "{msg}.{field}: {} samples -> {} points; extract {:.0} ms, downsample {:.0} ms, total {:.0} ms; {}",
                col.values.len(),
                keep.len(),
                t_extract * 1e3,
                t_ds * 1e3,
                (t_extract + t_ds) * 1e3,
                mem()
            );
            if let Some(path) = csv {
                let text: String = keep
                    .iter()
                    .map(|&i| format!("{},{}\n", x[i], col.values[i]))
                    .collect();
                std::fs::write(path, text).map_err(|e| format!("{}: {e}", path.display()))?;
            }
        }
    }
    Ok(())
}

fn ports(watch: Option<u64>, probe: Option<&str>) -> Result<(), String> {
    use link_serial::discovery::{
        DeviceEvent, DeviceMonitor, PROBE_BAUDS, PortInfo, list_ports, probe_baud,
    };
    let show = |p: &PortInfo| {
        let usb = p.usb.as_ref().map(|u| {
            format!(
                "{:04x}:{:04x} {} {}",
                u.vid,
                u.pid,
                u.manufacturer.as_deref().unwrap_or("?"),
                u.product.as_deref().unwrap_or("?")
            )
        });
        let hint = p
            .hint
            .as_ref()
            .map(|h| format!("{} ({:?})", h.vendor, h.role));
        format!(
            "{:<28} {:<40} {}",
            p.path,
            usb.unwrap_or_default(),
            hint.unwrap_or_default()
        )
    };
    if let Some(path) = probe {
        eprintln!("probing {path} at {PROBE_BAUDS:?} …");
        return match probe_baud(path, &PROBE_BAUDS, Duration::from_millis(1500))
            .map_err(|e| format!("{path}: {e}"))?
        {
            Some(r) => {
                println!(
                    "{path}: HEARTBEAT at {} baud from sysid {} compid {}",
                    r.baud, r.sysid, r.compid
                );
                Ok(())
            }
            None => Err(format!("{path}: no HEARTBEAT at any probed baud rate")),
        };
    }
    match watch {
        None => {
            for p in list_ports().map_err(|e| e.to_string())? {
                println!("{}", show(&p));
            }
        }
        Some(secs) => {
            let m = DeviceMonitor::start().map_err(|e| e.to_string())?;
            let end = Instant::now() + Duration::from_secs(secs);
            while let Some(left) = end.checked_duration_since(Instant::now()) {
                match m.events().recv_timeout(left) {
                    Ok(DeviceEvent::Added(p)) => println!("+ {}", show(&p)),
                    Ok(DeviceEvent::Removed { path }) => println!("- {path}"),
                    Err(_) => break,
                }
            }
        }
    }
    Ok(())
}

/// Serial soak test. Exit status is failure on any checksum error or lost frame.
fn soak(tx_path: &str, rx_path: &str, baud: u32, seconds: u64) -> Result<(), String> {
    use mavlink_codec::messages::Statustext;
    use mavlink_codec::{FrameParser, Header, Message, Version, encode};
    use std::io::{Read, Write};

    let open = |p: &str| {
        serialport::new(p, baud)
            .timeout(Duration::from_millis(20))
            .open()
            .map_err(|e| format!("{p}: {e}"))
    };
    let mut tx = open(tx_path)?;
    let mut rx = if rx_path == tx_path {
        tx.try_clone().map_err(|e| e.to_string())?
    } else {
        open(rx_path)?
    };
    let end = Instant::now() + Duration::from_secs(seconds);
    let reader = std::thread::spawn(move || {
        let mut parser = FrameParser::new();
        let (mut ok, mut lost, mut expect) = (0u64, 0u64, None::<u8>);
        let mut buf = [0u8; 4096];
        let quiet_until = |t: Instant| Instant::now() < t + Duration::from_secs(1);
        while quiet_until(end) {
            if let Ok(n) = rx.read(&mut buf) {
                parser.push(&buf[..n]);
                while let Some(f) = parser.next_frame() {
                    if let Some(e) = expect {
                        lost += u64::from(f.seq().wrapping_sub(e));
                    }
                    expect = Some(f.seq().wrapping_add(1));
                    ok += 1;
                }
            }
        }
        (ok, lost, parser.stats().clone())
    });

    // Frames of varying length (STATUSTEXT with 1..50 chars) at full line rate.
    let mut seq = 0u8;
    let mut sent = 0u64;
    while Instant::now() < end {
        let mut s = Statustext {
            severity: 6,
            ..Default::default()
        };
        let n = 1 + usize::from(seq) % 50;
        for (i, b) in s.text[..n].iter_mut().enumerate() {
            *b = b'A' + ((i + usize::from(seq)) % 26) as u8;
        }
        let f = encode(
            Version::V2,
            Header {
                seq,
                sysid: 255,
                compid: 190,
            },
            &Message::Statustext(s),
        )
        .map_err(|e| e.to_string())?;
        tx.write_all(f.as_bytes())
            .map_err(|e| format!("write: {e}"))?;
        seq = seq.wrapping_add(1);
        sent += 1;
    }
    let (ok, lost, stats) = reader.join().map_err(|_| "reader panicked".to_string())?;
    println!(
        "{tx_path} -> {rx_path} @ {baud}: sent {sent}, received {ok}, lost {lost}, crc_errors {}, skipped_bytes {}",
        stats.crc_errors, stats.skipped_bytes
    );
    if stats.crc_errors == 0 && lost == 0 && ok + 2 >= sent {
        Ok(())
    } else {
        Err("soak FAILED: errors or lost frames".into())
    }
}

async fn remote(url: &str, what: &RemoteCmd) -> Result<(), String> {
    use api::pb::{self, core_client::CoreClient};
    let mut c = CoreClient::connect(url.to_string())
        .await
        .map_err(|e| format!("{url}: {e}"))?;
    let err = |e: tonic::Status| format!("{}: {}", e.code(), e.message());
    match what {
        RemoteCmd::Info => {
            let i = c
                .get_info(pb::GetInfoRequest {})
                .await
                .map_err(err)?
                .into_inner();
            println!(
                "dc-core {} sysid {} compid {} dialect {}",
                i.version, i.sysid, i.compid, i.dialect
            );
        }
        RemoteCmd::Vehicles => {
            for v in c
                .list_vehicles(pb::ListVehiclesRequest {})
                .await
                .map_err(err)?
                .into_inner()
                .vehicles
            {
                println!("{v:?}");
            }
        }
        RemoteCmd::Params { out } => {
            let list = c
                .get_params(pb::GetParamsRequest {
                    sysid: 0,
                    compid: 0,
                })
                .await
                .map_err(err)?
                .into_inner();
            let params = list
                .params
                .into_iter()
                .map(|p| {
                    (
                        p.name,
                        protocols::params::Param {
                            value: p.value,
                            ty: p.mav_param_type as u8,
                            index: p.index as u16,
                        },
                    )
                })
                .collect();
            let text = to_param_file(&params);
            eprintln!("{} parameters from vehicle {}", params.len(), list.sysid);
            match out {
                Some(path) => {
                    std::fs::write(path, text).map_err(|e| format!("{}: {e}", path.display()))?
                }
                None => print!("{text}"),
            }
        }
        RemoteCmd::Items { kind } => {
            let kind = match kind {
                ListKind::Mission => pb::ItemKind::Mission,
                ListKind::Fence => pb::ItemKind::Fence,
                ListKind::Rally => pb::ItemKind::Rally,
            };
            let list = c
                .download_items(pb::DownloadItemsRequest {
                    sysid: 0,
                    compid: 0,
                    kind: kind as i32,
                })
                .await
                .map_err(err)?
                .into_inner();
            let items: Vec<dc_core::MissionItem> = list
                .items
                .iter()
                .map(|i| dc_core::MissionItem {
                    frame: i.frame as u8,
                    command: i.command as u16,
                    params: std::array::from_fn(|k| i.params.get(k).copied().unwrap_or(0.0)),
                    x: i.x,
                    y: i.y,
                    z: i.z,
                    autocontinue: i.autocontinue,
                })
                .collect();
            eprintln!("vehicle {}: {} {:?} items", list.sysid, items.len(), kind);
            print_mission(&items);
        }
        RemoteCmd::Logs { download, out } => match (download, out) {
            (Some(id), Some(out)) => {
                let mut s = c
                    .download_log(pb::DownloadLogRequest {
                        sysid: 0,
                        compid: 0,
                        id: *id,
                    })
                    .await
                    .map_err(err)?
                    .into_inner();
                let mut data = Vec::new();
                while let Some(u) = s.message().await.map_err(err)? {
                    if u.data.is_empty() {
                        eprintln!("  {}/{} bytes", u.received, u.size);
                    }
                    data.extend_from_slice(&u.data);
                    if u.done {
                        break;
                    }
                }
                let partial = out.with_extension("partial");
                std::fs::write(&partial, &data)
                    .map_err(|e| format!("{}: {e}", partial.display()))?;
                std::fs::rename(&partial, out).map_err(|e| format!("{}: {e}", out.display()))?;
                eprintln!("wrote {} ({} bytes)", out.display(), data.len());
            }
            _ => {
                let list = c
                    .list_logs(pb::ListLogsRequest {
                        sysid: 0,
                        compid: 0,
                    })
                    .await
                    .map_err(err)?
                    .into_inner();
                for l in &list.logs {
                    println!("{:5}  {:>12} bytes", l.id, l.size);
                }
                eprintln!("vehicle {}: {} logs", list.sysid, list.logs.len());
            }
        },
        RemoteCmd::Geotags => {
            let list = c
                .get_geotags(pb::GetGeotagsRequest { sysid: 0 })
                .await
                .map_err(err)?
                .into_inner();
            println!("cam,img,time_usec,lat,lon,alt_msl_m,alt_rel_m,roll,pitch,yaw,flags");
            for g in &list.geotags {
                println!(
                    "{},{},{},{:.7},{:.7},{:.2},{:.2},{:.2},{:.2},{:.2},{}",
                    g.cam_idx,
                    g.img_idx,
                    g.time_usec,
                    g.lat_deg,
                    g.lon_deg,
                    g.alt_msl_m,
                    g.alt_rel_m,
                    g.roll_deg,
                    g.pitch_deg,
                    g.yaw_deg,
                    g.flags
                );
            }
            eprintln!("vehicle {}: {} photos", list.sysid, list.geotags.len());
        }
        RemoteCmd::Messages => {
            let list = c
                .get_messages(pb::GetMessagesRequest { sysid: 0 })
                .await
                .map_err(err)?
                .into_inner();
            for m in &list.messages {
                println!(
                    "{}  sev{} [{}] {}",
                    crate::tlog::utc(m.unix_ms * 1000),
                    m.severity,
                    m.compid,
                    m.text
                );
            }
            eprintln!("vehicle {}: {} messages", list.sysid, list.messages.len());
        }
        RemoteCmd::Events { seconds } => {
            let mut stream = c
                .stream_events(pb::StreamEventsRequest {})
                .await
                .map_err(err)?
                .into_inner();
            let _ = tokio::time::timeout(Duration::from_secs(*seconds), async {
                while let Ok(Some(ev)) = stream.message().await {
                    println!("{:?}", ev.kind);
                }
            })
            .await;
        }
    }
    Ok(())
}

fn print_fleet(core: &Core) {
    let fleet = core.fleet();
    let now = Instant::now();
    for v in fleet.vehicles() {
        let hb = v.heartbeat.as_ref().map(|h| {
            // A mode is named only when the firmware is confirmed by its banner.
            let mode = v
                .mode_name()
                .map_or_else(|| h.value.custom_mode.to_string(), str::to_string);
            format!(
                "mode {mode} {}",
                if h.value.armed() { "ARMED" } else { "disarmed" }
            )
        });
        let att = v.attitude.as_ref().map(|a| {
            format!(
                "rpy {:+.1} {:+.1} {:+.1}°",
                a.value.roll.to_degrees(),
                a.value.pitch.to_degrees(),
                a.value.yaw.to_degrees()
            )
        });
        let pos = v.position.as_ref().map(|p| {
            format!(
                "{:.7},{:.7} alt {:.1} m",
                p.value.lat_deg, p.value.lon_deg, p.value.alt_rel_m
            )
        });
        let bat = v
            .power
            .as_ref()
            .and_then(|p| p.value.voltage)
            .map(|volt| format!("{volt:.2} V"));
        let age = v
            .heartbeat
            .as_ref()
            .map(|h| h.age(now).as_secs_f32())
            .unwrap_or(f32::NAN);
        let mission = v.mission.as_ref().map(|m| match m.value.total {
            Some(t) => format!("mission {}/{t} {:?}", m.value.current, m.value.state),
            None => format!("mission item {} {:?}", m.value.current, m.value.state),
        });
        let parts: Vec<String> = [hb, att, pos, bat, mission].into_iter().flatten().collect();
        println!(
            "sys {:3} | {} | hb age {:.1} s",
            v.sysid,
            parts.join(" | "),
            age
        );
    }
}
