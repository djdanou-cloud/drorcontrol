//! `dc-cli`: DrorControl from the command line.
//!
//! ```text
//! dc-cli params    --connect tcp:127.0.0.1:5760 [--out copter.param]
//! dc-cli telemetry --connect udp:0.0.0.0:14550 [--seconds 10]
//! dc-cli daemon    --connect tcp:127.0.0.1:5760 [--listen 127.0.0.1:50051]
//! dc-cli remote    --grpc http://127.0.0.1:50051 vehicles|params|events
//! ```
//!
//! Read-only by design for now: parameter writes and vehicle commands will be exposed only
//! through the safety interlocks (`CommandPolicy`, plan 08 §4) once they exist.

use std::process::ExitCode;
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand};
use dc_core::{Core, CoreConfig, CoreEvent, LinkSpec};
use protocols::params::to_param_file;

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
        /// Link, e.g. tcp:127.0.0.1:5760, udp:0.0.0.0:14550, serial:/dev/ttyACM0:115200.
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
            eprintln!("dc-core API on {listen} (Ctrl-C to stop)");
            tokio::select! {
                r = api::serve(core, *listen) => r.map_err(|e| format!("gRPC server: {e}")),
                _ = tokio::signal::ctrl_c() => Ok(()),
            }
        }
        Cmd::Remote { grpc, what } => remote(grpc, what).await,
        Cmd::Ports { watch, probe } => ports(*watch, probe.as_deref()),
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
                        Err(_) => {}
                    },
                }
            }
            Ok(())
        }
    }
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
            format!(
                "mode {} {}",
                h.value.custom_mode,
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
        let parts: Vec<String> = [hb, att, pos, bat].into_iter().flatten().collect();
        println!(
            "sys {:3} | {} | hb age {:.1} s",
            v.sysid,
            parts.join(" | "),
            age
        );
    }
}
