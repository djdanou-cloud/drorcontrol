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
//! dc-cli log summary|events|changes flight.bin
//! dc-cli log params flight.bin [--out flight.param]
//! ```
//!
//! Read-only apart from `param-set` (ADR-0012 item 1: one confirmation; a vehicle that is
//! armed needs its in-flight warning acknowledged as well). Vehicle commands go only through
//! the safety interlocks (`CommandPolicy`, plan 08 §4).

mod tlog;
mod tlog_health;
mod tlog_sanitize;
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
        /// Compare the vehicle with this .param file or .bin log instead of printing; exits
        /// with failure on any difference (read-only).
        #[arg(long, conflicts_with = "out")]
        compare: Option<std::path::PathBuf>,
        /// Seconds to wait for a vehicle heartbeat.
        #[arg(long, default_value_t = 15)]
        wait: u64,
    },
    /// Change parameters on the vehicle (ADR-0012): every change is listed and confirmed once;
    /// if the vehicle is armed, its in-flight warning must be acknowledged too. A baseline of
    /// all parameters is saved first, each value is read back, and every step is audited.
    ParamSet {
        /// Link, e.g. tcp:127.0.0.1:5760, udp:0.0.0.0:14550, serial:/dev/ttyACM0:115200.
        #[arg(long)]
        connect: LinkSpec,
        /// Target system id (default: first vehicle heard).
        #[arg(long)]
        target: Option<u8>,
        /// Changes as NAME=VALUE.
        #[arg(required = true, value_name = "NAME=VALUE")]
        changes: Vec<String>,
        /// Confirm the changes without asking (scripts).
        #[arg(long)]
        yes: bool,
        /// Acknowledge the in-flight warning without asking (scripts): write even if armed.
        #[arg(long)]
        armed_ok: bool,
        /// Baseline file to create (default: param-baseline-SYSID-UNIXTIME.param).
        #[arg(long)]
        snapshot: Option<std::path::PathBuf>,
        /// Hash-chained audit log to append to.
        #[arg(long, default_value = "dc-audit.log")]
        audit: std::path::PathBuf,
        /// Seconds to wait for a vehicle heartbeat.
        #[arg(long, default_value_t = 15)]
        wait: u64,
    },
    /// Run the core with the gRPC API (proto/dc/v1/core.proto) until interrupted.
    Daemon {
        /// Links to open at start (repeatable).
        #[arg(long)]
        connect: Vec<LinkSpec>,
        /// gRPC listen address, e.g. 0.0.0.0:50051 for a field network. Beyond loopback an
        /// API key is required (--token-file or DC_API_TOKEN).
        #[arg(long, default_value = "127.0.0.1:50051")]
        listen: std::net::SocketAddr,
        /// File holding the API key clients must present (else the DC_API_TOKEN variable).
        #[arg(long)]
        token_file: Option<std::path::PathBuf>,
        /// Allow parameter writes over the API (SetParams), saving each baseline in this
        /// directory; without it the API cannot change parameters.
        #[arg(long)]
        param_dir: Option<std::path::PathBuf>,
        /// Hash-chained audit log (commands and parameter writes).
        #[arg(long, default_value = "dc-audit.log")]
        audit: std::path::PathBuf,
    },
    /// Talk to a running daemon over gRPC.
    Remote {
        /// Daemon URL.
        #[arg(long, default_value = "http://127.0.0.1:50051")]
        grpc: String,
        /// File holding the daemon's API key (else the DC_API_TOKEN variable).
        #[arg(long)]
        token_file: Option<std::path::PathBuf>,
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
    /// Which safety nets a parameter set has switched off (advisory; .param or .bin).
    ParamsReview {
        /// A .param file or a .bin log.
        file: std::path::PathBuf,
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
    /// Cursor-on-Target position events for every vehicle (ATAK/WinTAK/TAK), printed or
    /// sent over UDP (e.g. the ATAK multicast 239.2.3.1:6969).
    Cot {
        /// Link to open.
        #[arg(long)]
        connect: LinkSpec,
        /// Send each event as one UDP datagram to this address instead of printing it.
        #[arg(long)]
        udp: Option<std::net::SocketAddr>,
        /// Events per second per vehicle.
        #[arg(long, default_value_t = 1.0)]
        rate: f64,
        /// Stop after this many seconds (0: until interrupted).
        #[arg(long, default_value_t = 0)]
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
    /// Index the recording for playback: checkpoints, key events, and what a seek to a time
    /// would restore (the latest message of each stream).
    Index {
        /// The .tlog file.
        file: std::path::PathBuf,
        /// Show the state a seek to this many seconds after the start restores.
        #[arg(long)]
        at: Option<f64>,
    },
    /// Health checks from the telemetry (vibration, battery, EKF, GPS, errors), for when there
    /// is no dataflash log: the same thresholds as `log health`.
    Health {
        /// The .tlog file.
        file: std::path::PathBuf,
        /// Print the findings as JSON (schema `dc.health/1`).
        #[arg(long)]
        json: bool,
        /// Exit with an error when any finding is at this level or worse.
        #[arg(long, value_enum)]
        fail_on: Option<FailOn>,
    },
    /// Write a copy with every position moved to a stand-in location (shapes and distances
    /// kept), so the recording can be shared without revealing where it was flown.
    Sanitize {
        /// The .tlog file.
        file: std::path::PathBuf,
        /// Output .tlog file.
        #[arg(long)]
        out: std::path::PathBuf,
        /// Stand-in location "lat,lon" (default: ArduPilot's SITL home, Canberra).
        #[arg(
            long,
            default_value = "-35.363261,149.165230",
            allow_hyphen_values = true
        )]
        to: String,
    },
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
    /// Parameters whose value changed during the recording (PARAM_VALUE reports).
    Changes {
        /// The .tlog file.
        file: std::path::PathBuf,
        /// Vehicle system id (default: the first that sent parameters).
        #[arg(long)]
        vehicle: Option<u8>,
    },
    /// The last complete mission, fence or rally list in the recording (a download or an
    /// upload) as .waypoints (QGC WPL 110).
    Mission {
        /// The .tlog file.
        file: std::path::PathBuf,
        /// Which list.
        #[arg(long = "type", value_enum, default_value_t = ListKind::Mission)]
        kind: ListKind,
        /// Output .waypoints file (default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Every message of one type (e.g. ATTITUDE) as CSV, with receive time and sender.
    Csv {
        /// The .tlog file.
        file: std::path::PathBuf,
        /// Message name, e.g. GLOBAL_POSITION_INT.
        msg: String,
        /// Only this system id (not `--sysid`, which is our own MAVLink id).
        #[arg(long)]
        vehicle: Option<u8>,
        /// Output file (default: print).
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
        /// Write a .waypoints file (QGC WPL 110; fence and rally items as the vehicle holds
        /// them).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Generate a survey grid ("lawnmower") over a polygon as a .waypoints mission: takeoff,
    /// parallel lines flown alternately, RTL. Offline; upload it like any other mission.
    Survey {
        /// Polygon vertices as "lat,lon;lat,lon;..." (at least 3; southern latitudes start
        /// with a minus sign).
        #[arg(
            long,
            allow_hyphen_values = true,
            required_unless_present = "polygon_kml"
        )]
        polygon: Option<String>,
        /// Read the polygon from a KML file instead (its first <Polygon>, e.g. drawn in
        /// Google Earth).
        #[arg(long, conflicts_with = "polygon")]
        polygon_kml: Option<std::path::PathBuf>,
        /// Distance between lines, m (or worked out from --camera and --sidelap).
        #[arg(long, required_unless_present = "camera")]
        spacing: Option<f64>,
        /// Camera as "sensor_w_mm,sensor_h_mm,focal_mm,image_w_px,image_h_px": prints the
        /// ground resolution and photo distance, and sets the line spacing from the overlap.
        /// The mission does not trigger the camera.
        #[arg(long)]
        camera: Option<String>,
        /// Side overlap between lines, % (with --camera).
        #[arg(long, default_value_t = 70.0)]
        sidelap: f64,
        /// Front overlap between photos, % (with --camera).
        #[arg(long, default_value_t = 80.0)]
        frontlap: f64,
        /// Trigger the camera by distance in the mission (DO_SET_CAM_TRIGG_DIST at the first
        /// survey waypoint, off before RTL), at the photo distance from --camera.
        #[arg(long, requires = "camera")]
        trigger: bool,
        /// Fly a second set of lines at 90 degrees to the first (crosshatch).
        #[arg(long)]
        crosshatch: bool,
        /// Extend each line this many metres beyond the polygon at both ends.
        #[arg(long, default_value_t = 0.0)]
        overshoot: f64,
        /// Line direction, degrees clockwise from north.
        #[arg(long, default_value_t = 0.0)]
        angle: f64,
        /// Altitude above home, m.
        #[arg(long)]
        alt: f32,
        /// Output .waypoints file (default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Read a .waypoints file and list its items (offline).
    Check {
        /// The file.
        file: std::path::PathBuf,
        /// The vehicle's parameters (.param or .bin) for the flight-time estimate's speeds
        /// (default: ArduCopter 4.7 defaults) and the parameter fences (FENCE_TYPE circle and
        /// altitude) of the --fence check.
        #[arg(long)]
        params: Option<std::path::PathBuf>,
        /// Check the whole path against this geofence (fence items as saved by
        /// `mission download --type fence --out`); exits with failure on a breach.
        #[arg(long)]
        fence: Option<std::path::PathBuf>,
    },
    /// Fly the route backwards (offline): home, TAKEOFF and the final RTL/LAND stay in place.
    Reverse {
        /// The .waypoints file.
        file: std::path::PathBuf,
        /// Output file (default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Move every located item by metres north/east (offline); home stays unless --home.
    Shift {
        /// The .waypoints file.
        file: std::path::PathBuf,
        /// Metres north (negative: south).
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        north: f64,
        /// Metres east (negative: west).
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        east: f64,
        /// Move home (item 0) too.
        #[arg(long)]
        home: bool,
        /// Output file (default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Compare two mission files (offline); exits with failure on any difference.
    Diff {
        /// First .waypoints file.
        a: std::path::PathBuf,
        /// Second .waypoints file.
        b: std::path::PathBuf,
        /// Also compare home (item 0), which the vehicle sets itself.
        #[arg(long)]
        with_home: bool,
    },
    /// The mission's flight path as GPX and/or KML (Google Earth), offline.
    Export {
        /// The .waypoints file.
        file: std::path::PathBuf,
        /// Write a GPX 1.1 track.
        #[arg(long)]
        gpx: Option<std::path::PathBuf>,
        /// Write a KML 2.2 line.
        #[arg(long)]
        kml: Option<std::path::PathBuf>,
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
    /// Two flights side by side: time, altitude, distance, speeds, battery, motors, tracking.
    Compare {
        /// First .bin log.
        a: std::path::PathBuf,
        /// Second .bin log.
        b: std::path::PathBuf,
    },
    /// Every analysis at once: summary, health, motors, attitude, battery, compass, wind,
    /// sensors, RC link, notch advice and parameter changes.
    Report {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Flight summary: armed time, highest altitude, top speed, distance, battery used.
    Summary {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Timeline of mode changes, arming, events, errors and messages.
    Events {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Parameters changed after boot (values the autopilot updates itself are marked).
    Changes {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Motor outputs while flying: how hard each motor works compared with the others.
    Motors {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Photos in the log (CAM records) as CSV for geotagging, with UTC from the GPS time.
    Geotags {
        /// Path to the .bin log.
        path: std::path::PathBuf,
        /// Trigger times (TRIG) instead of photos; differs from CAM only with a feedback pin.
        #[arg(long)]
        trig: bool,
        /// Output file (default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Throttle-based harmonic notch settings suggested from the flight (advice only;
    /// nothing is written to the vehicle).
    Notch {
        /// Path to the .bin log.
        path: std::path::PathBuf,
        /// Message and field, e.g. ACC.AccX or GYR.GyrX (raw IMU logging, LOG_BITMASK bit 19).
        #[arg(long, default_value = "ACC.AccX")]
        field: String,
        /// IMU instance.
        #[arg(long, default_value_t = 0)]
        imu: u8,
        /// Samples per FFT segment (any length; powers of two are fastest).
        #[arg(long, default_value_t = 1024)]
        segment: usize,
        /// Ignore peaks below this frequency, Hz (body motion, not vibration).
        #[arg(long, default_value_t = 20.0)]
        min_hz: f64,
    },
    /// RC link: receiver RSSI and link quality while armed, radio failsafes, late frames.
    Rc {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Optical flow and rangefinders while flying: could the EKF use them?
    Sensors {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Do the altitude sources agree while flying (barometer vs EKF, GPS vs EKF)?
    Altitude {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// GPS quality while armed: satellites, HDOP, fix types, glitches.
    Gps {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Wind while flying, as the EKF estimated it.
    Wind {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// IMU health: temperatures, error counts, unhealthy samples while armed, sample rates.
    Imu {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Write a copy with every position moved to a stand-in location (shapes and distances
    /// kept), so the log can be shared without revealing where it was flown.
    Sanitize {
        /// Path to the .bin log.
        path: std::path::PathBuf,
        /// Output .bin file.
        #[arg(long)]
        out: std::path::PathBuf,
        /// Stand-in location "lat,lon" (default: ArduPilot's SITL home, Canberra).
        #[arg(
            long,
            default_value = "-35.363261,149.165230",
            allow_hyphen_values = true
        )]
        to: String,
    },
    /// Write a smaller copy holding only a time window (default: the armed time with a
    /// margin), with the formats, parameters and boot messages that explain it.
    Trim {
        /// Path to the .bin log.
        path: std::path::PathBuf,
        /// Output .bin file.
        #[arg(long)]
        out: std::path::PathBuf,
        /// Window start, seconds since boot (instead of the armed time).
        #[arg(long, requires = "until")]
        from: Option<f64>,
        /// Window end, seconds since boot.
        #[arg(long, requires = "from")]
        until: Option<f64>,
        /// Seconds kept before the first arming and after the last disarm.
        #[arg(long, default_value_t = 10.0)]
        margin: f64,
    },
    /// Takeoffs: time from arming to liftoff, climb, attitude error just after liftoff.
    Takeoffs {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Landings: descent speed before touchdown, time to disarm.
    Landings {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// EKF lane switches (primary core changes), flagged when armed.
    Lanes {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Battery cell balance while flying (packs that report cell voltages).
    Cells {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Autopilot performance: loop rate and overruns, processor load, memory, internal errors.
    Perf {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// ESC telemetry while armed: RPM, error rate, temperature and current per ESC.
    Esc {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Compass interference: field strength against battery current while flying.
    Compass {
        /// Path to the .bin log.
        path: std::path::PathBuf,
        /// Compass instance (0 = first).
        #[arg(long, default_value_t = 0)]
        instance: u8,
    },
    /// The battery under load while flying: current, power, voltage sag, use, resistance.
    Battery {
        /// Path to the .bin log.
        path: std::path::PathBuf,
        /// Battery instance (0 = first).
        #[arg(long, default_value_t = 0)]
        instance: u8,
    },
    /// Attitude tracking while flying: desired minus achieved roll, pitch and yaw.
    Attitude {
        /// Path to the .bin log.
        path: std::path::PathBuf,
    },
    /// Vibration spectrum (Welch FFT) of an IMU axis while armed, and its strongest peaks.
    Fft {
        /// Path to the .bin log.
        path: std::path::PathBuf,
        /// Message and field, e.g. ACC.AccX or GYR.GyrZ.
        #[arg(long, default_value = "ACC.AccZ")]
        field: String,
        /// IMU instance.
        #[arg(long, default_value_t = 0)]
        imu: u8,
        /// Samples per FFT segment (any length; powers of two are fastest).
        #[arg(long, default_value_t = 1024)]
        segment: usize,
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
        /// Print the findings as JSON (schema `dc.health/1`) for scripts and CI.
        #[arg(long)]
        json: bool,
        /// Exit with an error when any finding is at this level or worse.
        #[arg(long, value_enum)]
        fail_on: Option<FailOn>,
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
    /// Change parameters through the daemon (it must run with --param-dir): confirm once; if
    /// the vehicle is armed, acknowledge its in-flight warning too.
    SetParams {
        /// Changes as NAME=VALUE.
        #[arg(required = true, value_name = "NAME=VALUE")]
        changes: Vec<String>,
        /// Confirm the changes without asking (scripts).
        #[arg(long)]
        yes: bool,
        /// Acknowledge the in-flight warning without asking (scripts).
        #[arg(long)]
        armed_ok: bool,
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
    /// The daemon's router: links, routes (which link each system is heard on), counters.
    Links,
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

/// Ask a yes/no question on the terminal; `None` when there is no terminal to ask on.
fn ask(question: &str) -> Option<bool> {
    use std::io::{BufRead, IsTerminal, Write};
    if !std::io::stdin().is_terminal() {
        return None;
    }
    eprint!("{question} [y/N] ");
    let _ = std::io::stderr().flush();
    let mut line = String::new();
    std::io::stdin().lock().read_line(&mut line).ok()?;
    Some(matches!(line.trim(), "y" | "Y" | "yes" | "YES"))
}

#[allow(clippy::too_many_arguments)]
async fn param_set(
    cli: &Cli,
    connect: &LinkSpec,
    target: Option<u8>,
    changes: &[String],
    yes: bool,
    armed_ok: bool,
    snapshot: Option<&std::path::Path>,
    audit: &std::path::Path,
    wait: u64,
) -> Result<(), String> {
    let mut edits = std::collections::BTreeMap::new();
    for c in changes {
        let (name, value) = c
            .split_once('=')
            .ok_or_else(|| format!("{c:?}: expected NAME=VALUE"))?;
        let value: f64 = value
            .trim()
            .parse()
            .map_err(|_| format!("{c:?}: value is not a number"))?;
        edits.insert(name.trim().to_ascii_uppercase(), value);
    }
    let cfg = CoreConfig {
        sysid: cli.sysid,
        compid: cli.compid,
        tlog: cli.tlog.clone(),
        audit_log: Some(audit.to_path_buf()),
        ..Default::default()
    };
    let core = Core::start(cfg)
        .await
        .map_err(|e| format!("cannot start core: {e}"))?;
    core.connect(connect)
        .await
        .map_err(|e| format!("cannot open {connect}: {e}"))?;
    let sysid = match target {
        Some(t) => t,
        None => core
            .wait_for_vehicle(Duration::from_secs(wait))
            .await
            .ok_or_else(|| format!("no vehicle heartbeat on {connect} within {wait} s"))?,
    };
    let armed = core.facts(sysid).armed == Some(true);
    eprintln!(
        "vehicle {sysid}{}: {} change(s)",
        if armed { " (ARMED)" } else { "" },
        edits.len()
    );
    for (name, value) in &edits {
        let label = match dc_core::parameter_risk(name) {
            dc_core::ParamRisk::Dangerous => "  (failsafe/arming/fence/safety setting)",
            dc_core::ParamRisk::Ordinary => "",
        };
        eprintln!("  {name} = {value}{label}");
    }
    if !yes {
        match ask("Write these parameters to the vehicle?") {
            Some(true) => {}
            Some(false) => return Err("not confirmed: nothing written".into()),
            None => return Err("no terminal to confirm on: pass --yes".into()),
        }
    }
    let path = snapshot.map_or_else(
        || {
            let t = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_secs());
            std::path::PathBuf::from(format!("param-baseline-{sysid}-{t}.param"))
        },
        std::path::Path::to_path_buf,
    );
    let mut mode = if armed_ok {
        if armed {
            eprintln!("WARNING: {}", dc_core::ARMED_WRITE_WARNING);
        }
        dc_core::ArmedWrite::Acknowledged
    } else {
        dc_core::ArmedWrite::DisarmedOnly
    };
    let report = loop {
        match core
            .write_params_with((sysid, 1), edits.clone(), &path, mode)
            .await
        {
            Ok(r) => break r,
            Err(dc_core::ParamOperationError::ArmedWarningRequired(w)) => {
                eprintln!("WARNING: {w}");
                match ask("The vehicle is ARMED. Write anyway, in flight?") {
                    Some(true) => mode = dc_core::ArmedWrite::Acknowledged,
                    Some(false) => {
                        return Err("in-flight warning not acknowledged: nothing written".into());
                    }
                    None => {
                        return Err(
                            "vehicle armed: pass --armed-ok to acknowledge the in-flight warning"
                                .into(),
                        );
                    }
                }
            }
            Err(e) => return Err(e.to_string()),
        }
    };
    eprintln!("baseline saved to {}", report.snapshot.display());
    for r in &report.results {
        println!("{} {:?}", r.name, r.status);
    }
    if let Some(why) = &report.interruption {
        eprintln!("stopped: {why}");
    }
    if report.verified() {
        println!("all {} change(s) verified by read-back", report.planned);
        Ok(())
    } else {
        Err(format!(
            "{} of {} change(s) verified; the baseline holds the previous values",
            report
                .results
                .iter()
                .filter(|r| matches!(r.status, protocols::params::SetStatus::Verified(_)))
                .count(),
            report.planned
        ))
    }
}

async fn run(cli: Cli) -> Result<(), String> {
    match &cli.cmd {
        Cmd::ParamSet {
            connect,
            target,
            changes,
            yes,
            armed_ok,
            snapshot,
            audit,
            wait,
        } => {
            param_set(
                &cli,
                connect,
                *target,
                changes,
                *yes,
                *armed_ok,
                snapshot.as_deref(),
                audit,
                *wait,
            )
            .await
        }
        Cmd::Params {
            connect,
            target,
            component,
            out,
            compare,
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
            if let Some(file) = compare {
                let on_vehicle: std::collections::BTreeMap<String, f64> = params
                    .iter()
                    .map(|(k, p)| (k.clone(), f64::from(p.value)))
                    .collect();
                let same = print_param_diff(
                    &file.display().to_string(),
                    &load_params(file)?,
                    &format!("vehicle {sysid}"),
                    &on_vehicle,
                );
                return if same {
                    Ok(())
                } else {
                    Err("the vehicle differs from the file".into())
                };
            }
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
        Cmd::Daemon {
            connect,
            listen,
            token_file,
            param_dir,
            audit,
        } => {
            if let Some(d) = param_dir
                && !d.is_dir()
            {
                return Err(format!("--param-dir {}: not a directory", d.display()));
            }
            let token = api_token(token_file.as_deref())?;
            if token.is_none() && !listen.ip().is_loopback() {
                return Err(format!(
                    "listening on {listen} needs an API key: --token-file FILE or DC_API_TOKEN"
                ));
            }
            let cfg = CoreConfig {
                sysid: cli.sysid,
                compid: cli.compid,
                tlog: cli.tlog.clone(),
                audit_log: Some(audit.clone()),
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
                r = api::serve_with(core, *listen, api::ServeOptions { token, param_dir: param_dir.clone() }) => r.map_err(|e| format!("gRPC server: {e}")),
                _ = shutdown_signal() => Ok(()),
            }
        }
        Cmd::Remote {
            grpc,
            token_file,
            what,
        } => remote(grpc, token_file.as_deref(), what).await,
        Cmd::Ports { watch, probe } => ports(*watch, probe.as_deref()),
        Cmd::Log {
            what: what @ (LogCmd::List { .. } | LogCmd::Download { .. }),
        } => log_transfer(&cli, what).await,
        Cmd::Log { what } => log_cmd(what),
        Cmd::Mission { what } => mission_cmd(&cli, what).await,
        Cmd::Tlog { what } => tlog_cmd(what),
        Cmd::ParamsDiff { a, b } => params_diff(a, b),
        Cmd::ParamsReview { file } => {
            print_params_review(&load_params(file)?);
            Ok(())
        }
        Cmd::Soak {
            tx,
            rx,
            baud,
            seconds,
        } => soak(tx, rx.as_deref().unwrap_or(tx), *baud, *seconds),
        Cmd::Cot {
            connect,
            udp,
            rate,
            seconds,
        } => {
            if !(rate.is_finite() && *rate > 0.0 && *rate <= 20.0) {
                return Err("--rate must be between 0 and 20 events per second".into());
            }
            let core = start(&cli, connect).await?;
            let sock = match udp {
                Some(addr) => {
                    let bind: std::net::SocketAddr = if addr.is_ipv4() {
                        "0.0.0.0:0".parse().unwrap()
                    } else {
                        "[::]:0".parse().unwrap()
                    };
                    let s = tokio::net::UdpSocket::bind(bind)
                        .await
                        .map_err(|e| format!("udp: {e}"))?;
                    Some((s, *addr))
                }
                None => None,
            };
            let mut tick = tokio::time::interval(Duration::from_secs_f64(1.0 / rate));
            let end = (*seconds > 0).then(|| Instant::now() + Duration::from_secs(*seconds));
            let mut sent = 0u64;
            loop {
                tokio::select! {
                    _ = tick.tick() => {}
                    _ = shutdown_signal() => break,
                }
                if end.is_some_and(|e| Instant::now() >= e) {
                    break;
                }
                let now_us = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_or(0, |d| d.as_micros() as u64);
                for v in core.fleet().vehicles() {
                    let Some(p) = &v.position else { continue };
                    let p = &p.value;
                    let (vn, ve) = (f64::from(p.vel_ned[0]), f64::from(p.vel_ned[1]));
                    let track = protocols::cot::CotTrack {
                        uid: format!("DrorControl-sys{}", v.sysid),
                        callsign: format!("UAV {}", v.sysid),
                        rotary: v.confirmed_firmware() != Some(dc_core::Firmware::Plane),
                        lat: p.lat_deg,
                        lon: p.lon_deg,
                        alt_m: f64::from(p.alt_msl_m),
                        course_deg: ve.atan2(vn).to_degrees(),
                        speed_ms: vn.hypot(ve),
                    };
                    let xml = protocols::cot::event(&track, now_us, 10);
                    match &sock {
                        Some((s, addr)) => {
                            s.send_to(xml.as_bytes(), addr)
                                .await
                                .map_err(|e| format!("udp {addr}: {e}"))?;
                        }
                        None => println!("{xml}"),
                    }
                    sent += 1;
                }
            }
            eprintln!("{sent} CoT events");
            Ok(())
        }
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

fn print_log_summary(log: &logs::Log) -> Result<(), String> {
    let s = logs::flight::summary(log);
    println!("flights: {}", s.armed.len());
    for p in &s.armed {
        let end = p.end_us.map_or("log end".to_string(), |e| {
            format!("{:.1} s", e as f64 / 1e6)
        });
        println!("  armed {:.1} s -> {end}", p.start_us as f64 / 1e6);
    }
    println!("armed time: {:.1} s", s.armed_s);
    let opt =
        |v: Option<f64>, unit: &str| v.map_or("not logged".into(), |v| format!("{v:.1} {unit}"));
    println!("highest above home: {}", opt(s.max_alt_rel_m, "m"));
    println!("top ground speed: {}", opt(s.max_speed_ms, "m/s"));
    println!("distance flown: {:.0} m", s.distance_m);
    println!("furthest from home: {}", opt(s.max_from_home_m, "m"));
    println!(
        "fastest climb: {}, fastest descent: {}",
        opt(s.max_climb_ms, "m/s"),
        opt(s.max_descent_ms, "m/s")
    );
    println!("battery used: {}", opt(s.used_mah, "mAh"));
    println!("lowest battery voltage: {}", opt(s.min_voltage, "V"));
    let fw = log_firmware(&logs::flight::events(log));
    let modes: Vec<(u8, f64)> = logs::flight::mode_times(log)
        .into_iter()
        .filter(|(_, t)| *t >= 0.05)
        .collect();
    if !modes.is_empty() {
        println!("armed time per mode:");
        for (m, t) in modes {
            println!("  {:<12} {t:7.1} s", mode_label(fw, m));
        }
    }
    Ok(())
}

fn print_log_motors(log: &logs::Log) -> Result<(), String> {
    if let Some(h) = logs::motors::hover_throttle(log) {
        let pct = |v: Option<f64>| v.map_or("-".into(), |v| format!("{:.1} %", v * 100.0));
        println!(
            "hover throttle: learned {} (MOT_THST_HOVER {}), level flight {} over {} samples{}",
            pct(h.learned),
            pct(h.param),
            pct(h.mean_level),
            h.samples,
            match h.at_limit {
                Some("upper") => " -- at the upper learning limit: underpowered or overloaded?",
                Some(_) => " -- at the lower learning limit: very overpowered",
                None => "",
            }
        );
    }
    let m = logs::motors::motor_balance(log)
        .ok_or("no motor outputs in flight (SERVOn_FUNCTION motors and RCOU needed)")?;
    println!(
        "{} samples while {}",
        m.samples,
        if m.land_detector {
            "flying (land detector)"
        } else {
            "armed (no land-detector events)"
        }
    );
    for o in &m.motors {
        println!(
            "  motor {:2} (SERVO{:<2}) {:6.0} us  {:5.1} %",
            o.motor,
            o.channel,
            o.mean_pwm,
            o.mean_output * 100.0
        );
    }
    if let Some((motor, dev)) = m.worst() {
        println!(
            "largest deviation: motor {motor} {:+.1} % from the mean",
            dev * 100.0
        );
    }
    if let Some(d) = m.cw_minus_ccw {
        println!(
            "quad: clockwise motors (3, 4) {:+.1} points of output vs counter-clockwise (1, 2)",
            d * 100.0
        );
    }
    Ok(())
}

fn print_log_rc(log: &logs::Log) -> Result<(), String> {
    let l = logs::rc::rc_link(log);
    let mm = |v: Option<(f64, f64)>| {
        v.map_or("not reported".into(), |(m, lo)| {
            format!("mean {m:.0} %, lowest {lo:.0} %")
        })
    };
    println!(
        "while armed ({} samples): RSSI {}; link quality {}",
        l.rssi_samples,
        mm(l.rssi_pct),
        mm(l.lq_pct)
    );
    println!(
        "radio failsafes: {}; late frames: {}",
        l.failsafes.len(),
        l.late_frames
    );
    for (start, end) in &l.failsafes {
        println!(
            "  failsafe at {:.1} s, {}",
            *start as f64 / 1e6,
            end.map_or("not resolved by the end of the log".into(), |e| format!(
                "resolved after {:.1} s",
                (e - start) as f64 / 1e6
            ))
        );
    }
    Ok(())
}

fn print_log_sensors(log: &logs::Log) -> Result<(), String> {
    let pct = |x: f64| x * 100.0;
    match logs::sensors::flow(log) {
        Some(f) => println!(
            "optical flow: {} samples, quality mean {:.0} (lowest {:.0}); usable by the EKF {:.1} %, quality 0 {:.1} %, rate >= 4.2 rad/s {:.1} %",
            f.samples,
            f.mean_quality,
            f.min_quality,
            pct(f.usable),
            pct(f.zero_quality),
            pct(f.too_fast)
        ),
        None => println!("optical flow: no OF samples in flight"),
    }
    let rf = logs::sensors::rangefinders(log);
    if rf.is_empty() {
        println!("rangefinder: no RFND samples in flight");
    }
    for r in rf {
        let s = r.status_share;
        println!(
            "rangefinder {} (orientation {}): {} samples; good {:.1} %, out of range low {:.1} %, high {:.1} %, no data {:.1} %, not connected {:.1} %{}",
            r.instance,
            r.orientation,
            r.samples,
            pct(s[4]),
            pct(s[2]),
            pct(s[3]),
            pct(s[1]),
            pct(s[0]),
            r.good_range_m.map_or(String::new(), |(a, b)| format!(
                "; good readings {a:.2} to {b:.2} m"
            ))
        );
    }
    Ok(())
}

fn print_log_wind(log: &logs::Log) -> Result<(), String> {
    let w = logs::wind::wind(log).ok_or(
        "no wind estimate in flight (a multicopter's EKF estimates wind only with drag fusion: EK3_DRAG_BCOEF_X/Y)",
    )?;
    println!(
        "mean wind {:.1} m/s from {:.0} deg; strongest {:.1} m/s at {:.1} s ({} samples)",
        w.mean_speed_ms,
        w.mean_from_deg,
        w.max_speed_ms,
        w.max_at_us as f64 / 1e6,
        w.samples
    );
    Ok(())
}

fn print_log_attitude(log: &logs::Log) -> Result<(), String> {
    let a = logs::tuning::attitude_tracking(log).ok_or("no ATT samples in flight")?;
    println!(
        "{} samples while {}",
        a.samples,
        if a.land_detector {
            "flying (land detector)"
        } else {
            "armed (no land-detector events)"
        }
    );
    for (name, e) in [("roll", a.roll), ("pitch", a.pitch), ("yaw", a.yaw)] {
        println!(
            "  {name:<5} error rms {:5.2} deg, max {:6.2} deg at {:.1} s",
            e.rms_deg,
            e.max_deg,
            e.max_at_us as f64 / 1e6
        );
    }
    Ok(())
}

fn print_log_changes(log: &logs::Log) -> Result<(), String> {
    let changes = logs::flight::param_changes(log);
    for c in &changes {
        println!(
            "{:10.3} s  {} {} -> {}{}",
            c.time_us as f64 / 1e6,
            c.name,
            c.old as f32,
            c.new as f32,
            if c.automatic {
                "  (updated by the autopilot)"
            } else {
                ""
            }
        );
    }
    eprintln!("{} changes", changes.len());
    Ok(())
}

/// Health level that fails `log health --fail-on`.
#[derive(Debug, Clone, Copy, clap::ValueEnum)]
enum FailOn {
    Caution,
    Warning,
}

fn level_name(l: logs::health::Level) -> &'static str {
    match l {
        logs::health::Level::Ok => "ok",
        logs::health::Level::Caution => "caution",
        logs::health::Level::Warning => "warning",
    }
}

/// `log health --json`: `{"schema": "dc.health/1", "log", "overall", "findings": [{"check",
/// "level", "text", "at_s"}]}`; levels are `ok`, `caution`, `warning`; `overall` is the worst
/// (null without findings); `at_s` is seconds since boot or null.
fn health_json(path: &std::path::Path, findings: &[logs::health::Finding]) -> String {
    let worst = findings.iter().map(|f| f.level).max();
    let list: Vec<serde_json::Value> = findings
        .iter()
        .map(|f| {
            serde_json::json!({
                "check": f.check,
                "level": level_name(f.level),
                "text": f.text,
                "at_s": f.at_us.map(|t| t as f64 / 1e6),
            })
        })
        .collect();
    serde_json::to_string_pretty(&serde_json::json!({
        "schema": "dc.health/1",
        "log": path.display().to_string(),
        "overall": worst.map(level_name),
        "findings": list,
    }))
    .expect("JSON values serialise")
}

fn log_health(
    path: &std::path::Path,
    log: &logs::Log,
    json: bool,
    fail_on: Option<FailOn>,
) -> Result<(), String> {
    report_health(path, &logs::health::assess(log), json, fail_on)
}

/// Print findings as text or JSON; error if any reaches `fail_on`.
fn report_health(
    path: &std::path::Path,
    findings: &[logs::health::Finding],
    json: bool,
    fail_on: Option<FailOn>,
) -> Result<(), String> {
    if json {
        println!("{}", health_json(path, findings));
    } else {
        print_findings(findings);
    }
    let limit = match fail_on {
        None => return Ok(()),
        Some(FailOn::Caution) => logs::health::Level::Caution,
        Some(FailOn::Warning) => logs::health::Level::Warning,
    };
    let bad = findings.iter().filter(|f| f.level >= limit).count();
    if bad > 0 {
        return Err(format!(
            "{bad} finding(s) at {} or worse",
            level_name(limit)
        ));
    }
    Ok(())
}

fn print_log_health(log: &logs::Log) -> Result<(), String> {
    print_findings(&logs::health::assess(log));
    Ok(())
}

fn print_findings(findings: &[logs::health::Finding]) {
    if findings.is_empty() {
        println!("no checks apply: none of the messages the checks read were found");
    }
    for f in findings {
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

fn print_log_battery(log: &logs::Log, instance: &u8) -> Result<(), String> {
    let b = logs::battery::battery_use(log, *instance)
        .ok_or("no battery voltage and current while flying")?;
    let opt = |v: Option<f64>, unit: &str, digits: usize| {
        v.map_or("not logged".into(), |v| format!("{v:.digits$} {unit}"))
    };
    println!(
        "{} samples over {:.1} s {}",
        b.samples,
        b.flying_s,
        if b.land_detector {
            "of flight (land detector)"
        } else {
            "armed (no land-detector events)"
        }
    );
    println!(
        "current: max {:.1} A, mean {:.1} A",
        b.max_current_a, b.mean_current_a
    );
    println!(
        "power: max {:.0} W, mean {:.0} W",
        b.max_power_w, b.mean_power_w
    );
    println!(
        "voltage: lowest {:.2} V under load, largest sag {}",
        b.min_voltage,
        opt(b.max_sag_v, "V", 2)
    );
    println!(
        "used: {}, {} ({})",
        opt(b.used_mah, "mAh", 0),
        opt(b.used_wh, "Wh", 1),
        opt(b.mah_per_min(), "mAh/min", 0)
    );
    println!(
        "internal resistance: logged {}, fitted {}",
        opt(b.logged_resistance_ohm.map(|r| r * 1000.0), "mOhm", 1),
        opt(b.fitted_resistance_ohm.map(|r| r * 1000.0), "mOhm", 1)
    );
    Ok(())
}

fn print_log_compass(log: &logs::Log, instance: &u8) -> Result<(), String> {
    let c = logs::compass::compass_interference(log, *instance)
        .ok_or("needs MAG and BAT current while flying, with the current varying")?;
    println!(
        "compass {}: {} samples, mean field {:.0} mGauss",
        c.instance, c.samples, c.mean_field_mgauss
    );
    println!(
        "field strength changes {:+.2} mGauss per A (correlation {:+.2}); at the highest current ({:.1} A): {:+.1} % of the field",
        c.mgauss_per_amp, c.correlation, c.max_current_a, c.change_at_max_pct
    );
    println!("(field strength only, a lower bound of the on-ground compassmot figure)");
    Ok(())
}

/// Aliasing note for IMU data logged at `fs_hz` (plan 05 §2.2): the Nyquist limit, and where
/// the motors' rotation (ESC telemetry) and the configured notch frequency would appear if
/// they are above it.
fn print_aliasing(log: &logs::Log, fs_hz: f64) {
    let nyq = fs_hz / 2.0;
    let mut tones: Vec<(String, f64)> = Vec::new();
    let esc = logs::esc::esc_stats(log);
    if let Some(rpm) = esc.iter().map(|e| e.mean_rpm).max_by(f64::total_cmp)
        && rpm > 0.0
    {
        tones.push((
            format!("motor rotation (ESC mean {rpm:.0} rpm)"),
            rpm / 60.0,
        ));
    }
    let params = log.params();
    if params.get("INS_HNTCH_ENABLE").is_some_and(|v| *v > 0.0)
        && let Some(f) = params.get("INS_HNTCH_FREQ").copied().filter(|f| *f > 0.0)
    {
        tones.push(("INS_HNTCH_FREQ".into(), f));
    }
    let above: Vec<String> = tones
        .iter()
        .filter(|(_, f)| *f > nyq)
        .map(|(name, f)| {
            format!(
                "{name} {f:.0} Hz would appear at {:.0} Hz",
                logs::spectrum::alias(*f, fs_hz)
            )
        })
        .collect();
    if above.is_empty() {
        println!("logged at {fs_hz:.0} Hz: frequencies up to {nyq:.0} Hz are visible");
    } else {
        println!(
            "CHECK: logged at {fs_hz:.0} Hz, so nothing above {nyq:.0} Hz is visible and higher tones fold back (alias): {}. Log faster (batch sampler INS_LOG_BAT_MASK, or fast IMU logging) before trusting peaks for notch settings",
            above.join("; ")
        );
    }
}

fn print_log_notch(
    log: &logs::Log,
    field: &str,
    imu: &u8,
    segment: &usize,
    min_hz: &f64,
) -> Result<(), String> {
    let (msg, fld) = field
        .split_once('.')
        .ok_or_else(|| format!("--field {field:?}: expected MSG.FIELD"))?;
    let n = logs::notch::suggest(log, msg, fld, *imu, *segment, *min_hz)
        .ok_or("no steady flight with enough samples (needs CTUN while flying and raw IMU data)")?;
    println!(
        "steady flight analysed: {:.1} s, {} samples of {field}",
        n.span_s, n.samples
    );
    print_aliasing(log, n.sample_rate_hz);
    println!("suggested throttle-based harmonic notch (review before use; nothing is written):");
    println!("  INS_HNTCH_ENABLE 1");
    println!("  INS_HNTCH_MODE   1  (throttle)");
    println!(
        "  INS_HNTCH_FREQ   {:.0}  (Hz, peak at the reference throttle)",
        n.freq_hz
    );
    println!(
        "  INS_HNTCH_BW     {:.0}  (Hz, half the frequency)",
        n.bw_hz
    );
    println!(
        "  INS_HNTCH_REF    {:.2}  (throttle during the measurement)",
        n.ref_throttle
    );
    if !n.current.is_empty() {
        let cur: Vec<String> = n
            .current
            .iter()
            .map(|(k, v)| format!("{k} {}", *v as f32))
            .collect();
        println!("currently: {}", cur.join(", "));
    }
    Ok(())
}

fn print_params_review(params: &std::collections::BTreeMap<String, f64>) {
    let findings = protocols::params::review::review(params);
    if findings.is_empty() {
        println!("no safety net switched off among the reviewed parameters");
    }
    for f in findings {
        let tag = match f.level {
            protocols::params::review::Level::Off => "OFF ",
            protocols::params::review::Level::Note => "note",
        };
        println!("{tag}  {:<16} {}", f.param, f.text);
    }
}

fn print_log_imu(log: &logs::Log) -> Result<(), String> {
    let h = logs::imu::imu_health(log);
    if h.is_empty() {
        return Err("no IMU records".into());
    }
    for i in h {
        println!(
            "IMU {}: {} samples; temperature {:.1} to {:.1} C; errors gyro {} accel {}; unhealthy while armed {}; rates gyro {:.0} Hz accel {:.0} Hz",
            i.instance,
            i.samples,
            i.temp_c.0,
            i.temp_c.1,
            i.gyro_errors,
            i.accel_errors,
            i.unhealthy_armed,
            i.rates_hz.0,
            i.rates_hz.1
        );
    }
    Ok(())
}

fn print_log_takeoffs(log: &logs::Log) -> Result<(), String> {
    let k = logs::takeoff::takeoffs(log);
    if k.is_empty() {
        return Err("no liftoff reported by the land detector (EV 28)".into());
    }
    for x in k {
        println!(
            "liftoff at {:.1} s ({}): first {:.0} s climb up to {:.2} m/s, attitude error up to roll {:.1}° pitch {:.1}°{}",
            x.time_us as f64 / 1e6,
            x.from_arm_s
                .map_or("arming not in this log".into(), |s| format!(
                    "{s:.1} s after arming"
                )),
            logs::takeoff::WINDOW_US as f64 / 1e6,
            x.max_climb_ms,
            x.max_roll_err_deg,
            x.max_pitch_err_deg,
            if x.max_roll_err_deg.max(x.max_pitch_err_deg) > 10.0 {
                "  CHECK: over 10° (motor order/direction, centre of gravity, props)"
            } else {
                ""
            }
        );
    }
    Ok(())
}

fn print_log_landings(log: &logs::Log) -> Result<(), String> {
    let l = logs::landing::landings(log);
    if l.is_empty() {
        return Err("no touchdown reported by the land detector (EV 18)".into());
    }
    for x in l {
        println!(
            "touchdown at {:.1} s: descent over the last 3 s mean {:.2} m/s, fastest {:.2} m/s; {}",
            x.time_us as f64 / 1e6,
            x.mean_descent_ms,
            x.max_descent_ms,
            x.to_disarm_s
                .map_or("not disarmed in this log".into(), |s| format!(
                    "disarmed {s:.1} s later"
                ))
        );
    }
    Ok(())
}

fn print_log_lanes(log: &logs::Log) -> Result<(), String> {
    if log.format_by_name("XKF4").is_none() {
        return Err("no XKF4 records (EKF3 not logged)".into());
    }
    let s = logs::lanes::lane_switches(log);
    let armed = s.iter().filter(|l| l.armed).count();
    println!("{} lane switches ({armed} while armed)", s.len());
    for l in s {
        println!(
            "  {:.1} s  core {} -> {}{}",
            l.time_us as f64 / 1e6,
            l.from,
            l.to,
            if l.armed { "  ARMED" } else { "" }
        );
    }
    Ok(())
}

fn print_log_cells(log: &logs::Log) -> Result<(), String> {
    let packs = logs::cells::cell_balance(log);
    if packs.is_empty() {
        return Err("no cell voltages (BCL) while flying".into());
    }
    for p in packs {
        println!(
            "battery {}: {} cells, {} samples; spread mean {:.3} V, largest {:.3} V at {:.1} s; lowest cell {} at {:.3} V",
            p.instance,
            p.cells,
            p.samples,
            p.mean_spread_v,
            p.max_spread_v,
            p.max_spread_at_us as f64 / 1e6,
            p.lowest_cell,
            p.lowest_v
        );
    }
    Ok(())
}

fn print_log_perf(log: &logs::Log) -> Result<(), String> {
    let p = logs::perf::perf(log).ok_or("no PM records")?;
    println!(
        "main loop {:.0} Hz; long loops (over 1.2 x the period) {:.2} %; longest {:.1} ms at {:.1} s ({} PM records; the boot record left out)",
        p.loop_rate_hz,
        p.long_loop_pct,
        p.max_loop_ms,
        p.max_loop_at_us as f64 / 1e6,
        p.records
    );
    println!(
        "processor load highest {:.1} %; least free memory {:.0} bytes",
        p.max_load_pct, p.min_free_mem
    );
    if p.internal_errors > 0 {
        println!(
            "INTERNAL ERRORS: {} (mask {:#x}; AP_InternalError::error_t)",
            p.internal_errors, p.internal_error_mask
        );
    } else {
        println!("internal errors: none");
    }
    Ok(())
}

fn print_log_esc(log: &logs::Log) -> Result<(), String> {
    let stats = logs::esc::esc_stats(log);
    if stats.is_empty() {
        return Err("no ESC telemetry while armed".into());
    }
    for e in &stats {
        let opt = |v: Option<f64>, unit: &str| v.map_or("-".into(), |v| format!("{v:.1} {unit}"));
        println!(
            "ESC {}: {} samples; RPM mean {:.0}, highest {:.0}; error rate mean {:.1} %, highest {:.1} %; temperature highest {}; current mean {}",
            e.instance,
            e.samples,
            e.mean_rpm,
            e.max_rpm,
            e.mean_err_pct,
            e.max_err_pct,
            opt(e.max_temp_c, "C"),
            opt(e.mean_current_a, "A")
        );
    }
    Ok(())
}

fn print_log_altitude(log: &logs::Log) -> Result<(), String> {
    let show = |name: &str, g: Option<logs::altitude::AltGap>| match g {
        Some(g) => println!(
            "{name}: mean {:+.2} m, largest {:.2} m at {:.1} s, drift over the flight {:+.2} m ({} samples)",
            g.mean_m,
            g.max_abs_m,
            g.max_at_us as f64 / 1e6,
            g.drift_m,
            g.samples
        ),
        None => println!("{name}: not logged in flight"),
    };
    show("barometer - EKF height", logs::altitude::baro_vs_ekf(log));
    show("GPS - EKF altitude", logs::altitude::gps_vs_ekf(log));
    match logs::altitude::hold_error(log) {
        Some(h) => println!(
            "altitude hold (EKF height - target, altitude-controlled modes): mean {:+.2} m, largest {:.2} m at {:.1} s ({} samples)",
            h.mean_m,
            h.max_abs_m,
            h.max_at_us as f64 / 1e6,
            h.samples
        ),
        None => println!("altitude hold: no altitude-controlled flight logged"),
    }
    Ok(())
}

fn print_log_gps(log: &logs::Log) -> Result<(), String> {
    let q = logs::gps::quality(log).ok_or("no GPS samples while armed")?;
    println!(
        "{} samples while armed: satellites mean {:.1}, fewest {:.0}; HDOP mean {:.2}, highest {:.2}; 3D fix or better {:.1} %",
        q.samples,
        q.mean_sats,
        q.min_sats,
        q.mean_hdop,
        q.max_hdop,
        q.fix_3d() * 100.0
    );
    let fixes: Vec<String> = q
        .status_share
        .iter()
        .enumerate()
        .filter(|(_, s)| **s > 0.0)
        .map(|(i, s)| format!("{} {:.1} %", logs::gps::STATUS_NAMES[i], s * 100.0))
        .collect();
    println!("fix types: {}", fixes.join(", "));
    if let Some(d) = logs::gps::gps_vs_ekf(log) {
        println!(
            "GPS vs EKF position while flying: mean {:.1} m, largest {:.1} m at {:.1} s ({} samples)",
            d.mean_m,
            d.max_m,
            d.max_at_us as f64 / 1e6,
            d.samples
        );
    }
    println!("GPS glitches: {}", q.glitches.len());
    for (start, end) in &q.glitches {
        println!(
            "  at {:.1} s, {}",
            *start as f64 / 1e6,
            end.map_or("not resolved by the end of the log".into(), |e| format!(
                "resolved after {:.1} s",
                (e - start) as f64 / 1e6
            ))
        );
    }
    Ok(())
}

/// The figures `log compare` puts side by side, as (label, value) in a fixed order.
fn compare_rows(log: &logs::Log) -> Vec<(&'static str, String)> {
    let f = |v: Option<f64>, digits: usize| v.map_or("-".to_string(), |v| format!("{v:.digits$}"));
    let s = logs::flight::summary(log);
    let bat = logs::battery::battery_use(log, 0);
    let mot = logs::motors::motor_balance(log);
    let hov = logs::motors::hover_throttle(log);
    let att = logs::tuning::attitude_tracking(log);
    let vibe = ["VibeX", "VibeY", "VibeZ"]
        .iter()
        .filter_map(|k| log.column("VIBE", k, Some(("IMU", 0.0))))
        .flat_map(|c| c.values)
        .filter(|v| v.is_finite())
        .reduce(f64::max);
    vec![
        ("flights", s.armed.len().to_string()),
        ("armed time, s", f(Some(s.armed_s), 1)),
        ("highest above home, m", f(s.max_alt_rel_m, 1)),
        ("furthest from home, m", f(s.max_from_home_m, 0)),
        ("distance flown, m", f(Some(s.distance_m), 0)),
        ("top ground speed, m/s", f(s.max_speed_ms, 1)),
        ("fastest climb, m/s", f(s.max_climb_ms, 1)),
        ("fastest descent, m/s", f(s.max_descent_ms, 1)),
        ("battery used, mAh", f(s.used_mah, 0)),
        ("lowest voltage, V", f(s.min_voltage, 2)),
        (
            "mean current, A",
            f(bat.as_ref().map(|b| b.mean_current_a), 1),
        ),
        (
            "consumption, mAh/min",
            f(bat.as_ref().and_then(|b| b.mah_per_min()), 0),
        ),
        (
            "hover throttle (learned), %",
            f(hov.and_then(|h| h.learned).map(|h| h * 100.0), 1),
        ),
        (
            "worst motor deviation, %",
            f(mot.and_then(|m| m.worst()).map(|w| w.1 * 100.0), 1),
        ),
        (
            "roll error rms, deg",
            f(att.as_ref().map(|a| a.roll.rms_deg), 2),
        ),
        (
            "pitch error rms, deg",
            f(att.as_ref().map(|a| a.pitch.rms_deg), 2),
        ),
        (
            "yaw error rms, deg",
            f(att.as_ref().map(|a| a.yaw.rms_deg), 2),
        ),
        ("highest vibration, m/s2", f(vibe, 1)),
    ]
}

/// The firmware that wrote a log, from its version banner (modes are named only then).
fn log_firmware(events: &[(u64, logs::flight::Event)]) -> Option<dc_core::Firmware> {
    events.iter().find_map(|(_, e)| match e {
        logs::flight::Event::Text(t) => dc_core::Firmware::from_banner(t),
        _ => None,
    })
}

/// A mode's name if the firmware is known, else its number.
fn mode_label(fw: Option<dc_core::Firmware>, mode: u8) -> String {
    fw.and_then(|f| f.mode_name(u32::from(mode)))
        .map_or(format!("mode {mode}"), str::to_string)
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
        TlogCmd::Index { file, at } => {
            let open = || std::fs::File::open(file).map_err(|e| format!("{}: {e}", file.display()));
            let t0 = Instant::now();
            let (ix, err) = recorder::TlogIndex::build(std::io::BufReader::new(open()?));
            let Some((first, last)) = ix.span else {
                return Err(format!("{}: no records", file.display()));
            };
            println!(
                "{} records, {:.1} s, {} checkpoints, {} message types, {} key events; indexed in {:.0} ms",
                ix.records,
                (last - first) as f64 / 1e6,
                ix.checkpoints.len(),
                ix.counts.len(),
                ix.events.len(),
                t0.elapsed().as_secs_f64() * 1e3
            );
            if let Some(e) = err {
                println!("warning: stopped at a bad record ({e})");
            }
            for e in ix
                .events
                .iter()
                .filter(|e| !matches!(e.kind, recorder::KeyEventKind::Text(..)))
            {
                println!(
                    "  {:8.1} s  sys {}  {:?}",
                    (e.unix_us - first) as f64 / 1e6,
                    e.sysid,
                    e.kind
                );
            }
            if let Some(s) = at {
                let t = first + (s.max(0.0) * 1e6) as u64;
                let t1 = Instant::now();
                let snap = ix
                    .snapshot(&mut std::io::BufReader::new(open()?), t)
                    .map_err(|e| format!("{}: {e}", file.display()))?;
                println!(
                    "seek to {s:.1} s: offset {}, {} streams restored in {:.1} ms",
                    ix.offset_at(t),
                    snap.len(),
                    t1.elapsed().as_secs_f64() * 1e3
                );
                for r in &snap {
                    let f = &r.frame;
                    let name = mavlink_codec::message_info(f.msgid())
                        .map_or(format!("#{}", f.msgid()), |i| i.name.to_string());
                    println!(
                        "  {:8.1} s  {}/{}  {name}",
                        (r.unix_us - first) as f64 / 1e6,
                        f.sysid(),
                        f.compid()
                    );
                }
            }
            return Ok(());
        }
        TlogCmd::Health {
            file,
            json,
            fail_on,
        } => {
            let f = std::fs::File::open(file).map_err(|e| format!("{}: {e}", file.display()))?;
            let findings = tlog_health::assess(std::io::BufReader::new(f))
                .map_err(|e| format!("{}: {e}", file.display()))?;
            return report_health(file, &findings, *json, *fail_on);
        }
        TlogCmd::Sanitize { file, out, to } => {
            let to = parse_polygon(to)?
                .first()
                .copied()
                .ok_or("--to: expected lat,lon")?;
            if out == file {
                return Err("--out must differ from the input".into());
            }
            let f = std::fs::File::open(file).map_err(|e| format!("{}: {e}", file.display()))?;
            let (bytes, r, err) = tlog_sanitize::sanitize(std::io::BufReader::new(f), to);
            std::fs::write(out, bytes).map_err(|e| format!("{}: {e}", out.display()))?;
            match r.reference {
                Some(from) => println!(
                    "moved {} position values in {} of {} frames from around {:.4},{:.4} to {:.4},{:.4}; wrote {}",
                    r.values,
                    r.changed,
                    r.frames,
                    from.0,
                    from.1,
                    to.0,
                    to.1,
                    out.display()
                ),
                None => println!(
                    "no GLOBAL_POSITION_INT/GPS_RAW_INT fix to move relative to: zeroed {} position values in {} of {} frames; wrote {}",
                    r.values,
                    r.changed,
                    r.frames,
                    out.display()
                ),
            }
            if !r.fields.is_empty() {
                let f: Vec<&str> = r.fields.iter().map(String::as_str).collect();
                println!("fields: {}", f.join(", "));
            }
            if r.unsigned > 0 {
                println!(
                    "{} signed frames changed and are now unsigned (no key to re-sign)",
                    r.unsigned
                );
            }
            if r.unknown > 0 {
                println!(
                    "warning: {} frames of messages outside the dialect copied unchanged (not checked for positions)",
                    r.unknown
                );
            }
            if let Some(e) = err {
                println!("warning: stopped at a bad record ({e}); the rest was not written");
            }
            println!("note: altitudes and times are unchanged");
            return Ok(());
        }
        TlogCmd::Changes { file, vehicle } => {
            let f = std::fs::File::open(file).map_err(|e| format!("{}: {e}", file.display()))?;
            let (sys, changes) = tlog::param_changes(std::io::BufReader::new(f), *vehicle)
                .map_err(|e| format!("{}: {e}", file.display()))?
                .ok_or_else(|| format!("{}: no PARAM_VALUE in this recording", file.display()))?;
            for c in &changes {
                println!(
                    "{}  {:<16} {} -> {}",
                    tlog::utc(c.unix_us),
                    c.name,
                    c.old,
                    c.new
                );
            }
            eprintln!("vehicle {sys}: {} parameter changes", changes.len());
            return Ok(());
        }
        TlogCmd::Csv {
            file,
            msg,
            vehicle,
            out,
        } => {
            let f = std::fs::File::open(file).map_err(|e| format!("{}: {e}", file.display()))?;
            let (text, rows) = tlog::csv(std::io::BufReader::new(f), msg, *vehicle)
                .map_err(|e| format!("{}: {e}", file.display()))?;
            if rows == 0 {
                return Err(format!("{}: no {msg} messages", file.display()));
            }
            return write_or_print(out.as_deref(), &text, &format!("{rows} {msg} rows"));
        }
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
        TlogCmd::Mission { file, kind, out } => {
            let (ty, what) = match kind {
                ListKind::Mission => (0, "mission"),
                ListKind::Fence => (1, "fence"),
                ListKind::Rally => (2, "rally list"),
            };
            let f = std::fs::File::open(file).map_err(|e| format!("{}: {e}", file.display()))?;
            let ((sys, comp), items) = tlog::mission(std::io::BufReader::new(f), ty)
                .map_err(|e| format!("{}: {e}", file.display()))?
                .ok_or_else(|| {
                    format!("{}: no complete {what} in this recording", file.display())
                })?;
            if ty == 0 {
                print_stats(&items);
            } else {
                print_mission(&items);
            }
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
    for (sys, f) in &s.flights {
        if !f.mode_s.is_empty() {
            let modes: Vec<String> = f
                .mode_s
                .iter()
                .map(|(m, t)| format!("{m} {t:.1} s"))
                .collect();
            println!("  vehicle {sys} armed time per mode: {}", modes.join(", "));
        }
    }
    for ((sys, comp), n) in &s.sources {
        let loss = s
            .sequences
            .get(&(*sys, *comp))
            .and_then(|q| Some((q.lost, q.loss_pct()?)))
            .map_or(String::new(), |(lost, pct)| {
                format!(", {lost} lost in transit ({pct:.1} %)")
            });
        println!("  source {sys}/{comp}: {n} frames{loss}");
    }
    let lost_s = dc_core::CoreConfig::default().link_lost.as_secs_f64();
    for (sys, gaps) in &s.link_gaps {
        let lost = gaps.iter().filter(|g| g.1 > lost_s).count();
        println!(
            "  vehicle {sys}: {} heartbeat gaps over 1.5 s (STALE), {lost} over {lost_s:.0} s (LOST)",
            gaps.len()
        );
        for (at, len) in gaps {
            println!(
                "    {}  {len:.1} s{}",
                tlog::utc(*at),
                if *len > lost_s { "  LOST" } else { "" }
            );
        }
    }
    for ((sys, comp), r) in &s.radios {
        let sig = |v: Option<(u8, f64)>| {
            v.map_or("unknown".to_string(), |(lo, m)| {
                format!("mean {m:.0}, lowest {lo}")
            })
        };
        let margin = |v: Option<i16>| v.map_or("unknown".to_string(), |m| m.to_string());
        println!(
            "  radio {sys}/{comp}: {} reports; signal local {}, remote {}; smallest margin over noise local {}, remote {} (radio units, about 2 per dB on SiK); {} receive errors, {} corrected; lowest free tx buffer {}",
            r.reports,
            sig(r.rssi),
            sig(r.remrssi),
            margin(r.margin),
            margin(r.rem_margin),
            r.rx_errors,
            r.fixed,
            r.min_txbuf.map_or("-".to_string(), |t| format!("{t} %"))
        );
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
    for (sys, f) in &s.flights {
        println!(
            "system {sys}: armed {} times, {:.1} s; {:.0} m flown; top ground speed {:.1} m/s",
            f.arms, f.armed_s, f.distance_m, f.max_speed_ms
        );
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
/// Polygon vertices from "lat,lon;lat,lon;...".
fn parse_polygon(text: &str) -> Result<Vec<(f64, f64)>, String> {
    text.split(';')
        .map(|v| {
            let (a, b) = v
                .split_once(',')
                .ok_or_else(|| format!("vertex {v:?}: expected lat,lon"))?;
            let num = |t: &str| {
                t.trim()
                    .parse::<f64>()
                    .map_err(|_| format!("vertex {v:?}: {t:?} is not a number"))
            };
            Ok((num(a)?, num(b)?))
        })
        .collect()
}

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
    let open = || std::fs::File::open(file).map_err(|e| format!("{}: {e}", file.display()));
    let t = track::from_tlog_modes(std::io::BufReader::new(open()?), sysid)
        .map_err(|e| format!("{}: {e}", file.display()))?
        .filter(|t| !t.points.is_empty())
        .ok_or_else(|| format!("{}: no vehicle position in this recording", file.display()))?;
    let sys = t.sysid;
    let points: Vec<track::Point> = t.points.iter().map(|(_, p)| *p).collect();
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
        let segments = track::split_by_mode(&t.points, |m| {
            if m == u32::MAX {
                "before the first heartbeat".into()
            } else {
                t.firmware
                    .and_then(|f| f.mode_name(m))
                    .map_or(format!("mode {m}"), str::to_string)
            }
        });
        let mission = tlog::mission(std::io::BufReader::new(open()?), 0)
            .ok()
            .flatten()
            .map(|(_, items)| track::waypoints(&items))
            .unwrap_or_default();
        write(p, track::kml_rich(&name, &segments, &mission, "absolute"))?;
        eprintln!(
            "  {} mode segments, {} waypoints",
            segments.len(),
            mission.len()
        );
    }
    Ok(())
}

/// Track points of a dataflash log tagged with the flight mode (`MODE` records; `u32::MAX`
/// before the first), and the mission waypoints from `CMD` records (latest per number).
fn bin_kml_parts(log: &logs::Log) -> (Vec<(u32, track::Point)>, Vec<track::Waypoint>) {
    let modes: Vec<(u64, u32)> = logs::flight::events(log)
        .into_iter()
        .filter_map(|(t, e)| match e {
            logs::flight::Event::Mode { mode, .. } => Some((t, u32::from(mode))),
            _ => None,
        })
        .collect();
    let points = logs::track::track(log)
        .into_iter()
        .map(|p| {
            let i = modes.partition_point(|(t, _)| *t <= p.time_us);
            (
                if i == 0 { u32::MAX } else { modes[i - 1].1 },
                track::Point {
                    unix_us: p.unix_us,
                    lat: p.lat,
                    lon: p.lon,
                    alt_msl: p.alt_msl,
                },
            )
        })
        .collect();
    let col = |f: &str| log.column("CMD", f, None);
    let mut wps: std::collections::BTreeMap<u16, track::Waypoint> = Default::default();
    if let (Some(n), Some(la), Some(lo), Some(al)) =
        (col("CNum"), col("Lat"), col("Lng"), col("Alt"))
    {
        for k in 0..n.values.len() {
            if la.values[k] != 0.0 || lo.values[k] != 0.0 {
                let seq = n.values[k] as u16;
                wps.insert(
                    seq,
                    track::Waypoint {
                        seq,
                        lat: la.values[k],
                        lon: lo.values[k],
                        alt_m: al.values[k],
                    },
                );
            }
        }
    }
    (points, wps.into_values().collect())
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

/// Path length, altitudes and ending of a mission (an estimate), then the lint findings.
fn print_stats(items: &[dc_core::MissionItem]) {
    print_stats_only(items);
    for f in protocols::mission::lint(items) {
        let at = f
            .seq
            .map_or_else(|| "mission".to_string(), |s| format!("item {s}"));
        eprintln!("{:?}: {at}: {}", f.level, f.text);
    }
}

fn print_time_estimate(
    items: &[dc_core::MissionItem],
    speeds: &protocols::mission::Speeds,
    from_vehicle: bool,
) {
    let e = protocols::mission::estimate_time(items, speeds);
    let t = e.total_s.round() as u64;
    eprintln!(
        "flight time at least {}:{:02} (holds {:.0} s) at {:.1} m/s, climb {:.1}, descent {:.1} ({}); acceleration, turns and wind add time (a short SITL test mission took 1.5x as long){}{}",
        t / 60,
        t % 60,
        e.hold_s,
        speeds.horizontal_ms,
        speeds.up_ms,
        speeds.down_ms,
        if from_vehicle {
            "vehicle parameters"
        } else {
            "ArduCopter 4.7 defaults; --params for the vehicle's"
        },
        if e.alt_unknown {
            "; climbs to MSL/terrain altitudes not counted"
        } else {
            ""
        },
        if e.open_ended {
            "; DO_JUMP repeats and LOITER_UNLIM not counted"
        } else {
            ""
        }
    );
}

fn print_stats_only(items: &[dc_core::MissionItem]) {
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
                let items = download_mission(cli, vehicle, kind).await?;
                eprintln!("{} {kind:?} items", items.len());
                print_mission(&items);
                if let Some(path) = out {
                    // The items as the vehicle holds them (mission protocol), in the same
                    // QGC WPL 110 text; DrorControl reads it back. Mission Planner's own fence
                    // and rally file formats are different and are not written here.
                    let text = protocols::wpl::to_string(&items);
                    std::fs::write(path, text).map_err(|e| format!("{}: {e}", path.display()))?;
                    eprintln!(
                        "wrote {} (QGC WPL 110 with {kind:?} items; not Mission Planner's fence/rally format)",
                        path.display()
                    );
                }
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
        MissionCmd::Survey {
            polygon,
            polygon_kml,
            spacing,
            camera,
            sidelap,
            frontlap,
            trigger,
            crosshatch,
            overshoot,
            angle,
            alt,
            out,
        } => {
            let fp = match camera {
                Some(c) => {
                    let v: Vec<f64> = c
                        .split(',')
                        .map(|t| t.trim().parse::<f64>())
                        .collect::<Result<_, _>>()
                        .map_err(|_| format!("--camera {c:?}: expected 5 numbers"))?;
                    let [w, h, f, px_w, px_h] = v[..] else {
                        return Err(format!("--camera {c:?}: expected 5 numbers"));
                    };
                    let cam = protocols::survey::Camera {
                        sensor_w_mm: w,
                        sensor_h_mm: h,
                        focal_mm: f,
                        image_w_px: px_w as u32,
                        image_h_px: px_h as u32,
                    };
                    let fp = protocols::survey::footprint(
                        &cam,
                        f64::from(*alt),
                        sidelap / 100.0,
                        frontlap / 100.0,
                    )
                    .ok_or("camera, altitude or overlaps out of range (overlaps 0 to <100 %)")?;
                    eprintln!(
                        "ground resolution {:.2} cm/px; each photo covers {:.0} x {:.0} m (flat ground)",
                        fp.gsd_cm, fp.across_m, fp.along_m
                    );
                    Some(fp)
                }
                None => None,
            };
            let spacing = match (spacing, fp) {
                (Some(s), _) => *s,
                (None, Some(fp)) => fp.spacing_m,
                (None, None) => return Err("--spacing or --camera is needed".into()),
            };
            let polygon = match (polygon, polygon_kml) {
                (_, Some(k)) => {
                    let text =
                        std::fs::read_to_string(k).map_err(|e| format!("{}: {e}", k.display()))?;
                    let p = protocols::kml::first_polygon(&text)
                        .map_err(|e| format!("{}: {e}", k.display()))?;
                    eprintln!("{}: polygon of {} vertices", k.display(), p.len());
                    p
                }
                (Some(p), None) => parse_polygon(p)?,
                (None, None) => return Err("--polygon or --polygon-kml is needed".into()),
            };
            let cfg_polygon = polygon.clone();
            let items = protocols::survey::mission(&protocols::survey::SurveyConfig {
                polygon,
                spacing_m: spacing,
                angle_deg: *angle,
                alt_m: *alt,
                trigger_m: fp.filter(|_| *trigger).map(|f| f.trigger_m),
                crosshatch: *crosshatch,
                overshoot_m: *overshoot,
            })
            .map_err(|e| e.to_string())?;
            print_stats(&items);
            eprintln!(
                "area {:.2} ha",
                protocols::survey::area_m2(&cfg_polygon) / 10_000.0
            );
            print_time_estimate(&items, &protocols::mission::Speeds::default(), false);
            if let Some(fp) = fp {
                let points: Vec<(f64, f64)> = items[2..items.len() - 1]
                    .iter()
                    .filter(|i| i.command == 16)
                    .map(|i| (f64::from(i.x) * 1e-7, f64::from(i.y) * 1e-7))
                    .collect();
                let lines = points.len() / 2;
                let length = protocols::survey::line_length_m(&points);
                eprintln!(
                    "lines {spacing:.1} m apart ({sidelap:.0} % side overlap); a photo every {:.1} m ({frontlap:.0} % front overlap): about {} photos over {length:.0} m of lines",
                    fp.trigger_m,
                    (length / fp.trigger_m).floor() as usize + lines
                );
                if *trigger {
                    eprintln!(
                        "the mission triggers the camera every {:.1} m from the first survey waypoint",
                        fp.trigger_m
                    );
                } else {
                    eprintln!(
                        "the mission does not trigger the camera (--trigger adds it): set {:.1} m on the camera or autopilot",
                        fp.trigger_m
                    );
                }
            }
            let text = protocols::wpl::to_string(&items);
            write_or_print(out.as_deref(), &text, &format!("{} items", items.len()))
        }
        MissionCmd::Export { file, gpx, kml } => {
            if gpx.is_none() && kml.is_none() {
                return Err("nothing to write: give --gpx and/or --kml".into());
            }
            let items = read_mission(file)?;
            if items.is_empty() {
                return Err(format!("{}: no mission items", file.display()));
            }
            let path = protocols::mission::path(&items);
            // With home's MSL altitude in the file the path is absolute; otherwise heights
            // are above home, drawn relative to the ground.
            let home_msl = items.first().map(|h| f64::from(h.z)).filter(|z| *z != 0.0);
            let points: Vec<track::Point> = path
                .iter()
                .map(|p| track::Point {
                    unix_us: None,
                    lat: p.lat,
                    lon: p.lon,
                    alt_msl: home_msl.unwrap_or(0.0) + p.alt_rel_m,
                })
                .collect();
            let name = file
                .file_stem()
                .map_or("mission".into(), |s| s.to_string_lossy().into_owned());
            if let Some(out) = gpx {
                let text = track::gpx(&name, &points);
                std::fs::write(out, text).map_err(|e| format!("{}: {e}", out.display()))?;
                eprintln!(
                    "wrote {} ({} points{})",
                    out.display(),
                    points.len(),
                    if home_msl.is_some() {
                        ""
                    } else {
                        "; heights above home: the file has no home altitude"
                    }
                );
            }
            if let Some(out) = kml {
                let mode = if home_msl.is_some() {
                    "absolute"
                } else {
                    "relativeToGround"
                };
                let text = track::kml_with(&name, &points, mode);
                std::fs::write(out, text).map_err(|e| format!("{}: {e}", out.display()))?;
                eprintln!("wrote {} ({} points, {mode})", out.display(), points.len());
            }
            Ok(())
        }
        MissionCmd::Check {
            file,
            params,
            fence,
        } => {
            let items = read_mission(file)?;
            print_mission(&items);
            eprintln!(
                "{}: {} items (including home), readable",
                file.display(),
                items.len()
            );
            print_stats(&items);
            let speeds = match params {
                Some(p) => protocols::mission::Speeds::from_params(&load_params(p)?),
                None => protocols::mission::Speeds::default(),
            };
            print_time_estimate(&items, &speeds, params.is_some());
            if let Some(fence_file) = fence {
                let zones = protocols::fence::Fence::from_items(&read_mission(fence_file)?)
                    .map_err(|e| format!("{}: {e}", fence_file.display()))?;
                let fp = match params {
                    Some(p) => protocols::fence::FenceParams::from_params(&load_params(p)?),
                    None => protocols::fence::FenceParams::default(),
                };
                if zones.is_empty() && fp.circle_radius_m.is_none() && fp.alt_max_m.is_none() {
                    return Err(format!("{}: no fence zones", fence_file.display()));
                }
                let path = protocols::mission::path(&items);
                let breaches = protocols::fence::check_path(&path, &zones, &fp, 5.0);
                for b in &breaches {
                    println!(
                        "fence: leg to item {} {} at {:.7},{:.7}",
                        b.seq, b.what, b.at.0, b.at.1
                    );
                }
                if !breaches.is_empty() {
                    return Err(format!(
                        "the path breaches the fence on {} legs",
                        breaches.len()
                    ));
                }
                eprintln!(
                    "fence: the whole path stays inside{}",
                    if params.is_some() {
                        ""
                    } else {
                        " (zones only; --params adds FENCE_TYPE circle and altitude)"
                    }
                );
            }
            Ok(())
        }
        MissionCmd::Reverse { file, out } => {
            let items = read_mission(file)?;
            let r = protocols::mission::reverse(&items).map_err(|e| e.to_string())?;
            print_stats(&r);
            write_or_print(
                out.as_deref(),
                &protocols::wpl::to_string(&r),
                &format!("{} items reversed", r.len()),
            )
        }
        MissionCmd::Shift {
            file,
            north,
            east,
            home,
            out,
        } => {
            let items = read_mission(file)?;
            let s = protocols::mission::shift(&items, *north, *east, *home);
            print_stats(&s);
            write_or_print(
                out.as_deref(),
                &protocols::wpl::to_string(&s),
                &format!("{} items, moved {north} m north, {east} m east", s.len()),
            )
        }
        MissionCmd::Diff { a, b, with_home } => {
            let (ma, mb) = (read_mission(a)?, read_mission(b)?);
            let diff = protocols::mission::verify(&ma, &mb, !with_home);
            if diff.is_empty() {
                println!(
                    "same mission ({} items{})",
                    ma.len(),
                    if *with_home {
                        ""
                    } else {
                        "; home not compared"
                    }
                );
                return Ok(());
            }
            for m in &diff {
                let at = m.seq.map_or_else(String::new, |s| format!("item {s} "));
                println!("{at}{}: first {}, second {}", m.field, m.draft, m.vehicle);
            }
            Err(format!("{} differences", diff.len()))
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
    print_param_diff(&a.display().to_string(), &pa, &b.display().to_string(), &pb);
    Ok(())
}

/// Print the differences between two parameter sets; true when there are none.
fn print_param_diff(
    a: &str,
    pa: &std::collections::BTreeMap<String, f64>,
    b: &str,
    pb: &std::collections::BTreeMap<String, f64>,
) -> bool {
    let d = diff_params(pa, pb);
    // Shortest text that round-trips the vehicle's f32.
    let f = |v: f64| v as f32;
    let mut by_autopilot = 0;
    for (k, x, y) in &d.changed {
        let auto = protocols::params::set_by_autopilot(k);
        by_autopilot += usize::from(auto);
        println!(
            "~ {k:<16} {} -> {}{}",
            f(*x),
            f(*y),
            if auto { "  (set by the autopilot)" } else { "" }
        );
    }
    for k in &d.only_left {
        println!("- {k:<16} {}", f(pa[k]));
    }
    for k in &d.only_right {
        println!("+ {k:<16} {}", f(pb[k]));
    }
    println!(
        "{a} vs {b}: {} vs {} parameters; {} changed ({by_autopilot} set by the autopilot itself), {} only left, {} only right",
        pa.len(),
        pb.len(),
        d.changed.len(),
        d.only_left.len(),
        d.only_right.len()
    );
    d.changed.len() == by_autopilot && d.only_left.is_empty() && d.only_right.is_empty()
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
        LogCmd::Fft {
            path,
            field,
            imu,
            segment,
        } => {
            let log = open(path)?;
            let (msg, f) = field
                .split_once('.')
                .ok_or("--field is MSG.FIELD, e.g. ACC.AccZ")?;
            let c = log
                .column(msg, f, Some(("I", f64::from(*imu))))
                .ok_or_else(|| format!("{}: no {field} for IMU {imu}", path.display()))?;
            let armed = logs::flight::armed_periods(&log);
            let in_flight = |t: u64| {
                armed.is_empty()
                    || armed
                        .iter()
                        .any(|p| t >= p.start_us && p.end_us.is_none_or(|e| t <= e))
            };
            let (t, v): (Vec<u64>, Vec<f64>) = c
                .time_us
                .iter()
                .zip(&c.values)
                .filter(|(t, _)| in_flight(**t))
                .map(|(t, v)| (*t, *v))
                .unzip();
            let sp = logs::spectrum::spectrum(&t, &v, *segment).ok_or_else(|| {
                format!("{} samples: fewer than one segment of {segment}", v.len())
            })?;
            println!(
                "{field} IMU {imu}: {} samples at {:.0} Hz{}, {} segments, {:.2} Hz bins",
                v.len(),
                sp.sample_rate_hz,
                if armed.is_empty() { "" } else { " while armed" },
                sp.segments,
                sp.sample_rate_hz / *segment as f64
            );
            for (freq, p) in logs::spectrum::peaks(&sp, 5.0, 5) {
                println!("  peak {freq:7.1} Hz  power {p:.3e}");
            }
            print_aliasing(&log, sp.sample_rate_hz);
        }
        LogCmd::Summary { path } => print_log_summary(&open(path)?)?,
        LogCmd::Compare { a, b } => {
            let (la, lb) = (open(a)?, open(b)?);
            let (ra, rb) = (compare_rows(&la), compare_rows(&lb));
            let name = |p: &std::path::Path| {
                p.file_name()
                    .map_or("?".into(), |n| n.to_string_lossy().into_owned())
            };
            println!("{:<28} {:>16} {:>16}", "", name(a), name(b));
            for ((label, x), (_, y)) in ra.iter().zip(&rb) {
                println!("{label:<28} {x:>16} {y:>16}");
            }
        }
        LogCmd::Report { path } => {
            let log = open(path)?;
            type Section<'a> = (&'a str, Box<dyn Fn() -> Result<(), String> + 'a>);
            let sections: Vec<Section> = vec![
                ("Flight summary", Box::new(|| print_log_summary(&log))),
                ("Health checks", Box::new(|| print_log_health(&log))),
                ("Motors and hover", Box::new(|| print_log_motors(&log))),
                ("Attitude tracking", Box::new(|| print_log_attitude(&log))),
                ("Takeoffs", Box::new(|| print_log_takeoffs(&log))),
                ("Landings", Box::new(|| print_log_landings(&log))),
                ("Battery", Box::new(|| print_log_battery(&log, &0))),
                ("Cell balance", Box::new(|| print_log_cells(&log))),
                (
                    "Compass interference",
                    Box::new(|| print_log_compass(&log, &0)),
                ),
                ("ESC telemetry", Box::new(|| print_log_esc(&log))),
                ("Autopilot performance", Box::new(|| print_log_perf(&log))),
                ("Wind", Box::new(|| print_log_wind(&log))),
                ("GPS", Box::new(|| print_log_gps(&log))),
                ("EKF lane switches", Box::new(|| print_log_lanes(&log))),
                ("IMU health", Box::new(|| print_log_imu(&log))),
                ("Altitude sources", Box::new(|| print_log_altitude(&log))),
                ("Flow and rangefinder", Box::new(|| print_log_sensors(&log))),
                ("RC link", Box::new(|| print_log_rc(&log))),
                (
                    "Harmonic notch (advice)",
                    Box::new(|| print_log_notch(&log, "ACC.AccX", &0, &1024, &20.0)),
                ),
                ("Parameter changes", Box::new(|| print_log_changes(&log))),
                (
                    "Configuration review",
                    Box::new(|| {
                        print_params_review(&log.params());
                        Ok(())
                    }),
                ),
            ];
            for (title, run) in sections {
                println!("\n== {title} ==");
                if let Err(e) = run() {
                    println!("not available: {e}");
                }
            }
        }
        LogCmd::Motors { path } => print_log_motors(&open(path)?)?,
        LogCmd::Geotags { path, trig, out } => {
            let log = open(path)?;
            let msg = if *trig { "TRIG" } else { "CAM" };
            let photos = logs::camera::photos(&log, msg);
            let mut csv =
                String::from("cam,img,time_usec,utc,lat,lon,alt_msl_m,alt_rel_m,roll,pitch,yaw\n");
            for p in &photos {
                let utc = p.unix_us.map_or(String::new(), crate::tlog::utc);
                csv += &format!(
                    "{},{},{},{utc},{:.7},{:.7},{:.2},{:.2},{:.2},{:.2},{:.2}\n",
                    p.instance,
                    p.image,
                    p.unix_us.map_or(String::new(), |u| u.to_string()),
                    p.lat,
                    p.lon,
                    p.alt_msl,
                    p.alt_rel,
                    p.roll,
                    p.pitch,
                    p.yaw
                );
            }
            write_or_print(
                out.as_deref(),
                &csv,
                &format!("{} {msg} records", photos.len()),
            )?;
        }
        LogCmd::Notch {
            path,
            field,
            imu,
            segment,
            min_hz,
        } => print_log_notch(&open(path)?, field, imu, segment, min_hz)?,
        LogCmd::Rc { path } => print_log_rc(&open(path)?)?,
        LogCmd::Sensors { path } => print_log_sensors(&open(path)?)?,
        LogCmd::Wind { path } => print_log_wind(&open(path)?)?,
        LogCmd::Altitude { path } => print_log_altitude(&open(path)?)?,
        LogCmd::Esc { path } => print_log_esc(&open(path)?)?,
        LogCmd::Perf { path } => print_log_perf(&open(path)?)?,
        LogCmd::Cells { path } => print_log_cells(&open(path)?)?,
        LogCmd::Lanes { path } => print_log_lanes(&open(path)?)?,
        LogCmd::Takeoffs { path } => print_log_takeoffs(&open(path)?)?,
        LogCmd::Trim {
            path,
            out,
            from,
            until,
            margin,
        } => {
            if out == path {
                return Err("--out must differ from the input".into());
            }
            let log = open(path)?;
            let us = |s: f64| (s.max(0.0) * 1e6) as u64;
            let (t0, t1) = match (from, until) {
                (Some(a), Some(b)) if b > a => (us(*a), us(*b)),
                (Some(_), Some(_)) => return Err("--until must be after --from".into()),
                _ => logs::trim::armed_window(&log, us(*margin))
                    .ok_or("the log never armed: give --from and --until")?,
            };
            let (bytes, r) = logs::trim::trim(&log, t0, t1);
            std::fs::write(out, &bytes).map_err(|e| format!("{}: {e}", out.display()))?;
            let end = if t1 == u64::MAX {
                "end of log".to_string()
            } else {
                format!("{:.1} s", t1 as f64 / 1e6)
            };
            println!(
                "kept {} of {} records ({:.1} of {:.1} MiB), {:.1} s to {end}; wrote {}",
                r.records_out,
                r.records_in,
                r.bytes_out as f64 / 1048576.0,
                log.bytes().len() as f64 / 1048576.0,
                t0 as f64 / 1e6,
                out.display()
            );
        }
        LogCmd::Landings { path } => print_log_landings(&open(path)?)?,
        LogCmd::Sanitize { path, out, to } => {
            let log = open(path)?;
            let to = parse_polygon(to)?
                .first()
                .copied()
                .ok_or("--to: expected lat,lon")?;
            if out == path {
                return Err("--out must differ from the input".into());
            }
            let (bytes, r) = logs::sanitize::sanitize(&log, to);
            std::fs::write(out, bytes).map_err(|e| format!("{}: {e}", out.display()))?;
            match r.reference {
                Some(from) => println!(
                    "moved {} position values in {} fields from around {:.4},{:.4} to {:.4},{:.4}; wrote {}",
                    r.values,
                    r.fields.len(),
                    from.0,
                    from.1,
                    r.stand_in.0,
                    r.stand_in.1,
                    out.display()
                ),
                None => println!(
                    "no POS/GPS fix to move relative to: zeroed {} position values; wrote {}",
                    r.values,
                    out.display()
                ),
            }
            if !r.zeroed.is_empty() {
                let what = if r.reference.is_some() {
                    "zeroed (position fields not named latitude/longitude)"
                } else {
                    "position fields (zeroed)"
                };
                println!("{what}: {}", r.zeroed.join(", "));
            }
            println!("note: altitudes and GPS times are unchanged");
        }
        LogCmd::Imu { path } => print_log_imu(&open(path)?)?,
        LogCmd::Gps { path } => print_log_gps(&open(path)?)?,
        LogCmd::Compass { path, instance } => print_log_compass(&open(path)?, instance)?,
        LogCmd::Battery { path, instance } => print_log_battery(&open(path)?, instance)?,
        LogCmd::Attitude { path } => print_log_attitude(&open(path)?)?,
        LogCmd::Events { path } => {
            let log = open(path)?;
            let ev = logs::flight::events(&log);
            let fw = log_firmware(&ev);
            for (t, e) in &ev {
                let text = match e {
                    logs::flight::Event::Mode { mode, reason } => {
                        format!("MODE {} (reason {reason})", mode_label(fw, *mode))
                    }
                    logs::flight::Event::Arm { armed, forced } => format!(
                        "{}{}",
                        if *armed { "ARMED" } else { "DISARMED" },
                        if *forced { " (forced)" } else { "" }
                    ),
                    logs::flight::Event::Ev(id) => {
                        logs::flight::ev_name(*id).map_or(format!("EV {id}"), |n| format!("EV {n}"))
                    }
                    logs::flight::Event::Err(sub, code) => format!("ERR {sub} {code}"),
                    logs::flight::Event::Text(t) => t.clone(),
                };
                println!("{:10.3} s  {text}", *t as f64 / 1e6);
            }
        }
        LogCmd::Changes { path } => print_log_changes(&open(path)?)?,
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
            let rich_kml = || {
                let (tagged, wps) = bin_kml_parts(&log);
                let fw = log_firmware(&logs::flight::events(&log));
                let segments = track::split_by_mode(&tagged, |m| {
                    if m == u32::MAX {
                        "before the first mode change".into()
                    } else {
                        mode_label(fw, m as u8)
                    }
                });
                eprintln!(
                    "  {} mode segments, {} waypoints",
                    segments.len(),
                    wps.len()
                );
                track::kml_rich(&name, &segments, &wps, "absolute")
            };
            for (out, text) in [
                (gpx, gpx.as_ref().map(|_| track::gpx(&name, &points))),
                (kml, kml.as_ref().map(|_| rich_kml())),
            ] {
                if let (Some(p), Some(t)) = (out, text) {
                    std::fs::write(p, t).map_err(|e| format!("{}: {e}", p.display()))?;
                    eprintln!("wrote {} ({} points)", p.display(), points.len());
                }
            }
        }
        LogCmd::Health {
            path,
            json,
            fail_on,
        } => log_health(path, &open(path)?, *json, *fail_on)?,
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

/// The API key from `file` (trimmed) or the `DC_API_TOKEN` variable; `None` if neither is set.
fn api_token(file: Option<&std::path::Path>) -> Result<Option<String>, String> {
    let token = match file {
        Some(f) => std::fs::read_to_string(f)
            .map_err(|e| format!("{}: {e}", f.display()))?
            .trim()
            .to_string(),
        None => std::env::var("DC_API_TOKEN").unwrap_or_default(),
    };
    if file.is_some() && token.is_empty() {
        return Err("the API key file is empty".into());
    }
    Ok((!token.is_empty()).then_some(token))
}

async fn remote(
    url: &str,
    token_file: Option<&std::path::Path>,
    what: &RemoteCmd,
) -> Result<(), String> {
    use api::pb::{self, core_client::CoreClient};
    let channel = tonic::transport::Endpoint::from_shared(url.to_string())
        .map_err(|e| format!("{url}: {e}"))?
        .connect()
        .await
        .map_err(|e| format!("{url}: {e}"))?;
    let auth = match api_token(token_file)? {
        Some(t) => Some(
            tonic::metadata::MetadataValue::try_from(format!("Bearer {t}"))
                .map_err(|_| "the API key has characters a header cannot carry".to_string())?,
        ),
        None => None,
    };
    let mut c = CoreClient::with_interceptor(channel, move |mut req: tonic::Request<()>| {
        if let Some(a) = &auth {
            req.metadata_mut().insert("authorization", a.clone());
        }
        Ok(req)
    });
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
        RemoteCmd::SetParams {
            changes,
            yes,
            armed_ok,
        } => {
            let mut assignments = Vec::new();
            for ch in changes {
                let (name, value) = ch
                    .split_once('=')
                    .ok_or_else(|| format!("{ch:?}: expected NAME=VALUE"))?;
                let value: f64 = value
                    .trim()
                    .parse()
                    .map_err(|_| format!("{ch:?}: value is not a number"))?;
                let name = name.trim().to_ascii_uppercase();
                let label = match dc_core::parameter_risk(&name) {
                    dc_core::ParamRisk::Dangerous => "  (failsafe/arming/fence/safety setting)",
                    dc_core::ParamRisk::Ordinary => "",
                };
                eprintln!("  {name} = {value}{label}");
                assignments.push(pb::ParamAssignment { name, value });
            }
            if !*yes {
                match ask("Write these parameters to the vehicle?") {
                    Some(true) => {}
                    Some(false) => return Err("not confirmed: nothing written".into()),
                    None => return Err("no terminal to confirm on: pass --yes".into()),
                }
            }
            let mut req = pb::SetParamsRequest {
                sysid: 0,
                compid: 0,
                changes: assignments,
                acknowledge_armed_warning: *armed_ok,
            };
            let mut reply = c.set_params(req.clone()).await.map_err(err)?.into_inner();
            if !reply.armed_warning.is_empty() {
                eprintln!("WARNING: {}", reply.armed_warning);
                match ask("The vehicle is ARMED. Write anyway, in flight?") {
                    Some(true) => {}
                    Some(false) => {
                        return Err("in-flight warning not acknowledged: nothing written".into());
                    }
                    None => {
                        return Err(
                            "vehicle armed: pass --armed-ok to acknowledge the in-flight warning"
                                .into(),
                        );
                    }
                }
                req.acknowledge_armed_warning = true;
                reply = c.set_params(req).await.map_err(err)?.into_inner();
            }
            eprintln!("baseline on the daemon's host: {}", reply.snapshot);
            for r in &reply.results {
                println!("{} {:?} {}", r.name, r.status(), r.value);
            }
            if !reply.interruption.is_empty() {
                eprintln!("stopped: {}", reply.interruption);
            }
            if !reply.verified {
                return Err(format!(
                    "not all {} change(s) verified; the baseline holds the previous values",
                    reply.planned
                ));
            }
            println!("all {} change(s) verified by read-back", reply.planned);
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
        RemoteCmd::Links => {
            let s = c
                .get_link_stats(pb::GetLinkStatsRequest {})
                .await
                .map_err(err)?
                .into_inner();
            for l in &s.links {
                println!("link {}  {}", l.id, l.name);
            }
            for r in &s.routes {
                println!("route {}/{} -> link {}", r.sysid, r.compid, r.link_id);
            }
            println!(
                "frames {}  flooded {}  loops dropped {}  queue drops {}  link-event drops {}",
                s.frames, s.flooded, s.loops_dropped, s.app_dropped, s.links_dropped
            );
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
        let fw = v.autopilot_version.as_ref().map(|a| {
            let note = match v.firmware_tested() {
                Some(false) => " (not covered by DrorControl's tests: ArduCopter 4.6 and 4.7)",
                _ => "",
            };
            format!("fw {}{note}", a.value.version_string())
        });
        // Packet loss over the last 10 s, all components of this vehicle together.
        let (rx, lost) = core
            .stream_quality(v.sysid)
            .iter()
            .fold((0u32, 0u32), |(r, l), (_, q)| {
                (r + q.recent_received, l + q.recent_lost)
            });
        let loss = (rx + lost > 0).then(|| {
            format!(
                "loss {:.1}% ({lost}/{})",
                f64::from(lost) * 100.0 / f64::from(rx + lost),
                rx + lost
            )
        });
        let parts: Vec<String> = [hb, att, pos, bat, mission, loss, fw]
            .into_iter()
            .flatten()
            .collect();
        println!(
            "sys {:3} | {} | hb age {:.1} s",
            v.sysid,
            parts.join(" | "),
            age
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn health_json_schema() {
        let f = [
            logs::health::Finding {
                check: "gps",
                level: logs::health::Level::Ok,
                text: "fine".into(),
                at_us: None,
            },
            logs::health::Finding {
                check: "arming",
                level: logs::health::Level::Caution,
                text: "PreArm: \"x\"".into(),
                at_us: Some(1_500_000),
            },
        ];
        let v: serde_json::Value =
            serde_json::from_str(&health_json(std::path::Path::new("a.bin"), &f)).unwrap();
        assert_eq!(v["schema"], "dc.health/1");
        assert_eq!(v["overall"], "caution");
        assert_eq!(v["findings"][0]["at_s"], serde_json::Value::Null);
        assert_eq!(v["findings"][1]["at_s"], 1.5);
        assert_eq!(v["findings"][1]["text"], "PreArm: \"x\"");
        let empty: serde_json::Value =
            serde_json::from_str(&health_json(std::path::Path::new("a.bin"), &[])).unwrap();
        assert_eq!(empty["overall"], serde_json::Value::Null);
    }
}
