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
mod tlog_link;
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
    /// Read named parameters without downloading the whole list (fast over a radio).
    ParamGet {
        /// Link, e.g. tcp:127.0.0.1:5760.
        #[arg(long)]
        connect: LinkSpec,
        /// Target system id (default: first vehicle heard).
        #[arg(long)]
        target: Option<u8>,
        /// Parameter names.
        #[arg(required = true)]
        names: Vec<String>,
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
        #[arg(required_unless_present = "file", value_name = "NAME=VALUE")]
        changes: Vec<String>,
        /// Apply a .param file (or a .bin log's parameters): only values that differ from the
        /// vehicle are written; names it lacks and values the autopilot maintains are left
        /// out and listed.
        #[arg(long, conflicts_with = "changes")]
        file: Option<std::path::PathBuf>,
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
    /// Restore parameters from a baseline saved by `param-set` (the vehicle in the baseline):
    /// the named ones, or with --changed every one that now differs (except values the
    /// autopilot maintains itself). Same confirmation,
    /// in-flight warning, new recovery baseline, read-back and audit as `param-set`.
    ParamRollback {
        /// Link, e.g. tcp:127.0.0.1:5760.
        #[arg(long)]
        connect: LinkSpec,
        /// Baseline file written by `param-set` (or the daemon's --param-dir).
        #[arg(long)]
        baseline: std::path::PathBuf,
        /// Parameters to restore.
        names: Vec<String>,
        /// Restore every parameter whose value now differs from the baseline.
        #[arg(long, conflicts_with = "names")]
        changed: bool,
        /// Confirm without asking (scripts).
        #[arg(long)]
        yes: bool,
        /// Acknowledge the in-flight warning without asking (scripts).
        #[arg(long)]
        armed_ok: bool,
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
    /// This build and machine, for test reports: version, OS and architecture, serial ports
    /// seen, and the firmware versions tested in CI.
    About,
    /// Check an audit log's hash chain (commands and parameter writes): every record links
    /// to the one before, so an edited, removed or reordered line is found. Prints the
    /// records with --print.
    AuditVerify {
        /// The audit log (default dc-audit.log).
        #[arg(default_value = "dc-audit.log")]
        file: std::path::PathBuf,
        /// Also print the records (time UTC, vehicle, command id, text).
        #[arg(long)]
        print: bool,
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
        /// A .param file or a .bin log (or use --connect for the vehicle itself).
        #[arg(required_unless_present = "connect")]
        file: Option<std::path::PathBuf>,
        /// Review the parameters on a vehicle instead (downloads them; read-only).
        #[arg(long, conflicts_with = "file")]
        connect: Option<LinkSpec>,
        /// Seconds to wait for a vehicle heartbeat (with --connect).
        #[arg(long, default_value_t = 15)]
        wait: u64,
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
    /// Vehicle-change settings: in-flight mission upload, geofence/rally upload and log
    /// erase. All off by default; each shows its danger when turned on. Without a
    /// subcommand: show them.
    Settings {
        #[command(subcommand)]
        what: Option<SettingsCmd>,
    },
    /// Mission files (QGC WPL 110, Mission Planner's .waypoints, QGC .plan): download,
    /// upload, check, compare, generate.
    Mission {
        #[command(subcommand)]
        what: MissionCmd,
    },
    /// A position in every display format: decimal degrees, DMS, degrees and decimal
    /// minutes, UTM and MGRS (WGS-84).
    Coord {
        /// "lat,lon" in decimal degrees.
        #[arg(allow_hyphen_values = true)]
        point: String,
    },
    /// Ground elevation from SRTM .hgt tiles on disk (offline).
    Terrain {
        #[command(subcommand)]
        what: TerrainCmd,
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

#[derive(Subcommand)]
enum TerrainCmd {
    /// Elevation above the geoid (EGM96) at each point, bilinear between samples.
    Elevation {
        /// Directory holding the tiles (e.g. S36E149.hgt, 1″ or 3″).
        #[arg(long)]
        dem: std::path::PathBuf,
        /// Points as "lat,lon;lat,lon;...".
        #[arg(allow_hyphen_values = true)]
        points: String,
    },
}

/// `--pattern` of `mission search`.
#[derive(Clone, Copy, PartialEq, clap::ValueEnum)]
enum SearchPattern {
    /// Expanding square (IAMSAR): legs of 1, 1, 2, 2, 3, 3 … × --spacing, turning right.
    Square,
    /// Sector search (IAMSAR): three triangles of side --radius through the datum, 120°
    /// right turns.
    Sector,
}

#[derive(Subcommand)]
enum SettingsCmd {
    /// Show each setting, whether it is on, and its danger (and where the file is).
    Show,
    /// Turn a setting on, after showing its danger and asking once.
    Enable {
        /// armed-mission-upload, fence-rally-upload or log-erase.
        name: String,
        /// Turn it on without asking (scripts).
        #[arg(long)]
        yes: bool,
        /// Hash-chained audit log to record the change in.
        #[arg(long, default_value = "dc-audit.log")]
        audit: std::path::PathBuf,
    },
    /// Turn a setting off.
    Disable {
        /// armed-mission-upload, fence-rally-upload or log-erase.
        name: String,
        /// Hash-chained audit log to record the change in.
        #[arg(long, default_value = "dc-audit.log")]
        audit: std::path::PathBuf,
    },
}

/// `--type` of `mission download` and `mission upload`.
#[derive(Clone, Copy, PartialEq, clap::ValueEnum)]
enum ListKind {
    Mission,
    Fence,
    Rally,
}

#[derive(Subcommand)]
enum TlogCmd {
    /// Link quality over time (range tests): frames received and lost per window, the
    /// radio's signal, and the longest heartbeat gap.
    Link {
        /// The .tlog file.
        file: std::path::PathBuf,
        /// Window length, s.
        #[arg(long, default_value_t = 10.0)]
        window: f64,
        /// Vehicle system id (default: the first autopilot heard).
        #[arg(long)]
        vehicle: Option<u8>,
    },
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
    /// Upload a mission, geofence or rally points from a file (.plan, or .waypoints), then
    /// read it back and compare. Asks once. A mission to a disarmed vehicle needs no
    /// setting; to an armed one it needs the `armed-mission-upload` setting and the in-flight
    /// warning acknowledged. Fence and rally need the `fence-rally-upload` setting (and the
    /// warning when armed). An empty list clears it on the vehicle. Audited.
    Upload {
        #[command(flatten)]
        vehicle: VehicleArgs,
        /// The file: a .plan (its mission, geofence or rally points) or QGC WPL 110.
        file: std::path::PathBuf,
        /// Which list to upload.
        #[arg(long = "type", value_enum, default_value_t = ListKind::Mission)]
        kind: ListKind,
        /// Confirm without asking (scripts).
        #[arg(long)]
        yes: bool,
        /// Acknowledge the in-flight warning without asking (scripts): upload even if armed.
        #[arg(long)]
        armed_ok: bool,
        /// Hash-chained audit log to append to.
        #[arg(long, default_value = "dc-audit.log")]
        audit: std::path::PathBuf,
    },
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
        /// `mission download --type fence --out` or `mission fence-from-kml`, or a .plan's
        /// geofence); exits with failure on a breach.
        #[arg(long)]
        fence: Option<std::path::PathBuf>,
        /// Check the height above the ground along every leg with the SRTM tiles in this
        /// directory (heights above home taken from the ground at home); exits with failure
        /// below --min-clearance.
        #[arg(long)]
        dem: Option<std::path::PathBuf>,
        /// Lowest acceptable height above the ground with --dem, m.
        #[arg(long, default_value_t = 20.0, requires = "dem")]
        min_clearance: f64,
    },
    /// Corridor scan along a path (a road, river, power line): parallel lines covering
    /// --width centred on the path, flown back and forth. Offline; check and upload as usual.
    Corridor {
        /// Path points as "lat,lon;lat,lon;..." (at least 2).
        #[arg(long, allow_hyphen_values = true, required_unless_present = "path_kml")]
        path: Option<String>,
        /// Read the path from a KML file instead (its first LineString).
        #[arg(long, conflicts_with = "path")]
        path_kml: Option<std::path::PathBuf>,
        /// Width to cover, m, centred on the path.
        #[arg(long)]
        width: f64,
        /// Distance between lines, m.
        #[arg(long)]
        spacing: f64,
        /// Extend each line this many metres past both ends of the path.
        #[arg(long, default_value_t = 0.0)]
        overshoot: f64,
        /// Altitude above home, m.
        #[arg(long)]
        alt: f32,
        /// Output file (.waypoints or .plan; default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Search and rescue pattern from a datum (last known position). Offline; check and
    /// upload as usual.
    Search {
        /// The pattern.
        #[arg(long, value_enum)]
        pattern: SearchPattern,
        /// Datum as "lat,lon".
        #[arg(long, allow_hyphen_values = true)]
        datum: String,
        /// Track spacing, m (square).
        #[arg(long, required_if_eq("pattern", "square"))]
        spacing: Option<f64>,
        /// Side of the square searched, m (square).
        #[arg(long, required_if_eq("pattern", "square"))]
        extent: Option<f64>,
        /// Leg length, m (sector).
        #[arg(long, required_if_eq("pattern", "sector"))]
        radius: Option<f64>,
        /// First leg's direction, degrees clockwise from north.
        #[arg(long, default_value_t = 0.0)]
        track: f64,
        /// Altitude above home, m.
        #[arg(long)]
        alt: f32,
        /// Output file (.waypoints or .plan; default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Terrain following without terrain data on the vehicle: sets every waypoint above
    /// home to --agl above the ground under it (SRTM tiles in --dem), optionally adding
    /// waypoints every --step m between them. Offline; prints the resulting clearance.
    TerrainFollow {
        /// The mission file.
        file: std::path::PathBuf,
        /// Directory holding the SRTM tiles.
        #[arg(long)]
        dem: std::path::PathBuf,
        /// Height above the ground, m.
        #[arg(long)]
        agl: f32,
        /// Add waypoints this often between waypoints, m (0: none).
        #[arg(long, default_value_t = 0.0)]
        step: f64,
        /// Output file (.waypoints or .plan; default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// The mission's altitude against the ground along the path, as CSV (dist_m, seq, lat,
    /// lon, ground_m, alt_msl_m, agl_m; SRTM tiles in --dem), for a profile chart.
    Profile {
        /// The mission file.
        file: std::path::PathBuf,
        /// Directory holding the SRTM tiles.
        #[arg(long)]
        dem: std::path::PathBuf,
        /// Sample spacing along the legs, m.
        #[arg(long, default_value_t = 25.0)]
        step: f64,
        /// Output CSV (default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// A mission from a path drawn in Google Earth (the KML's first LineString): takeoff, a
    /// waypoint per point in drawing order, RTL. Offline; check and upload it as usual.
    FromKml {
        /// The KML file.
        kml: std::path::PathBuf,
        /// Altitude above home, m.
        #[arg(long)]
        alt: f32,
        /// Output file (.waypoints or .plan; default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// A geofence from polygons drawn in Google Earth: every KML polygon becomes an inclusion
    /// zone, or an exclusion zone when its name contains "exclusion", "no-fly", "no fly" or
    /// "nofly" (any case). Offline; check a mission against it with `mission check --fence`.
    FenceFromKml {
        /// The KML file.
        kml: std::path::PathBuf,
        /// Output file (.waypoints fence items, or the geofence of a .plan; default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Rally points from points (placemarks) drawn in Google Earth, at one altitude above
    /// home. Offline.
    RallyFromKml {
        /// The KML file.
        kml: std::path::PathBuf,
        /// Altitude above home, m.
        #[arg(long)]
        alt: f32,
        /// Output file (.waypoints rally items, or the rally points of a .plan; default: print).
        #[arg(long)]
        out: Option<std::path::PathBuf>,
    },
    /// Convert a mission between QGroundControl `.plan` and `.waypoints` (QGC WPL 110), by the
    /// file extensions. A .plan's geofence and rally points are listed; only the mission is
    /// written to a .waypoints file.
    Convert {
        /// Input file (.plan or .waypoints).
        input: std::path::PathBuf,
        /// Output file (.plan or .waypoints).
        output: std::path::PathBuf,
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
    /// The mission's flight path as GPX and/or KML (Google Earth), or its items as CSV
    /// (spreadsheets), offline.
    Export {
        /// The mission file (.waypoints or .plan).
        file: std::path::PathBuf,
        /// Write a GPX 1.1 track.
        #[arg(long)]
        gpx: Option<std::path::PathBuf>,
        /// Write a KML 2.2 line.
        #[arg(long)]
        kml: Option<std::path::PathBuf>,
        /// Write the items as CSV: seq, command, frame, params, lat, lon, alt, leg and total
        /// distance (m).
        #[arg(long)]
        csv: Option<std::path::PathBuf>,
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
    /// Written to OUT.partial and renamed when complete. Downloading never erases a log.
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
    /// Erase every dataflash log on the vehicle (LOG_ERASE), then list them again to check.
    /// Needs the `log-erase` setting and a disarmed vehicle; asks once. Cannot be undone.
    /// Audited.
    Erase {
        #[command(flatten)]
        vehicle: VehicleArgs,
        /// Erase without asking (scripts).
        #[arg(long)]
        yes: bool,
        /// Hash-chained audit log to append to.
        #[arg(long, default_value = "dc-audit.log")]
        audit: std::path::PathBuf,
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
        /// Print a machine-readable report instead (schema `dc.report/1`): summary, health,
        /// takeoffs, landings and time per mode.
        #[arg(long)]
        json: bool,
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
        /// Write CSV (time_s,kind,detail) to this file instead of printing.
        #[arg(long)]
        csv: Option<std::path::PathBuf>,
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
        /// Write the filter response of the suggested and the current settings (Hz, dB) to
        /// this CSV file.
        #[arg(long)]
        response: Option<std::path::PathBuf>,
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
    /// Every failsafe in order: what triggered, what the vehicle did, when it cleared.
    Failsafes {
        /// Path to the .bin log.
        path: std::path::PathBuf,
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
    /// How the vibration spectrum changes over the flight and with throttle: the strongest
    /// frequency per throttle band (motor-harmonic lines move with throttle), and optionally
    /// the full spectrogram as CSV (one row per second).
    Spectrogram {
        /// Path to the .bin log.
        path: std::path::PathBuf,
        /// Message and field, e.g. IMU.AccZ or ACC.AccZ.
        #[arg(long, default_value = "IMU.AccZ")]
        field: String,
        /// IMU instance.
        #[arg(long, default_value_t = 0)]
        imu: u8,
        /// Throttle bands (equal widths from 0 to 1).
        #[arg(long, default_value_t = 10)]
        bands: usize,
        /// Ignore peaks below this frequency, Hz.
        #[arg(long, default_value_t = 20.0)]
        min_hz: f64,
        /// Write the spectrogram (1 s slices, armed time) to this CSV file.
        #[arg(long)]
        csv: Option<std::path::PathBuf>,
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
    // `dc-cli … | head`: when the reader closes the pipe, printing fails and `println!`
    // panics. End quietly instead, like other command-line tools (Unix EPIPE is os error 32,
    // Windows ERROR_NO_DATA 232); every other panic is reported as before.
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let p = info.payload();
        let msg = p
            .downcast_ref::<String>()
            .map(String::as_str)
            .or_else(|| p.downcast_ref::<&str>().copied())
            .unwrap_or("");
        if msg.starts_with("failed printing to stdout")
            && (msg.contains("os error 32") || msg.contains("os error 232"))
        {
            std::process::exit(0);
        }
        default_hook(info);
    }));
    // Parsing this many subcommands and the large `run` future need more than the 1 MB main
    // thread stack Windows gives (a debug build overflowed it in CI): run on a thread with a
    // stack of its own.
    match std::thread::Builder::new()
        .name("dc-cli".into())
        .stack_size(16 << 20)
        .spawn(real_main)
        .map(std::thread::JoinHandle::join)
    {
        Ok(Ok(code)) => code,
        Ok(Err(_)) => ExitCode::FAILURE, // panicked; the hook has reported it
        Err(e) => {
            eprintln!("error: cannot start: {e}");
            ExitCode::FAILURE
        }
    }
}

fn real_main() -> ExitCode {
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

/// One parameter's outcome, for people (bench: the debug form read
/// `Verified(Param { value: 12.0, ty: 2, index: 65535 })`).
fn set_status_text(status: &protocols::params::SetStatus) -> String {
    use protocols::params::SetStatus;
    match status {
        SetStatus::Verified(p) => format!("= {} (verified by read-back)", p.value),
        SetStatus::Failed(Some(p)) => format!("NOT set: the vehicle reports {}", p.value),
        SetStatus::Failed(None) => "NOT confirmed: no read-back from the vehicle".into(),
        SetStatus::Pending => "not confirmed yet".into(),
    }
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

/// Where the vehicle-change settings are kept: `$DC_SETTINGS`, else the platform's
/// configuration directory.
fn settings_path() -> Result<std::path::PathBuf, String> {
    use std::path::PathBuf;
    if let Some(p) = std::env::var_os("DC_SETTINGS") {
        return Ok(PathBuf::from(p));
    }
    let home = || std::env::var_os("HOME").map(PathBuf::from);
    let dir = if cfg!(windows) {
        std::env::var_os("APPDATA").map(|d| PathBuf::from(d).join("DrorControl"))
    } else if cfg!(target_os = "macos") {
        home().map(|h| h.join("Library/Application Support/DrorControl"))
    } else {
        std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| home().map(|h| h.join(".config")))
            .map(|d| d.join("drorcontrol"))
    };
    dir.map(|d| d.join("vehicle-changes.conf"))
        .ok_or_else(|| "no configuration directory: set DC_SETTINGS to a file path".into())
}

/// The vehicle-change settings in force (all off when there is no settings file).
fn load_settings() -> Result<dc_core::VehicleChangeSettings, String> {
    let path = settings_path()?;
    match std::fs::read_to_string(&path) {
        Ok(text) => dc_core::VehicleChangeSettings::parse(&text)
            .map_err(|e| format!("{}: {e}", path.display())),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Default::default()),
        Err(e) => Err(format!("{}: {e}", path.display())),
    }
}

/// The refusal, with how to turn the setting on when that is what is missing.
fn change_error(e: &dc_core::VehicleChangeError) -> String {
    match e {
        dc_core::VehicleChangeError::SettingOff(c) => format!(
            "{e}: nothing was sent. To allow it: dc-cli settings enable {}",
            c.key()
        ),
        _ => format!("{e}: nothing was sent"),
    }
}

async fn settings_cmd(what: &SettingsCmd) -> Result<(), String> {
    let path = settings_path()?;
    let mut s = load_settings()?;
    let (name, on, yes, audit) = match what {
        SettingsCmd::Show => {
            println!("settings file: {}", path.display());
            for c in dc_core::VehicleChange::ALL {
                println!(
                    "{:<22} {:<3}  {}\n    {}",
                    c.key(),
                    if s.get(c) { "ON" } else { "off" },
                    c.title(),
                    c.warning()
                );
            }
            return Ok(());
        }
        SettingsCmd::Enable { name, yes, audit } => (name, true, *yes, audit),
        SettingsCmd::Disable { name, audit } => (name, false, true, audit),
    };
    let c = dc_core::VehicleChange::from_key(name).ok_or_else(|| {
        format!(
            "unknown setting {name:?}: one of {}",
            dc_core::VehicleChange::ALL.map(|c| c.key()).join(", ")
        )
    })?;
    if s.get(c) == on {
        eprintln!("{} is already {}", c.key(), if on { "on" } else { "off" });
        return Ok(());
    }
    if on {
        eprintln!("{}\nWARNING: {}", c.title(), c.warning());
        if !yes {
            match ask(&format!("Turn on {}?", c.key())) {
                Some(true) => {}
                Some(false) => return Err("not confirmed: the setting stays off".into()),
                None => return Err("no terminal to confirm on: pass --yes".into()),
            }
        }
    }
    s.set(c, on);
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    std::fs::write(&path, s.to_text()).map_err(|e| format!("{}: {e}", path.display()))?;
    // Record the change in the audit log, through the core like every other change.
    let mut before = s;
    before.set(c, !on);
    let core = Core::start(CoreConfig {
        audit_log: Some(audit.clone()),
        vehicle_changes: before,
        ..Default::default()
    })
    .await
    .map_err(|e| format!("cannot start core: {e}"))?;
    core.set_vehicle_change(c, on);
    drop(core);
    // The audit writer runs on its own thread; give it a moment to write the record.
    tokio::time::sleep(Duration::from_millis(200)).await;
    eprintln!(
        "{} turned {} ({})",
        c.key(),
        if on { "ON" } else { "off" },
        path.display()
    );
    Ok(())
}

/// Start a core that writes the audit log and carries the vehicle-change settings, connect,
/// and find the vehicle (waiting a moment for its armed state).
async fn start_for_change(
    cli: &Cli,
    vehicle: &VehicleArgs,
    audit: &std::path::Path,
    settings: dc_core::VehicleChangeSettings,
) -> Result<(Core, u8), String> {
    let core = Core::start(CoreConfig {
        sysid: cli.sysid,
        compid: cli.compid,
        tlog: cli.tlog.clone(),
        audit_log: Some(audit.to_path_buf()),
        vehicle_changes: settings,
        ..Default::default()
    })
    .await
    .map_err(|e| format!("cannot start core: {e}"))?;
    core.connect(&vehicle.connect)
        .await
        .map_err(|e| format!("cannot open {}: {e}", vehicle.connect))?;
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
    let t0 = Instant::now();
    while core.facts(sysid).armed.is_none() && t0.elapsed() < Duration::from_secs(3) {
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    // ArduPilot reports MAV_STATE_BOOT until its storage is ready: wait (up to --wait).
    let booting = |core: &Core| {
        core.fleet()
            .get(sysid)
            .and_then(|v| v.heartbeat.as_ref())
            .is_none_or(|h| h.value.system_status == mavlink_codec::enums::MavState::BOOT.0)
    };
    if booting(&core) {
        eprintln!("vehicle {sysid} is still booting; waiting");
        let t0 = Instant::now();
        while booting(&core) && t0.elapsed() < Duration::from_secs(vehicle.wait) {
            tokio::time::sleep(Duration::from_millis(200)).await;
        }
    }
    Ok((core, sysid))
}

#[allow(clippy::too_many_arguments)]
async fn mission_upload(
    cli: &Cli,
    vehicle: &VehicleArgs,
    file: &std::path::Path,
    kind: ListKind,
    yes: bool,
    armed_ok: bool,
    audit: &std::path::Path,
) -> Result<(), String> {
    use dc_core::{ArmedWrite, MissionType, VehicleChangeError};
    let settings = load_settings()?;
    let (kind, items) = match kind {
        ListKind::Mission => (MissionType::Mission, read_mission(file)?),
        ListKind::Fence => (MissionType::Fence, read_fence(file)?),
        ListKind::Rally => (MissionType::Rally, read_rally(file)?),
    };
    let what = match kind {
        MissionType::Mission => "mission",
        MissionType::Fence => "geofence",
        MissionType::Rally => "rally points",
    };
    // Refuse before connecting when the setting is off.
    if kind != MissionType::Mission && !settings.fence_rally_upload {
        return Err(change_error(&VehicleChangeError::SettingOff(
            dc_core::VehicleChange::FenceRallyUpload,
        )));
    }
    let (core, sysid) = start_for_change(cli, vehicle, audit, settings).await?;
    let target = (sysid, vehicle.component);
    let armed = core.facts(sysid).armed != Some(false);
    eprintln!(
        "vehicle {sysid}{}: {what}, {} items from {}{}",
        match core.facts(sysid).armed {
            Some(true) => " (ARMED)",
            Some(false) => "",
            None => " (armed state unknown)",
        },
        items.len(),
        file.display(),
        if items.is_empty() {
            " (clears it on the vehicle)"
        } else {
            ""
        }
    );
    if armed && kind == MissionType::Mission && !settings.armed_mission_upload {
        return Err(change_error(&VehicleChangeError::SettingOff(
            dc_core::VehicleChange::ArmedMissionUpload,
        )));
    }
    if !yes {
        match ask(&format!("Upload the {what} to vehicle {sysid}?")) {
            Some(true) => {}
            Some(false) => return Err("not confirmed: nothing was sent".into()),
            None => return Err("no terminal to confirm on: pass --yes".into()),
        }
    }
    let mut mode = if armed_ok {
        ArmedWrite::Acknowledged
    } else {
        ArmedWrite::DisarmedOnly
    };
    let report = loop {
        match core.upload_items(target, kind, items.clone(), mode).await {
            Ok(r) => break r,
            Err(VehicleChangeError::ArmedWarningRequired(w)) => {
                eprintln!("WARNING: {w}");
                match ask("Upload anyway, to the armed vehicle?") {
                    Some(true) => mode = ArmedWrite::Acknowledged,
                    Some(false) => {
                        return Err("in-flight warning not acknowledged: nothing was sent".into());
                    }
                    None => {
                        return Err(
                            "vehicle armed: pass --armed-ok to acknowledge the in-flight warning"
                                .into(),
                        );
                    }
                }
            }
            Err(e) => return Err(change_error(&e)),
        }
    };
    // Let the audit writer catch up before the process exits.
    tokio::time::sleep(Duration::from_millis(200)).await;
    if report.verified() {
        eprintln!(
            "verified: the vehicle holds exactly these {} items (read back)",
            items.len()
        );
        return Ok(());
    }
    for m in &report.mismatches {
        eprintln!(
            "  item {}: {} sent {} vehicle {}",
            m.seq.map_or("-".into(), |s| s.to_string()),
            m.field,
            m.draft,
            m.vehicle
        );
    }
    Err(format!(
        "{what} NOT verified: upload {:?}, read-back {:?}, {} mismatches; check the vehicle",
        report.upload,
        report.readback,
        report.mismatches.len()
    ))
}

async fn log_erase(
    cli: &Cli,
    vehicle: &VehicleArgs,
    yes: bool,
    audit: &std::path::Path,
) -> Result<(), String> {
    let settings = load_settings()?;
    if !settings.log_erase {
        return Err(change_error(&dc_core::VehicleChangeError::SettingOff(
            dc_core::VehicleChange::LogErase,
        )));
    }
    let (core, sysid) = start_for_change(cli, vehicle, audit, settings).await?;
    let target = (sysid, vehicle.component);
    let logs = core.list_logs(target).await?;
    let bytes: u64 = logs.iter().map(|e| u64::from(e.size)).sum();
    eprintln!("vehicle {sysid}: {} logs, {bytes} bytes", logs.len());
    eprintln!("WARNING: {}", dc_core::VehicleChange::LogErase.warning());
    if !yes {
        match ask(&format!(
            "Erase all {} logs on vehicle {sysid}?",
            logs.len()
        )) {
            Some(true) => {}
            Some(false) => return Err("not confirmed: nothing was erased".into()),
            None => return Err("no terminal to confirm on: pass --yes".into()),
        }
    }
    let report = core
        .erase_logs(target)
        .await
        .map_err(|e| change_error(&e))?;
    tokio::time::sleep(Duration::from_millis(200)).await;
    if report.verified() {
        let new = report.new_logs();
        eprintln!(
            "erased: the {} earlier logs are gone{}",
            report.before.len(),
            if new.is_empty() {
                String::new()
            } else {
                format!(
                    "; the vehicle started {} new log(s) after the erase (ArduPilot does that when its log file was open, e.g. after arming since boot)",
                    new.len()
                )
            }
        );
        Ok(())
    } else {
        let old = report.still_listed();
        for e in &old {
            eprintln!("  still listed: log {} ({} bytes)", e.id, e.size);
        }
        Err(format!(
            "NOT verified after 30 s: {}",
            dc_core::ERASE_NOT_VERIFIED_HINT
        ))
    }
}

#[allow(clippy::too_many_arguments)]
async fn param_set(
    cli: &Cli,
    connect: &LinkSpec,
    target: Option<u8>,
    changes: &[String],
    file: Option<&std::path::Path>,
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
    if let Some(f) = file {
        let wanted = load_params(f)?;
        eprintln!(
            "vehicle {sysid}: downloading parameters to compare with {}…",
            f.display()
        );
        let now = core
            .download_params((sysid, 1), |_, _| {})
            .await
            .map_err(|e| e.to_string())?;
        let (mut unknown, mut auto) = (Vec::new(), Vec::new());
        for (name, value) in wanted {
            match now.get(&name) {
                None => unknown.push(name),
                Some(_) if protocols::params::set_by_autopilot(&name) => auto.push(name),
                Some(p) if p.value != value as f32 => {
                    eprintln!("  {name}: {} -> {value}", p.value);
                    edits.insert(name, value);
                }
                Some(_) => {}
            }
        }
        if !unknown.is_empty() {
            eprintln!(
                "not on this vehicle, left out ({}): {}",
                unknown.len(),
                unknown.join(", ")
            );
        }
        if !auto.is_empty() {
            eprintln!(
                "maintained by the autopilot, left out ({}): {}",
                auto.len(),
                auto.join(", ")
            );
        }
        if edits.is_empty() {
            println!("the vehicle already matches {}", f.display());
            return Ok(());
        }
    }
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
        println!("{} {}", r.name, set_status_text(&r.status));
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

#[allow(clippy::too_many_arguments)]
async fn param_rollback(
    cli: &Cli,
    connect: &LinkSpec,
    baseline: &std::path::Path,
    names: &[String],
    changed: bool,
    yes: bool,
    armed_ok: bool,
    audit: &std::path::Path,
    wait: u64,
) -> Result<(), String> {
    let text =
        std::fs::read_to_string(baseline).map_err(|e| format!("{}: {e}", baseline.display()))?;
    let snap = protocols::params::snapshot::ParamSnapshot::from_text(&text)
        .map_err(|e| format!("{}: {e}", baseline.display()))?;
    let target = snap.target();
    if !changed && names.is_empty() {
        return Err("name the parameters to restore, or pass --changed".into());
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
    let deadline = Instant::now() + Duration::from_secs(wait);
    while core.facts(target.0).armed.is_none() {
        if Instant::now() > deadline {
            return Err(format!(
                "vehicle {} not heard on {connect} within {wait} s",
                target.0
            ));
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    let mut restore: Vec<String> = names.iter().map(|n| n.to_ascii_uppercase()).collect();
    if changed {
        eprintln!("vehicle {}: downloading parameters to compare…", target.0);
        let now = core
            .download_params(target, |_, _| {})
            .await
            .map_err(|e| e.to_string())?;
        restore = snap
            .params()
            .iter()
            .filter(|(k, p)| now.get(*k).is_some_and(|n| n.value != p.value))
            .map(|(k, _)| k.clone())
            .collect();
        // Values the autopilot maintains itself (ground pressure, sensor IDs, statistics,
        // learned hover throttle) drift on their own and must not be restored.
        let skipped: Vec<String> = restore
            .iter()
            .filter(|n| protocols::params::set_by_autopilot(n))
            .cloned()
            .collect();
        restore.retain(|n| !protocols::params::set_by_autopilot(n));
        if !skipped.is_empty() {
            eprintln!(
                "left alone (the autopilot maintains these): {}",
                skipped.join(", ")
            );
        }
        if restore.is_empty() {
            println!("nothing differs from the baseline");
            return Ok(());
        }
    }
    for n in &restore {
        let p = snap
            .params()
            .get(n)
            .ok_or_else(|| format!("{n} is not in the baseline"))?;
        eprintln!("  {n} back to {}", p.value);
    }
    if !yes {
        match ask("Restore these parameters on the vehicle?") {
            Some(true) => {}
            Some(false) => return Err("not confirmed: nothing written".into()),
            None => return Err("no terminal to confirm on: pass --yes".into()),
        }
    }
    let t = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let recovery = std::path::PathBuf::from(format!("param-recovery-{}-{t}.param", target.0));
    let mut mode = if armed_ok {
        dc_core::ArmedWrite::Acknowledged
    } else {
        dc_core::ArmedWrite::DisarmedOnly
    };
    let report = loop {
        match core
            .rollback_params_with(baseline, &restore, &recovery, mode)
            .await
        {
            Ok(r) => break r,
            Err(dc_core::ParamOperationError::ArmedWarningRequired(w)) => {
                eprintln!("WARNING: {w}");
                match ask("The vehicle is ARMED. Restore anyway, in flight?") {
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
    eprintln!("recovery baseline saved to {}", report.snapshot.display());
    for r in &report.results {
        println!("{} {}", r.name, set_status_text(&r.status));
    }
    if report.verified() {
        println!("all {} parameter(s) restored and verified", report.planned);
        Ok(())
    } else {
        Err(format!(
            "restore incomplete{}",
            report
                .interruption
                .as_ref()
                .map_or(String::new(), |w| format!(": {w}"))
        ))
    }
}

fn audit_verify(file: &std::path::Path, print: bool) -> Result<(), String> {
    let open = || std::fs::File::open(file).map_err(|e| format!("{}: {e}", file.display()));
    let (n, last) = protocols::audit::verify(std::io::BufReader::new(open()?)).map_err(|b| {
        format!(
            "{}: chain broken at line {}: {} (the log was edited, truncated mid-line or reordered there)",
            file.display(),
            b.line,
            b.reason
        )
    })?;
    if print {
        use std::io::BufRead;
        for line in std::io::BufReader::new(open()?).lines() {
            let line = line.map_err(|e| e.to_string())?;
            let c: Vec<&str> = line.split('\t').collect();
            let t = c[0].parse::<u64>().map_or(c[0].to_string(), tlog::utc);
            println!("{t}  sys {}  cmd {}  {}", c[1], c[2], c[3]);
        }
    }
    println!(
        "{}: {n} records, chain intact; last hash {last}",
        file.display()
    );
    Ok(())
}

async fn run(cli: Cli) -> Result<(), String> {
    match &cli.cmd {
        Cmd::AuditVerify { file, print } => audit_verify(file, *print),
        Cmd::About => {
            // CI builds carry the commit (GitHub sets GITHUB_SHA while compiling).
            println!(
                "dc-cli {} ({})",
                env!("CARGO_PKG_VERSION"),
                option_env!("GITHUB_SHA").map_or("local build".to_string(), |c| format!(
                    "commit {}",
                    &c[..c.len().min(12)]
                ))
            );
            println!(
                "os {} / {} ({} build)",
                std::env::consts::OS,
                std::env::consts::ARCH,
                if cfg!(debug_assertions) {
                    "debug"
                } else {
                    "release"
                }
            );
            match link_serial::discovery::list_ports() {
                Ok(p) => println!("serial ports seen: {}", p.len()),
                Err(e) => println!("serial ports: cannot list ({e})"),
            }
            let tested: Vec<String> = dc_core::TESTED_FIRMWARE
                .iter()
                .map(|(f, a, b)| format!("{f:?} {a}.{b}"))
                .collect();
            println!("firmware tested in CI (SITL): {}", tested.join(", "));
            Ok(())
        }
        Cmd::ParamGet {
            connect,
            target,
            names,
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
            let mut missing = 0;
            for n in names {
                let n = n.to_ascii_uppercase();
                match core.read_param((sysid, 1), &n).await {
                    Ok(p) => println!("{n},{}", p.value),
                    Err(e) => {
                        eprintln!("{e}");
                        missing += 1;
                    }
                }
            }
            if missing > 0 {
                return Err(format!("{missing} parameter(s) not read"));
            }
            Ok(())
        }
        Cmd::ParamRollback {
            connect,
            baseline,
            names,
            changed,
            yes,
            armed_ok,
            audit,
            wait,
        } => {
            param_rollback(
                &cli, connect, baseline, names, *changed, *yes, *armed_ok, audit, *wait,
            )
            .await
        }
        Cmd::ParamSet {
            connect,
            target,
            changes,
            file,
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
                file.as_deref(),
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
        Cmd::Log {
            what:
                LogCmd::Erase {
                    vehicle,
                    yes,
                    audit,
                },
        } => log_erase(&cli, vehicle, *yes, audit).await,
        Cmd::Log { what } => log_cmd(what),
        Cmd::Settings { what } => settings_cmd(what.as_ref().unwrap_or(&SettingsCmd::Show)).await,
        Cmd::Mission { what } => mission_cmd(&cli, what).await,
        Cmd::Tlog { what } => tlog_cmd(what),
        Cmd::Terrain { what } => terrain_cmd(what),
        Cmd::Coord { point } => {
            let [(lat, lon)] = parse_polygon(point)?[..] else {
                return Err("give one lat,lon".into());
            };
            if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
                return Err(format!("{lat},{lon} is not a position"));
            }
            use protocols::units::{CoordFormat as F, position};
            for (name, f) in [
                ("decimal", F::Decimal),
                ("DMS", F::Dms),
                ("DDM", F::Ddm),
                ("UTM", F::Utm),
                ("MGRS", F::Mgrs),
            ] {
                let v = position(lat, lon, f).unwrap_or_else(|| "(polar region: UPS)".into());
                println!("{name:<8} {v}");
            }
            Ok(())
        }
        Cmd::ParamsDiff { a, b } => params_diff(a, b),
        Cmd::ParamsReview {
            file,
            connect,
            wait,
        } => {
            let params = match (file, connect) {
                (Some(f), _) => load_params(f)?,
                (None, Some(link)) => {
                    let core = start(&cli, link).await?;
                    let sysid = core
                        .wait_for_vehicle(Duration::from_secs(*wait))
                        .await
                        .ok_or_else(|| format!("no vehicle heartbeat on {link} within {wait} s"))?;
                    eprintln!("vehicle {sysid}: downloading parameters…");
                    core.download_params((sysid, 1), |_, _| {})
                        .await
                        .map_err(|e| e.to_string())?
                        .into_iter()
                        .map(|(k, p)| (k, f64::from(p.value)))
                        .collect()
                }
                (None, None) => return Err("give a file or --connect".into()),
            };
            print_params_review(&params);
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
    let osc = logs::tuning::rate_oscillation(log);
    if !osc.is_empty() {
        println!("rate-controller error spectrum (longest flight):");
        for o in &osc {
            println!(
                "  {:<5} rms {:.1} deg/s; strongest peak {:.1} Hz holds {:.0} % of the error above 1 Hz ({} samples at {:.0} Hz)",
                o.axis,
                o.rms_degs,
                o.peak_hz,
                o.peak_share * 100.0,
                o.samples,
                o.sample_rate_hz
            );
        }
        println!(
            "  a single narrow peak holding much of the error is typical of an oscillation (often too much P or D on that axis)"
        );
        if osc.iter().any(|o| o.sample_rate_hz < 50.0) {
            println!(
                "  RATE is logged slowly here: only oscillations below half the logging rate can be seen (LOG_BITMASK ATTITUDE_FAST logs it at the loop rate)"
            );
        }
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

/// `log report --json` (schema `dc.report/1`): `log`, `firmware`, `summary` (armed_s,
/// flights, distance_m and the optional maxima, null when not logged), `modes` [{mode,
/// armed_s}], `health` (as `dc.health/1` without its schema/log keys), `takeoffs`,
/// `landings`. Times are seconds since boot.
fn report_json(path: &std::path::Path, log: &logs::Log) -> String {
    let s = logs::flight::summary(log);
    let events = logs::flight::events(log);
    let fw = log_firmware(&events);
    let findings = logs::health::assess(log);
    let health: serde_json::Value =
        serde_json::from_str(&health_json(path, &findings)).expect("health JSON parses");
    let secs = |us: u64| us as f64 / 1e6;
    serde_json::to_string_pretty(&serde_json::json!({
        "schema": "dc.report/1",
        "log": path.display().to_string(),
        "firmware": events.iter().find_map(|(_, e)| match e {
            logs::flight::Event::Text(t) if dc_core::Firmware::from_banner(t).is_some() => Some(t.clone()),
            _ => None,
        }),
        "summary": {
            "flights": s.armed.len(),
            "armed_s": s.armed_s,
            "distance_m": s.distance_m,
            "max_alt_rel_m": s.max_alt_rel_m,
            "max_speed_ms": s.max_speed_ms,
            "max_from_home_m": s.max_from_home_m,
            "max_climb_ms": s.max_climb_ms,
            "max_descent_ms": s.max_descent_ms,
            "used_mah": s.used_mah,
            "min_voltage": s.min_voltage,
        },
        "modes": logs::flight::mode_times(log).into_iter().map(|(m, t)| serde_json::json!({
            "mode": mode_label(fw, m),
            "armed_s": t,
        })).collect::<Vec<_>>(),
        "health": {
            "overall": health["overall"],
            "findings": health["findings"],
        },
        "takeoffs": logs::takeoff::takeoffs(log).into_iter().map(|k| serde_json::json!({
            "at_s": secs(k.time_us),
            "from_arm_s": k.from_arm_s,
            "max_climb_ms": k.max_climb_ms,
            "climbed": k.climbed(),
            "max_roll_err_deg": k.max_roll_err_deg,
            "max_pitch_err_deg": k.max_pitch_err_deg,
        })).collect::<Vec<_>>(),
        "landings": logs::landing::landings(log).into_iter().map(|l| serde_json::json!({
            "at_s": secs(l.time_us),
            "max_descent_ms": l.max_descent_ms,
            "mean_descent_ms": l.mean_descent_ms,
            "to_disarm_s": l.to_disarm_s,
        })).collect::<Vec<_>>(),
    }))
    .expect("JSON values serialise")
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
    response: Option<&std::path::Path>,
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
    let max_alt = logs::flight::summary(log).max_alt_rel_m;
    for c in notch_caveats(n.span_s, max_alt) {
        println!("CAUTION: {c}");
    }
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
    // What the filters would do (the notch runs at the gyro rate).
    let params = log.params();
    let p = |k: &str, d: f64| params.get(k).copied().unwrap_or(d);
    let gyro_hz = logs::imu::imu_health(log)
        .first()
        .map(|h| h.rates_hz.0)
        .filter(|r| *r > 0.0)
        .unwrap_or(1000.0);
    let suggested = logs::notch_design::NotchConfig {
        freq_hz: n.freq_hz,
        bw_hz: n.bw_hz,
        att_db: p("INS_HNTCH_ATT", 40.0),
        harmonics: p("INS_HNTCH_HMNCS", 3.0) as u32,
        sample_hz: gyro_hz,
    };
    let current = (p("INS_HNTCH_ENABLE", 0.0) > 0.0).then(|| logs::notch_design::NotchConfig {
        freq_hz: p("INS_HNTCH_FREQ", 80.0),
        bw_hz: p("INS_HNTCH_BW", 40.0),
        ..suggested
    });
    println!(
        "at the measured peak ({:.0} Hz, gyro rate {gyro_hz:.0} Hz): suggested settings {:+.1} dB, current settings {}",
        n.freq_hz,
        suggested.gain_db(n.freq_hz),
        current.map_or("off".to_string(), |c| format!(
            "{:+.1} dB",
            c.gain_db(n.freq_hz)
        ))
    );
    if let Some(out) = response {
        let mut csv = String::from("hz,suggested_db,current_db\n");
        for (f, g) in suggested.response(500) {
            let c = current.map_or(String::new(), |c| format!("{:.2}", c.gain_db(f)));
            csv.push_str(&format!("{f:.2},{g:.2},{c}\n"));
        }
        std::fs::write(out, csv).map_err(|e| format!("{}: {e}", out.display()))?;
        eprintln!("wrote {}", out.display());
    }
    Ok(())
}

/// Steady flight shorter than this gives a notch suggestion too few averages to trust, s.
const NOTCH_MIN_SPAN_S: f64 = 30.0;

/// Why a notch suggestion may not be worth using (bench, 2026-10-13: 23 Hz suggested from 19 s
/// of a props-off bench session, against a configured 120 Hz).
fn notch_caveats(span_s: f64, max_alt_rel_m: Option<f64>) -> Vec<String> {
    let mut out = Vec::new();
    if max_alt_rel_m.is_some_and(|a| a < 1.0) {
        out.push(format!(
            "the vehicle never climbed above 1 m ({:.1} m): a bench or ground run, where the \
             noise is not the noise in flight; do not use this suggestion",
            max_alt_rel_m.unwrap_or(0.0)
        ));
    }
    if span_s < NOTCH_MIN_SPAN_S {
        out.push(format!(
            "only {span_s:.0} s of steady flight (want {NOTCH_MIN_SPAN_S:.0} s or more): the \
             peak may not be the motors' noise; analyse a longer hover"
        ));
    }
    out
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

fn print_log_failsafes(log: &logs::Log) -> Result<(), String> {
    use logs::failsafe::FailsafeEvent as F;
    let fs = logs::failsafe::failsafes(log);
    if fs.is_empty() {
        println!("no failsafe in this log");
        return Ok(());
    }
    let fw = log_firmware(&logs::flight::events(log));
    for (t, e) in fs {
        let what = match e {
            F::Triggered { what, code } => format!("{what} triggered (code {code})"),
            F::Resolved { what } => format!("{what} cleared"),
            F::ModeChange { mode, reason } => {
                format!("mode -> {} ({reason})", mode_label(fw, mode))
            }
        };
        println!("{:9.1} s  {what}", t as f64 / 1e6);
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
            if !x.climbed() {
                "  note: no climb after it: the land detector said not landed, but the vehicle \
                 did not leave the ground (bench, or a hop)"
            } else if x.max_roll_err_deg.max(x.max_pitch_err_deg) > 10.0 {
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
        TlogCmd::Link {
            file,
            window,
            vehicle,
        } => {
            let f = std::fs::File::open(file).map_err(|e| format!("{}: {e}", file.display()))?;
            let (sys, tl) = tlog_link::timeline(std::io::BufReader::new(f), *vehicle, *window)
                .map_err(|e| format!("{}: {e}", file.display()))?;
            let sys = sys.ok_or_else(|| format!("{}: no autopilot heard", file.display()))?;
            println!("vehicle {sys}, {window} s windows:");
            println!("     start   received  lost   loss  radio rssi/remote  longest hb gap");
            let opt = |v: Option<u8>| v.map_or("-".to_string(), |x| x.to_string());
            let first = tl.first().map_or(0, |w| w.start_us);
            for w in &tl {
                println!(
                    "{:8.0} s  {:8}  {:4}  {:5.1} %  {:>5}/{:<5}         {:5.1} s",
                    (w.start_us - first) as f64 / 1e6,
                    w.received,
                    w.lost,
                    w.loss() * 100.0,
                    opt(w.rssi),
                    opt(w.remrssi),
                    w.max_hb_gap_s
                );
            }
            return Ok(());
        }
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
            let text = mission_text(out.as_deref(), &items);
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

/// The SRTM tiles under `dir` covering `points` (file names as published, upper or lower
/// case). Tiles that are not there are listed by name.
fn load_terrain(
    dir: &std::path::Path,
    points: impl IntoIterator<Item = (f64, f64)>,
) -> Result<(protocols::terrain::Terrain, Vec<String>), String> {
    let mut t = protocols::terrain::Terrain::default();
    let mut missing = Vec::new();
    let names: std::collections::BTreeSet<String> = points
        .into_iter()
        .map(|(lat, lon)| protocols::terrain::tile_name(lat, lon))
        .collect();
    for name in names {
        let found = [name.clone(), name.to_lowercase()]
            .into_iter()
            .map(|n| dir.join(n))
            .find(|p| p.is_file());
        match found {
            Some(p) => {
                let bytes = std::fs::read(&p).map_err(|e| format!("{}: {e}", p.display()))?;
                t.add(
                    protocols::terrain::HgtTile::from_bytes(&name, &bytes)
                        .map_err(|e| format!("{}: {e}", p.display()))?,
                );
            }
            None => missing.push(name),
        }
    }
    Ok((t, missing))
}

/// `mission check --dem`: the lowest height above the ground per leg.
fn terrain_check(
    items: &[dc_core::MissionItem],
    dir: &std::path::Path,
    min_m: f64,
) -> Result<(), String> {
    const STEP_M: f64 = 10.0;
    let (t, missing) = load_terrain(dir, protocols::terrain::sample_points(items, STEP_M))?;
    if !missing.is_empty() {
        eprintln!(
            "terrain: missing tiles in {}: {}",
            dir.display(),
            missing.join(", ")
        );
    }
    let r = protocols::terrain::clearance(items, &t, STEP_M);
    let Some(home) = r.home_ground_m else {
        return Err("terrain: no elevation at home, nothing checked".into());
    };
    let low: Vec<_> = r.legs.iter().filter(|l| l.min_m < min_m).collect();
    for l in &low {
        let how = if l.min_m < 0.0 {
            format!("{:.0} m BELOW the ground", -l.min_m)
        } else {
            format!("only {:.0} m above the ground", l.min_m)
        };
        println!(
            "terrain: leg to item {} {how} ({:.0} m) at {:.7},{:.7}",
            l.seq, l.ground_m, l.at.0, l.at.1
        );
    }
    if r.no_data > 0 {
        eprintln!("terrain: {} samples without data (not checked)", r.no_data);
    }
    if let Some(m) = r.legs.iter().min_by(|a, b| a.min_m.total_cmp(&b.min_m)) {
        eprintln!(
            "terrain: ground at home {home:.0} m; lowest {:.0} m above the ground on the leg to item {} (SRTM, every {STEP_M} m)",
            m.min_m, m.seq
        );
    }
    if low.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "{} legs come closer than {min_m} m to the ground",
            low.len()
        ))
    }
}

fn terrain_cmd(what: &TerrainCmd) -> Result<(), String> {
    match what {
        TerrainCmd::Elevation { dem, points } => {
            let pts = parse_polygon(points)?;
            let (t, missing) = load_terrain(dem, pts.iter().copied())?;
            for (lat, lon) in &pts {
                match t.elevation(*lat, *lon) {
                    Some(e) => println!("{lat:.7},{lon:.7}  {e:.1} m"),
                    None => println!("{lat:.7},{lon:.7}  no data"),
                }
            }
            if !missing.is_empty() {
                eprintln!("missing tiles in {}: {}", dem.display(), missing.join(", "));
            }
            Ok(())
        }
    }
}

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

/// Mission items as CSV. Distances are along the located items (global frames, not 0,0),
/// on a local flat earth (exact enough for mission legs).
fn mission_csv(items: &[dc_core::MissionItem]) -> String {
    const GLOBAL: [u8; 6] = [0, 3, 5, 6, 10, 11];
    let mut s = String::from("seq,command,frame,p1,p2,p3,p4,lat,lon,alt,leg_m,total_m\n");
    let mut last: Option<(f64, f64)> = None;
    let mut total = 0.0;
    for (i, it) in items.iter().enumerate() {
        let name = mavlink_codec::enums::MavCmd(it.command)
            .name()
            .map_or(it.command.to_string(), |n| {
                n.trim_start_matches("MAV_CMD_").to_string()
            });
        let located = GLOBAL.contains(&it.frame) && (it.x != 0 || it.y != 0);
        let (lat, lon) = (f64::from(it.x) * 1e-7, f64::from(it.y) * 1e-7);
        let leg = if located {
            let d = last.map_or(0.0, |(a, b)| {
                let n = (lat - a).to_radians() * 6_371_008.8;
                let e = (lon - b).to_radians() * 6_371_008.8 * a.to_radians().cos();
                n.hypot(e)
            });
            last = Some((lat, lon));
            total += d;
            format!("{d:.1}")
        } else {
            String::new()
        };
        let p = |v: f32| {
            if v.is_finite() {
                format!("{v}")
            } else {
                String::new()
            }
        };
        s.push_str(&format!(
            "{i},{name},{},{},{},{},{},{},{},{},{leg},{}\n",
            it.frame,
            p(it.params[0]),
            p(it.params[1]),
            p(it.params[2]),
            p(it.params[3]),
            if located {
                format!("{lat:.7}")
            } else {
                String::new()
            },
            if located {
                format!("{lon:.7}")
            } else {
                String::new()
            },
            it.z,
            if located {
                format!("{total:.1}")
            } else {
                String::new()
            },
        ));
    }
    s
}

fn is_plan(path: &std::path::Path) -> bool {
    path.extension()
        .is_some_and(|e| e.eq_ignore_ascii_case("plan"))
}

/// A mission file: QGroundControl `.plan` (JSON) by extension, else QGC WPL 110
/// (`.waypoints`, Mission Planner).
fn read_mission(path: &std::path::Path) -> Result<Vec<dc_core::MissionItem>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if is_plan(path) {
        return protocols::plan::parse(&text)
            .map(|p| p.mission)
            .map_err(|e| format!("{}: {e}", path.display()));
    }
    protocols::wpl::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
}

/// Fence items from a file: a `.plan`'s geofence, else QGC WPL 110 fence items.
fn read_fence(path: &std::path::Path) -> Result<Vec<dc_core::MissionItem>, String> {
    if is_plan(path) {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        return protocols::plan::parse(&text)
            .map(|p| p.fence)
            .map_err(|e| format!("{}: {e}", path.display()));
    }
    read_mission(path)
}

/// Rally points from a file: a `.plan`'s rally points, else QGC WPL 110 rally items.
fn read_rally(path: &std::path::Path) -> Result<Vec<dc_core::MissionItem>, String> {
    if is_plan(path) {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        return protocols::plan::parse(&text)
            .map(|p| p.rally)
            .map_err(|e| format!("{}: {e}", path.display()));
    }
    read_mission(path)
}

/// A .plan's geofence as named polygons (circles as 36-sided polygons; exclusion zones
/// named "No-fly …" so `fence-from-kml` reads them back as exclusions) and its rally
/// points.
#[allow(clippy::type_complexity)]
fn plan_shapes(
    path: &std::path::Path,
) -> Result<(Vec<(String, Vec<(f64, f64)>)>, Vec<(String, (f64, f64))>), String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let plan = protocols::plan::parse(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    let f = protocols::fence::Fence::from_items(&plan.fence)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    let circle = |(lat, lon): (f64, f64), r: f64| -> Vec<(f64, f64)> {
        const R_EARTH: f64 = 6_371_008.8;
        (0..36)
            .map(|k| {
                let a = f64::from(k) * 10f64.to_radians();
                (
                    lat + (r * a.cos() / R_EARTH).to_degrees(),
                    lon + (r * a.sin() / (R_EARTH * lat.to_radians().cos())).to_degrees(),
                )
            })
            .collect()
    };
    let mut zones = Vec::new();
    for (k, p) in f.inclusion_polygons.iter().enumerate() {
        zones.push((format!("Inclusion {}", k + 1), p.clone()));
    }
    for (k, p) in f.exclusion_polygons.iter().enumerate() {
        zones.push((format!("No-fly {}", k + 1), p.clone()));
    }
    for (k, (c, r)) in f.inclusion_circles.iter().enumerate() {
        zones.push((
            format!("Inclusion circle {} ({r:.0} m)", k + 1),
            circle(*c, *r),
        ));
    }
    for (k, (c, r)) in f.exclusion_circles.iter().enumerate() {
        zones.push((
            format!("No-fly circle {} ({r:.0} m)", k + 1),
            circle(*c, *r),
        ));
    }
    let rally = plan
        .rally
        .iter()
        .enumerate()
        .map(|(k, r)| {
            (
                format!("Rally {}", k + 1),
                (f64::from(r.x) * 1e-7, f64::from(r.y) * 1e-7),
            )
        })
        .collect();
    Ok((zones, rally))
}

/// A KML polygon named as a no-fly zone.
fn is_exclusion_name(name: &str) -> bool {
    let n = name.to_lowercase();
    ["exclusion", "no-fly", "no fly", "nofly"]
        .iter()
        .any(|k| n.contains(k))
}

/// The mission as text for `path`: `.plan` JSON for a `.plan` path, else QGC WPL 110.
fn mission_text(path: Option<&std::path::Path>, items: &[dc_core::MissionItem]) -> String {
    if path.is_some_and(is_plan) {
        protocols::plan::to_string(&protocols::plan::Plan {
            mission: items.to_vec(),
            ..Default::default()
        })
    } else {
        protocols::wpl::to_string(items)
    }
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
        MissionCmd::Upload {
            vehicle,
            file,
            kind,
            yes,
            armed_ok,
            audit,
        } => mission_upload(cli, vehicle, file, *kind, *yes, *armed_ok, audit).await,
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
            let text = mission_text(out.as_deref(), &items);
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
            let text = mission_text(out.as_deref(), &items);
            write_or_print(out.as_deref(), &text, &format!("{} items", items.len()))
        }
        MissionCmd::Export {
            file,
            gpx,
            kml,
            csv,
        } => {
            if gpx.is_none() && kml.is_none() && csv.is_none() {
                return Err("nothing to write: give --gpx, --kml and/or --csv".into());
            }
            let items = read_mission(file)?;
            if items.is_empty() {
                return Err(format!("{}: no mission items", file.display()));
            }
            if let Some(out) = csv {
                std::fs::write(out, mission_csv(&items))
                    .map_err(|e| format!("{}: {e}", out.display()))?;
                eprintln!("wrote {} ({} items)", out.display(), items.len());
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
                let mut text = track::kml_with(&name, &points, mode);
                // A .plan's geofence and rally points go along as polygons and points.
                let (zones, rally) = if is_plan(file) {
                    plan_shapes(file)?
                } else {
                    (Vec::new(), Vec::new())
                };
                if !zones.is_empty() || !rally.is_empty() {
                    text = track::kml_add_shapes(&text, &zones, &rally);
                }
                std::fs::write(out, text).map_err(|e| format!("{}: {e}", out.display()))?;
                eprintln!(
                    "wrote {} ({} points, {mode}; {} fence zones, {} rally points)",
                    out.display(),
                    points.len(),
                    zones.len(),
                    rally.len()
                );
            }
            Ok(())
        }
        MissionCmd::Check {
            file,
            params,
            fence,
            dem,
            min_clearance,
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
                let fence_items = read_fence(fence_file)?;
                let fp = match params {
                    Some(p) => protocols::fence::FenceParams::from_params(&load_params(p)?),
                    None => protocols::fence::FenceParams::default(),
                };
                // A .plan's rally points are checked against its fence too.
                let rally = if is_plan(fence_file) {
                    let text = std::fs::read_to_string(fence_file)
                        .map_err(|e| format!("{}: {e}", fence_file.display()))?;
                    protocols::plan::parse(&text)
                        .map_err(|e| format!("{}: {e}", fence_file.display()))?
                        .rally
                } else {
                    Vec::new()
                };
                let home = items
                    .first()
                    .filter(|h| h.x != 0 || h.y != 0)
                    .map(|h| (f64::from(h.x) * 1e-7, f64::from(h.y) * 1e-7));
                for f in protocols::fence::lint(&fence_items, home, &rally, &fp) {
                    let at = f
                        .seq
                        .map_or_else(|| "fence".to_string(), |s| format!("fence item {s}"));
                    eprintln!("{:?}: {at}: {}", f.level, f.text);
                }
                let zones = protocols::fence::Fence::from_items(&fence_items)
                    .map_err(|e| format!("{}: {e}", fence_file.display()))?;
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
            if let Some(dir) = dem {
                terrain_check(&items, dir, *min_clearance)?;
            }
            Ok(())
        }
        MissionCmd::Corridor {
            path,
            path_kml,
            width,
            spacing,
            overshoot,
            alt,
            out,
        } => {
            if !(alt.is_finite() && *alt > 0.0) {
                return Err("--alt must be above 0 m".into());
            }
            let pts = match (path, path_kml) {
                (Some(p), _) => parse_polygon(p)?,
                (None, Some(k)) => {
                    let text =
                        std::fs::read_to_string(k).map_err(|e| format!("{}: {e}", k.display()))?;
                    protocols::kml::first_line(&text)
                        .map_err(|e| format!("{}: no usable path ({e})", k.display()))?
                }
                (None, None) => return Err("give --path or --path-kml".into()),
            };
            let wps = protocols::pattern::corridor(&pts, *width, *spacing, *overshoot)
                .map_err(|e| e.to_string())?;
            let lines = (width / spacing).ceil().max(1.0);
            let items = protocols::mission::route(&wps, *alt);
            print_stats(&items);
            write_or_print(
                out.as_deref(),
                &mission_text(out.as_deref(), &items),
                &format!("{lines} lines {spacing} m apart over {width} m"),
            )
        }
        MissionCmd::Search {
            pattern,
            datum,
            spacing,
            extent,
            radius,
            track,
            alt,
            out,
        } => {
            if !(alt.is_finite() && *alt > 0.0) {
                return Err("--alt must be above 0 m".into());
            }
            let d = match parse_polygon(datum)?[..] {
                [d] => d,
                _ => return Err("--datum: one lat,lon".into()),
            };
            let (wps, what) = match pattern {
                SearchPattern::Square => (
                    protocols::pattern::expanding_square(
                        d,
                        spacing.ok_or("--spacing is needed")?,
                        extent.ok_or("--extent is needed")?,
                        *track,
                    )
                    .map_err(|e| e.to_string())?,
                    "expanding square",
                ),
                SearchPattern::Sector => (
                    protocols::pattern::sector(d, radius.ok_or("--radius is needed")?, *track)
                        .map_err(|e| e.to_string())?,
                    "sector search",
                ),
            };
            let items = protocols::mission::route(&wps, *alt);
            print_stats(&items);
            write_or_print(
                out.as_deref(),
                &mission_text(out.as_deref(), &items),
                &format!("{what}: {} legs", wps.len() - 1),
            )
        }
        MissionCmd::TerrainFollow {
            file,
            dem,
            agl,
            step,
            out,
        } => {
            if !(agl.is_finite() && *agl > 0.0) {
                return Err("--agl must be above 0 m".into());
            }
            let items = read_mission(file)?;
            let mut pts = protocols::terrain::sample_points(&items, step.max(10.0));
            pts.extend(
                items
                    .iter()
                    .filter(|i| i.x != 0 || i.y != 0)
                    .map(|i| (f64::from(i.x) * 1e-7, f64::from(i.y) * 1e-7)),
            );
            let (t, missing) = load_terrain(dem, pts)?;
            if !missing.is_empty() {
                eprintln!("missing tiles in {}: {}", dem.display(), missing.join(", "));
            }
            let f =
                protocols::terrain::follow(&items, &t, *agl, *step).map_err(|e| e.to_string())?;
            print_stats(&f);
            let r = protocols::terrain::clearance(&f, &t, 10.0);
            if let Some(m) = r.legs.iter().min_by(|a, b| a.min_m.total_cmp(&b.min_m)) {
                eprintln!(
                    "lowest {:.0} m above the ground, on the leg to item {} (--step adds waypoints where the ground rises between them)",
                    m.min_m, m.seq
                );
            }
            write_or_print(
                out.as_deref(),
                &mission_text(out.as_deref(), &f),
                &format!(
                    "{} items ({} added), waypoints {agl} m above the ground",
                    f.len(),
                    f.len() - items.len()
                ),
            )
        }
        MissionCmd::Profile {
            file,
            dem,
            step,
            out,
        } => {
            if !(step.is_finite() && *step >= 1.0) {
                return Err("--step must be at least 1 m".into());
            }
            let items = read_mission(file)?;
            let (t, missing) = load_terrain(dem, protocols::terrain::sample_points(&items, *step))?;
            if !missing.is_empty() {
                eprintln!("missing tiles in {}: {}", dem.display(), missing.join(", "));
            }
            let p = protocols::terrain::profile(&items, &t, *step);
            let home = p.first().and_then(|h| h.ground_m);
            let mut csv = String::from("dist_m,seq,lat,lon,ground_m,alt_msl_m,agl_m\n");
            let opt = |v: Option<f64>| v.map_or(String::new(), |v| format!("{v:.1}"));
            for q in &p {
                let msl = home.map(|h| h + q.alt_rel_m);
                let agl = msl.zip(q.ground_m).map(|(m, g)| m - g);
                csv.push_str(&format!(
                    "{:.1},{},{:.7},{:.7},{},{},{}\n",
                    q.dist_m,
                    q.seq,
                    q.lat,
                    q.lon,
                    opt(q.ground_m),
                    opt(msl),
                    opt(agl)
                ));
            }
            write_or_print(
                out.as_deref(),
                &csv,
                &format!(
                    "{} samples over {:.0} m{}",
                    p.len(),
                    p.last().map_or(0.0, |q| q.dist_m),
                    if home.is_none() {
                        " (no terrain at home: heights only)"
                    } else {
                        ""
                    }
                ),
            )
        }
        MissionCmd::FromKml { kml, alt, out } => {
            if !(alt.is_finite() && *alt > 0.0) {
                return Err("--alt must be above 0 m".into());
            }
            let text =
                std::fs::read_to_string(kml).map_err(|e| format!("{}: {e}", kml.display()))?;
            let pts = protocols::kml::first_line(&text)
                .map_err(|e| format!("{}: no usable path ({e})", kml.display()))?;
            let items = protocols::mission::route(&pts, *alt);
            print_stats(&items);
            write_or_print(
                out.as_deref(),
                &mission_text(out.as_deref(), &items),
                &format!("{} waypoints from the path", pts.len()),
            )
        }
        MissionCmd::FenceFromKml { kml, out } => {
            let text =
                std::fs::read_to_string(kml).map_err(|e| format!("{}: {e}", kml.display()))?;
            let marks =
                protocols::kml::placemarks(&text).map_err(|e| format!("{}: {e}", kml.display()))?;
            let mut fence = protocols::fence::Fence::default();
            for m in marks.iter().filter(|m| m.kind == "Polygon") {
                let name = m.name.as_deref().unwrap_or("(unnamed)");
                if m.points.len() < 3 {
                    return Err(format!("polygon {name}: fewer than 3 vertices"));
                }
                let exclude = is_exclusion_name(name);
                eprintln!(
                    "{} zone {name}: {} vertices",
                    if exclude { "exclusion" } else { "inclusion" },
                    m.points.len()
                );
                if exclude {
                    fence.exclusion_polygons.push(m.points.clone());
                } else {
                    fence.inclusion_polygons.push(m.points.clone());
                }
            }
            if fence.is_empty() {
                return Err(format!("{}: no polygons", kml.display()));
            }
            let items = fence.to_items();
            let text = if out.as_deref().is_some_and(is_plan) {
                protocols::plan::to_string(&protocols::plan::Plan {
                    fence: items.clone(),
                    ..Default::default()
                })
            } else {
                protocols::wpl::to_string(&items)
            };
            write_or_print(
                out.as_deref(),
                &text,
                &format!(
                    "{} inclusion and {} exclusion zones, {} fence items",
                    fence.inclusion_polygons.len(),
                    fence.exclusion_polygons.len(),
                    items.len()
                ),
            )
        }
        MissionCmd::RallyFromKml { kml, alt, out } => {
            if !(alt.is_finite() && *alt > 0.0) {
                return Err("--alt must be above 0 m".into());
            }
            let text =
                std::fs::read_to_string(kml).map_err(|e| format!("{}: {e}", kml.display()))?;
            let marks =
                protocols::kml::placemarks(&text).map_err(|e| format!("{}: {e}", kml.display()))?;
            let items: Vec<dc_core::MissionItem> = marks
                .iter()
                .filter(|m| m.kind == "Point")
                .filter_map(|m| {
                    let &(lat, lon) = m.points.first()?;
                    eprintln!(
                        "rally {}: {lat:.7}, {lon:.7}",
                        m.name.as_deref().unwrap_or("(unnamed)")
                    );
                    Some(dc_core::MissionItem {
                        frame: 3,
                        command: 5100,
                        params: [0.0; 4],
                        x: (lat * 1e7).round() as i32,
                        y: (lon * 1e7).round() as i32,
                        z: *alt,
                        autocontinue: true,
                    })
                })
                .collect();
            if items.is_empty() {
                return Err(format!("{}: no points", kml.display()));
            }
            let text = if out.as_deref().is_some_and(is_plan) {
                protocols::plan::to_string(&protocols::plan::Plan {
                    rally: items.clone(),
                    ..Default::default()
                })
            } else {
                protocols::wpl::to_string(&items)
            };
            write_or_print(
                out.as_deref(),
                &text,
                &format!("{} rally points at {alt} m above home", items.len()),
            )
        }
        MissionCmd::Convert { input, output } => {
            let text =
                std::fs::read_to_string(input).map_err(|e| format!("{}: {e}", input.display()))?;
            let plan = if is_plan(input) {
                protocols::plan::parse(&text).map_err(|e| format!("{}: {e}", input.display()))?
            } else {
                protocols::plan::Plan {
                    mission: protocols::wpl::parse(&text)
                        .map_err(|e| format!("{}: {e}", input.display()))?,
                    ..Default::default()
                }
            };
            let out = if is_plan(output) {
                protocols::plan::to_string(&plan)
            } else {
                if !plan.fence.is_empty() || !plan.rally.is_empty() {
                    eprintln!(
                        "note: {} fence and {} rally items are not written to a .waypoints file (keep the .plan, or download/upload them separately)",
                        plan.fence.len(),
                        plan.rally.len()
                    );
                }
                protocols::wpl::to_string(&plan.mission)
            };
            print_stats(&plan.mission);
            std::fs::write(output, out).map_err(|e| format!("{}: {e}", output.display()))?;
            eprintln!(
                "wrote {} ({} mission items incl. home, {} fence, {} rally)",
                output.display(),
                plan.mission.len(),
                plan.fence.len(),
                plan.rally.len()
            );
            Ok(())
        }
        MissionCmd::Reverse { file, out } => {
            let items = read_mission(file)?;
            let r = protocols::mission::reverse(&items).map_err(|e| e.to_string())?;
            print_stats(&r);
            write_or_print(
                out.as_deref(),
                &mission_text(out.as_deref(), &r),
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
                &mission_text(out.as_deref(), &s),
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
        // Handled by log_transfer / log_erase (they need a link).
        LogCmd::List { .. } | LogCmd::Download { .. } | LogCmd::Erase { .. } => {}
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
            // ACC and GYR are only logged with raw IMU logging (LOG_BITMASK bit 19); IMU has the
            // same fields at the logging rate (bench, 4.6.3: no ACC messages). Fall back to it.
            let (c, shown) = match log.column(msg, f, Some(("I", f64::from(*imu)))) {
                Some(c) => (c, field.clone()),
                None if matches!(msg, "ACC" | "GYR") => {
                    let c = log
                        .column("IMU", f, Some(("I", f64::from(*imu))))
                        .ok_or_else(|| {
                            format!("{}: no {field} or IMU.{f} for IMU {imu}", path.display())
                        })?;
                    eprintln!(
                        "note: no {msg} messages (raw IMU logging off); using IMU.{f}, at the \
                         logging rate, so the highest frequency seen is lower"
                    );
                    (c, format!("IMU.{f}"))
                }
                None => return Err(format!("{}: no {field} for IMU {imu}", path.display())),
            };
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
                "{shown} IMU {imu}: {} samples at {:.0} Hz{}, {} segments, {:.2} Hz bins",
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
        LogCmd::Spectrogram {
            path,
            field,
            imu,
            bands,
            min_hz,
            csv,
        } => {
            let log = open(path)?;
            let (msg, f) = field
                .split_once('.')
                .ok_or("--field is MSG.FIELD, e.g. IMU.AccZ")?;
            let m = logs::spectrogram::throttle_map(&log, msg, f, *imu, *bands, *min_hz)
                .ok_or_else(|| format!("no {field} (IMU {imu}) with CTUN while armed"))?;
            println!(
                "strongest vibration at or above {min_hz} Hz per throttle band ({field}, IMU {imu}):"
            );
            for ((lo, hi, n, _), peak) in m.bands.iter().zip(&m.peak_hz) {
                println!("  throttle {lo:.2}-{hi:.2}: {peak:6.1} Hz  ({n} s)");
            }
            println!(
                "a peak that rises with throttle is a motor harmonic (throttle-based notch); one that stays put is a frame or prop resonance"
            );
            if let Some(out) = csv {
                let c = log
                    .column(msg, f, Some(("I", f64::from(*imu))))
                    .ok_or_else(|| format!("no {field}"))?;
                let armed = logs::flight::armed_periods(&log);
                let (t, v): (Vec<u64>, Vec<f64>) = c
                    .time_us
                    .iter()
                    .zip(&c.values)
                    .filter(|(t, _)| logs::flight::in_periods(&armed, **t))
                    .map(|(t, v)| (*t, *v))
                    .unzip();
                let sg = logs::spectrogram::spectrogram(&t, &v, 1_000_000, 64)
                    .ok_or("too few samples for a spectrogram")?;
                std::fs::write(out, logs::spectrogram::spectrogram_csv(&sg))
                    .map_err(|e| format!("{}: {e}", out.display()))?;
                eprintln!(
                    "wrote {} ({} slices, {} bins)",
                    out.display(),
                    sg.slices.len(),
                    sg.freqs.len()
                );
            }
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
        LogCmd::Report { path, json: true } => {
            println!("{}", report_json(path, &open(path)?));
        }
        LogCmd::Report { path, json: false } => {
            let log = open(path)?;
            type Section<'a> = (&'a str, Box<dyn Fn() -> Result<(), String> + 'a>);
            let sections: Vec<Section> = vec![
                ("Flight summary", Box::new(|| print_log_summary(&log))),
                ("Health checks", Box::new(|| print_log_health(&log))),
                ("Motors and hover", Box::new(|| print_log_motors(&log))),
                ("Attitude tracking", Box::new(|| print_log_attitude(&log))),
                ("Failsafes", Box::new(|| print_log_failsafes(&log))),
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
                    Box::new(|| print_log_notch(&log, "ACC.AccX", &0, &1024, &20.0, None)),
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
            response,
        } => print_log_notch(
            &open(path)?,
            field,
            imu,
            segment,
            min_hz,
            response.as_deref(),
        )?,
        LogCmd::Rc { path } => print_log_rc(&open(path)?)?,
        LogCmd::Sensors { path } => print_log_sensors(&open(path)?)?,
        LogCmd::Wind { path } => print_log_wind(&open(path)?)?,
        LogCmd::Altitude { path } => print_log_altitude(&open(path)?)?,
        LogCmd::Esc { path } => print_log_esc(&open(path)?)?,
        LogCmd::Perf { path } => print_log_perf(&open(path)?)?,
        LogCmd::Cells { path } => print_log_cells(&open(path)?)?,
        LogCmd::Lanes { path } => print_log_lanes(&open(path)?)?,
        LogCmd::Failsafes { path } => print_log_failsafes(&open(path)?)?,
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
        LogCmd::Events { path, csv } => {
            let log = open(path)?;
            let ev = logs::flight::events(&log);
            let fw = log_firmware(&ev);
            let mut rows = String::from("time_s,kind,detail\n");
            for (t, e) in &ev {
                let kind = match e {
                    logs::flight::Event::Mode { .. } => "mode",
                    logs::flight::Event::Arm { .. } => "arm",
                    logs::flight::Event::Ev(_) => "event",
                    logs::flight::Event::Err(..) => "error",
                    logs::flight::Event::Text(_) => "message",
                };
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
                if csv.is_some() {
                    rows.push_str(&format!(
                        "{:.6},{kind},\"{}\"\n",
                        *t as f64 / 1e6,
                        text.replace('"', "\"\"")
                    ));
                } else {
                    println!("{:10.3} s  {text}", *t as f64 / 1e6);
                }
            }
            if let Some(out) = csv {
                std::fs::write(out, &rows).map_err(|e| format!("{}: {e}", out.display()))?;
                eprintln!("wrote {} ({} events)", out.display(), ev.len());
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
mod notch_caveat_tests {
    use super::notch_caveats;

    #[test]
    fn short_or_ground_runs_are_flagged() {
        assert!(notch_caveats(120.0, Some(25.0)).is_empty());
        let bench = notch_caveats(19.0, Some(0.2));
        assert_eq!(bench.len(), 2, "{bench:?}");
        assert!(bench[0].contains("never climbed"));
        assert!(bench[1].contains("only 19 s"));
        // No altitude in the log: only the length is judged.
        assert_eq!(notch_caveats(45.0, None).len(), 0);
    }
}

#[cfg(test)]
mod set_status_tests {
    use super::set_status_text;
    use protocols::params::{Param, SetStatus};

    #[test]
    fn parameter_outcomes_read_as_text() {
        let p = |value| Param {
            value,
            ty: 2,
            index: 65535,
        };
        assert_eq!(
            set_status_text(&SetStatus::Verified(p(12.0))),
            "= 12 (verified by read-back)"
        );
        assert_eq!(
            set_status_text(&SetStatus::Failed(Some(p(0.5)))),
            "NOT set: the vehicle reports 0.5"
        );
        assert!(set_status_text(&SetStatus::Failed(None)).starts_with("NOT confirmed"));
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
