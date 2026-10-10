//! Spike S0-1 (ADR-0001): can the Rust core ingest MAVLink at 200 k msg/s while a Qt Quick
//! UI, fed through a cxx-qt bridge, keeps rendering every frame?
//!
//! The ingest path is the real one: synthetic frames → UDP → `dc-core` link → router hub →
//! decode → fleet snapshot. The UI pulls the snapshot once per frame (`Telemetry.tick()`).
//!
//! Env: `S01_RATE` offered msg/s (default 200000, 0 = no load), `S01_DURATION` seconds
//! measured (default 20), `S01_WARMUP` seconds ignored (default 3), `S01_REPORT` JSON path.

mod telemetry;

use std::net::UdpSocket;
use std::sync::OnceLock;
use std::time::{Duration, Instant};

use cxx_qt_lib::{QGuiApplication, QQmlApplicationEngine, QUrl};
use dc_core::{Core, CoreConfig, LinkSpec};
use mavlink_codec::messages::{Attitude, GlobalPositionInt, Heartbeat, VfrHud};
use mavlink_codec::{Header, Message, Version, encode};

static CORE: OnceLock<Core> = OnceLock::new();
static RT: OnceLock<tokio::runtime::Runtime> = OnceLock::new();
/// Reference time for S01_FAKE animation.
pub static START: OnceLock<Instant> = OnceLock::new();

fn env_u64(name: &str, default: u64) -> u64 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

/// Seconds ignored at start (JIT, caches, first layout).
pub fn warmup_s() -> u64 {
    env_u64("S01_WARMUP", 3)
}

/// Seconds measured.
pub fn duration_s() -> u64 {
    env_u64("S01_DURATION", 20)
}

pub fn core() -> &'static Core {
    CORE.get().expect("core started")
}

/// Write the JSON report (and print it).
pub fn report(frame_ms: &[f64], tick_us: &[f64], load_us: &[f64], rates: &[f64]) {
    use telemetry::percentile as p;
    let offered = env_u64("S01_RATE", 200_000);
    let json = format!(
        concat!(
            "{{\"offered_msgs_per_s\":{},\"achieved_msgs_per_s_median\":{:.0},\"frames\":{},",
            "\"frame_ms\":{{\"p50\":{:.2},\"p99\":{:.2},\"max\":{:.2}}},\"dropped_frames_gt_25ms\":{},",
            "\"tick_us\":{{\"p50\":{:.1},\"p99\":{:.1},\"max\":{:.1}}},\"snapshot_load_us\":{{\"p50\":{:.1},\"p99\":{:.1}}},",
            "\"qt\":\"{}\",\"platform\":\"{}\",\"render_loop\":\"{}\"}}"
        ),
        offered,
        p(rates, 0.5),
        frame_ms.len(),
        p(frame_ms, 0.5),
        p(frame_ms, 0.99),
        p(frame_ms, 1.0),
        frame_ms.iter().filter(|&&x| x > 25.0).count(),
        p(tick_us, 0.5),
        p(tick_us, 0.99),
        p(tick_us, 1.0),
        p(load_us, 0.5),
        p(load_us, 0.99),
        std::env::var("S01_QT_VERSION").unwrap_or_default(),
        std::env::var("QT_QPA_PLATFORM").unwrap_or_else(|_| "default".into()),
        std::env::var("QSG_RENDER_LOOP").unwrap_or_else(|_| "default".into()),
    );
    println!("{json}");
    if let Ok(path) = std::env::var("S01_REPORT") {
        let _ = std::fs::write(path, json + "\n");
    }
}

/// Offer `rate` msg/s of telemetry from "vehicle 1" in UDP datagrams of 20 frames.
fn generator(target: std::net::SocketAddr, rate: u64) {
    let sock = UdpSocket::bind("127.0.0.1:0").expect("bind");
    let mut seq = 0u8;
    let mut i: u64 = 0;
    let start = Instant::now();
    let mut buf = Vec::with_capacity(4096);
    loop {
        buf.clear();
        for _ in 0..20 {
            let t = i as f32 * 1e-4;
            let msg = match i % 4 {
                0 => Message::Attitude(Attitude {
                    roll: (t * 0.7).sin() * 0.5,
                    pitch: (t * 0.5).sin() * 0.2,
                    yaw: t % std::f32::consts::TAU,
                    ..Default::default()
                }),
                1 => Message::VfrHud(VfrHud {
                    groundspeed: 15.0,
                    alt: 100.0 + t.sin(),
                    ..Default::default()
                }),
                2 => Message::GlobalPositionInt(GlobalPositionInt {
                    lat: -353_632_610,
                    lon: 1_491_652_300,
                    relative_alt: (50_000.0 + 1000.0 * t.sin()) as i32,
                    ..Default::default()
                }),
                _ => Message::Heartbeat(Heartbeat {
                    r#type: 2,
                    autopilot: 3,
                    mavlink_version: 3,
                    ..Default::default()
                }),
            };
            let f = encode(
                Version::V2,
                Header {
                    seq,
                    sysid: 1,
                    compid: 1,
                },
                &msg,
            )
            .expect("encode");
            buf.extend_from_slice(f.as_bytes());
            seq = seq.wrapping_add(1);
            i += 1;
        }
        let _ = sock.send_to(&buf, target);
        // Pace to the offered rate. Sleep rather than spin so the generator does not
        // steal a whole core from the system under test (bursts of up to ~2 ms are fine).
        let due = Duration::from_secs_f64(i as f64 / rate as f64);
        let now = start.elapsed();
        if due > now + Duration::from_millis(2) {
            std::thread::sleep(due - now);
        }
    }
}

fn main() {
    let rate = env_u64("S01_RATE", 200_000);
    let rt = RT.get_or_init(|| {
        tokio::runtime::Builder::new_multi_thread()
            .worker_threads(3)
            .enable_all()
            .build()
            .expect("runtime")
    });
    let core = rt.block_on(async {
        let core = Core::start(CoreConfig {
            stream_rates: vec![],
            ..Default::default()
        })
        .await
        .expect("core");
        core.connect(&LinkSpec::Udp("127.0.0.1:14599".parse().unwrap()))
            .await
            .expect("udp link");
        core
    });
    let _ = CORE.set(core);
    if rate > 0 {
        std::thread::spawn(move || generator("127.0.0.1:14599".parse().unwrap(), rate));
    }

    let mut app = QGuiApplication::new();
    let mut engine = QQmlApplicationEngine::new();
    if let Some(engine) = engine.as_mut() {
        engine.load(&QUrl::from(
            "qrc:/qt/qml/org/drorcontrol/spike/qml/main.qml",
        ));
    }
    if let Some(app) = app.as_mut() {
        app.exec();
    }
}
