//! Spike S0-5 (T0-09): 1080p30 H.264 decode latency and CPU with the platform's hardware
//! decoder (plan 01 §6; pass: decode ≤ 8 ms, CPU ≤ 5 % on M1).
//!
//! ```text
//! spike-s0-5-video make-sample <out.mkv> [seconds] [--profile high|main|baseline] [--refs N]
//!                                                       # 1080p30 H.264 test clip (no B-frames)
//! spike-s0-5-video run <rtsp://… | file.mkv> [options]  # measure, print JSON
//!     --seconds N    measured time after warm-up (default 60)
//!     --warmup N     seconds ignored at the start (default 3)
//!     --decoder NAME force a GStreamer decoder (default: first available, hardware first)
//!     --report PATH  also write the JSON report here
//!     --output any   let the decoder pick its output memory (default: system memory)
//! ```
//!
//! Decode latency = time from a buffer entering the decoder to the decoded frame with the same
//! PTS leaving it (includes the decoder's own queueing, as the pipeline would see it). CPU =
//! process user + system time over wall time, in % of one core (Activity Monitor's scale).
//! A file is played in real time (sink synced to the clock) so the CPU figure matches a live
//! 30 fps stream; an RTSP source runs unsynced, as the GCS will (plan 01 §6).
//!
//! Out of scope here: importing the decoded surface into the Qt scene graph and the telemetry
//! overlay (measured with the Qt video item). The report says whether a zero-copy handle is
//! available: on macOS the decoded buffer carries its IOSurface-backed `CVPixelBuffer`
//! (`GstCoreVideoMeta`), which the GCS imports into Metal with `CVMetalTextureCache`; elsewhere
//! the caps name GPU memory (`memory:VAMemory`, `memory:DMABuf`, `memory:D3D11Memory`, …).
//!
//! The default output is system-memory caps: VideoToolbox still decodes into IOSurfaces and
//! attaches the `CVPixelBuffer`, and no OpenGL is involved. (`--output any` lets `vtdec` choose
//! `memory:GLMemory`, which on macOS needs a Cocoa run loop and an OpenGL context; the GCS
//! renders with Metal, so that path is not the one to measure.)

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use gstreamer as gst;
use gstreamer::prelude::*;

/// Decoders in order of preference per platform. Hardware first; software last, flagged.
#[cfg(target_os = "macos")]
const DECODERS: &[&str] = &["vtdec_hw", "vtdec", "avdec_h264"];
#[cfg(target_os = "windows")]
const DECODERS: &[&str] = &["d3d11h264dec", "nvh264dec", "avdec_h264"];
#[cfg(not(any(target_os = "macos", target_os = "windows")))]
const DECODERS: &[&str] = &["vah264dec", "nvh264dec", "v4l2slh264dec", "avdec_h264"];
/// Software decoders: reported as `hardware: false` (the GCS shows a "SW decode" badge).
const SOFTWARE: &[&str] = &["avdec_h264", "openh264dec"];
/// Encoders for the test clip, in order of preference.
const ENCODERS: &[(&str, &str)] = &[
    (
        "x264enc",
        "x264enc tune=zerolatency speed-preset=medium pass=cbr bitrate=8000 vbv-buf-capacity=1000 key-int-max=60 bframes=0",
    ),
    (
        "vtenc_h264",
        "vtenc_h264 bitrate=8000 max-keyframe-interval=60 allow-frame-reordering=false realtime=true",
    ),
    ("openh264enc", "openh264enc bitrate=8000000 gop-size=60"),
];

fn main() {
    // On macOS GStreamer's GL and VideoToolbox elements expect an NSApplication on the main
    // thread; gst_macos_main runs one and calls us on another thread.
    #[cfg(target_os = "macos")]
    let result = gst::macos_main(real_main);
    #[cfg(not(target_os = "macos"))]
    let result = real_main();
    if let Err(e) = result {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn real_main() -> Result<(), String> {
    gst::init().map_err(|e| e.to_string())?;
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("make-sample") => {
            let out = args.get(1).ok_or("make-sample <out.mkv> [seconds]")?;
            let secs: u32 = args
                .get(2)
                .filter(|a| !a.starts_with("--"))
                .map_or(Ok(60), |s| s.parse())
                .map_err(|_| "seconds")?;
            let opt = |name: &str| {
                args.iter()
                    .position(|a| a == name)
                    .and_then(|i| args.get(i + 1))
                    .cloned()
            };
            let profile = opt("--profile").unwrap_or_else(|| "high".into());
            if !["high", "main", "baseline"].contains(&profile.as_str()) {
                return Err(format!("--profile {profile}: high, main or baseline"));
            }
            let refs = opt("--refs")
                .map(|r| r.parse::<u32>().map_err(|_| format!("--refs {r}")))
                .transpose()?;
            make_sample(out, secs, &profile, refs)
        }
        Some("run") => {
            let source = args.get(1).ok_or("run <rtsp://… | file.mkv> [options]")?;
            let opt = |name: &str| {
                args.iter()
                    .position(|a| a == name)
                    .and_then(|i| args.get(i + 1))
                    .cloned()
            };
            let num = |name: &str, default: u64| -> Result<u64, String> {
                opt(name).map_or(Ok(default), |v| {
                    v.parse().map_err(|_| format!("{name} {v}"))
                })
            };
            let report = run(
                source,
                Duration::from_secs(num("--seconds", 60)?),
                Duration::from_secs(num("--warmup", 3)?),
                opt("--decoder"),
                opt("--output").as_deref() == Some("any"),
            )?;
            println!("{report}");
            if let Some(path) = opt("--report") {
                std::fs::write(&path, report + "\n").map_err(|e| format!("{path}: {e}"))?;
            }
            Ok(())
        }
        _ => Err("usage: make-sample <out.mkv> [seconds] | run <source> [options]".into()),
    }
}

fn available(name: &str) -> bool {
    gst::ElementFactory::find(name).is_some()
}

/// `profile` and `refs` exist to test GStreamer's `vtdec`, which holds
/// `max(max_dec_frame_buffering, num_ref_frames)` decoded frames in its own reorder queue
/// unless the stream is Baseline profile (see RESULTS.md, "Why the hardware path holds frames").
fn make_sample(out: &str, secs: u32, profile: &str, refs: Option<u32>) -> Result<(), String> {
    let (name, enc) = ENCODERS
        .iter()
        .find(|(n, _)| available(n))
        .ok_or("no H.264 encoder (install x264 or use vtenc_h264 on macOS)")?;
    let enc = match refs {
        Some(r) if *name == "x264enc" => format!("{enc} ref={r}"),
        Some(_) => return Err(format!("--refs needs x264enc; using {name}")),
        None => enc.to_string(),
    };
    let desc = format!(
        // A moving zone plate: fine detail everywhere and motion in every frame, so the
        // encoder spends its bitrate like on real aerial footage. 4:2:0 8-bit, as cameras send.
        "videotestsrc pattern=zone-plate kx2=20 ky2=20 kt=1 num-buffers={n} ! \
         video/x-raw,width=1920,height=1080,framerate=30/1 ! videoconvert ! \
         video/x-raw,format=I420 ! {enc} ! video/x-h264,profile={profile} ! \
         h264parse ! matroskamux ! filesink location=\"{out}\"",
        n = secs * 30
    );
    eprintln!("encoding {secs} s of 1080p30 with {name}, profile {profile} → {out}");
    let pipeline = launch(&desc)?;
    wait(&pipeline, None)?;
    Ok(())
}

fn launch(desc: &str) -> Result<gst::Pipeline, String> {
    let el = gst::parse::launch(desc).map_err(|e| format!("{e}: {desc}"))?;
    let pipeline = el
        .downcast::<gst::Pipeline>()
        .map_err(|_| "not a pipeline")?;
    pipeline
        .set_state(gst::State::Playing)
        .map_err(|e| format!("cannot start: {e}"))?;
    Ok(pipeline)
}

/// Run until EOS, error, or `limit`. Always leaves the pipeline stopped.
fn wait(pipeline: &gst::Pipeline, limit: Option<Duration>) -> Result<(), String> {
    let bus = pipeline.bus().ok_or("no bus")?;
    let deadline = limit.map(|l| Instant::now() + l);
    let result = loop {
        let left = deadline.map(|d| d.saturating_duration_since(Instant::now()));
        if left == Some(Duration::ZERO) {
            break Ok(());
        }
        let timeout = left.map_or(gst::ClockTime::NONE, |l| {
            Some(gst::ClockTime::from_mseconds(l.as_millis() as u64))
        });
        match bus.timed_pop_filtered(timeout, &[gst::MessageType::Eos, gst::MessageType::Error]) {
            None => continue,
            Some(msg) => match msg.view() {
                gst::MessageView::Eos(_) => break Ok(()),
                gst::MessageView::Error(e) => {
                    break Err(format!(
                        "{} ({})",
                        e.error(),
                        e.debug().map(|d| d.to_string()).unwrap_or_default()
                    ));
                }
                _ => continue,
            },
        }
    };
    let _ = pipeline.set_state(gst::State::Null);
    result
}

#[derive(Default)]
struct Probe {
    /// Entry time per PTS of buffers that went into the decoder.
    pending: HashMap<u64, Instant>,
    /// Decode latencies after warm-up, µs.
    latencies_us: Vec<u64>,
    /// Frames out after warm-up.
    frames: u64,
    /// Decoded frames whose PTS never went in (decoder retimestamped): latency unknown.
    unmatched: u64,
    /// Meta APIs on the first decoded buffer (e.g. `GstCoreVideoMetaAPI`).
    metas: Option<Vec<String>>,
    measuring: bool,
}

fn run(
    source: &str,
    seconds: Duration,
    warmup: Duration,
    forced: Option<String>,
    any_output: bool,
) -> Result<String, String> {
    let decoder = match forced {
        Some(d) if available(&d) => d,
        Some(d) => return Err(format!("decoder {d} not available")),
        None => DECODERS
            .iter()
            .find(|d| available(d))
            .ok_or("no H.264 decoder available")?
            .to_string(),
    };
    let live = source.starts_with("rtsp://");
    let head = if live {
        format!("rtspsrc location=\"{source}\" latency=0 ! rtph264depay")
    } else {
        format!("filesrc location=\"{source}\" ! matroskademux")
    };
    // Decode only: the sink takes whatever the decoder produces, so its output memory type
    // (GPU or system) is what the GCS would import.
    // libav's default frame threading holds several frames (≈ 100 ms at 30 fps); the GCS
    // would use slice threading for its software fallback, so measure that.
    let props = if decoder == "avdec_h264" {
        " thread-type=slice"
    } else {
        ""
    };
    let output = if any_output { "" } else { " ! video/x-raw" };
    let desc = format!(
        "{head} ! h264parse ! {decoder} name=dec{props}{output} ! fakesink name=sink sync={sync}",
        sync = !live
    );
    let el = gst::parse::launch(&desc).map_err(|e| format!("{e}: {desc}"))?;
    let pipeline = el
        .downcast::<gst::Pipeline>()
        .map_err(|_| "not a pipeline")?;
    let dec = pipeline.by_name("dec").ok_or("no decoder element")?;
    let sink_pad = dec.static_pad("sink").ok_or("decoder has no sink pad")?;
    let src_pad = dec.static_pad("src").ok_or("decoder has no src pad")?;

    let probe = Arc::new(Mutex::new(Probe::default()));
    let p_in = probe.clone();
    sink_pad.add_probe(gst::PadProbeType::BUFFER, move |_, info| {
        if let Some(pts) = info.buffer().and_then(|b| b.pts()) {
            p_in.lock()
                .unwrap()
                .pending
                .insert(pts.nseconds(), Instant::now());
        }
        gst::PadProbeReturn::Ok
    });
    let p_out = probe.clone();
    src_pad.add_probe(gst::PadProbeType::BUFFER, move |_, info| {
        let now = Instant::now();
        let mut p = p_out.lock().unwrap();
        let buffer = info.buffer();
        if p.metas.is_none()
            && let Some(b) = buffer
        {
            p.metas = Some(
                b.iter_meta::<gst::Meta>()
                    .map(|m| m.api().name().to_string())
                    .collect(),
            );
        }
        let entered = buffer
            .and_then(|b| b.pts())
            .and_then(|pts| p.pending.remove(&pts.nseconds()));
        if p.measuring {
            p.frames += 1;
            match entered {
                Some(t) => p
                    .latencies_us
                    .push(now.duration_since(t).as_micros() as u64),
                None => p.unmatched += 1,
            }
        }
        // Bound the map if a decoder drops frames.
        if p.pending.len() > 300 {
            p.pending.clear();
        }
        gst::PadProbeReturn::Ok
    });

    pipeline
        .set_state(gst::State::Playing)
        .map_err(|e| format!("cannot start: {e}"))?;
    let bus = pipeline.bus().ok_or("no bus")?;
    // Warm-up, then measure CPU and latency over `seconds`.
    if let Some(err) = pop_error(&bus, warmup) {
        let _ = pipeline.set_state(gst::State::Null);
        return Err(err);
    }
    let caps = src_pad
        .current_caps()
        .map(|c| c.to_string())
        .unwrap_or_else(|| "not negotiated".into());
    probe.lock().unwrap().measuring = true;
    let cpu0 = cpu_time::ProcessTime::now();
    let t0 = Instant::now();
    let ended_early = wait(&pipeline, Some(seconds));
    let wall = t0.elapsed();
    let cpu = cpu0.elapsed();
    ended_early?;

    let p = probe.lock().unwrap();
    let mut lat = p.latencies_us.clone();
    lat.sort_unstable();
    let pct = |q: f64| -> f64 {
        if lat.is_empty() {
            return f64::NAN;
        }
        let i = ((lat.len() as f64 - 1.0) * q).round() as usize;
        lat[i] as f64 / 1000.0
    };
    let cpu_pct = 100.0 * cpu.as_secs_f64() / wall.as_secs_f64();
    let hardware = !SOFTWARE.contains(&decoder.as_str());
    let metas = p.metas.clone().unwrap_or_default();
    let zero_copy = (caps.contains("memory:") && !caps.contains("memory:SystemMemory"))
        || metas.iter().any(|m| m.contains("CoreVideo"));
    let metas_json = metas
        .iter()
        .map(|m| format!("\"{m}\""))
        .collect::<Vec<_>>()
        .join(",");
    let pass_decode = pct(0.99) <= 8.0;
    let pass_cpu = cpu_pct <= 5.0;
    Ok(format!(
        concat!(
            "{{\"spike\":\"S0-5\",\"source\":\"{}\",\"live\":{},\"decoder\":\"{}{}\",",
            "\"hardware\":{},\"output\":\"{}\",\"zero_copy_handle\":{},\"metas\":[{}],\"caps\":\"{}\",",
            "\"os\":\"{}\",\"arch\":\"{}\",\"cores\":{},\"gstreamer\":\"{}\",",
            "\"measured_s\":{:.1},\"frames\":{},\"fps\":{:.2},\"unmatched\":{},",
            "\"decode_ms\":{{\"p50\":{:.2},\"p95\":{:.2},\"p99\":{:.2},\"max\":{:.2}}},",
            "\"cpu_percent_of_one_core\":{:.2},",
            "\"pass\":{{\"decode_p99_le_8ms\":{},\"cpu_le_5pct\":{}}}}}"
        ),
        source.replace('"', "'"),
        live,
        decoder,
        props,
        hardware,
        if any_output {
            "any"
        } else {
            "system-memory caps"
        },
        zero_copy,
        metas_json,
        caps.replace('"', "'"),
        std::env::consts::OS,
        std::env::consts::ARCH,
        std::thread::available_parallelism().map_or(0, |n| n.get()),
        gst::version_string(),
        wall.as_secs_f64(),
        p.frames,
        p.frames as f64 / wall.as_secs_f64(),
        p.unmatched,
        pct(0.50),
        pct(0.95),
        pct(0.99),
        lat.last().map_or(f64::NAN, |&m| m as f64 / 1000.0),
        cpu_pct,
        pass_decode,
        pass_cpu,
    ))
}

/// Wait `d`, returning the first error the pipeline posts in that time.
fn pop_error(bus: &gst::Bus, d: Duration) -> Option<String> {
    let end = Instant::now() + d;
    loop {
        let left = end.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return None;
        }
        let msg = bus.timed_pop_filtered(
            gst::ClockTime::from_mseconds(left.as_millis() as u64),
            &[gst::MessageType::Error, gst::MessageType::Eos],
        )?;
        match msg.view() {
            gst::MessageView::Error(e) => return Some(e.error().to_string()),
            gst::MessageView::Eos(_) => return Some("stream ended during warm-up".into()),
            _ => {}
        }
    }
}
