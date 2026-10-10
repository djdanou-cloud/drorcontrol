//! Stable C ABI over `dc-core` (header: `include/dc_core.h`).
//!
//! This is the only crate allowed to contain `unsafe` for FFI. Every function checks its
//! pointers; ownership and threading rules are documented in the header. The UI reads
//! telemetry with [`dc_core_telemetry`] once per frame: it copies from a lock-free snapshot
//! and never blocks.

#![allow(unsafe_code)]

use std::ffi::{CStr, c_char};
use std::time::{Duration, Instant};

use dc_core::{Core, CoreConfig, LinkSpec};

/// ABI version; bump on any incompatible change to the header.
pub const DC_ABI_VERSION: u32 = 1;
/// A required pointer was NULL.
pub const DC_ERR_NULL: i32 = -1;
/// Malformed argument.
pub const DC_ERR_ARGUMENT: i32 = -2;
/// Link could not be opened.
pub const DC_ERR_IO: i32 = -3;
/// Unknown vehicle.
pub const DC_ERR_NOT_FOUND: i32 = -4;

/// `DcTelemetry.flags` bits.
pub mod flags {
    /// Heartbeat group valid.
    pub const HEARTBEAT: u8 = 1 << 0;
    /// Attitude group valid.
    pub const ATTITUDE: u8 = 1 << 1;
    /// Position group valid.
    pub const POSITION: u8 = 1 << 2;
    /// Battery voltage valid.
    pub const BATTERY: u8 = 1 << 3;
    /// Motors armed.
    pub const ARMED: u8 = 1 << 4;
}

/// Mirror of `DcTelemetry` in `dc_core.h` (64 bytes; layout checked by tests).
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DcTelemetry {
    /// Bytes filled in by the library.
    pub struct_size: u32,
    /// System id.
    pub sysid: u8,
    /// `flags::*` bits.
    pub flags: u8,
    /// `MAV_TYPE`.
    pub mav_type: u8,
    /// Reserved, zero.
    pub reserved0: u8,
    /// ArduPilot flight mode number.
    pub custom_mode: u32,
    /// Heartbeat age, ms.
    pub heartbeat_age_ms: u32,
    /// Roll, rad.
    pub roll_rad: f32,
    /// Pitch, rad.
    pub pitch_rad: f32,
    /// Yaw, rad.
    pub yaw_rad: f32,
    /// Attitude age, ms.
    pub attitude_age_ms: u32,
    /// Latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees.
    pub lon_deg: f64,
    /// Altitude above home, m.
    pub alt_rel_m: f32,
    /// Position age, ms.
    pub position_age_ms: u32,
    /// Battery voltage, V.
    pub battery_v: f32,
    /// Battery age, ms.
    pub battery_age_ms: u32,
}

/// Opaque handle: the core plus the runtime that drives it.
pub struct DcCore {
    rt: tokio::runtime::Runtime,
    core: Core,
}

fn ms(d: Duration) -> u32 {
    u32::try_from(d.as_millis()).unwrap_or(u32::MAX)
}

/// ABI version implemented by this library.
#[unsafe(no_mangle)]
pub extern "C" fn dc_abi_version() -> u32 {
    DC_ABI_VERSION
}

/// Start a core. Returns NULL on failure.
#[unsafe(no_mangle)]
pub extern "C" fn dc_core_new(sysid: u8, compid: u8) -> *mut DcCore {
    let Ok(rt) = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
    else {
        return std::ptr::null_mut();
    };
    let cfg = CoreConfig {
        sysid,
        compid,
        ..Default::default()
    };
    match rt.block_on(Core::start(cfg)) {
        Ok(core) => Box::into_raw(Box::new(DcCore { rt, core })),
        Err(_) => std::ptr::null_mut(),
    }
}

/// Stop the core and free the handle.
///
/// # Safety
/// `core` must be NULL or a handle from [`dc_core_new`] not yet freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dc_core_free(core: *mut DcCore) {
    if !core.is_null() {
        // SAFETY: caller guarantees `core` came from dc_core_new and is freed once.
        let h = unsafe { Box::from_raw(core) };
        h.rt.shutdown_timeout(Duration::from_millis(500));
    }
}

/// Open a link described by `spec`.
///
/// # Safety
/// `core` must be a live handle; `spec` a NUL-terminated string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dc_core_connect(core: *mut DcCore, spec: *const c_char) -> i32 {
    if core.is_null() || spec.is_null() {
        return DC_ERR_NULL;
    }
    // SAFETY: non-null, caller guarantees validity.
    let (h, spec) = unsafe { (&*core, CStr::from_ptr(spec)) };
    let Ok(spec) = spec
        .to_str()
        .map_err(|_| ())
        .and_then(|s| s.parse::<LinkSpec>().map_err(|_| ()))
    else {
        return DC_ERR_ARGUMENT;
    };
    match h.rt.block_on(h.core.connect(&spec)) {
        Ok(id) => i32::try_from(id.0).unwrap_or(i32::MAX),
        Err(_) => DC_ERR_IO,
    }
}

/// List vehicle system ids.
///
/// # Safety
/// `core` must be a live handle; `out` must point to at least `cap` bytes (or be NULL
/// when `cap` is 0).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dc_core_vehicles(core: *const DcCore, out: *mut u8, cap: usize) -> i32 {
    if core.is_null() || (out.is_null() && cap > 0) {
        return DC_ERR_NULL;
    }
    // SAFETY: non-null, caller guarantees validity.
    let h = unsafe { &*core };
    let ids: Vec<u8> = h
        .core
        .fleet()
        .vehicles()
        .filter(|v| v.heartbeat.is_some())
        .map(|v| v.sysid)
        .collect();
    for (i, id) in ids.iter().take(cap).enumerate() {
        // SAFETY: i < cap and caller guarantees `out` has `cap` bytes.
        unsafe { *out.add(i) = *id };
    }
    i32::try_from(ids.len()).unwrap_or(i32::MAX)
}

/// Copy the latest telemetry of `sysid` into `out`.
///
/// # Safety
/// `core` must be a live handle; `out` must point to a writable `DcTelemetry`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dc_core_telemetry(
    core: *const DcCore,
    sysid: u8,
    out: *mut DcTelemetry,
) -> i32 {
    if core.is_null() || out.is_null() {
        return DC_ERR_NULL;
    }
    // SAFETY: non-null, caller guarantees validity.
    let h = unsafe { &*core };
    let fleet = h.core.fleet();
    let Some(v) = fleet.get(sysid) else {
        return DC_ERR_NOT_FOUND;
    };
    let now = Instant::now();
    let mut t = DcTelemetry {
        struct_size: size_of::<DcTelemetry>() as u32,
        sysid,
        ..Default::default()
    };
    if let Some(hb) = &v.heartbeat {
        t.flags |= flags::HEARTBEAT | if hb.value.armed() { flags::ARMED } else { 0 };
        t.mav_type = hb.value.mav_type;
        t.custom_mode = hb.value.custom_mode;
        t.heartbeat_age_ms = ms(hb.age(now));
    }
    if let Some(a) = &v.attitude {
        t.flags |= flags::ATTITUDE;
        (t.roll_rad, t.pitch_rad, t.yaw_rad) = (a.value.roll, a.value.pitch, a.value.yaw);
        t.attitude_age_ms = ms(a.age(now));
    }
    if let Some(p) = &v.position {
        t.flags |= flags::POSITION;
        (t.lat_deg, t.lon_deg, t.alt_rel_m) = (p.value.lat_deg, p.value.lon_deg, p.value.alt_rel_m);
        t.position_age_ms = ms(p.age(now));
    }
    if let Some(p) = &v.power
        && let Some(volt) = p.value.voltage
    {
        t.flags |= flags::BATTERY;
        t.battery_v = volt;
        t.battery_age_ms = ms(p.age(now));
    }
    // SAFETY: caller guarantees `out` is writable.
    unsafe { out.write(t) };
    0
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::{align_of, offset_of};

    #[test]
    fn layout_matches_header() {
        // Keep in sync with dc_core.h (which static_asserts sizeof == 64).
        assert_eq!(size_of::<DcTelemetry>(), 64);
        assert_eq!(align_of::<DcTelemetry>(), 8);
        assert_eq!(offset_of!(DcTelemetry, sysid), 4);
        assert_eq!(offset_of!(DcTelemetry, custom_mode), 8);
        assert_eq!(offset_of!(DcTelemetry, roll_rad), 16);
        assert_eq!(offset_of!(DcTelemetry, attitude_age_ms), 28);
        assert_eq!(offset_of!(DcTelemetry, lat_deg), 32);
        assert_eq!(offset_of!(DcTelemetry, alt_rel_m), 48);
        assert_eq!(offset_of!(DcTelemetry, battery_age_ms), 60);
    }

    #[test]
    fn header_constants_match() {
        let h = include_str!("../include/dc_core.h");
        assert!(h.contains(&format!("#define DC_ABI_VERSION {DC_ABI_VERSION}u")));
        for (name, v) in [
            ("DC_ERR_NULL", DC_ERR_NULL),
            ("DC_ERR_ARGUMENT", DC_ERR_ARGUMENT),
            ("DC_ERR_IO", DC_ERR_IO),
            ("DC_ERR_NOT_FOUND", DC_ERR_NOT_FOUND),
        ] {
            assert!(h.contains(&format!("#define {name:<18} ({v})")), "{name}");
        }
    }

    #[test]
    fn handles_null_and_bad_input() {
        unsafe {
            assert_eq!(
                dc_core_connect(std::ptr::null_mut(), c"tcp:127.0.0.1:1".as_ptr()),
                DC_ERR_NULL
            );
            assert_eq!(
                dc_core_vehicles(std::ptr::null(), std::ptr::null_mut(), 0),
                DC_ERR_NULL
            );
            dc_core_free(std::ptr::null_mut());
        }
        let core = dc_core_new(255, 190);
        assert!(!core.is_null());
        unsafe {
            assert_eq!(
                dc_core_connect(core, c"not-a-spec".as_ptr()),
                DC_ERR_ARGUMENT
            );
            assert_eq!(dc_core_vehicles(core, std::ptr::null_mut(), 0), 0);
            let mut t = DcTelemetry::default();
            assert_eq!(dc_core_telemetry(core, 1, &mut t), DC_ERR_NOT_FOUND);
            dc_core_free(core);
        }
    }

    #[test]
    fn reads_telemetry_through_the_c_abi() {
        use mavlink_codec::messages::{Attitude, Heartbeat};
        use mavlink_codec::{Header, Message, Version, encode};
        use std::io::Write;

        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        std::thread::spawn(move || {
            let (mut s, _) = listener.accept().unwrap();
            for _ in 0..100 {
                for m in [
                    Message::Heartbeat(Heartbeat {
                        r#type: 2,
                        autopilot: 3,
                        base_mode: 128,
                        custom_mode: 5,
                        ..Default::default()
                    }),
                    Message::Attitude(Attitude {
                        roll: 0.25,
                        ..Default::default()
                    }),
                ] {
                    let f = encode(
                        Version::V2,
                        Header {
                            seq: 0,
                            sysid: 3,
                            compid: 1,
                        },
                        &m,
                    )
                    .unwrap();
                    if s.write_all(f.as_bytes()).is_err() {
                        return;
                    }
                }
                std::thread::sleep(Duration::from_millis(50));
            }
        });

        let core = dc_core_new(255, 190);
        let spec = std::ffi::CString::new(format!("tcp:{addr}")).unwrap();
        unsafe {
            assert!(dc_core_connect(core, spec.as_ptr()) >= 0);
            let mut ids = [0u8; 4];
            let deadline = Instant::now() + Duration::from_secs(5);
            while dc_core_vehicles(core, ids.as_mut_ptr(), ids.len()) < 1 {
                assert!(Instant::now() < deadline, "no vehicle");
                std::thread::sleep(Duration::from_millis(20));
            }
            assert_eq!(ids[0], 3);
            let mut t = DcTelemetry::default();
            while t.flags & flags::ATTITUDE == 0 {
                assert!(Instant::now() < deadline, "no attitude");
                assert_eq!(dc_core_telemetry(core, 3, &mut t), 0);
            }
            assert_eq!(t.struct_size, 64);
            assert_eq!((t.custom_mode, t.roll_rad), (5, 0.25));
            assert!(t.flags & flags::ARMED != 0);
            dc_core_free(core);
        }
    }
}
