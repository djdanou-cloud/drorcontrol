//! Photos recorded in a dataflash log, for geotagging.
//!
//! ArduPilot 4.7.1 (`AP_Camera_Backend::log_picture`, `check_feedback`): without a feedback
//! pin it writes `CAM` when it triggers the camera; with one (`CAMn_FEEDBAK_PIN`) it writes
//! `TRIG` when it triggers and `CAM` when the camera reports the shutter, so `CAM` is the
//! photo either way. Both = TimeUS, I, Img, GPSTime (ms of week), GPSWeek, Lat, Lng, Alt
//! (m MSL), RelAlt (m above home), GPSAlt, R, P, Y (degrees) (`AP_Camera/LogStructure.h`).

use crate::dataflash::Log;
use crate::track::{UNIX_OFFSET_MSEC, boot_to_unix_us};

/// One photo.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Photo {
    /// µs since boot.
    pub time_us: u64,
    /// UTC, µs since the Unix epoch: from the record's GPS time, else from the log's GPS clock.
    pub unix_us: Option<u64>,
    /// Camera instance.
    pub instance: u8,
    /// Image number.
    pub image: u16,
    /// Latitude, degrees.
    pub lat: f64,
    /// Longitude, degrees.
    pub lon: f64,
    /// Altitude above MSL, m.
    pub alt_msl: f64,
    /// Altitude above home, m.
    pub alt_rel: f64,
    /// Roll, pitch and yaw, degrees.
    pub roll: f64,
    /// See `roll`.
    pub pitch: f64,
    /// See `roll`.
    pub yaw: f64,
}

/// Every `msg` record (`CAM`, or `TRIG` for trigger times), in log order.
pub fn photos(log: &Log, msg: &str) -> Vec<Photo> {
    let col = |f: &str| log.column(msg, f, None).map(|c| c.values);
    let Some(time) = log.column(msg, "TimeUS", None).map(|c| c.values) else {
        return Vec::new();
    };
    let get = |f: &str| col(f).unwrap_or_default();
    let (inst, img, gms, gwk) = (get("I"), get("Img"), get("GPSTime"), get("GPSWeek"));
    let (lat, lon, alt, rel) = (get("Lat"), get("Lng"), get("Alt"), get("RelAlt"));
    let (r, p, y) = (get("R"), get("P"), get("Y"));
    let offset = boot_to_unix_us(log);
    let at = |v: &[f64], i: usize| v.get(i).copied().unwrap_or(f64::NAN);
    (0..time.len())
        .map(|i| {
            let t = time[i] as u64;
            let week = at(&gwk, i);
            let unix_us = if week > 0.0 {
                Some((week as u64 * 604_800_000 + at(&gms, i) as u64 + UNIX_OFFSET_MSEC) * 1000)
            } else {
                offset.and_then(|o| u64::try_from(i128::from(t) + o).ok())
            };
            Photo {
                time_us: t,
                unix_us,
                instance: at(&inst, i) as u8,
                image: at(&img, i) as u16,
                lat: at(&lat, i),
                lon: at(&lon, i),
                alt_msl: at(&alt, i),
                alt_rel: at(&rel, i),
                roll: at(&r, i),
                pitch: at(&p, i),
                yaw: at(&y, i),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{LogBuilder, V::N};

    #[test]
    fn photos_with_utc_from_gps_time() {
        let mut b = LogBuilder::default();
        b.format(
            "CAM",
            "QBHIHLLeeefff",
            "TimeUS,I,Img,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,GPSAlt,R,P,Y",
        );
        // GPS week 2388, 302 400 000 ms (Wednesday 2025-10-15 12:00:00 GPS time).
        b.rec(
            "CAM",
            &[
                N(5e6),
                N(0.0),
                N(1.0),
                N(302_400_000.0),
                N(2388.0),
                N(-35.363_261_1),
                N(149.165_230_2),
                N(604.12),
                N(20.05),
                N(605.0),
                N(1.5),
                N(-2.0),
                N(270.0),
            ],
        );
        let p = photos(&b.open(), "CAM");
        assert_eq!(p.len(), 1);
        let p = p[0];
        assert_eq!((p.time_us, p.instance, p.image), (5_000_000, 0, 1));
        assert!((p.lat + 35.363_261_1).abs() < 1e-7 && (p.lon - 149.165_230_2).abs() < 1e-7);
        assert!((p.alt_msl - 604.12).abs() < 1e-9 && (p.alt_rel - 20.05).abs() < 1e-9);
        assert_eq!((p.roll, p.pitch, p.yaw), (1.5, -2.0, 270.0));
        // 2025-10-15T11:59:42Z: GPS time runs 18 s ahead of UTC.
        assert_eq!(p.unix_us, Some(1_760_529_582_000_000));
    }
}
