//! Flight track from a dataflash log: the EKF position (`POS`), the same fused position the
//! GCS shows live, with UTC times from the GPS clock.
//!
//! Sources (ArduPilot 4.7.1): `POS` = TimeUS, Lat, Lng (degrees), Alt (m MSL), …
//! (`AP_AHRS/LogStructure.h`); `GPS` = TimeUS, I, Status, GMS, GWk, … (`AP_GPS/LogStructure.h`);
//! Unix time = GPS week × 604 800 000 ms + GMS + `UNIX_OFFSET_MSEC` (`AP_GPS.h`: the GPS
//! epoch in Unix milliseconds minus `GPS_LEAPSECONDS_MILLIS`, 18 s).

use crate::dataflash::Log;

/// GPS epoch (1980-01-06) in Unix ms, less the 18 leap seconds (`AP_GPS.h`).
pub(crate) const UNIX_OFFSET_MSEC: u64 = 17_000 * 86_400 + 52 * 10 * 604_800_000 - 18_000;
/// `GPS_OK_FIX_3D`: the GPS clock is trusted from a 3D fix on.
const GPS_OK_FIX_3D: f64 = 3.0;

/// One track point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackPoint {
    /// Microseconds since boot (`TimeUS`).
    pub time_us: u64,
    /// UTC, µs since the Unix epoch, when the log has a GPS time reference.
    pub unix_us: Option<u64>,
    /// Latitude, degrees.
    pub lat: f64,
    /// Longitude, degrees.
    pub lon: f64,
    /// Altitude above MSL, m.
    pub alt_msl: f64,
}

/// Offset from boot time to Unix time, µs, from the first `GPS` record (instance 0) with a
/// 3D fix and a week number.
pub fn boot_to_unix_us(log: &Log) -> Option<i128> {
    let inst = Some(("I", 0.0));
    let status = log.column("GPS", "Status", inst)?;
    let gms = log.column("GPS", "GMS", inst)?;
    let gwk = log.column("GPS", "GWk", inst)?;
    let i = status
        .values
        .iter()
        .zip(&gwk.values)
        .position(|(s, w)| *s >= GPS_OK_FIX_3D && *w > 0.0)?;
    let unix_ms = gwk.values[i] as u64 * 604_800_000 + gms.values[i] as u64 + UNIX_OFFSET_MSEC;
    Some(i128::from(unix_ms) * 1000 - i128::from(status.time_us[i]))
}

/// The `POS` track. Samples at exactly 0, 0 (no position yet) are skipped.
pub fn track(log: &Log) -> Vec<TrackPoint> {
    let (Some(lat), Some(lon), Some(alt)) = (
        log.column("POS", "Lat", None),
        log.column("POS", "Lng", None),
        log.column("POS", "Alt", None),
    ) else {
        return Vec::new();
    };
    let offset = boot_to_unix_us(log);
    lat.values
        .iter()
        .zip(&lon.values)
        .zip(&alt.values)
        .zip(&lat.time_us)
        .filter(|(((la, lo), _), _)| !(**la == 0.0 && **lo == 0.0))
        .map(|(((la, lo), al), t)| TrackPoint {
            time_us: *t,
            unix_us: offset.and_then(|o| u64::try_from(i128::from(*t) + o).ok()),
            lat: *la,
            lon: *lo,
            alt_msl: *al,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gps_epoch_matches_unix() {
        // 1980-01-06T00:00:00Z is Unix 315 964 800 s; the constant is that minus 18 s.
        assert_eq!(UNIX_OFFSET_MSEC, 315_964_800_000 - 18_000);
    }
}
