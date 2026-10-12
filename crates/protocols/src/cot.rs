//! Cursor-on-Target (CoT) position events, the XML that ATAK/WinTAK and TAK servers read
//! (event schema 2.0: `<event>` with `<point>` and `<detail>`). Parity: CoT is in scope.
//!
//! Type strings follow the MIL-STD-2525 hierarchy CoT uses: `a-f-A-M-H-Q` (friendly, air,
//! military, rotary wing, UAV) and `a-f-A-M-F-Q` (fixed wing UAV). `hae` is height above the
//! WGS-84 ellipsoid; MAVLink gives MSL, so the event carries MSL and says so in `le`
//! (unknown accuracy is CoT's 9999999).

/// What the event describes.
#[derive(Debug, Clone, PartialEq)]
pub struct CotTrack {
    /// Stable identifier, e.g. `DrorControl-sys1`.
    pub uid: String,
    /// Display name.
    pub callsign: String,
    /// Rotary wing (true) or fixed wing.
    pub rotary: bool,
    /// Latitude and longitude, degrees.
    pub lat: f64,
    /// See `lat`.
    pub lon: f64,
    /// Altitude, m (MSL; see the module notes).
    pub alt_m: f64,
    /// Course over ground, degrees true, and ground speed, m/s.
    pub course_deg: f64,
    /// See `course_deg`.
    pub speed_ms: f64,
}

/// `YYYY-MM-DDTHH:MM:SS.mmmZ` from µs since the Unix epoch.
pub fn iso8601(unix_us: u64) -> String {
    let secs = unix_us / 1_000_000;
    let ms = (unix_us / 1000) % 1000;
    let days = (secs / 86_400) as i64;
    let rem = secs % 86_400;
    // Civil date from days since 1970-01-01 (Howard Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}.{ms:03}Z",
        rem / 3600,
        (rem / 60) % 60,
        rem % 60
    )
}

fn xml_attr(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('"', "&quot;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

/// One CoT event for `t`, valid for `stale_s` seconds.
pub fn event(t: &CotTrack, unix_us: u64, stale_s: u64) -> String {
    let now = iso8601(unix_us);
    let stale = iso8601(unix_us + stale_s * 1_000_000);
    let ty = if t.rotary {
        "a-f-A-M-H-Q"
    } else {
        "a-f-A-M-F-Q"
    };
    format!(
        "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\
<event version=\"2.0\" uid=\"{uid}\" type=\"{ty}\" how=\"m-g\" time=\"{now}\" start=\"{now}\" stale=\"{stale}\">\
<point lat=\"{lat:.7}\" lon=\"{lon:.7}\" hae=\"{alt:.1}\" ce=\"9999999.0\" le=\"9999999.0\"/>\
<detail><contact callsign=\"{cs}\"/><track course=\"{course:.1}\" speed=\"{speed:.2}\"/>\
<remarks>altitude MSL (not ellipsoid); DrorControl</remarks></detail></event>",
        uid = xml_attr(&t.uid),
        lat = t.lat,
        lon = t.lon,
        alt = t.alt_m,
        cs = xml_attr(&t.callsign),
        course = t.course_deg.rem_euclid(360.0),
        speed = t.speed_ms,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dates_are_utc_iso8601() {
        assert_eq!(iso8601(0), "1970-01-01T00:00:00.000Z");
        assert_eq!(iso8601(1_760_529_582_123_000), "2025-10-15T11:59:42.123Z");
        assert_eq!(
            iso8601(951_782_400_000_000),
            "2000-02-29T00:00:00.000Z",
            "leap day"
        );
    }

    #[test]
    fn event_carries_position_track_and_stale_time() {
        let e = event(
            &CotTrack {
                uid: "DrorControl-sys1".into(),
                callsign: "UAV <1> & co".into(),
                rotary: true,
                lat: -35.363_261,
                lon: 149.165_23,
                alt_m: 604.2,
                course_deg: -10.0,
                speed_ms: 4.5,
            },
            1_760_529_582_000_000,
            10,
        );
        assert!(e.contains("type=\"a-f-A-M-H-Q\""), "{e}");
        assert!(e.contains("time=\"2025-10-15T11:59:42.000Z\""));
        assert!(e.contains("stale=\"2025-10-15T11:59:52.000Z\""));
        assert!(e.contains("<point lat=\"-35.3632610\" lon=\"149.1652300\" hae=\"604.2\""));
        assert!(e.contains("course=\"350.0\" speed=\"4.50\""));
        assert!(
            e.contains("callsign=\"UAV &lt;1&gt; &amp; co\""),
            "escaped: {e}"
        );
    }
}
