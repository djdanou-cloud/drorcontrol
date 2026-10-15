//! The RC link as the GCS can see it over MAVLink (plan 09 Step 2).
//!
//! ArduPilot never sends CRSF/ELRS link quality as a MAVLink field. The GCS gets
//! `RC_CHANNELS.rssi`, which is the receiver's RSSI, or its link quality when `RC_OPTIONS`
//! bit 11 (`USE_CRSF_LQ_AS_RSSI`) is set (`AP_RSSI`, `AP_RCProtocol_CRSF`, 4.6.3 and 4.7.1).
//! On a CRSF/ELRS receiver it also says so in `STATUSTEXT`s (`AP_CRSF_Telem`):
//! `"<proto>: Link rate <n>Hz, Telemetry rate <m>Hz"` and `"<proto>: RSSI now displays as LQ"`
//! (or `normally`), with `<proto>` = `ELRS`, `CRSFv3` or `CRSFv2`.

/// What the vehicle said about its CRSF/ELRS receiver.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct RcLinkInfo {
    /// `ELRS`, `CRSFv3` or `CRSFv2`.
    pub protocol: String,
    /// RF packet rate, Hz.
    pub link_rate_hz: Option<u16>,
    /// Telemetry rate, Hz.
    pub telemetry_rate_hz: Option<u16>,
    /// `RC_CHANNELS.rssi` carries link quality (`Some(true)`) or RSSI (`Some(false)`).
    pub rssi_as_lq: Option<bool>,
}

/// One recognised `STATUSTEXT`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RcLinkText {
    /// `"<proto>: Link rate <n>Hz, Telemetry rate <m>Hz"`.
    Rates {
        /// Protocol.
        protocol: String,
        /// Link rate, Hz.
        link_rate_hz: u16,
        /// Telemetry rate, Hz.
        telemetry_rate_hz: u16,
    },
    /// `"<proto>: RSSI now displays as LQ"` / `"… normally"`.
    RssiDisplay {
        /// Protocol.
        protocol: String,
        /// Shown as link quality.
        as_lq: bool,
    },
}

const PROTOCOLS: [&str; 3] = ["ELRS", "CRSFv3", "CRSFv2"];

/// Recognise one of ArduPilot's CRSF/ELRS link messages.
pub fn parse(text: &str) -> Option<RcLinkText> {
    let (protocol, rest) = text.split_once(": ")?;
    if !PROTOCOLS.contains(&protocol) {
        return None;
    }
    let protocol = protocol.to_string();
    if let Some(rest) = rest.strip_prefix("Link rate ") {
        let (link, rest) = rest.split_once("Hz, Telemetry rate ")?;
        let telem = rest.strip_suffix("Hz")?;
        return Some(RcLinkText::Rates {
            protocol,
            link_rate_hz: link.trim().parse().ok()?,
            telemetry_rate_hz: telem.trim().parse().ok()?,
        });
    }
    // ArduPilot formats "%s" with " as LQ" (leading space) or "normally".
    let shown = rest.strip_prefix("RSSI now displays")?.trim();
    let as_lq = match shown {
        "as LQ" => true,
        "normally" => false,
        _ => return None,
    };
    Some(RcLinkText::RssiDisplay { protocol, as_lq })
}

impl RcLinkInfo {
    /// Fold one message in.
    pub fn apply(&mut self, t: RcLinkText) {
        match t {
            RcLinkText::Rates {
                protocol,
                link_rate_hz,
                telemetry_rate_hz,
            } => {
                self.protocol = protocol;
                self.link_rate_hz = Some(link_rate_hz);
                self.telemetry_rate_hz = Some(telemetry_rate_hz);
            }
            RcLinkText::RssiDisplay { protocol, as_lq } => {
                self.protocol = protocol;
                self.rssi_as_lq = Some(as_lq);
            }
        }
    }
}

/// What `RC_CHANNELS.rssi` measures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignalKind {
    /// Received signal strength.
    Rssi,
    /// Link quality (share of packets received).
    LinkQuality,
    /// Not known yet (no CRSF message and `RSSI_TYPE` not read).
    Unknown,
}

/// `RSSI_TYPE` 3: the receiver protocol supplies RSSI (`AP_RSSI.h`).
pub const RSSI_TYPE_RECEIVER_PROTOCOL: u8 = 3;
/// `RC_OPTIONS` bit 11: CRSF link quality shown as RSSI (`RC_Channel.h`).
pub const RC_OPTIONS_CRSF_LQ_AS_RSSI: u32 = 1 << 11;

/// What `RC_CHANNELS.rssi` measures. The vehicle's own message wins; otherwise the
/// parameters decide (`rssi_type` = `RSSI_TYPE`, `rc_options` = `RC_OPTIONS`).
pub fn signal_kind(
    rssi_as_lq: Option<bool>,
    rssi_type: Option<u8>,
    rc_options: Option<u32>,
) -> SignalKind {
    match (rssi_as_lq, rssi_type, rc_options) {
        (Some(true), _, _) => SignalKind::LinkQuality,
        (Some(false), _, _) => SignalKind::Rssi,
        (None, Some(RSSI_TYPE_RECEIVER_PROTOCOL), Some(o))
            if o & RC_OPTIONS_CRSF_LQ_AS_RSSI != 0 =>
        {
            SignalKind::LinkQuality
        }
        (None, Some(RSSI_TYPE_RECEIVER_PROTOCOL), Some(_)) => SignalKind::Rssi,
        (None, Some(t), _) if t != RSSI_TYPE_RECEIVER_PROTOCOL => SignalKind::Rssi,
        _ => SignalKind::Unknown,
    }
}

/// `RC_CHANNELS.rssi` (0–254) as a percentage. RSSI: of ArduPilot's 0–254 scale. Link quality:
/// ArduPilot sends `round(LQ × 2.5)` on 0–255, then × 254/255
/// (`AP_RCProtocol_CRSF::derive_scaled_lq_value`, `GCS_MAVLINK::receiver_rssi`), so LQ ≈
/// value × 255 / 254 / 2.5, at most 100.
pub fn signal_pct(value: u8, kind: SignalKind) -> f32 {
    match kind {
        SignalKind::LinkQuality => (f32::from(value) * 255.0 / 254.0 / 2.5).min(100.0),
        SignalKind::Rssi | SignalKind::Unknown => f32::from(value) * 100.0 / 254.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_arducopter_crsf_messages() {
        assert_eq!(
            parse("ELRS: Link rate 250Hz, Telemetry rate 62Hz"),
            Some(RcLinkText::Rates {
                protocol: "ELRS".into(),
                link_rate_hz: 250,
                telemetry_rate_hz: 62
            })
        );
        // "%s" with " as LQ" gives a double space.
        assert_eq!(
            parse("CRSFv3: RSSI now displays  as LQ"),
            Some(RcLinkText::RssiDisplay {
                protocol: "CRSFv3".into(),
                as_lq: true
            })
        );
        assert_eq!(
            parse("ELRS: RSSI now displays normally"),
            Some(RcLinkText::RssiDisplay {
                protocol: "ELRS".into(),
                as_lq: false
            })
        );
        for other in [
            "ELRS: running on non-DMA serial port",
            "PreArm: RC not found",
            "SiK: Link rate 50Hz, Telemetry rate 5Hz",
            "ELRS: Link rate fastHz, Telemetry rate 5Hz",
        ] {
            assert_eq!(parse(other), None, "{other}");
        }
        let mut info = RcLinkInfo::default();
        info.apply(parse("ELRS: Link rate 500Hz, Telemetry rate 125Hz").unwrap());
        info.apply(parse("ELRS: RSSI now displays  as LQ").unwrap());
        assert_eq!(
            info,
            RcLinkInfo {
                protocol: "ELRS".into(),
                link_rate_hz: Some(500),
                telemetry_rate_hz: Some(125),
                rssi_as_lq: Some(true)
            }
        );
    }

    #[test]
    fn kind_comes_from_the_message_then_the_parameters() {
        use SignalKind::*;
        assert_eq!(signal_kind(Some(true), Some(1), Some(0)), LinkQuality);
        assert_eq!(signal_kind(Some(false), Some(3), Some(1 << 11)), Rssi);
        assert_eq!(signal_kind(None, Some(3), Some((1 << 11) | 1)), LinkQuality);
        assert_eq!(signal_kind(None, Some(3), Some(0)), Rssi);
        assert_eq!(
            signal_kind(None, Some(3), None),
            Unknown,
            "RC_OPTIONS not read"
        );
        assert_eq!(signal_kind(None, Some(1), None), Rssi, "analog pin");
        assert_eq!(signal_kind(None, None, None), Unknown);
    }

    #[test]
    fn percentages_invert_arducopter_scaling() {
        // LQ 100 % -> round(250) / 255 * 254 -> 249 on the wire.
        let wire = |lq: f32| ((lq * 2.5).round() / 255.0 * 254.0) as u8;
        for lq in [0.0, 37.0, 50.0, 99.0, 100.0] {
            let back = signal_pct(wire(lq), SignalKind::LinkQuality);
            assert!((back - lq).abs() <= 0.5, "LQ {lq} -> {back}");
        }
        assert_eq!(signal_pct(254, SignalKind::LinkQuality), 100.0);
        assert_eq!(signal_pct(254, SignalKind::Rssi), 100.0);
        assert_eq!(signal_pct(127, SignalKind::Rssi), 50.0);
    }
}
