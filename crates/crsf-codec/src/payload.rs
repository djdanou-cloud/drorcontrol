//! Typed payloads for the frame types DrorControl uses. Telemetry fields are big-endian.
//! Layouts and scaling from ArduPilot's `AP_CRSF_Telem` / `AP_RCProtocol_CRSF` (4.6.3).

use std::fmt;

use crate::{frame_type as ft, rc};

/// Why a payload does not decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Shorter than the type's layout.
    Short {
        /// Frame type.
        frame_type: u8,
        /// Bytes needed.
        need: usize,
        /// Bytes present.
        got: usize,
    },
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DecodeError::Short {
                frame_type,
                need,
                got,
            } => write!(
                f,
                "type 0x{frame_type:02X}: {got}-byte payload, {need} needed"
            ),
        }
    }
}

impl std::error::Error for DecodeError {}

/// GPS (0x02).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Gps {
    /// Latitude, degrees × 1e7.
    pub lat: i32,
    /// Longitude, degrees × 1e7.
    pub lon: i32,
    /// Ground speed, km/h × 10.
    pub groundspeed: u16,
    /// Course over ground, degrees × 100.
    pub heading: u16,
    /// Altitude, metres + 1000 (ArduPilot: AMSL, clamped to 0..5000 m).
    pub altitude: u16,
    /// Satellites.
    pub satellites: u8,
}

impl Gps {
    /// Latitude, degrees.
    pub fn lat_deg(&self) -> f64 {
        f64::from(self.lat) * 1e-7
    }
    /// Longitude, degrees.
    pub fn lon_deg(&self) -> f64 {
        f64::from(self.lon) * 1e-7
    }
    /// Ground speed, m/s.
    pub fn groundspeed_m_s(&self) -> f32 {
        f32::from(self.groundspeed) / 36.0
    }
    /// Course over ground, degrees.
    pub fn heading_deg(&self) -> f32 {
        f32::from(self.heading) / 100.0
    }
    /// Altitude, metres.
    pub fn altitude_m(&self) -> i32 {
        i32::from(self.altitude) - 1000
    }
}

/// Battery (0x08).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Battery {
    /// Volts × 10.
    pub voltage_dv: u16,
    /// Amps × 10.
    pub current_da: u16,
    /// Capacity used, mAh (24 bits on the wire).
    pub used_mah: u32,
    /// Remaining, percent.
    pub remaining_pct: u8,
}

impl Battery {
    /// Volts.
    pub fn voltage_v(&self) -> f32 {
        f32::from(self.voltage_dv) / 10.0
    }
    /// Amps.
    pub fn current_a(&self) -> f32 {
        f32::from(self.current_da) / 10.0
    }
}

/// Attitude (0x1E): radians × 10 000.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Attitude {
    /// Pitch.
    pub pitch: i16,
    /// Roll.
    pub roll: i16,
    /// Yaw.
    pub yaw: i16,
}

impl Attitude {
    /// (pitch, roll, yaw) in radians.
    pub fn radians(&self) -> (f32, f32, f32) {
        let r = |v: i16| f32::from(v) / 10_000.0;
        (r(self.pitch), r(self.roll), r(self.yaw))
    }
}

/// Barometric altitude and vertical speed (0x09), as ArduPilot packs them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BaroAltitude {
    /// Packed altitude: below 0x8000, decimetres + 10 000; with bit 15 set, whole metres.
    /// ArduPilot sends the height above home.
    pub altitude_packed: u16,
    /// Packed vertical speed: sign × ln(|cm/s| / 100 + 1) / 0.026.
    pub vertical_speed_packed: i8,
}

impl BaroAltitude {
    /// Altitude, metres.
    pub fn altitude_m(&self) -> f32 {
        if self.altitude_packed & 0x8000 == 0 {
            (f32::from(self.altitude_packed) - 10_000.0) / 10.0
        } else {
            f32::from(self.altitude_packed & 0x7FFF)
        }
    }
    /// Vertical speed, m/s (the packing loses precision away from zero).
    pub fn vertical_speed_m_s(&self) -> f32 {
        let v = f32::from(self.vertical_speed_packed);
        let cm_s = ((v.abs() * 0.026).exp() - 1.0) * 100.0;
        v.signum() * cm_s / 100.0
    }
}

/// Link statistics (0x14).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkStatistics {
    /// Uplink RSSI antenna 1, −dBm.
    pub uplink_rssi_ant1: u8,
    /// Uplink RSSI antenna 2, −dBm.
    pub uplink_rssi_ant2: u8,
    /// Uplink link quality, %.
    pub uplink_lq: u8,
    /// Uplink SNR, dB.
    pub uplink_snr: i8,
    /// Active antenna (0 = 1, 1 = 2).
    pub active_antenna: u8,
    /// RF mode (link-rate index; see ArduPilot's `RF_MODE_RATES`).
    pub rf_mode: u8,
    /// Uplink TX power index ([`LinkStatistics::tx_power_mw`]).
    pub uplink_tx_power: u8,
    /// Downlink RSSI, −dBm.
    pub downlink_rssi: u8,
    /// Downlink link quality, %.
    pub downlink_lq: u8,
    /// Downlink SNR, dB.
    pub downlink_snr: i8,
}

/// TX power table for [`LinkStatistics::uplink_tx_power`], mW (`AP_RCProtocol_CRSF.h`).
pub const TX_POWER_MW: [u16; 9] = [0, 10, 25, 100, 500, 1000, 2000, 250, 50];

impl LinkStatistics {
    /// Uplink RSSI of the active antenna, dBm.
    pub fn uplink_rssi_dbm(&self) -> i16 {
        -i16::from(if self.active_antenna == 0 {
            self.uplink_rssi_ant1
        } else {
            self.uplink_rssi_ant2
        })
    }
    /// Uplink TX power, mW (`None` for an index outside the table).
    pub fn tx_power_mw(&self) -> Option<u16> {
        TX_POWER_MW.get(usize::from(self.uplink_tx_power)).copied()
    }
}

/// Link statistics of one side (0x1C receiver, 0x1D transmitter).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkStatisticsSide {
    /// RSSI, −dBm.
    pub rssi_db: u8,
    /// RSSI, %.
    pub rssi_pct: u8,
    /// Link quality, %.
    pub lq: u8,
    /// SNR, dB.
    pub snr: i8,
    /// RF power, dBm.
    pub rf_power_dbm: u8,
    /// Frames per second / 10 (transmitter side only).
    pub fps_div10: Option<u8>,
}

/// A decoded payload.
#[derive(Debug, Clone, PartialEq)]
pub enum Payload {
    /// 0x02.
    Gps(Gps),
    /// 0x07: vertical speed, cm/s.
    Vario(i16),
    /// 0x08.
    Battery(Battery),
    /// 0x09.
    BaroAltitude(BaroAltitude),
    /// 0x0B: origin address.
    Heartbeat(u8),
    /// 0x14.
    LinkStatistics(LinkStatistics),
    /// 0x16: raw 11-bit values ([`rc::us_from_raw`] for µs).
    RcChannels([u16; rc::CHANNELS]),
    /// 0x1C.
    LinkStatisticsRx(LinkStatisticsSide),
    /// 0x1D.
    LinkStatisticsTx(LinkStatisticsSide),
    /// 0x1E.
    Attitude(Attitude),
    /// 0x21: text up to the first NUL.
    FlightMode(String),
    /// Any other type: the raw payload.
    Other(u8, Vec<u8>),
}

fn be16(b: &[u8], i: usize) -> u16 {
    u16::from_be_bytes([b[i], b[i + 1]])
}

fn be32(b: &[u8], i: usize) -> u32 {
    u32::from_be_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]])
}

impl Payload {
    /// Decode `payload` of `frame_type`. Extra trailing bytes are ignored (newer senders may
    /// append fields).
    pub fn decode(frame_type: u8, b: &[u8]) -> Result<Payload, DecodeError> {
        let need = |n: usize| {
            if b.len() < n {
                Err(DecodeError::Short {
                    frame_type,
                    need: n,
                    got: b.len(),
                })
            } else {
                Ok(())
            }
        };
        Ok(match frame_type {
            ft::GPS => {
                need(15)?;
                Payload::Gps(Gps {
                    lat: be32(b, 0) as i32,
                    lon: be32(b, 4) as i32,
                    groundspeed: be16(b, 8),
                    heading: be16(b, 10),
                    altitude: be16(b, 12),
                    satellites: b[14],
                })
            }
            ft::VARIO => {
                need(2)?;
                Payload::Vario(be16(b, 0) as i16)
            }
            ft::BATTERY => {
                need(8)?;
                Payload::Battery(Battery {
                    voltage_dv: be16(b, 0),
                    current_da: be16(b, 2),
                    used_mah: u32::from_be_bytes([0, b[4], b[5], b[6]]),
                    remaining_pct: b[7],
                })
            }
            ft::BARO_ALTITUDE => {
                need(3)?;
                Payload::BaroAltitude(BaroAltitude {
                    altitude_packed: be16(b, 0),
                    vertical_speed_packed: b[2] as i8,
                })
            }
            ft::HEARTBEAT => {
                // Origin address: one byte from ArduPilot (`HeartbeatFrame`); a sender that
                // writes it as 16-bit big-endian puts it in the last byte.
                need(1)?;
                Payload::Heartbeat(*b.last().unwrap_or(&0))
            }
            ft::LINK_STATISTICS => {
                need(10)?;
                Payload::LinkStatistics(LinkStatistics {
                    uplink_rssi_ant1: b[0],
                    uplink_rssi_ant2: b[1],
                    uplink_lq: b[2],
                    uplink_snr: b[3] as i8,
                    active_antenna: b[4],
                    rf_mode: b[5],
                    uplink_tx_power: b[6],
                    downlink_rssi: b[7],
                    downlink_lq: b[8],
                    downlink_snr: b[9] as i8,
                })
            }
            ft::RC_CHANNELS_PACKED => {
                need(rc::PACKED_LEN)?;
                let mut packed = [0u8; rc::PACKED_LEN];
                packed.copy_from_slice(&b[..rc::PACKED_LEN]);
                Payload::RcChannels(rc::unpack(&packed))
            }
            ft::LINK_STATISTICS_RX | ft::LINK_STATISTICS_TX => {
                let tx = frame_type == ft::LINK_STATISTICS_TX;
                need(if tx { 6 } else { 5 })?;
                let side = LinkStatisticsSide {
                    rssi_db: b[0],
                    rssi_pct: b[1],
                    lq: b[2],
                    snr: b[3] as i8,
                    rf_power_dbm: b[4],
                    fps_div10: tx.then(|| b[5]),
                };
                if tx {
                    Payload::LinkStatisticsTx(side)
                } else {
                    Payload::LinkStatisticsRx(side)
                }
            }
            ft::ATTITUDE => {
                need(6)?;
                Payload::Attitude(Attitude {
                    pitch: be16(b, 0) as i16,
                    roll: be16(b, 2) as i16,
                    yaw: be16(b, 4) as i16,
                })
            }
            ft::FLIGHT_MODE => {
                let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
                Payload::FlightMode(String::from_utf8_lossy(&b[..end]).into_owned())
            }
            other => Payload::Other(other, b.to_vec()),
        })
    }

    /// Frame type and payload bytes.
    pub fn encode(&self) -> (u8, Vec<u8>) {
        match self {
            Payload::Gps(g) => {
                let mut v = Vec::with_capacity(15);
                v.extend_from_slice(&g.lat.to_be_bytes());
                v.extend_from_slice(&g.lon.to_be_bytes());
                v.extend_from_slice(&g.groundspeed.to_be_bytes());
                v.extend_from_slice(&g.heading.to_be_bytes());
                v.extend_from_slice(&g.altitude.to_be_bytes());
                v.push(g.satellites);
                (ft::GPS, v)
            }
            Payload::Vario(cm_s) => (ft::VARIO, cm_s.to_be_bytes().to_vec()),
            Payload::Battery(b) => {
                let mut v = Vec::with_capacity(8);
                v.extend_from_slice(&b.voltage_dv.to_be_bytes());
                v.extend_from_slice(&b.current_da.to_be_bytes());
                v.extend_from_slice(&b.used_mah.min(0xFF_FFFF).to_be_bytes()[1..]);
                v.push(b.remaining_pct);
                (ft::BATTERY, v)
            }
            Payload::BaroAltitude(a) => {
                let mut v = a.altitude_packed.to_be_bytes().to_vec();
                v.push(a.vertical_speed_packed as u8);
                (ft::BARO_ALTITUDE, v)
            }
            Payload::Heartbeat(origin) => (ft::HEARTBEAT, vec![*origin]),
            Payload::LinkStatistics(l) => (
                ft::LINK_STATISTICS,
                vec![
                    l.uplink_rssi_ant1,
                    l.uplink_rssi_ant2,
                    l.uplink_lq,
                    l.uplink_snr as u8,
                    l.active_antenna,
                    l.rf_mode,
                    l.uplink_tx_power,
                    l.downlink_rssi,
                    l.downlink_lq,
                    l.downlink_snr as u8,
                ],
            ),
            Payload::RcChannels(raw) => (ft::RC_CHANNELS_PACKED, rc::pack(raw).to_vec()),
            Payload::LinkStatisticsRx(s) | Payload::LinkStatisticsTx(s) => {
                let mut v = vec![s.rssi_db, s.rssi_pct, s.lq, s.snr as u8, s.rf_power_dbm];
                let t = if let Payload::LinkStatisticsTx(_) = self {
                    v.push(s.fps_div10.unwrap_or(0));
                    ft::LINK_STATISTICS_TX
                } else {
                    ft::LINK_STATISTICS_RX
                };
                (t, v)
            }
            Payload::Attitude(a) => {
                let mut v = Vec::with_capacity(6);
                for x in [a.pitch, a.roll, a.yaw] {
                    v.extend_from_slice(&x.to_be_bytes());
                }
                (ft::ATTITUDE, v)
            }
            Payload::FlightMode(s) => {
                // Whole characters only, so a truncated name still decodes to itself.
                let mut end = 0;
                for (i, c) in s.char_indices() {
                    if i + c.len_utf8() > crate::PAYLOAD_MAX - 1 {
                        break;
                    }
                    end = i + c.len_utf8();
                }
                let mut v = s.as_bytes()[..end].to_vec();
                v.push(0);
                (ft::FLIGHT_MODE, v)
            }
            Payload::Other(t, b) => (*t, b.clone()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Frame, Parser, addr};

    fn round_trip(p: Payload) {
        let (t, bytes) = p.encode();
        let f = Frame {
            addr: addr::RADIO_TRANSMITTER,
            frame_type: t,
            payload: bytes,
        };
        let mut parser = Parser::default();
        parser.push(&f.encode().unwrap());
        assert_eq!(parser.next_frame().unwrap().decode().unwrap(), p);
    }

    #[test]
    fn every_payload_round_trips_through_a_frame() {
        round_trip(Payload::Gps(Gps {
            lat: -353_632_610,
            lon: 1_491_652_300,
            groundspeed: 362,
            heading: 35_999,
            altitude: 1584,
            satellites: 14,
        }));
        round_trip(Payload::Vario(-250));
        round_trip(Payload::Battery(Battery {
            voltage_dv: 168,
            current_da: 123,
            used_mah: 0xFF_FFFF,
            remaining_pct: 77,
        }));
        round_trip(Payload::BaroAltitude(BaroAltitude {
            altitude_packed: 0x8000 | 3000,
            vertical_speed_packed: -40,
        }));
        round_trip(Payload::Heartbeat(addr::FLIGHT_CONTROLLER));
        round_trip(Payload::LinkStatistics(LinkStatistics {
            uplink_rssi_ant1: 67,
            uplink_rssi_ant2: 71,
            uplink_lq: 100,
            uplink_snr: -3,
            active_antenna: 1,
            rf_mode: 7,
            uplink_tx_power: 3,
            downlink_rssi: 80,
            downlink_lq: 98,
            downlink_snr: 9,
        }));
        let mut raw = [rc::RAW_MID; rc::CHANNELS];
        raw[2] = rc::RAW_MIN;
        raw[15] = 2047;
        round_trip(Payload::RcChannels(raw));
        let side = LinkStatisticsSide {
            rssi_db: 70,
            rssi_pct: 60,
            lq: 99,
            snr: 5,
            rf_power_dbm: 20,
            fps_div10: None,
        };
        round_trip(Payload::LinkStatisticsRx(side));
        round_trip(Payload::LinkStatisticsTx(LinkStatisticsSide {
            fps_div10: Some(25),
            ..side
        }));
        round_trip(Payload::Attitude(Attitude {
            pitch: -31_415,
            roll: 31_415,
            yaw: 0,
        }));
        round_trip(Payload::FlightMode("LOITER*".into()));
        // Too long for a frame: cut on a character boundary.
        let (_, b) = Payload::FlightMode("é".repeat(40)).encode();
        assert_eq!(b.len(), 59, "29 two-byte characters and the NUL");
        assert_eq!(
            Payload::decode(ft::FLIGHT_MODE, &b).unwrap(),
            Payload::FlightMode("é".repeat(29))
        );
        round_trip(Payload::Other(0x7F, vec![1, 2, 3]));
    }

    #[test]
    fn units_and_scaling() {
        let g = Gps {
            lat: -353_632_610,
            lon: 1_491_652_300,
            groundspeed: 360, // 36.0 km/h = 10 m/s
            heading: 18_000,
            altitude: 1584,
            satellites: 9,
        };
        assert!((g.lat_deg() + 35.363_261).abs() < 1e-9);
        assert!((g.lon_deg() - 149.165_23).abs() < 1e-9);
        assert!((g.groundspeed_m_s() - 10.0).abs() < 1e-6);
        assert_eq!((g.heading_deg(), g.altitude_m()), (180.0, 584));
        let b = Battery {
            voltage_dv: 168,
            current_da: 55,
            used_mah: 1200,
            remaining_pct: 50,
        };
        assert_eq!((b.voltage_v(), b.current_a()), (16.8, 5.5));
        let (p, r, y) = Attitude {
            pitch: 5000,
            roll: -10_000,
            yaw: 31_415,
        }
        .radians();
        assert_eq!((p, r), (0.5, -1.0));
        assert!((y - std::f32::consts::PI).abs() < 1e-4);
        // Baro altitude: decimetre range, metre range (ArduPilot get_altitude_packed).
        let alt = |packed| {
            BaroAltitude {
                altitude_packed: packed,
                vertical_speed_packed: 0,
            }
            .altitude_m()
        };
        assert_eq!(alt(10_000 + 1234), 123.4);
        assert_eq!(alt(0), -1000.0);
        assert_eq!(alt(0x8000 | 5000), 5000.0);
        // Vertical speed: ArduPilot packs 2 m/s as int8(ln(200/100 + 1)/0.026) = 42.
        let vs = |p| {
            BaroAltitude {
                altitude_packed: 0,
                vertical_speed_packed: p,
            }
            .vertical_speed_m_s()
        };
        assert!((vs(42) - 1.98).abs() < 0.03, "{}", vs(42));
        assert!((vs(-42) + 1.98).abs() < 0.03);
        assert_eq!(vs(0), 0.0);
        let l = LinkStatistics {
            uplink_rssi_ant1: 60,
            uplink_rssi_ant2: 90,
            uplink_lq: 100,
            uplink_snr: 0,
            active_antenna: 1,
            rf_mode: 0,
            uplink_tx_power: 6,
            downlink_rssi: 0,
            downlink_lq: 0,
            downlink_snr: 0,
        };
        assert_eq!((l.uplink_rssi_dbm(), l.tx_power_mw()), (-90, Some(2000)));
        assert_eq!(
            LinkStatistics {
                uplink_tx_power: 9,
                ..l
            }
            .tx_power_mw(),
            None
        );
    }

    #[test]
    fn short_payloads_are_errors_and_flight_mode_stops_at_nul() {
        assert_eq!(
            Payload::decode(ft::GPS, &[0; 14]),
            Err(DecodeError::Short {
                frame_type: ft::GPS,
                need: 15,
                got: 14
            })
        );
        assert!(Payload::decode(ft::LINK_STATISTICS_TX, &[0; 5]).is_err());
        assert!(Payload::decode(ft::LINK_STATISTICS_RX, &[0; 5]).is_ok());
        assert_eq!(
            Payload::decode(ft::FLIGHT_MODE, b"AUTO\0garbage").unwrap(),
            Payload::FlightMode("AUTO".into())
        );
        // A longer payload than the layout is accepted (fields appended by newer senders).
        assert!(matches!(
            Payload::decode(ft::ATTITUDE, &[0; 8]).unwrap(),
            Payload::Attitude(_)
        ));
    }
}
