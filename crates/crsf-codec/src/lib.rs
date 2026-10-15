//! CRSF frames (TBS Crossfire and ExpressLRS), as DrorControl reads and writes them
//! (plan 09 §2.4, Step 3). Pure: no I/O, no ArduPilot knowledge.
//!
//! Written from the frame description, checked against the pinned ArduPilot source
//! (`AP_RCProtocol_CRSF`, `AP_CRSF_Telem`, `AP_Math/crc.cpp`, 4.6.3 and 4.7.1) and ExpressLRS
//! 3.6.4 (`crsf_protocol.h`). No code was copied.
//!
//! A frame is `addr, len, type, payload, crc`: `len` counts type + payload + CRC, so a frame
//! is `len + 2` bytes, at most [`FRAME_MAX`]. The CRC is CRC-8/DVB-S2 over type + payload.
//! Telemetry fields are big-endian; RC channels are 11-bit little-endian bit fields.

#![forbid(unsafe_code)]

use std::fmt;

pub mod payload;
pub mod rc;

/// Largest frame on the wire, bytes (`CRSF_FRAMELEN_MAX`).
pub const FRAME_MAX: usize = 64;
/// Largest `len` field (type + payload + CRC).
pub const LEN_MAX: u8 = 62;
/// Smallest `len` field (type + CRC, empty payload).
pub const LEN_MIN: u8 = 2;
/// Largest payload (without type and CRC).
pub const PAYLOAD_MAX: usize = LEN_MAX as usize - 2;

/// Device addresses (first byte of a frame; destination/origin of extended frames).
pub mod addr {
    /// Broadcast.
    pub const BROADCAST: u8 = 0x00;
    /// USB.
    pub const USB: u8 = 0x10;
    /// Flight controller; also the "sync byte" flight-controller-side parsers wait for.
    pub const FLIGHT_CONTROLLER: u8 = 0xC8;
    /// Radio transmitter (the handset).
    pub const RADIO_TRANSMITTER: u8 = 0xEA;
    /// Receiver.
    pub const RECEIVER: u8 = 0xEC;
    /// TX module.
    pub const TRANSMITTER: u8 = 0xEE;
}

/// Frame types used by DrorControl (others pass through as raw frames).
pub mod frame_type {
    /// GPS.
    pub const GPS: u8 = 0x02;
    /// Vario (vertical speed).
    pub const VARIO: u8 = 0x07;
    /// Battery.
    pub const BATTERY: u8 = 0x08;
    /// Barometric altitude and vertical speed.
    pub const BARO_ALTITUDE: u8 = 0x09;
    /// Heartbeat.
    pub const HEARTBEAT: u8 = 0x0B;
    /// Link statistics (uplink and downlink).
    pub const LINK_STATISTICS: u8 = 0x14;
    /// 16 RC channels, 11 bits each.
    pub const RC_CHANNELS_PACKED: u8 = 0x16;
    /// Link statistics, receiver side.
    pub const LINK_STATISTICS_RX: u8 = 0x1C;
    /// Link statistics, transmitter side.
    pub const LINK_STATISTICS_TX: u8 = 0x1D;
    /// Attitude.
    pub const ATTITUDE: u8 = 0x1E;
    /// Flight mode text.
    pub const FLIGHT_MODE: u8 = 0x21;
    /// First extended type: from here the payload starts with destination and origin.
    pub const EXTENDED_FIRST: u8 = 0x28;
    /// Command (extended); some sub-commands carry an inner CRC ([`crate::command_crc`]).
    pub const COMMAND: u8 = 0x32;
}

/// CRC-8 with polynomial `poly`, MSB first, initial value 0 (`crc8_dvb` in ArduPilot).
pub fn crc8(poly: u8, data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ poly
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// CRC-8/DVB-S2 (polynomial 0xD5): the frame CRC.
pub fn crc8_dvb_s2(data: &[u8]) -> u8 {
    crc8(0xD5, data)
}

/// The inner CRC some COMMAND (0x32) frames carry (VTX, speed negotiation, bind): CRC-8
/// polynomial 0xBA over the type byte, destination, origin, command id and its payload
/// (`AP_CRSF_Telem` 4.6.3).
pub fn command_crc(dest: u8, origin: u8, command: &[u8]) -> u8 {
    let mut data = vec![frame_type::COMMAND, dest, origin];
    data.extend_from_slice(command);
    crc8(0xBA, &data)
}

/// One frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// First byte: destination address (or the 0xC8 sync byte).
    pub addr: u8,
    /// Frame type.
    pub frame_type: u8,
    /// Payload (without type and CRC).
    pub payload: Vec<u8>,
}

/// Why a frame cannot be built.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// Payload longer than [`PAYLOAD_MAX`].
    TooLong(usize),
}

impl fmt::Display for EncodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EncodeError::TooLong(n) => write!(f, "{n}-byte payload: at most {PAYLOAD_MAX}"),
        }
    }
}

impl std::error::Error for EncodeError {}

impl Frame {
    /// Bytes on the wire.
    pub fn encode(&self) -> Result<Vec<u8>, EncodeError> {
        if self.payload.len() > PAYLOAD_MAX {
            return Err(EncodeError::TooLong(self.payload.len()));
        }
        let mut out = Vec::with_capacity(self.payload.len() + 4);
        out.push(self.addr);
        out.push(self.payload.len() as u8 + 2);
        out.push(self.frame_type);
        out.extend_from_slice(&self.payload);
        out.push(crc8_dvb_s2(&out[2..]));
        Ok(out)
    }

    /// Extended frame (type ≥ 0x28): `(destination, origin, rest of the payload)`.
    pub fn extended(&self) -> Option<(u8, u8, &[u8])> {
        (self.frame_type >= frame_type::EXTENDED_FIRST && self.payload.len() >= 2)
            .then(|| (self.payload[0], self.payload[1], &self.payload[2..]))
    }

    /// The typed payload ([`payload::Payload::Other`] for types not decoded here).
    pub fn decode(&self) -> Result<payload::Payload, payload::DecodeError> {
        payload::Payload::decode(self.frame_type, &self.payload)
    }
}

/// Counters of what the parser has seen.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ParserStats {
    /// Bytes pushed.
    pub bytes: u64,
    /// Good frames.
    pub frames: u64,
    /// Frames with a bad CRC (dropped).
    pub crc_errors: u64,
    /// Bytes skipped to find a frame start.
    pub skipped_bytes: u64,
}

/// Turns a byte stream into frames. A frame starts with one of the accepted addresses;
/// anything else, a bad length or a bad CRC skips one byte and searches again.
#[derive(Debug, Clone)]
pub struct Parser {
    starts: Vec<u8>,
    buf: Vec<u8>,
    stats: ParserStats,
}

impl Default for Parser {
    /// Accepts frames to the flight controller, handset, receiver and TX module: what a GCS
    /// on a TX module's CRSF pin or on a receiver's line can see.
    fn default() -> Self {
        Parser::new(&[
            addr::FLIGHT_CONTROLLER,
            addr::RADIO_TRANSMITTER,
            addr::RECEIVER,
            addr::TRANSMITTER,
        ])
    }
}

impl Parser {
    /// A parser that accepts frames starting with any of `starts`.
    pub fn new(starts: &[u8]) -> Parser {
        Parser {
            starts: starts.to_vec(),
            buf: Vec::with_capacity(2 * FRAME_MAX),
            stats: ParserStats::default(),
        }
    }

    /// Counters so far.
    pub fn stats(&self) -> ParserStats {
        self.stats
    }

    /// Add received bytes.
    pub fn push(&mut self, bytes: &[u8]) {
        self.stats.bytes += bytes.len() as u64;
        self.buf.extend_from_slice(bytes);
    }

    fn skip(&mut self, n: usize) {
        self.buf.drain(..n);
        self.stats.skipped_bytes += n as u64;
    }

    /// The next complete frame, if one is buffered.
    pub fn next_frame(&mut self) -> Option<Frame> {
        loop {
            // Find a start byte.
            match self.buf.iter().position(|b| self.starts.contains(b)) {
                Some(0) => {}
                Some(n) => self.skip(n),
                None => {
                    let n = self.buf.len();
                    self.skip(n);
                    return None;
                }
            }
            let len = *self.buf.get(1)?;
            if !(LEN_MIN..=LEN_MAX).contains(&len) {
                self.skip(1);
                continue;
            }
            let total = len as usize + 2;
            if self.buf.len() < total {
                return None;
            }
            let crc = self.buf[total - 1];
            if crc8_dvb_s2(&self.buf[2..total - 1]) != crc {
                self.stats.crc_errors += 1;
                self.skip(1);
                continue;
            }
            let frame = Frame {
                addr: self.buf[0],
                frame_type: self.buf[2],
                payload: self.buf[3..total - 1].to_vec(),
            };
            self.buf.drain(..total);
            self.stats.frames += 1;
            return Some(frame);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_check_values() {
        // CRC catalogue check value for CRC-8/DVB-S2: "123456789" -> 0xBC.
        assert_eq!(crc8_dvb_s2(b"123456789"), 0xBC);
        assert_eq!(crc8_dvb_s2(&[]), 0);
        // Polynomial 0xBA, same routine (AP_Math crc8_dvb).
        assert_eq!(crc8(0xBA, &[]), 0);
        assert_ne!(crc8(0xBA, b"123456789"), crc8_dvb_s2(b"123456789"));
    }

    #[test]
    fn frames_round_trip_and_bounds_hold() {
        let f = Frame {
            addr: addr::FLIGHT_CONTROLLER,
            frame_type: frame_type::FLIGHT_MODE,
            payload: b"LOITER\0".to_vec(),
        };
        let bytes = f.encode().unwrap();
        assert_eq!(bytes[1] as usize, f.payload.len() + 2);
        assert_eq!(bytes.len(), bytes[1] as usize + 2);
        let mut p = Parser::default();
        p.push(&bytes);
        assert_eq!(p.next_frame(), Some(f));
        // The largest frame is 64 bytes; one more byte of payload is refused.
        let max = Frame {
            addr: addr::RADIO_TRANSMITTER,
            frame_type: 0x7F,
            payload: vec![0xA5; PAYLOAD_MAX],
        };
        assert_eq!(max.encode().unwrap().len(), FRAME_MAX);
        let too = Frame {
            payload: vec![0; PAYLOAD_MAX + 1],
            ..max.clone()
        };
        assert_eq!(too.encode(), Err(EncodeError::TooLong(PAYLOAD_MAX + 1)));
        p.push(&max.encode().unwrap());
        assert_eq!(p.next_frame(), Some(max));
    }

    #[test]
    fn parser_resyncs_and_counts() {
        let good = Frame {
            addr: addr::FLIGHT_CONTROLLER,
            frame_type: frame_type::VARIO,
            payload: vec![0x00, 0x64],
        }
        .encode()
        .unwrap();
        let mut bad = good.clone();
        *bad.last_mut().unwrap() ^= 0xFF;
        let mut p = Parser::default();
        // Garbage, a bad length after a start byte, a bad CRC, then a good frame.
        p.push(&[0x01, 0x02, 0xC8, 0x01, 0x55]);
        p.push(&bad);
        // Truncated: waits for the rest.
        p.push(&good[..3]);
        assert_eq!(p.next_frame(), None);
        p.push(&good[3..]);
        assert_eq!(p.next_frame().unwrap().payload, vec![0x00, 0x64]);
        assert_eq!(p.next_frame(), None);
        let s = p.stats();
        assert_eq!((s.frames, s.crc_errors), (1, 1));
        assert!(s.skipped_bytes >= 5 + bad.len() as u64 - 1);
        assert_eq!(s.bytes, 5 + bad.len() as u64 + good.len() as u64);
    }

    #[test]
    fn random_bytes_never_panic_or_yield_a_bad_frame() {
        // xorshift64*, seeded: deterministic.
        let mut x = 0x9E37_79B9_7F4A_7C15u64;
        let mut next = || {
            x ^= x >> 12;
            x ^= x << 25;
            x ^= x >> 27;
            (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 56) as u8
        };
        let mut p = Parser::default();
        let mut frames = 0;
        for _ in 0..2000 {
            let chunk: Vec<u8> = (0..(next() as usize % 97)).map(|_| next()).collect();
            p.push(&chunk);
            while let Some(f) = p.next_frame() {
                frames += 1;
                // Every frame that comes out re-encodes to bytes with a valid CRC.
                let b = f.encode().unwrap();
                assert_eq!(crc8_dvb_s2(&b[2..b.len() - 1]), *b.last().unwrap());
            }
        }
        let s = p.stats();
        assert_eq!(s.frames, frames);
        assert!(s.bytes > 50_000);
    }

    #[test]
    fn extended_frames_carry_destination_and_origin() {
        let f = Frame {
            addr: addr::TRANSMITTER,
            frame_type: frame_type::COMMAND,
            payload: vec![addr::RECEIVER, addr::RADIO_TRANSMITTER, 0x10, 0x01],
        };
        assert_eq!(
            f.extended(),
            Some((addr::RECEIVER, addr::RADIO_TRANSMITTER, &[0x10, 0x01][..]))
        );
        let short = Frame {
            frame_type: frame_type::GPS,
            ..f.clone()
        };
        assert_eq!(short.extended(), None);
        // The inner CRC covers type, destination, origin and the command bytes.
        let c = command_crc(addr::RECEIVER, addr::RADIO_TRANSMITTER, &[0x10, 0x01]);
        assert_eq!(
            c,
            crc8(
                0xBA,
                &[
                    frame_type::COMMAND,
                    addr::RECEIVER,
                    addr::RADIO_TRANSMITTER,
                    0x10,
                    0x01
                ]
            )
        );
    }
}
