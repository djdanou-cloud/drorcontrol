//! MAVLink frames: validation, stream parsing with resynchronisation, and encoding.
//!
//! Layouts (all multi-byte values little-endian):
//!
//! ```text
//! v1: STX=0xFE len seq sysid compid msgid(1)                     payload crc(2)
//! v2: STX=0xFD len incompat compat seq sysid compid msgid(3)     payload crc(2) [signature(13)]
//! ```
//!
//! The checksum covers everything after STX up to the end of the payload, then the
//! message's CRC_EXTRA byte. MAVLink 2 senders drop trailing zero bytes from the payload;
//! receivers zero-extend.

use crate::{Message, UnknownMessage, crc, message_info};

/// MAVLink 1 start byte.
pub const STX_V1: u8 = 0xFE;
/// MAVLink 2 start byte.
pub const STX_V2: u8 = 0xFD;
/// `incompat_flags` bit: the frame carries a 13-byte signature.
pub const IFLAG_SIGNED: u8 = 0x01;
/// Signature block length (link id + 48-bit timestamp + 48-bit signature).
pub const SIGNATURE_LEN: usize = 13;
/// Largest possible frame: v2 header + 255-byte payload + CRC + signature.
pub const MAX_FRAME_LEN: usize = 10 + 255 + 2 + SIGNATURE_LEN;

/// Protocol version of a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Version {
    /// MAVLink 1 (`0xFE`).
    V1,
    /// MAVLink 2 (`0xFD`).
    V2,
}

impl Version {
    fn header_len(self) -> usize {
        match self {
            Version::V1 => 6,
            Version::V2 => 10,
        }
    }
}

/// Sender identity and sequence number for an outgoing frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Header {
    /// Packet sequence number.
    pub seq: u8,
    /// Sender system ID.
    pub sysid: u8,
    /// Sender component ID.
    pub compid: u8,
}

/// One complete MAVLink frame, stored inline (no heap allocation).
#[derive(Clone)]
pub struct Frame {
    buf: [u8; MAX_FRAME_LEN],
    len: usize,
    crc_checked: bool,
}

impl std::fmt::Debug for Frame {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Frame")
            .field("version", &self.version())
            .field("msgid", &self.msgid())
            .field("sysid", &self.sysid())
            .field("compid", &self.compid())
            .field("seq", &self.seq())
            .field("payload_len", &self.payload().len())
            .field("signed", &self.is_signed())
            .field("crc_checked", &self.crc_checked)
            .finish()
    }
}

impl PartialEq for Frame {
    fn eq(&self, other: &Self) -> bool {
        self.as_bytes() == other.as_bytes()
    }
}

impl Eq for Frame {}

impl Frame {
    /// The raw frame bytes, exactly as on the wire.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    /// Protocol version.
    pub fn version(&self) -> Version {
        if self.buf[0] == STX_V1 {
            Version::V1
        } else {
            Version::V2
        }
    }

    fn header_len(&self) -> usize {
        self.version().header_len()
    }

    /// Header byte offset for v1 or v2 layout.
    fn at(&self, v1: usize, v2: usize) -> usize {
        match self.version() {
            Version::V1 => v1,
            Version::V2 => v2,
        }
    }

    /// `incompat_flags` (always 0 for v1).
    pub fn incompat_flags(&self) -> u8 {
        match self.version() {
            Version::V1 => 0,
            Version::V2 => self.buf[2],
        }
    }

    /// `compat_flags` (always 0 for v1).
    pub fn compat_flags(&self) -> u8 {
        match self.version() {
            Version::V1 => 0,
            Version::V2 => self.buf[3],
        }
    }

    /// Sequence number.
    pub fn seq(&self) -> u8 {
        self.buf[self.at(2, 4)]
    }

    /// Sender system ID.
    pub fn sysid(&self) -> u8 {
        self.buf[self.at(3, 5)]
    }

    /// Sender component ID.
    pub fn compid(&self) -> u8 {
        self.buf[self.at(4, 6)]
    }

    /// Message ID.
    pub fn msgid(&self) -> u32 {
        match self.version() {
            Version::V1 => u32::from(self.buf[5]),
            Version::V2 => u32::from_le_bytes([self.buf[7], self.buf[8], self.buf[9], 0]),
        }
    }

    /// Payload bytes as transmitted (v2 payloads may be truncated).
    pub fn payload(&self) -> &[u8] {
        let h = self.header_len();
        &self.buf[h..h + usize::from(self.buf[1])]
    }

    /// The transmitted checksum.
    pub fn checksum(&self) -> u16 {
        let at = self.header_len() + usize::from(self.buf[1]);
        u16::from_le_bytes([self.buf[at], self.buf[at + 1]])
    }

    /// True if the v2 signed flag is set.
    pub fn is_signed(&self) -> bool {
        self.incompat_flags() & IFLAG_SIGNED != 0
    }

    /// The 13-byte signature block (link id, timestamp, signature), if signed.
    pub fn signature_block(&self) -> Option<&[u8]> {
        self.is_signed()
            .then(|| &self.buf[self.len - SIGNATURE_LEN..self.len])
    }

    /// Bytes covered by the signature hash: STX through the checksum.
    pub(crate) fn unsigned_part(&self) -> &[u8] {
        &self.buf[..self.header_len() + usize::from(self.buf[1]) + 2]
    }

    /// Whether the checksum was verified against a known CRC_EXTRA. False only for
    /// frames of unknown message IDs accepted by a pass-through parser.
    pub fn crc_checked(&self) -> bool {
        self.crc_checked
    }

    /// Routing target `(target_system, target_component)` read straight from the payload
    /// (no decoding). `None` if the message has no target fields or its id is unknown;
    /// 0 means broadcast. Truncated (v2) payloads read missing bytes as 0.
    pub fn target(&self) -> Option<(u8, u8)> {
        let info = message_info(self.msgid())?;
        let byte = |ofs: Option<usize>| ofs.map(|o| self.payload().get(o).copied().unwrap_or(0));
        let sys = byte(info.target_system_ofs)?;
        Some((sys, byte(info.target_component_ofs).unwrap_or(0)))
    }

    /// Decode the payload into a typed message.
    pub fn decode(&self) -> Result<Message, UnknownMessage> {
        Message::parse(self.msgid(), self.payload())
    }

    /// Validate one complete frame (exactly one, no trailing bytes).
    pub fn from_bytes(bytes: &[u8]) -> Result<Frame, FrameError> {
        let mut p = FrameParser::new();
        p.push(bytes);
        match p.next_frame() {
            Some(f) if f.len == bytes.len() && p.stats().skipped_bytes == 0 => Ok(f),
            Some(_) => Err(FrameError::TrailingOrLeadingBytes),
            None => {
                let s = p.stats();
                Err(if s.crc_errors > 0 {
                    FrameError::BadChecksum
                } else if s.unknown_ids > 0 {
                    FrameError::UnknownMessage
                } else if s.incompatible > 0 {
                    FrameError::IncompatibleFlags
                } else {
                    FrameError::Incomplete
                })
            }
        }
    }

    pub(crate) fn set_signature(&mut self, block: &[u8; SIGNATURE_LEN]) {
        let end = self.header_len() + usize::from(self.buf[1]) + 2;
        self.buf[end..end + SIGNATURE_LEN].copy_from_slice(block);
        self.len = end + SIGNATURE_LEN;
    }

    /// Set the signed flag and recompute the checksum (the flag is covered by it).
    pub(crate) fn mark_signed(&mut self, crc_extra: u8) {
        self.buf[2] |= IFLAG_SIGNED;
        let end = self.header_len() + usize::from(self.buf[1]);
        let c = crc::frame_crc(&self.buf[1..end], crc_extra);
        self.buf[end..end + 2].copy_from_slice(&c.to_le_bytes());
    }
}

/// Why [`Frame::from_bytes`] rejected its input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Not enough bytes for a complete frame.
    Incomplete,
    /// Checksum mismatch.
    BadChecksum,
    /// Message ID not in the dialect, so the checksum cannot be verified.
    UnknownMessage,
    /// Unsupported `incompat_flags` bits.
    IncompatibleFlags,
    /// Input is not exactly one frame.
    TrailingOrLeadingBytes,
}

/// Why a message could not be encoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// MAVLink 1 only carries message IDs 0..=255.
    IdTooLargeForV1(u32),
    /// Signing requires MAVLink 2.
    SigningRequiresV2,
    /// Message ID not in the dialect.
    UnknownMessage(u32),
}

impl std::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EncodeError::IdTooLargeForV1(id) => {
                write!(f, "message id {id} cannot be sent as MAVLink 1")
            }
            EncodeError::SigningRequiresV2 => f.write_str("signing requires MAVLink 2"),
            EncodeError::UnknownMessage(id) => write!(f, "message id {id} is not in the dialect"),
        }
    }
}

impl std::error::Error for EncodeError {}

/// Encode `msg` into an unsigned frame.
///
/// MAVLink 1 sends only the base fields; MAVLink 2 sends extensions and drops trailing
/// zero bytes (keeping at least one payload byte), matching the reference implementations.
pub fn encode(version: Version, header: Header, msg: &Message) -> Result<Frame, EncodeError> {
    let id = msg.id();
    let info = message_info(id).ok_or(EncodeError::UnknownMessage(id))?;
    let mut payload = [0u8; 255];
    let full = msg.serialize(&mut payload);
    let plen = match version {
        Version::V1 => {
            if id > 255 {
                return Err(EncodeError::IdTooLargeForV1(id));
            }
            info.base_len
        }
        Version::V2 => {
            let mut n = full;
            while n > 1 && payload[n - 1] == 0 {
                n -= 1;
            }
            n
        }
    };

    let mut f = Frame {
        buf: [0u8; MAX_FRAME_LEN],
        len: 0,
        crc_checked: true,
    };
    let h = version.header_len();
    let plen_u8 = u8::try_from(plen).expect("generator guarantees payload <= 255");
    match version {
        Version::V1 => {
            f.buf[..6].copy_from_slice(&[
                STX_V1,
                plen_u8,
                header.seq,
                header.sysid,
                header.compid,
                id as u8,
            ]);
        }
        Version::V2 => {
            let idb = id.to_le_bytes();
            f.buf[..10].copy_from_slice(&[
                STX_V2,
                plen_u8,
                0,
                0,
                header.seq,
                header.sysid,
                header.compid,
                idb[0],
                idb[1],
                idb[2],
            ]);
        }
    }
    f.buf[h..h + plen].copy_from_slice(&payload[..plen]);
    let c = crc::frame_crc(&f.buf[1..h + plen], info.crc_extra);
    f.buf[h + plen..h + plen + 2].copy_from_slice(&c.to_le_bytes());
    f.len = h + plen + 2;
    Ok(f)
}

/// Counters describing link quality as seen by the parser.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ParserStats {
    /// Bytes pushed into the parser.
    pub bytes: u64,
    /// Frames returned.
    pub frames: u64,
    /// Candidate frames rejected for a bad checksum.
    pub crc_errors: u64,
    /// Candidate frames with a message ID not in the dialect.
    pub unknown_ids: u64,
    /// Candidate v2 frames with unsupported `incompat_flags`.
    pub incompatible: u64,
    /// Bytes discarded while searching for a valid frame.
    pub skipped_bytes: u64,
}

/// Incremental parser: push bytes as they arrive, pull complete frames.
///
/// On any invalid candidate the parser discards one byte and rescans, so a corrupt or
/// truncated frame never hides the valid frame that follows it.
#[derive(Debug, Default)]
pub struct FrameParser {
    buf: Vec<u8>,
    start: usize,
    stats: ParserStats,
    accept_unknown: bool,
}

impl FrameParser {
    /// A parser that drops frames of unknown message IDs.
    pub fn new() -> Self {
        Self::default()
    }

    /// A parser that also returns frames of unknown message IDs (checksum unverified,
    /// see [`Frame::crc_checked`]) — for routers that forward other dialects.
    pub fn with_unknown_passthrough() -> Self {
        FrameParser {
            accept_unknown: true,
            ..Self::default()
        }
    }

    /// Statistics since creation.
    pub fn stats(&self) -> &ParserStats {
        &self.stats
    }

    /// Append received bytes.
    pub fn push(&mut self, data: &[u8]) {
        self.stats.bytes += data.len() as u64;
        if self.start > 0 && self.start >= self.buf.len() / 2 {
            self.buf.drain(..self.start);
            self.start = 0;
        }
        self.buf.extend_from_slice(data);
    }

    fn skip(&mut self, n: usize) {
        self.start += n;
        self.stats.skipped_bytes += n as u64;
    }

    /// Next complete, valid frame, or `None` if more bytes are needed.
    pub fn next_frame(&mut self) -> Option<Frame> {
        loop {
            let window = &self.buf[self.start..];
            let Some(i) = window.iter().position(|&b| b == STX_V1 || b == STX_V2) else {
                self.skip(window.len());
                return None;
            };
            self.skip(i);
            let w = &self.buf[self.start..];
            let version = if w[0] == STX_V1 {
                Version::V1
            } else {
                Version::V2
            };
            let h = version.header_len();
            if w.len() < h {
                return None;
            }
            let plen = usize::from(w[1]);
            let (signed, msgid) = match version {
                Version::V1 => (false, u32::from(w[5])),
                Version::V2 => {
                    if w[2] & !IFLAG_SIGNED != 0 {
                        self.stats.incompatible += 1;
                        self.skip(1);
                        continue;
                    }
                    (
                        w[2] & IFLAG_SIGNED != 0,
                        u32::from_le_bytes([w[7], w[8], w[9], 0]),
                    )
                }
            };
            // Reject impossible candidates from the header alone, so a false start byte
            // claiming a long payload cannot stall the stream while we wait for bytes.
            let info = message_info(msgid);
            match info {
                None if !self.accept_unknown => {
                    self.stats.unknown_ids += 1;
                    self.skip(1);
                    continue;
                }
                // MAVLink 1 payloads carry the base fields (C library) or, from some
                // senders such as pymavlink, the full payload: anything else is noise.
                Some(i) if version == Version::V1 && !(i.base_len..=i.max_len).contains(&plen) => {
                    self.skip(1);
                    continue;
                }
                _ => {}
            }
            let total = h + plen + 2 + if signed { SIGNATURE_LEN } else { 0 };
            if w.len() < total {
                return None;
            }
            let crc_checked = match info {
                Some(info) => {
                    let expect = crc::frame_crc(&w[1..h + plen], info.crc_extra);
                    if expect != u16::from_le_bytes([w[h + plen], w[h + plen + 1]]) {
                        self.stats.crc_errors += 1;
                        self.skip(1);
                        continue;
                    }
                    true
                }
                None => {
                    self.stats.unknown_ids += 1;
                    false
                }
            };
            let mut f = Frame {
                buf: [0u8; MAX_FRAME_LEN],
                len: total,
                crc_checked,
            };
            f.buf[..total].copy_from_slice(&w[..total]);
            self.start += total;
            self.stats.frames += 1;
            return Some(f);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::messages::{Heartbeat, Statustext, SysStatus};

    fn hb() -> Message {
        Message::Heartbeat(Heartbeat {
            r#type: 2,
            autopilot: 3,
            base_mode: 81,
            custom_mode: 4,
            system_status: 4,
            mavlink_version: 3,
        })
    }

    const H: Header = Header {
        seq: 9,
        sysid: 1,
        compid: 1,
    };

    #[test]
    fn heartbeat_v1_matches_reference_bytes() {
        // Reference produced by pymavlink for the same message and header.
        let f = encode(Version::V1, H, &hb()).unwrap();
        assert_eq!(f.as_bytes()[..6], [0xFE, 9, 9, 1, 1, 0]);
        assert_eq!(f.payload(), [4, 0, 0, 0, 2, 3, 81, 4, 3]);
        assert_eq!((f.seq(), f.sysid(), f.compid(), f.msgid()), (9, 1, 1, 0));
        assert_eq!(f.decode().unwrap(), hb());
    }

    #[test]
    fn v2_truncates_trailing_zeros_and_round_trips() {
        let m = Message::SysStatus(SysStatus::default());
        let f = encode(Version::V2, H, &m).unwrap();
        assert_eq!(f.payload(), [0], "all-zero payload keeps one byte");
        assert_eq!(f.decode().unwrap(), m);
        assert_eq!(Frame::from_bytes(f.as_bytes()).unwrap(), f);
    }

    #[test]
    fn v1_rejects_large_ids() {
        let m = Message::parse(12_900, &[]).ok();
        if let Some(m) = m {
            assert!(matches!(
                encode(Version::V1, H, &m),
                Err(EncodeError::IdTooLargeForV1(_))
            ));
        }
    }

    #[test]
    fn parser_resyncs_after_noise_and_corruption() {
        let a = encode(Version::V2, H, &hb()).unwrap();
        let mut text = Statustext {
            severity: 6,
            ..Default::default()
        };
        text.text[..5].copy_from_slice(b"hello");
        let b = encode(
            Version::V1,
            Header { seq: 10, ..H },
            &Message::Statustext(text),
        )
        .unwrap();
        let mut corrupt = a.as_bytes().to_vec();
        corrupt[12] ^= 0xFF;

        let mut stream = vec![0x00, 0xFD, 0x05, 0xFE];
        stream.extend_from_slice(&corrupt);
        stream.extend_from_slice(a.as_bytes());
        stream.extend_from_slice(&[0xAA; 3]);
        stream.extend_from_slice(b.as_bytes());

        let mut p = FrameParser::new();
        let mut got = Vec::new();
        // Feed one byte at a time: worst case for incremental parsing.
        for byte in &stream {
            p.push(std::slice::from_ref(byte));
            while let Some(f) = p.next_frame() {
                got.push(f);
            }
        }
        assert_eq!(got, vec![a, b]);
        assert_eq!(p.stats().frames, 2);
        assert!(p.stats().crc_errors >= 1);
    }

    #[test]
    fn unknown_ids_dropped_or_passed_through() {
        let mut f = encode(Version::V2, H, &hb()).unwrap();
        // Rewrite the msgid to an unassigned one; checksum can no longer be verified.
        f.buf[7..10].copy_from_slice(&[0xFE, 0xFF, 0xFF]);
        let mut strict = FrameParser::new();
        strict.push(f.as_bytes());
        assert!(strict.next_frame().is_none());
        assert_eq!(strict.stats().unknown_ids, 1);

        let mut lax = FrameParser::with_unknown_passthrough();
        lax.push(f.as_bytes());
        let got = lax.next_frame().unwrap();
        assert!(!got.crc_checked());
        assert_eq!(got.msgid(), 0xFF_FFFE);
    }

    #[test]
    fn reads_routing_target_without_decoding() {
        use crate::messages::{CommandLong, ParamRequestList};
        let cmd = Message::CommandLong(CommandLong {
            target_system: 7,
            target_component: 1,
            command: 400,
            ..Default::default()
        });
        assert_eq!(encode(Version::V2, H, &cmd).unwrap().target(), Some((7, 1)));
        let all = Message::ParamRequestList(ParamRequestList {
            target_system: 3,
            target_component: 0,
        });
        assert_eq!(encode(Version::V2, H, &all).unwrap().target(), Some((3, 0)));
        assert_eq!(
            encode(Version::V2, H, &hb()).unwrap().target(),
            None,
            "HEARTBEAT has no target"
        );
    }

    #[test]
    fn rejects_unknown_incompat_flags() {
        let mut f = encode(Version::V2, H, &hb()).unwrap();
        f.buf[2] = 0x02;
        assert_eq!(
            Frame::from_bytes(f.as_bytes()),
            Err(FrameError::IncompatibleFlags)
        );
    }
}
