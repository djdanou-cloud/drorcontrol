//! MAVLink v1/v2 codec for DrorControl.
//!
//! * [`dialect`]: messages and enums generated at build time from the vendored ArduPilot
//!   XML (`definitions/`, root `ardupilotmega.xml`; feature `dialect-all` uses `all.xml`).
//! * [`frame`]: framing, checksum validation, a resynchronising stream parser, and encoding.
//! * [`signing`]: MAVLink 2 packet signing and replay protection.
//!
//! Decoding is lazy: the parser validates frames using only the per-message metadata table
//! ([`message_info`]), and a payload is turned into a typed [`Message`] only when asked.
//!
//! ```
//! use mavlink_codec::{encode, messages::Heartbeat, FrameParser, Header, Message, Version};
//!
//! let hb = Message::Heartbeat(Heartbeat { custom_mode: 5, mavlink_version: 3, ..Default::default() });
//! let frame = encode(Version::V2, Header { seq: 7, sysid: 255, compid: 190 }, &hb).unwrap();
//!
//! let mut parser = FrameParser::new();
//! parser.push(b"noise");
//! parser.push(frame.as_bytes());
//! let rx = parser.next_frame().expect("one frame");
//! assert_eq!(rx.decode().unwrap(), hb);
//! ```

pub mod crc;
pub mod frame;
pub mod signing;
pub mod wire;

/// Generated dialect: [`dialect::messages`], [`dialect::enums`], [`dialect::Message`].
#[allow(
    missing_docs,
    non_upper_case_globals,
    non_snake_case,
    clippy::derivable_impls,
    clippy::field_reassign_with_default,
    clippy::useless_conversion,
    rustdoc::broken_intra_doc_links,
    rustdoc::invalid_html_tags,
    rustdoc::bare_urls,
    rustdoc::invalid_rust_codeblocks
)]
pub mod dialect {
    include!(concat!(env!("OUT_DIR"), "/dialect.rs"));
}

pub use dialect::{Message, enums, message_info, message_infos, messages};
pub use frame::{EncodeError, Frame, FrameParser, Header, ParserStats, Version, encode};

/// Behaviour shared by every generated message struct.
pub trait MessageData: Sized + Default + Clone {
    /// Message ID.
    const ID: u32;
    /// Upstream message name, e.g. `HEARTBEAT`.
    const NAME: &'static str;
    /// CRC_EXTRA seed derived from the message layout.
    const CRC_EXTRA: u8;
    /// Payload length of the MAVLink 1 (base) fields.
    const BASE_LEN: usize;
    /// Payload length including MAVLink 2 extension fields.
    const MAX_LEN: usize;

    /// Write the full payload in wire order; returns `MAX_LEN`.
    fn serialize(&self, out: &mut [u8; 255]) -> usize;
    /// Read a payload; missing trailing bytes are zero, surplus bytes are ignored.
    fn deserialize(payload: &[u8]) -> Self;
    /// Field names and values in XML declaration order.
    fn fields(&self) -> Vec<(&'static str, FieldValue)>;
}

/// Wire metadata for one message ID (what the framer needs without decoding).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MessageInfo {
    /// Message ID.
    pub id: u32,
    /// Upstream name.
    pub name: &'static str,
    /// CRC_EXTRA seed.
    pub crc_extra: u8,
    /// Base (MAVLink 1) payload length.
    pub base_len: usize,
    /// Full payload length including extensions.
    pub max_len: usize,
    /// Payload offset of a `target_system` field, if any (used for routing).
    pub target_system_ofs: Option<usize>,
    /// Payload offset of a `target_component` field, if any.
    pub target_component_ofs: Option<usize>,
}

/// A message ID that is not part of the compiled dialect.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnknownMessage(pub u32);

impl std::fmt::Display for UnknownMessage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "message id {} is not in the {} dialect",
            self.0,
            dialect::DIALECT_ROOT
        )
    }
}

impl std::error::Error for UnknownMessage {}

/// A field value in a generic form, for inspectors, logging and cross-implementation tests.
#[derive(Debug, Clone, PartialEq)]
pub enum FieldValue {
    /// Unsigned integer.
    UInt(u64),
    /// Signed integer.
    Int(i64),
    /// `float` or `double`.
    Float(f64),
    /// Unsigned integer array.
    UIntArray(Vec<u64>),
    /// Signed integer array.
    IntArray(Vec<i64>),
    /// Floating-point array.
    FloatArray(Vec<f64>),
    /// `char[N]` bytes, including any trailing NULs.
    Chars(Vec<u8>),
}
