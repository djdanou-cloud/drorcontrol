//! MAVLink XML dialect parser and Rust code generator.
//!
//! Used at build time by `mavlink-codec` (and later by the documentation generator).
//! The XML definitions are the single source of truth: message IDs, wire layouts and
//! CRC_EXTRA values are always derived from them, never typed by hand.

pub mod codegen;
pub mod crc;
pub mod model;
pub mod parse;

pub use codegen::generate;
pub use model::{Dialect, Entry, Enum, Field, FieldType, Message, Prim};
pub use parse::{GenError, load_dialect};
