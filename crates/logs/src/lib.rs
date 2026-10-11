//! Log analytics for DrorControl (plan 05).
//!
//! * [`dataflash`]: ArduPilot `.bin` logs — FMT discovery, parallel index, lazy columns.
//! * [`downsample`]: LTTB and MinMaxLTTB for plotting millions of points.

pub mod dataflash;
pub mod downsample;

pub use dataflash::{Column, Format, Log};
