//! Log analytics for DrorControl (plan 05).
//!
//! * [`dataflash`]: ArduPilot `.bin` logs — FMT discovery, parallel index, lazy columns.
//! * [`downsample`]: LTTB and MinMaxLTTB for plotting millions of points.
//! * [`Log::params`]: parameter values recorded in the log.

pub mod dataflash;
pub mod downsample;
mod params;

pub use dataflash::{Column, Format, Log};
