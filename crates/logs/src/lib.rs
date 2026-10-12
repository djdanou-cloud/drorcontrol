//! Log analytics for DrorControl (plan 05).
//!
//! * [`dataflash`]: ArduPilot `.bin` logs — FMT discovery, parallel index, lazy columns.
//! * [`downsample`]: LTTB and MinMaxLTTB for plotting millions of points.
//! * [`Log::params`]: parameter values recorded in the log.
//! * [`export`]: one message as CSV; the recorded parameters as a .param file.
//! * [`health`]: automatic checks (vibration, battery, EKF, errors) against sourced thresholds.
//! * [`track`]: the flight track (EKF position) with UTC from the GPS clock.

pub mod dataflash;
pub mod downsample;
pub mod export;
pub mod health;
mod params;
pub mod track;

pub use dataflash::{Column, Format, Log};
