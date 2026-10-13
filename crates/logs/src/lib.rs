//! Log analytics for DrorControl (plan 05).
//!
//! * [`dataflash`]: ArduPilot `.bin` logs — FMT discovery, parallel index, lazy columns.
//! * [`downsample`]: LTTB and MinMaxLTTB for plotting millions of points.
//! * [`Log::params`]: parameter values recorded in the log.
//! * [`export`]: one message as CSV; the recorded parameters as a .param file.
//! * [`flight`]: armed periods and flight summary, event timeline, in-flight parameter changes.
//! * [`health`]: automatic checks (vibration, battery, EKF, errors) against sourced thresholds.
//! * [`spectrum`]: IMU vibration spectrum (Welch FFT) and its peaks.
//! * [`track`]: the flight track (EKF position) with UTC from the GPS clock.

pub mod altitude;
pub mod battery;
pub mod camera;
pub mod cells;
pub mod compass;
pub mod dataflash;
pub mod downsample;
pub mod esc;
pub mod export;
pub mod flight;
pub mod gps;
pub mod health;
pub mod imu;
pub mod landing;
pub mod lanes;
pub mod motors;
pub mod notch;
pub mod notch_design;
mod params;
pub mod perf;
pub mod rc;
pub mod sanitize;
pub mod sensors;
pub mod spectrogram;
pub mod spectrum;
pub mod takeoff;
#[cfg(test)]
mod testlog;
pub mod track;
pub mod trim;
pub mod tuning;
pub mod wind;

pub use dataflash::{Column, Format, Log};
