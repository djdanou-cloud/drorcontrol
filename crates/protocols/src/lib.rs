//! Sans-I/O MAVLink protocol state machines.
//!
//! Each machine consumes messages and the current time and returns the messages to send,
//! so protocols are tested deterministically and reused by the daemon, CLI and CI pilots.

pub mod audit;
pub mod command;
pub mod cot;
pub mod fence;
pub mod kml;
pub mod log_transfer;
pub mod mission;
pub mod mission_edit;
pub mod params;
pub mod pattern;
pub mod plan;
pub mod survey;
pub mod terrain;
pub mod units;
pub mod wpl;
