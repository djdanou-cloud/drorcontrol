//! Sans-I/O MAVLink protocol state machines.
//!
//! Each machine consumes messages and the current time and returns the messages to send,
//! so protocols are tested deterministically and reused by the daemon, CLI and CI pilots.

pub mod audit;
pub mod command;
pub mod params;
