//! MAVLink routing for DrorControl.
//!
//! * [`router`]: pure routing decisions (route learning, targeted vs broadcast forwarding,
//!   loop prevention, per-link policies).
//! * [`hub`]: the async task that connects links, the router, the application and the
//!   recorder.
//!
//! See `docs/plan/06-documentation-system.md` §5 for the routing flowchart.

pub mod hub;
pub mod router;

pub use hub::{HubHandle, HubOutput, HubOutputs, HubSnapshot, Tapped, spawn_hub};
pub use router::{Decision, LinkPolicy, Router, RouterConfig, RouterStats};
