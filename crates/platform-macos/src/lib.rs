//! macOS platform layer.
//!
//! [`SerialNotifier`]: event-driven serial hotplug via IOKit matching notifications
//! (`kIOFirstMatchNotification` / `kIOTerminatedNotification` on `IOSerialBSDClient`),
//! reporting callout device paths (`/dev/cu.*`, never `/dev/tty.*`, which blocks on
//! carrier detect). No kernel extensions, no polling.
//!
//! On other platforms this crate is empty; `link-serial` falls back to polling there.

#[cfg(target_os = "macos")]
mod imp;

#[cfg(target_os = "macos")]
pub use imp::{RawSerialEvent, SerialNotifier};
