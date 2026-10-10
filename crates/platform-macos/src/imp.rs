//! IOKit serial notifications (macOS only).

#![allow(unsafe_code)]

use std::ffi::{c_char, c_void};
use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Sender;
use std::thread::JoinHandle;
use std::time::Duration;

use core_foundation::base::{CFType, TCFType, kCFAllocatorDefault};
use core_foundation::string::CFString;
use core_foundation_sys::runloop::{
    CFRunLoopAddSource, CFRunLoopGetCurrent, CFRunLoopRunInMode, kCFRunLoopDefaultMode,
};
use io_kit_sys::keys::{kIOFirstMatchNotification, kIOTerminatedNotification};
use io_kit_sys::serial::keys::kIOSerialBSDServiceValue;
use io_kit_sys::types::io_iterator_t;
use io_kit_sys::{
    IOIteratorNext, IONotificationPortCreate, IONotificationPortDestroy,
    IONotificationPortGetRunLoopSource, IOObjectRelease, IORegistryEntryCreateCFProperty,
    IOServiceAddMatchingNotification, IOServiceMatching, kIOMasterPortDefault,
};

/// A serial device appeared or disappeared. Paths are callout devices (`/dev/cu.*`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RawSerialEvent {
    /// Device present. Devices already attached when the notifier starts are reported
    /// once as `Added`, so consumers get one consistent stream.
    Added(String),
    /// Device removed (unplugged, or the board rebooted into another USB personality).
    Removed(String),
}

struct Ctx {
    tx: Sender<RawSerialEvent>,
    added: bool,
}

/// Read the `IOCalloutDevice` property of a serial service.
unsafe fn callout_path(service: io_iterator_t) -> Option<String> {
    let key = CFString::from_static_string("IOCalloutDevice");
    // SAFETY: valid service object and CF string; we own the returned reference.
    let r = unsafe {
        IORegistryEntryCreateCFProperty(service, key.as_concrete_TypeRef(), kCFAllocatorDefault, 0)
    };
    if r.is_null() {
        return None;
    }
    // SAFETY: "create rule": we own `r`; wrapping transfers that ownership.
    let value = unsafe { CFType::wrap_under_create_rule(r) };
    value.downcast::<CFString>().map(|s| s.to_string())
}

/// Matching callback: drain the iterator (this also re-arms the notification).
unsafe extern "C" fn on_match(refcon: *mut c_void, iter: io_iterator_t) {
    // SAFETY: refcon is the `Ctx` boxed in `run`, alive until after the run loop exits.
    let ctx = unsafe { &*(refcon as *const Ctx) };
    loop {
        // SAFETY: valid iterator from IOServiceAddMatchingNotification.
        let service = unsafe { IOIteratorNext(iter) };
        if service == 0 {
            break;
        }
        // SAFETY: `service` is a live object we release right after use.
        if let Some(path) = unsafe { callout_path(service) } {
            let ev = if ctx.added {
                RawSerialEvent::Added(path)
            } else {
                RawSerialEvent::Removed(path)
            };
            let _ = ctx.tx.send(ev);
        }
        // SAFETY: we own the reference returned by IOIteratorNext.
        unsafe { IOObjectRelease(service) };
    }
}

/// Runs an IOKit notification port on a dedicated CFRunLoop thread until dropped.
pub struct SerialNotifier {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl SerialNotifier {
    /// Start watching; events go to `tx`. Fails if IOKit refuses the registration.
    pub fn start(tx: Sender<RawSerialEvent>) -> io::Result<SerialNotifier> {
        let stop = Arc::new(AtomicBool::new(false));
        let (ready_tx, ready_rx) = std::sync::mpsc::channel::<io::Result<()>>();
        let stop_flag = stop.clone();
        let thread = std::thread::Builder::new()
            .name("iokit-serial-hotplug".into())
            .spawn(move || run(tx, stop_flag, ready_tx))?;
        ready_rx
            .recv_timeout(Duration::from_secs(5))
            .map_err(|_| io::Error::other("IOKit notifier did not start"))??;
        Ok(SerialNotifier {
            stop,
            thread: Some(thread),
        })
    }
}

impl Drop for SerialNotifier {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn run(tx: Sender<RawSerialEvent>, stop: Arc<AtomicBool>, ready: Sender<io::Result<()>>) {
    // SAFETY: plain IOKit / CoreFoundation calls on this thread; every object created here
    // is released below after the run loop exits.
    unsafe {
        let port = IONotificationPortCreate(kIOMasterPortDefault);
        if port.is_null() {
            let _ = ready.send(Err(io::Error::other("IONotificationPortCreate failed")));
            return;
        }
        CFRunLoopAddSource(
            CFRunLoopGetCurrent(),
            IONotificationPortGetRunLoopSource(port),
            kCFRunLoopDefaultMode,
        );

        let contexts = [
            Box::new(Ctx {
                tx: tx.clone(),
                added: true,
            }),
            Box::new(Ctx { tx, added: false }),
        ];
        let kinds = [kIOFirstMatchNotification, kIOTerminatedNotification];
        let mut iters: Vec<io_iterator_t> = Vec::new();
        for (ctx, kind) in contexts.iter().zip(kinds) {
            // IOServiceAddMatchingNotification consumes one reference to `matching`.
            let matching = IOServiceMatching(kIOSerialBSDServiceValue);
            let mut iter: io_iterator_t = 0;
            let refcon = &**ctx as *const Ctx as *mut c_void;
            let kr = IOServiceAddMatchingNotification(
                port,
                kind as *mut c_char,
                matching as _,
                on_match,
                refcon,
                &mut iter,
            );
            if kr != 0 {
                for it in iters {
                    IOObjectRelease(it);
                }
                IONotificationPortDestroy(port);
                let _ = ready.send(Err(io::Error::other(format!(
                    "IOServiceAddMatchingNotification: {kr:#x}"
                ))));
                return;
            }
            // Drain once: arms the notification and reports devices already present.
            on_match(refcon, iter);
            iters.push(iter);
        }
        let _ = ready.send(Ok(()));

        // Short slices so Drop is honoured even if it races with start-up.
        while !stop.load(Ordering::Relaxed) {
            CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.25, 0);
        }

        for it in iters {
            IOObjectRelease(it);
        }
        IONotificationPortDestroy(port);
        drop(contexts);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn starts_reports_existing_devices_and_stops() {
        let (tx, rx) = std::sync::mpsc::channel();
        let n = SerialNotifier::start(tx).expect("IOKit notifier");
        let initial: Vec<RawSerialEvent> = rx.try_iter().collect();
        // Runners usually have none; developer Macs often have /dev/cu.Bluetooth-Incoming-Port.
        eprintln!("initial serial devices: {initial:?}");
        assert!(
            initial
                .iter()
                .all(|e| matches!(e, RawSerialEvent::Added(p) if p.starts_with("/dev/cu.")))
        );
        let t = Instant::now();
        drop(n);
        assert!(
            t.elapsed() < Duration::from_secs(1),
            "stops within one run-loop slice"
        );
    }
}
