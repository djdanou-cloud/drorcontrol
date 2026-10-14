//! MAVLink inspector: every message stream per sender (system, component, message), with
//! count and rate, and the latest field values of a chosen message, live or from a .tlog.

use std::collections::BTreeMap;

use mavlink_codec::Message;

/// One sender's stream of one message.
#[derive(Debug, Clone)]
struct Stream {
    count: u64,
    first_us: u64,
    last_us: u64,
    last: Message,
}

/// Streams seen so far.
#[derive(Debug, Default)]
pub struct Inspector {
    streams: BTreeMap<(u8, u8, u32), Stream>,
}

impl Inspector {
    /// One message from `sysid`/`compid` at `t_us` (any monotonic microsecond clock).
    pub fn add(&mut self, t_us: u64, sysid: u8, compid: u8, msg: &Message) {
        self.streams
            .entry((sysid, compid, msg.id()))
            .and_modify(|s| {
                s.count += 1;
                s.last_us = t_us;
                s.last = msg.clone();
            })
            .or_insert(Stream {
                count: 1,
                first_us: t_us,
                last_us: t_us,
                last: msg.clone(),
            });
    }

    /// One line per stream: sender, message, count and rate (Hz, from the first to the last).
    pub fn table(&self) -> String {
        let mut out = format!(
            "{:<9} {:<28} {:>8} {:>8}\n",
            "sender", "message", "count", "rate Hz"
        );
        for ((sysid, compid, _), s) in &self.streams {
            let rate = if s.count > 1 && s.last_us > s.first_us {
                format!(
                    "{:.2}",
                    (s.count - 1) as f64 * 1e6 / (s.last_us - s.first_us) as f64
                )
            } else {
                "-".into()
            };
            out.push_str(&format!(
                "{:<9} {:<28} {:>8} {:>8}\n",
                format!("{sysid}/{compid}"),
                s.last.name(),
                s.count,
                rate
            ));
        }
        out
    }

    /// The latest fields of message `name` (any case) from each sender that sent it.
    pub fn fields(&self, name: &str) -> Option<String> {
        let mut out = String::new();
        for ((sysid, compid, _), s) in &self.streams {
            if !s.last.name().eq_ignore_ascii_case(name) {
                continue;
            }
            out.push_str(&format!("{} from {sysid}/{compid}:\n", s.last.name()));
            for (field, value) in s.last.fields() {
                out.push_str(&format!(
                    "  {field:<24} {}\n",
                    crate::tlog::csv_cell(&value)
                ));
            }
        }
        (!out.is_empty()).then_some(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::messages::{Attitude, Heartbeat};

    #[test]
    fn rates_per_sender_and_latest_fields() {
        let mut i = Inspector::default();
        for k in 0..11u64 {
            i.add(
                k * 100_000,
                1,
                1,
                &Message::Attitude(Attitude {
                    roll: k as f32,
                    ..Default::default()
                }),
            );
        }
        i.add(0, 1, 1, &Message::Heartbeat(Heartbeat::default()));
        let t = i.table();
        assert!(t.contains("1/1       ATTITUDE"), "{t}");
        assert!(
            t.lines()
                .any(|l| l.starts_with("1/1       ATTITUDE") && l.ends_with("10.00")),
            "{t}"
        );
        assert!(
            t.lines()
                .any(|l| l.starts_with("1/1       HEARTBEAT") && l.trim_end().ends_with('-')),
            "{t}"
        );
        let f = i.fields("attitude").unwrap();
        assert!(f.starts_with("ATTITUDE from 1/1:"), "{f}");
        assert!(f.contains("  roll                     10\n"), "{f}");
        assert!(i.fields("GPS_RAW_INT").is_none());
    }
}
