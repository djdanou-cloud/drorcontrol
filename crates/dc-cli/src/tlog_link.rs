//! Link quality over time from a `.tlog`, for range tests: per time window, the vehicle's
//! frames received and lost (MAVLink sequence numbers of its autopilot, as the live core
//! counts them), the telemetry radio's signal (`RADIO_STATUS`, radio units, 255 = unknown)
//! and the longest gap between the vehicle's heartbeats.

use std::io::{self, Read};

use dc_core::SeqCount;
use mavlink_codec::Message;
use recorder::TlogReader;

/// One time window.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LinkWindow {
    /// Window start, µs since the Unix epoch.
    pub start_us: u64,
    /// Vehicle frames received and lost.
    pub received: u64,
    /// See `received`.
    pub lost: u64,
    /// Lowest local and remote signal reported by the radio in the window.
    pub rssi: Option<u8>,
    /// See `rssi`.
    pub remrssi: Option<u8>,
    /// Longest time without a vehicle heartbeat ending in this window, s.
    pub max_hb_gap_s: f64,
}

impl LinkWindow {
    /// Lost share of the vehicle's frames, 0..1.
    pub fn loss(&self) -> f64 {
        let all = self.received + self.lost;
        if all == 0 {
            0.0
        } else {
            self.lost as f64 / all as f64
        }
    }
}

/// The timeline of `sysid` (default: the first autopilot heard) in windows of `window_s`.
/// A bad record ends the read there.
pub fn timeline(
    input: impl Read,
    sysid: Option<u8>,
    window_s: f64,
) -> io::Result<(Option<u8>, Vec<LinkWindow>)> {
    let window_us = (window_s.max(0.1) * 1e6) as u64;
    let mut r = TlogReader::new(input);
    let mut vehicle = sysid;
    let mut seq = SeqCount::default();
    let mut out: Vec<LinkWindow> = Vec::new();
    let mut first = None::<u64>;
    let mut last_hb = None::<u64>;
    while let Ok(Some(rec)) = r.next_record() {
        let t = rec.unix_us;
        let t0 = *first.get_or_insert(t);
        let k = ((t - t0) / window_us) as usize;
        while out.len() <= k {
            let n = out.len() as u64;
            out.push(LinkWindow {
                start_us: t0 + n * window_us,
                ..Default::default()
            });
        }
        let w = &mut out[k];
        let f = &rec.frame;
        let msg = f.decode().ok();
        if vehicle.is_none()
            && f.compid() == 1
            && matches!(&msg, Some(Message::Heartbeat(h)) if h.autopilot != 8)
        {
            vehicle = Some(f.sysid());
        }
        if let Some(Message::RadioStatus(rs)) = &msg {
            let low = |cur: Option<u8>, v: u8| {
                (v != u8::MAX).then(|| cur.map_or(v, |c| c.min(v))).or(cur)
            };
            w.rssi = low(w.rssi, rs.rssi);
            w.remrssi = low(w.remrssi, rs.remrssi);
        }
        if Some(f.sysid()) != vehicle || f.compid() != 1 {
            continue;
        }
        if let Some((lost, late)) = seq.on_seq(f.seq()) {
            w.received += 1;
            w.lost += u64::from(lost);
            if late && w.lost > 0 {
                w.lost -= 1;
            }
        }
        if matches!(msg, Some(Message::Heartbeat(_))) {
            if let Some(p) = last_hb {
                let gap = (t - p) as f64 / 1e6;
                w.max_hb_gap_s = w.max_hb_gap_s.max(gap);
            }
            last_hb = Some(t);
        }
    }
    Ok((vehicle, out))
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::messages::{Heartbeat, RadioStatus};
    use mavlink_codec::{Header, Version, encode};
    use recorder::{TlogWriter, from_unix_micros};

    #[test]
    fn loss_signal_and_heartbeat_gaps_per_window() {
        let mut w = TlogWriter::new(Vec::new());
        let t0 = 1_700_000_000_000_000u64;
        let mut put = |t: u64, seq: u8, sysid: u8, m: Message| {
            let f = encode(
                Version::V2,
                Header {
                    seq,
                    sysid,
                    compid: if sysid == 51 { 68 } else { 1 },
                },
                &m,
            )
            .unwrap();
            w.write(from_unix_micros(t0 + t), &f).unwrap();
        };
        let hb = || {
            Message::Heartbeat(Heartbeat {
                autopilot: 3,
                ..Default::default()
            })
        };
        // Window 0 (0-10 s): heartbeats every second, seq 0..9, none lost.
        for i in 0..10u8 {
            put(u64::from(i) * 1_000_000, i, 1, hb());
        }
        put(
            5_000_000,
            0,
            51,
            Message::RadioStatus(RadioStatus {
                rssi: 180,
                remrssi: 170,
                ..Default::default()
            }),
        );
        // Window 1 (10-20 s): a 4 s gap and 5 frames lost (seq jumps 9 -> 15).
        put(13_000_000, 15, 1, hb());
        put(14_000_000, 16, 1, hb());
        put(
            15_000_000,
            1,
            51,
            Message::RadioStatus(RadioStatus {
                rssi: 90,
                remrssi: u8::MAX,
                ..Default::default()
            }),
        );
        let bytes = w.into_inner().unwrap();
        let (v, tl) = timeline(&bytes[..], None, 10.0).unwrap();
        assert_eq!(v, Some(1));
        assert_eq!(tl.len(), 2);
        assert_eq!((tl[0].received, tl[0].lost), (10, 0));
        assert_eq!((tl[0].rssi, tl[0].remrssi), (Some(180), Some(170)));
        assert_eq!((tl[1].received, tl[1].lost), (2, 5));
        assert_eq!((tl[1].rssi, tl[1].remrssi), (Some(90), None));
        assert!((tl[1].max_hb_gap_s - 4.0).abs() < 1e-9);
        assert!((tl[1].loss() - 5.0 / 7.0).abs() < 1e-9);
    }
}
