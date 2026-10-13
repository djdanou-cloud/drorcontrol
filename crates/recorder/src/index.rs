//! An index over a `.tlog` for playback (plan 05 §3): time → byte-offset checkpoints every
//! 100 ms so a seek costs a binary search, which messages the recording holds, the key events
//! to jump to (arming, mode changes, STATUSTEXT, mission item reached), and keyframes so a
//! seek can restore the last value of every message up to that time ("stateful seek": the
//! HUD is right immediately after a jump).

use std::collections::BTreeMap;
use std::io::{self, Read, Seek, SeekFrom};

use mavlink_codec::Message;

use crate::{TlogReader, TlogRecord};

/// Spacing of the time checkpoints, µs.
pub const CHECKPOINT_US: u64 = 100_000;
/// Spacing of the keyframes, µs.
pub const KEYFRAME_US: u64 = 10_000_000;

const ARMED: u8 = 128;

/// A moment worth jumping to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KeyEvent {
    /// Record time, µs since the Unix epoch.
    pub unix_us: u64,
    /// Sender system id.
    pub sysid: u8,
    /// What happened.
    pub kind: KeyEventKind,
}

/// Kinds of [`KeyEvent`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum KeyEventKind {
    /// The vehicle armed (true) or disarmed.
    Armed(bool),
    /// The autopilot's custom mode changed to this number.
    Mode(u32),
    /// A STATUSTEXT: severity and text.
    Text(u8, String),
    /// MISSION_ITEM_REACHED with this sequence number.
    ItemReached(u16),
}

/// Identity of a message stream: sender and message id.
pub type StreamKey = (u8, u8, u32);

/// The index of one recording.
#[derive(Debug, Clone, Default)]
pub struct TlogIndex {
    /// `(time, byte offset)` of the first record at or after each 100 ms step, ascending.
    pub checkpoints: Vec<(u64, u64)>,
    /// Records per message id.
    pub counts: BTreeMap<u32, u64>,
    /// Key events in order.
    pub events: Vec<KeyEvent>,
    /// First and last record time.
    pub span: Option<(u64, u64)>,
    /// Records indexed.
    pub records: u64,
    /// `(time, offset of the next record, offset of the latest record of each stream)`.
    keyframes: Vec<(u64, u64, BTreeMap<StreamKey, u64>)>,
}

impl TlogIndex {
    /// Read a whole recording. A bad record ends the index there; what came before is kept
    /// and the error is returned alongside.
    pub fn build(input: impl Read) -> (TlogIndex, Option<io::Error>) {
        let mut r = TlogReader::new(input);
        let mut ix = TlogIndex::default();
        let mut latest: BTreeMap<StreamKey, u64> = BTreeMap::new();
        let mut armed: BTreeMap<u8, bool> = BTreeMap::new();
        let mut mode: BTreeMap<u8, u32> = BTreeMap::new();
        let mut next_checkpoint = 0;
        let mut next_keyframe = 0;
        loop {
            let at = r.offset();
            let rec = match r.next_record() {
                Ok(Some(rec)) => rec,
                Ok(None) => return (ix, None),
                Err(e) => return (ix, Some(e)),
            };
            let t = rec.unix_us;
            if t >= next_keyframe && ix.records > 0 {
                ix.keyframes.push((t, at, latest.clone()));
                next_keyframe = t - t % KEYFRAME_US + KEYFRAME_US;
            } else if ix.records == 0 {
                next_keyframe = t - t % KEYFRAME_US + KEYFRAME_US;
            }
            if t >= next_checkpoint {
                ix.checkpoints.push((t, at));
                next_checkpoint = t - t % CHECKPOINT_US + CHECKPOINT_US;
            }
            ix.records += 1;
            ix.span = Some((ix.span.map_or(t, |s| s.0), t));
            let f = &rec.frame;
            *ix.counts.entry(f.msgid()).or_default() += 1;
            latest.insert((f.sysid(), f.compid(), f.msgid()), at);
            let sysid = f.sysid();
            let ev = |kind| KeyEvent {
                unix_us: t,
                sysid,
                kind,
            };
            match f.decode() {
                Ok(Message::Heartbeat(h)) if f.compid() == 1 && h.autopilot != 8 => {
                    let a = h.base_mode & ARMED != 0;
                    if armed.insert(sysid, a) != Some(a) {
                        ix.events.push(ev(KeyEventKind::Armed(a)));
                    }
                    if mode.insert(sysid, h.custom_mode) != Some(h.custom_mode) {
                        ix.events.push(ev(KeyEventKind::Mode(h.custom_mode)));
                    }
                }
                Ok(Message::Statustext(s)) => {
                    let end = s.text.iter().position(|c| *c == 0).unwrap_or(s.text.len());
                    let text = String::from_utf8_lossy(&s.text[..end]).into_owned();
                    ix.events.push(ev(KeyEventKind::Text(s.severity, text)));
                }
                Ok(Message::MissionItemReached(m)) => {
                    ix.events.push(ev(KeyEventKind::ItemReached(m.seq)));
                }
                _ => {}
            }
        }
    }

    /// Byte offset to start reading from to see every record at or after `unix_us` (the
    /// checkpoint at or before it; the start for earlier times, the end-most checkpoint
    /// for later ones).
    pub fn offset_at(&self, unix_us: u64) -> u64 {
        let i = self.checkpoints.partition_point(|(t, _)| *t <= unix_us);
        if i == 0 { 0 } else { self.checkpoints[i - 1].1 }
    }

    /// The latest record of every stream at or before `unix_us`, oldest first: the state a
    /// replay shows right after seeking there. Reads from the nearest keyframe.
    pub fn snapshot<R: Read + Seek>(
        &self,
        file: &mut R,
        unix_us: u64,
    ) -> io::Result<Vec<TlogRecord>> {
        let k = self.keyframes.partition_point(|(t, _, _)| *t <= unix_us);
        let (start, mut latest) = match k {
            0 => (0, BTreeMap::new()),
            _ => {
                let (_, next, map) = &self.keyframes[k - 1];
                (*next, map.clone())
            }
        };
        file.seek(SeekFrom::Start(start))?;
        let mut r = TlogReader::new(&mut *file);
        let mut at = start;
        while let Some(rec) = r.next_record()? {
            if rec.unix_us > unix_us {
                break;
            }
            let f = &rec.frame;
            latest.insert((f.sysid(), f.compid(), f.msgid()), at);
            at = start + r.offset();
        }
        let mut offsets: Vec<u64> = latest.into_values().collect();
        offsets.sort_unstable();
        let mut out = Vec::with_capacity(offsets.len());
        for o in offsets {
            file.seek(SeekFrom::Start(o))?;
            if let Some(rec) = TlogReader::new(&mut *file).next_record()? {
                out.push(rec);
            }
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{TlogWriter, from_unix_micros};
    use mavlink_codec::messages::{Attitude, Heartbeat, MissionItemReached, Statustext};
    use mavlink_codec::{Header, Version, encode};
    use std::io::Cursor;

    /// 30 s at 10 Hz: ATTITUDE every record with roll = record number, a heartbeat every
    /// second (armed from 5 s, mode 3 from 12 s), a text at 20 s, item 2 reached at 25 s.
    fn recording() -> Vec<u8> {
        let mut w = TlogWriter::new(Vec::new());
        let t0 = 1_700_000_000_000_000u64;
        for i in 0..300u64 {
            let t = t0 + i * 100_000;
            let mut put = |m: Message| {
                let f = encode(
                    Version::V2,
                    Header {
                        seq: i as u8,
                        sysid: 1,
                        compid: 1,
                    },
                    &m,
                )
                .unwrap();
                w.write(from_unix_micros(t), &f).unwrap();
            };
            put(Message::Attitude(Attitude {
                roll: i as f32,
                ..Default::default()
            }));
            if i % 10 == 0 {
                put(Message::Heartbeat(Heartbeat {
                    autopilot: 3,
                    base_mode: if i >= 50 { ARMED } else { 0 },
                    custom_mode: if i >= 120 { 3 } else { 0 },
                    ..Default::default()
                }));
            }
            if i == 200 {
                let mut s = Statustext::default();
                s.text[..5].copy_from_slice(b"hello");
                put(Message::Statustext(s));
            }
            if i == 250 {
                put(Message::MissionItemReached(MissionItemReached { seq: 2 }));
            }
        }
        w.into_inner().unwrap()
    }

    #[test]
    fn checkpoints_events_and_stateful_seek() {
        let bytes = recording();
        let (ix, err) = TlogIndex::build(&bytes[..]);
        assert!(err.is_none());
        let t0 = 1_700_000_000_000_000u64;
        assert_eq!(ix.records, 300 + 30 + 2);
        assert_eq!(ix.checkpoints.len(), 300);
        assert_eq!(ix.span, Some((t0, t0 + 299 * 100_000)));
        assert_eq!(ix.counts[&30], 300, "ATTITUDE");
        let kinds: Vec<_> = ix.events.iter().map(|e| e.kind.clone()).collect();
        assert_eq!(
            kinds,
            [
                KeyEventKind::Armed(false),
                KeyEventKind::Mode(0),
                KeyEventKind::Armed(true),
                KeyEventKind::Mode(3),
                KeyEventKind::Text(0, "hello".into()),
                KeyEventKind::ItemReached(2),
            ]
        );
        assert_eq!(ix.events[2].unix_us, t0 + 5_000_000);

        // Seeking: reading from offset_at(t) starts exactly at the record at t.
        let t = t0 + 17_350_000;
        let mut r = TlogReader::new(&bytes[ix.offset_at(t) as usize..]);
        let first = r.next_record().unwrap().unwrap();
        assert_eq!(first.unix_us, t0 + 17_300_000);
        assert_eq!(ix.offset_at(0), 0);

        // Stateful seek to 17.35 s: the latest attitude (roll 173) and heartbeat (armed, mode
        // 3), plus the statustext? No: it came at 20 s.
        let snap = ix.snapshot(&mut Cursor::new(&bytes), t).unwrap();
        let msgs: Vec<Message> = snap.iter().map(|r| r.frame.decode().unwrap()).collect();
        assert_eq!(msgs.len(), 2, "{msgs:?}");
        assert!(
            msgs.iter()
                .any(|m| matches!(m, Message::Attitude(a) if a.roll == 173.0))
        );
        assert!(msgs.iter().any(|m| matches!(m,
            Message::Heartbeat(h) if h.base_mode == ARMED && h.custom_mode == 3)));
        // Later than the end: everything, including the text and the item reached.
        let all = ix.snapshot(&mut Cursor::new(&bytes), u64::MAX).unwrap();
        assert_eq!(all.len(), 4);
        // Before the start: nothing.
        assert!(
            ix.snapshot(&mut Cursor::new(&bytes), t0 - 1)
                .unwrap()
                .is_empty()
        );
    }
}
