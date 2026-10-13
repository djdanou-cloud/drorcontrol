//! Telemetry logs (`.tlog`).
//!
//! Format (compatible with Mission Planner, MAVProxy and pymavlink): a sequence of records,
//! each an 8-byte **big-endian** count of microseconds since the Unix epoch followed by one
//! raw MAVLink frame exactly as it was on the wire (v1 or v2, signed or not).
//!
//! Recording must be lossless and byte-exact; [`TlogReader`] returns frames whose bytes are
//! identical to what [`TlogWriter`] was given, so replay is indistinguishable from the link.

pub mod index;

pub use index::{KeyEvent, KeyEventKind, TlogIndex};

use std::io::{self, BufWriter, Read, Write};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use mavlink_codec::frame::{IFLAG_SIGNED, MAX_FRAME_LEN, SIGNATURE_LEN, STX_V1, STX_V2};
use mavlink_codec::{Frame, FrameParser};

/// Microseconds since the Unix epoch (0 for times before it).
pub fn unix_micros(t: SystemTime) -> u64 {
    t.duration_since(UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

/// Inverse of [`unix_micros`].
pub fn from_unix_micros(us: u64) -> SystemTime {
    UNIX_EPOCH + Duration::from_micros(us)
}

/// Writes `.tlog` records. Buffered; call [`TlogWriter::flush`] periodically (the
/// recorder flushes at least once per second) and before closing.
pub struct TlogWriter<W: Write> {
    out: BufWriter<W>,
    records: u64,
}

impl<W: Write> TlogWriter<W> {
    /// Wrap a writer (usually a `File`).
    pub fn new(inner: W) -> Self {
        TlogWriter {
            out: BufWriter::with_capacity(64 * 1024, inner),
            records: 0,
        }
    }

    /// Append one frame with its timestamp.
    pub fn write(&mut self, time: SystemTime, frame: &Frame) -> io::Result<()> {
        self.write_raw(unix_micros(time), frame.as_bytes())
    }

    /// Append raw bytes with a microsecond timestamp.
    pub fn write_raw(&mut self, unix_us: u64, frame_bytes: &[u8]) -> io::Result<()> {
        self.out.write_all(&unix_us.to_be_bytes())?;
        self.out.write_all(frame_bytes)?;
        self.records += 1;
        Ok(())
    }

    /// Records written.
    pub fn records(&self) -> u64 {
        self.records
    }

    /// Push buffered data to the underlying writer.
    pub fn flush(&mut self) -> io::Result<()> {
        self.out.flush()
    }

    /// Flush and return the underlying writer.
    pub fn into_inner(self) -> io::Result<W> {
        self.out.into_inner().map_err(|e| e.into_error())
    }
}

/// One `.tlog` record.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TlogRecord {
    /// Microseconds since the Unix epoch.
    pub unix_us: u64,
    /// The frame, byte-identical to the recording.
    pub frame: Frame,
}

/// Reads `.tlog` records in order.
pub struct TlogReader<R: Read> {
    input: R,
    offset: u64,
}

impl<R: Read> TlogReader<R> {
    /// Wrap a reader (usually a `BufReader<File>`).
    pub fn new(input: R) -> Self {
        TlogReader { input, offset: 0 }
    }

    /// Byte offset of the next record.
    pub fn offset(&self) -> u64 {
        self.offset
    }

    fn bad(&self, what: &str) -> io::Error {
        io::Error::new(
            io::ErrorKind::InvalidData,
            format!("tlog record at byte {}: {what}", self.offset),
        )
    }

    /// Next record; `Ok(None)` at a clean end of file. A truncated final record is an
    /// `UnexpectedEof` error; a corrupt record is an `InvalidData` error naming its offset.
    pub fn next_record(&mut self) -> io::Result<Option<TlogRecord>> {
        let mut ts = [0u8; 8];
        match self.input.read(&mut ts[..1])? {
            0 => return Ok(None),
            _ => self.input.read_exact(&mut ts[1..])?,
        }
        let mut buf = [0u8; MAX_FRAME_LEN];
        self.input.read_exact(&mut buf[..1])?;
        let header_len = match buf[0] {
            STX_V1 => 6,
            STX_V2 => 10,
            _ => return Err(self.bad("missing start byte")),
        };
        self.input.read_exact(&mut buf[1..header_len])?;
        let signed = buf[0] == STX_V2 && buf[2] & IFLAG_SIGNED != 0;
        let total = header_len + usize::from(buf[1]) + 2 + if signed { SIGNATURE_LEN } else { 0 };
        self.input.read_exact(&mut buf[header_len..total])?;

        // Unknown message ids are kept (their checksum cannot be verified, but the bytes
        // are what was received and replay must reproduce them).
        let mut p = FrameParser::with_unknown_passthrough();
        p.push(&buf[..total]);
        let frame = match p.next_frame() {
            Some(f) if f.as_bytes().len() == total => f,
            _ => return Err(self.bad("invalid frame (checksum or flags)")),
        };
        self.offset += 8 + total as u64;
        Ok(Some(TlogRecord {
            unix_us: u64::from_be_bytes(ts),
            frame,
        }))
    }
}

impl<R: Read> Iterator for TlogReader<R> {
    type Item = io::Result<TlogRecord>;

    fn next(&mut self) -> Option<Self::Item> {
        self.next_record().transpose()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::messages::{Heartbeat, Statustext};
    use mavlink_codec::signing::{SecretKey, sign};
    use mavlink_codec::{Header, Message, Version, encode};

    fn frames() -> Vec<Frame> {
        let hb = Message::Heartbeat(Heartbeat {
            custom_mode: 4,
            ..Default::default()
        });
        let mut text = Statustext {
            severity: 4,
            ..Default::default()
        };
        text.text[..7].copy_from_slice(b"PreArm:");
        let mut signed = encode(
            Version::V2,
            Header {
                seq: 3,
                sysid: 1,
                compid: 1,
            },
            &hb,
        )
        .unwrap();
        sign(&mut signed, &SecretKey([1; 32]), 0, 123_456).unwrap();
        let mut unknown = encode(
            Version::V2,
            Header {
                seq: 4,
                sysid: 1,
                compid: 1,
            },
            &hb,
        )
        .unwrap()
        .as_bytes()
        .to_vec();
        unknown[7..10].copy_from_slice(&[0xFE, 0xFF, 0xFF]);
        let mut p = FrameParser::with_unknown_passthrough();
        p.push(&unknown);
        vec![
            encode(
                Version::V1,
                Header {
                    seq: 1,
                    sysid: 1,
                    compid: 1,
                },
                &hb,
            )
            .unwrap(),
            encode(
                Version::V2,
                Header {
                    seq: 2,
                    sysid: 1,
                    compid: 1,
                },
                &Message::Statustext(text),
            )
            .unwrap(),
            signed,
            p.next_frame().unwrap(),
        ]
    }

    #[test]
    fn round_trip_is_byte_exact() {
        let fs = frames();
        let t0 = 1_760_000_000_000_000u64;
        let mut w = TlogWriter::new(Vec::new());
        for (i, f) in fs.iter().enumerate() {
            w.write_raw(t0 + i as u64 * 20_000, f.as_bytes()).unwrap();
        }
        assert_eq!(w.records(), 4);
        let bytes = w.into_inner().unwrap();
        assert_eq!(&bytes[..8], &t0.to_be_bytes(), "big-endian timestamp first");

        let recs: Vec<TlogRecord> = TlogReader::new(bytes.as_slice())
            .collect::<io::Result<_>>()
            .unwrap();
        assert_eq!(recs.iter().map(|r| r.frame.clone()).collect::<Vec<_>>(), fs);
        assert_eq!(recs[3].unix_us, t0 + 60_000);

        // Re-recording the replay reproduces the file exactly.
        let mut again = TlogWriter::new(Vec::new());
        for r in &recs {
            again.write_raw(r.unix_us, r.frame.as_bytes()).unwrap();
        }
        assert_eq!(again.into_inner().unwrap(), bytes);
    }

    #[test]
    fn reports_truncation_and_corruption() {
        let mut w = TlogWriter::new(Vec::new());
        for f in frames() {
            w.write(SystemTime::now(), &f).unwrap();
        }
        let bytes = w.into_inner().unwrap();

        let truncated = &bytes[..bytes.len() - 3];
        let r: Vec<io::Result<TlogRecord>> = TlogReader::new(truncated).collect();
        assert_eq!(r.len(), 4);
        assert_eq!(
            r[3].as_ref().unwrap_err().kind(),
            io::ErrorKind::UnexpectedEof
        );

        let mut corrupt = bytes.clone();
        corrupt[8 + 6] ^= 0xFF; // first frame's payload: checksum now fails
        let err = TlogReader::new(corrupt.as_slice())
            .next_record()
            .unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::InvalidData);
        assert!(err.to_string().contains("byte 0"));
    }

    #[test]
    fn time_conversion_round_trips() {
        let t = from_unix_micros(1_760_000_000_123_456);
        assert_eq!(unix_micros(t), 1_760_000_000_123_456);
    }
}
