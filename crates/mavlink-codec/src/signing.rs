//! MAVLink 2 packet signing.
//!
//! Signature block (13 bytes, appended after the checksum):
//! `link_id (1) | timestamp (6, LE, units of 10 µs since 2015-01-01 00:00:00 UTC) | signature (6)`.
//!
//! `signature` = first 48 bits of `SHA-256(secret_key || frame[STX..=crc] || link_id || timestamp)`.
//! This is a keyed hash with the key as a prefix, *not* HMAC — it must match the
//! reference implementations byte for byte (verified against pymavlink vectors).

use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

use sha2::{Digest, Sha256};

use crate::frame::{Frame, SIGNATURE_LEN, Version};
use crate::message_info;

/// Seconds from the Unix epoch to the MAVLink signing epoch (2015-01-01T00:00:00Z).
pub const MAVLINK_EPOCH_UNIX_SECS: u64 = 1_420_070_400;
/// One minute in signing-timestamp units (10 µs).
pub const ONE_MINUTE: u64 = 6_000_000;
const TIMESTAMP_MASK: u64 = (1 << 48) - 1;

/// A 32-byte shared signing key. `Debug` never prints the key.
#[derive(Clone, PartialEq, Eq)]
pub struct SecretKey(pub [u8; 32]);

impl std::fmt::Debug for SecretKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("SecretKey(..)")
    }
}

/// Current time as a signing timestamp.
pub fn timestamp_now() -> u64 {
    let since_unix = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let micros = since_unix
        .as_micros()
        .saturating_sub(u128::from(MAVLINK_EPOCH_UNIX_SECS) * 1_000_000);
    (micros / 10) as u64 & TIMESTAMP_MASK
}

/// Why a frame could not be signed or failed verification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignatureError {
    /// Signing requires MAVLink 2.
    NotV2,
    /// The frame is already signed.
    AlreadySigned,
    /// The message ID is not in the dialect (CRC cannot be recomputed).
    UnknownMessage,
    /// The frame carries no signature.
    Unsigned,
    /// Signature does not match the key.
    BadSignature,
    /// Timestamp not newer than the last one seen on this stream.
    Replay,
    /// New stream whose timestamp is more than one minute older than the newest seen.
    TooOld,
}

fn compute(key: &SecretKey, unsigned_part: &[u8], link_id: u8, timestamp: u64) -> [u8; 6] {
    let ts = (timestamp & TIMESTAMP_MASK).to_le_bytes();
    let mut h = Sha256::new();
    h.update(key.0);
    h.update(unsigned_part);
    h.update([link_id]);
    h.update(&ts[..6]);
    let digest = h.finalize();
    let mut sig = [0u8; 6];
    sig.copy_from_slice(&digest[..6]);
    sig
}

/// Sign an unsigned MAVLink 2 frame in place: sets the signed flag, recomputes the
/// checksum, and appends the signature block.
pub fn sign(
    frame: &mut Frame,
    key: &SecretKey,
    link_id: u8,
    timestamp: u64,
) -> Result<(), SignatureError> {
    if frame.version() != Version::V2 {
        return Err(SignatureError::NotV2);
    }
    if frame.is_signed() {
        return Err(SignatureError::AlreadySigned);
    }
    let info = message_info(frame.msgid()).ok_or(SignatureError::UnknownMessage)?;
    frame.mark_signed(info.crc_extra);
    let sig = compute(key, frame.unsigned_part(), link_id, timestamp);
    let mut block = [0u8; SIGNATURE_LEN];
    block[0] = link_id;
    block[1..7].copy_from_slice(&(timestamp & TIMESTAMP_MASK).to_le_bytes()[..6]);
    block[7..].copy_from_slice(&sig);
    frame.set_signature(&block);
    Ok(())
}

/// `(link_id, timestamp)` of a signed frame.
pub fn signature_fields(frame: &Frame) -> Option<(u8, u64)> {
    let b = frame.signature_block()?;
    let mut ts = [0u8; 8];
    ts[..6].copy_from_slice(&b[1..7]);
    Some((b[0], u64::from_le_bytes(ts)))
}

/// Check the signature only (no replay protection).
pub fn verify(frame: &Frame, key: &SecretKey) -> Result<(), SignatureError> {
    let block = frame.signature_block().ok_or(SignatureError::Unsigned)?;
    let (link_id, ts) = signature_fields(frame).ok_or(SignatureError::Unsigned)?;
    if compute(key, frame.unsigned_part(), link_id, ts) == block[7..] {
        Ok(())
    } else {
        Err(SignatureError::BadSignature)
    }
}

/// Signature verification with replay protection, per stream `(sysid, compid, link_id)`.
///
/// A known stream must present strictly increasing timestamps. A new stream is accepted
/// unless its timestamp is more than one minute older than the newest timestamp seen.
#[derive(Debug, Default)]
pub struct ReplayGuard {
    streams: HashMap<(u8, u8, u8), u64>,
    newest: u64,
}

impl ReplayGuard {
    /// Empty guard.
    pub fn new() -> Self {
        Self::default()
    }

    /// Seed the newest-seen timestamp from the local clock (recommended at start-up).
    pub fn with_local_time(timestamp: u64) -> Self {
        ReplayGuard {
            newest: timestamp,
            ..Self::default()
        }
    }

    /// Verify `frame` with `key` and record its timestamp if accepted.
    pub fn accept(&mut self, frame: &Frame, key: &SecretKey) -> Result<(), SignatureError> {
        verify(frame, key)?;
        let (link_id, ts) = signature_fields(frame).ok_or(SignatureError::Unsigned)?;
        let stream = (frame.sysid(), frame.compid(), link_id);
        match self.streams.get(&stream) {
            Some(&last) if ts <= last => return Err(SignatureError::Replay),
            None if ts + ONE_MINUTE < self.newest => return Err(SignatureError::TooOld),
            _ => {}
        }
        self.streams.insert(stream, ts);
        self.newest = self.newest.max(ts);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Header, Message, encode, messages::Heartbeat};

    fn frame(seq: u8) -> Frame {
        let m = Message::Heartbeat(Heartbeat {
            custom_mode: 1,
            ..Default::default()
        });
        encode(
            Version::V2,
            Header {
                seq,
                sysid: 1,
                compid: 1,
            },
            &m,
        )
        .unwrap()
    }

    const KEY: SecretKey = SecretKey([7; 32]);

    #[test]
    fn sign_then_verify_and_parse() {
        let mut f = frame(0);
        sign(&mut f, &KEY, 2, 1_000).unwrap();
        assert!(f.is_signed());
        assert_eq!(signature_fields(&f), Some((2, 1_000)));
        assert_eq!(verify(&f, &KEY), Ok(()));
        assert_eq!(
            verify(&f, &SecretKey([8; 32])),
            Err(SignatureError::BadSignature)
        );
        assert_eq!(
            crate::Frame::from_bytes(f.as_bytes()).unwrap(),
            f,
            "parser accepts signed frames"
        );
        assert_eq!(
            sign(&mut f, &KEY, 2, 1_001),
            Err(SignatureError::AlreadySigned)
        );
    }

    #[test]
    fn tampering_breaks_signature() {
        let mut f = frame(0);
        sign(&mut f, &KEY, 0, 5).unwrap();
        let mut bytes = f.as_bytes().to_vec();
        let last = bytes.len() - 1;
        bytes[last] ^= 1;
        let tampered = crate::Frame::from_bytes(&bytes).unwrap();
        assert_eq!(verify(&tampered, &KEY), Err(SignatureError::BadSignature));
    }

    #[test]
    fn replay_guard() {
        let mut g = ReplayGuard::with_local_time(10 * ONE_MINUTE);
        let signed = |seq, ts| {
            let mut f = frame(seq);
            sign(&mut f, &KEY, 0, ts).unwrap();
            f
        };
        assert_eq!(g.accept(&signed(0, 10 * ONE_MINUTE), &KEY), Ok(()));
        assert_eq!(
            g.accept(&signed(1, 10 * ONE_MINUTE), &KEY),
            Err(SignatureError::Replay)
        );
        assert_eq!(g.accept(&signed(2, 10 * ONE_MINUTE + 1), &KEY), Ok(()));

        // A new stream (different sysid) more than a minute behind the newest timestamp.
        let m = Message::Heartbeat(Heartbeat::default());
        let mut other = encode(
            Version::V2,
            Header {
                seq: 0,
                sysid: 2,
                compid: 1,
            },
            &m,
        )
        .unwrap();
        sign(&mut other, &KEY, 0, 8 * ONE_MINUTE).unwrap();
        assert_eq!(g.accept(&other, &KEY), Err(SignatureError::TooOld));
    }

    #[test]
    fn timestamp_now_is_after_2026() {
        // 2026-01-01 is 11 years (~3.47e8 s) after the epoch: ~3.47e13 units of 10 µs.
        assert!(timestamp_now() > 34_700_000_000_000);
    }
}
