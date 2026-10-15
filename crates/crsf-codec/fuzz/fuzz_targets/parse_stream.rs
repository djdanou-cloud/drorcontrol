//! Arbitrary bytes, fed in arbitrary chunk sizes, through the CRSF parser.
//! Invariants: no panic; every frame re-encodes to bytes that parse back to itself; every
//! payload either decodes or reports a short payload, and a decoded payload re-encodes to
//! a payload that decodes to the same value.
#![no_main]

use crsf_codec::payload::Payload;
use crsf_codec::{Frame, Parser};
use libfuzzer_sys::fuzz_target;

fn check(f: &Frame) {
    let bytes = f.encode().expect("a parsed frame fits");
    let mut p = Parser::default();
    p.push(&bytes);
    assert_eq!(p.next_frame().as_ref(), Some(f));
    if let Ok(payload) = f.decode() {
        let (t, b) = payload.encode();
        assert_eq!(t, f.frame_type);
        assert_eq!(Payload::decode(t, &b).as_ref(), Ok(&payload));
    }
}

fuzz_target!(|data: &[u8]| {
    let (chunk, bytes) = match data.split_first() {
        Some((c, rest)) => (usize::from(*c).max(1), rest),
        None => return,
    };
    let mut parser = Parser::default();
    for piece in bytes.chunks(chunk) {
        parser.push(piece);
        while let Some(f) = parser.next_frame() {
            check(&f);
        }
    }
    assert_eq!(parser.stats().bytes, bytes.len() as u64);
});
