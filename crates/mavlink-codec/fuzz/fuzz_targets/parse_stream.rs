//! Arbitrary bytes, fed in arbitrary chunk sizes, through both parser modes.
//! Invariants: no panic; every returned frame re-validates to itself; known frames decode,
//! and decode → serialize is a fixed point.
#![no_main]

use libfuzzer_sys::fuzz_target;
use mavlink_codec::{Frame, FrameParser, Message};

fn check(f: &Frame) {
    if !f.crc_checked() {
        return;
    }
    assert_eq!(Frame::from_bytes(f.as_bytes()).as_ref(), Ok(f));
    let msg = f.decode().expect("crc-checked frame has a known id");
    let mut a = [0u8; 255];
    let n = msg.serialize(&mut a);
    let again = Message::parse(msg.id(), &a[..n]).unwrap();
    let mut b = [0u8; 255];
    assert_eq!(again.serialize(&mut b), n);
    assert_eq!(a, b);
}

fuzz_target!(|data: &[u8]| {
    let (chunk, bytes) = match data.split_first() {
        Some((c, rest)) => (usize::from(*c).max(1), rest),
        None => return,
    };
    for mut parser in [FrameParser::new(), FrameParser::with_unknown_passthrough()] {
        for piece in bytes.chunks(chunk) {
            parser.push(piece);
            while let Some(f) = parser.next_frame() {
                check(&f);
            }
        }
        let s = parser.stats();
        assert_eq!(s.bytes, bytes.len() as u64);
    }
});
