//! Arbitrary payloads for every message id. Invariants: decoding never panics, serialize
//! always writes MAX_LEN bytes, and decode/serialize is idempotent; encode accepts any
//! decoded message and the result parses back.
#![no_main]

use libfuzzer_sys::fuzz_target;
use mavlink_codec::{Frame, Header, Message, Version, encode, message_infos};

fuzz_target!(|data: &[u8]| {
    if data.len() < 2 {
        return;
    }
    let infos = message_infos();
    let info = &infos[usize::from(u16::from_le_bytes([data[0], data[1]])) % infos.len()];
    let payload = &data[2..];
    let msg = Message::parse(info.id, payload).unwrap();
    let mut a = [0u8; 255];
    assert_eq!(msg.serialize(&mut a), info.max_len);
    let mut b = [0u8; 255];
    Message::parse(info.id, &a[..info.max_len]).unwrap().serialize(&mut b);
    assert_eq!(a, b);

    for version in [Version::V1, Version::V2] {
        if let Ok(f) = encode(version, Header { seq: data[0], sysid: data[1], compid: 0 }, &msg) {
            let parsed = Frame::from_bytes(f.as_bytes()).expect("encoded frame parses");
            assert_eq!(parsed.msgid(), info.id);
        }
    }
});
