//! Signing: arbitrary key/link/timestamp/message. Invariants: sign → verify succeeds,
//! the signed frame parses, any single-bit flip in the signed frame is rejected
//! (by the parser's checksum or by verification), and a different key fails.
#![no_main]

use libfuzzer_sys::fuzz_target;
use mavlink_codec::signing::{self, SecretKey};
use mavlink_codec::{Frame, Header, Message, Version, encode, message_infos};

fuzz_target!(|data: &[u8]| {
    if data.len() < 32 + 1 + 6 + 2 + 2 {
        return;
    }
    let key = SecretKey(data[..32].try_into().unwrap());
    let link_id = data[32];
    let mut ts = [0u8; 8];
    ts[..6].copy_from_slice(&data[33..39]);
    let ts = u64::from_le_bytes(ts);
    let infos = message_infos();
    let info = &infos[usize::from(u16::from_le_bytes([data[39], data[40]])) % infos.len()];
    let flip = usize::from(u16::from_le_bytes([data[41], data[42]]));
    let msg = Message::parse(info.id, &data[43..]).unwrap();

    let mut f = encode(Version::V2, Header { seq: 1, sysid: 2, compid: 3 }, &msg).unwrap();
    signing::sign(&mut f, &key, link_id, ts).unwrap();
    let parsed = Frame::from_bytes(f.as_bytes()).expect("signed frame parses");
    assert_eq!(signing::verify(&parsed, &key), Ok(()));
    assert_eq!(signing::signature_fields(&parsed), Some((link_id, ts)));

    let mut other = key.0;
    other[0] ^= 1;
    assert!(signing::verify(&parsed, &SecretKey(other)).is_err());

    let mut bytes = f.as_bytes().to_vec();
    let bit = flip % (bytes.len() * 8);
    bytes[bit / 8] ^= 1 << (bit % 8);
    if let Ok(t) = Frame::from_bytes(&bytes) {
        if t.is_signed() {
            assert!(signing::verify(&t, &key).is_err(), "bit {bit} flip not detected");
        }
    }
});
