//! Deterministic mutation testing on stable Rust (the coverage-guided fuzzers in `fuzz/`
//! need nightly). Valid frames of every message are mutated — bit flips, truncation,
//! splicing, junk insertion — and streamed through the parser in random chunk sizes.
//! Invariants: no panic; every frame returned is valid; and every intact frame
//! surrounded by garbage is still recovered.

use mavlink_codec::{Frame, FrameParser, Header, Message, Version, encode, message_infos};

/// xorshift64*: tiny, deterministic, good enough to drive mutations.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
    fn bytes(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.next() as u8).collect()
    }
}

fn random_frame(rng: &mut Rng) -> Frame {
    let infos = message_infos();
    let info = &infos[rng.below(infos.len())];
    let len = rng.below(info.max_len + 1);
    let msg = Message::parse(info.id, &rng.bytes(len)).unwrap();
    let version = if info.id <= 255 && rng.below(2) == 0 {
        Version::V1
    } else {
        Version::V2
    };
    let hdr = Header {
        seq: rng.next() as u8,
        sysid: rng.next() as u8,
        compid: rng.next() as u8,
    };
    encode(version, hdr, &msg).unwrap()
}

fn assert_valid(f: &Frame) {
    assert!(f.crc_checked());
    assert_eq!(Frame::from_bytes(f.as_bytes()).as_ref(), Ok(f));
    f.decode().expect("valid frame decodes");
}

#[test]
fn mutated_streams_never_panic_and_only_yield_valid_frames() {
    let mut rng = Rng(0x00D0_0A1E_C0DE_C0DE);
    for _ in 0..20_000 {
        let mut stream = Vec::new();
        for _ in 0..1 + rng.below(6) {
            let mut b = random_frame(&mut rng).as_bytes().to_vec();
            match rng.below(5) {
                0 => {
                    let i = rng.below(b.len());
                    b[i] ^= 1 << rng.below(8);
                }
                1 => b.truncate(rng.below(b.len())),
                2 => {
                    let at = rng.below(b.len());
                    let n = rng.below(8);
                    let junk = rng.bytes(n);
                    b.splice(at..at, junk);
                }
                _ => {}
            }
            stream.extend(b);
        }
        let mut p = FrameParser::new();
        let mut i = 0;
        while i < stream.len() {
            let n = 1 + rng.below(64);
            p.push(&stream[i..(i + n).min(stream.len())]);
            i += n;
            while let Some(f) = p.next_frame() {
                assert_valid(&f);
            }
        }
        assert_eq!(p.stats().bytes, stream.len() as u64);
    }
}

#[test]
fn intact_frames_survive_surrounding_garbage() {
    let mut rng = Rng(42);
    for _ in 0..10_000 {
        let good = random_frame(&mut rng);
        let n = rng.below(40);
        let mut stream = rng.bytes(n);
        stream.extend_from_slice(good.as_bytes());
        // Enough trailing bytes that a false start byte in the garbage cannot hold back
        // the parser waiting for a long claimed payload.
        stream.extend(rng.bytes(300));
        let mut p = FrameParser::new();
        p.push(&stream);
        let mut found = false;
        while let Some(f) = p.next_frame() {
            assert_valid(&f);
            found |= f == good;
        }
        assert!(found, "lost {good:?}");
    }
}
