# mavlink-codec

MAVLink v1/v2 framing, checksums, signing and the generated ArduPilot dialect (task T0-02).

| Piece | Where |
|-------|-------|
| XML definitions (ArduPilot fork, pinned) | `definitions/` — see `definitions/UPSTREAM.md` |
| Generator (XML → Rust, CRC_EXTRA, wire order) | `../mavlink-gen` (build-dependency, runs in `build.rs`) |
| Framing, stream parser, encoder | `src/frame.rs` |
| Signing + replay protection | `src/signing.rs` |
| Cross-check against pymavlink | `tests/pymavlink_vectors.rs`, vectors from `scripts/mavlink_vectors.py` |
| Mutation robustness (stable) | `tests/robustness.rs` |
| Coverage-guided fuzzing (nightly) | `fuzz/` |

Dialect: `ardupilotmega.xml` by default; `--features dialect-all` builds `all.xml`.

## Design notes

* **Lazy decode.** The parser validates frames using only `message_info(id)` (CRC_EXTRA, lengths). Typed decoding happens on `Frame::decode()`.
* **No allocation per frame.** `Frame` stores up to 280 bytes inline.
* **Resynchronisation.** Any invalid candidate costs one byte; candidates that are impossible from the header alone (unknown id, bad v1 length, unknown incompat flags) are rejected before waiting for their payload, so noise cannot stall the stream.
* **Unknown message IDs** are dropped by default; `FrameParser::with_unknown_passthrough()` returns them with `crc_checked() == false` for routers.
* **Enums are newtypes** (`enums::MavCmd(u16)` with associated consts), so unknown values from newer firmware survive a round trip. Message fields keep the wire primitive type.
* **Names follow upstream** (`Vcc`, `GOPRO_RESOLUTION_480p`); Rust keywords use raw identifiers (`r#type`).
* **MAVLink 1** encodes base fields only (as the C library does); parsing accepts base..=full length (pymavlink sends full).
* **Signing** = first 48 bits of SHA-256(key ‖ frame ‖ link id ‖ timestamp) — not HMAC.

## Fuzzing

```bash
rustup toolchain install nightly && cargo +nightly install cargo-fuzz
cd crates/mavlink-codec
cargo +nightly fuzz run parse_stream   -- -max_total_time=600
cargo +nightly fuzz run decode_payload -- -max_total_time=600
cargo +nightly fuzz run signing        -- -max_total_time=600
```
