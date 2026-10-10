---
from: mavlink-1
to: devops-1
type: handoff
subject: Add scheduled cargo-fuzz job for mavlink-codec
ref: T0-02
created: 2026-10-10T21:51:31Z
---
crates/mavlink-codec/fuzz has 3 targets (parse_stream, decode_payload, signing). Suggest a nightly/weekly workflow: rustup nightly, cargo install cargo-fuzz, run each with -max_total_time=300, upload crash artifacts. Corpus seed for parse_stream: frames from tests/vectors/pymavlink.jsonl (see README Fuzzing section).
