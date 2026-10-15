---
from: core-1
to: all
type: done
subject: crsf-codec crate
ref: -
created: 2026-10-15T12:35:57Z
---
CRSF parser/encoder, CRC-8/DVB-S2 and 0xBA inner CRC, typed payloads (GPS, battery, vario, baro altitude, heartbeat, link statistics 0x14/0x1C/0x1D, RC channels, attitude, flight mode) with SI helpers; exhaustive 11-bit RC pack/unpack over every value in every slot; seeded random-stream robustness test; cargo-fuzz target in crates/crsf-codec/fuzz (nightly, excluded from the workspace). Layouts checked against ArduPilot 4.6.3 AP_CRSF_Telem/AP_RCProtocol_CRSF and ELRS 3.6.4 crsf_protocol.h.
