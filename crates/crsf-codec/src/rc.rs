//! RC channels (`RC_CHANNELS_PACKED`, 0x16): 16 channels × 11 bits in 22 bytes, packed
//! little-endian, least significant bit first. Raw values map to microseconds as ArduPilot
//! decodes them (`decode_11bit_channels(…, 5, 8, 880)`, `AP_RCProtocol_CRSF.cpp`):
//! µs = raw × 5 / 8 + 880 in integer arithmetic, so 172 → 987, 992 → 1500, 1811 → 2011.

/// Channels per frame.
pub const CHANNELS: usize = 16;
/// Payload bytes.
pub const PACKED_LEN: usize = 22;
/// Raw value at the low end of the normal range (`CRSF_DIGITAL_CHANNEL_MIN`).
pub const RAW_MIN: u16 = 172;
/// Raw value at centre.
pub const RAW_MID: u16 = 992;
/// Raw value at the high end of the normal range (`CRSF_DIGITAL_CHANNEL_MAX`).
pub const RAW_MAX: u16 = 1811;

/// Pack 16 raw 11-bit values (higher bits are ignored).
pub fn pack(raw: &[u16; CHANNELS]) -> [u8; PACKED_LEN] {
    let mut out = [0u8; PACKED_LEN];
    let mut bit = 0usize;
    for &v in raw {
        let v = u32::from(v & 0x7FF);
        for i in 0..11 {
            if v & (1 << i) != 0 {
                out[(bit + i) / 8] |= 1 << ((bit + i) % 8);
            }
        }
        bit += 11;
    }
    out
}

/// Unpack 16 raw 11-bit values.
pub fn unpack(bytes: &[u8; PACKED_LEN]) -> [u16; CHANNELS] {
    let mut out = [0u16; CHANNELS];
    for (ch, v) in out.iter_mut().enumerate() {
        let bit = ch * 11;
        for i in 0..11 {
            if bytes[(bit + i) / 8] & (1 << ((bit + i) % 8)) != 0 {
                *v |= 1 << i;
            }
        }
    }
    out
}

/// Microseconds for a raw value, as ArduPilot decodes it.
pub fn us_from_raw(raw: u16) -> u16 {
    ((u32::from(raw & 0x7FF) * 5) / 8 + 880) as u16
}

/// The raw value whose decoded microseconds are closest to `us` (clamped to 0..=2047).
pub fn raw_from_us(us: u16) -> u16 {
    let x = (i32::from(us) - 880) * 8;
    // Round to nearest, then correct for the decoder's truncation.
    let mut raw = ((x + 4) / 5).clamp(0, 2047) as u16;
    while raw > 0 && us_from_raw(raw) > us {
        raw -= 1;
    }
    while raw < 2047 && us_from_raw(raw + 1) <= us {
        raw += 1;
    }
    raw
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoints_map_as_arducopter_decodes_them() {
        assert_eq!(us_from_raw(RAW_MIN), 987);
        assert_eq!(us_from_raw(RAW_MID), 1500);
        assert_eq!(us_from_raw(RAW_MAX), 2011);
        for us in [987, 1000, 1100, 1500, 1501, 1999, 2000, 2011] {
            let back = us_from_raw(raw_from_us(us));
            assert!(back.abs_diff(us) <= 1, "{us} -> {back}");
            assert!(back <= us, "never above the request: {us} -> {back}");
        }
        assert_eq!(raw_from_us(0), 0);
        assert_eq!(raw_from_us(u16::MAX), 2047);
    }

    #[test]
    fn every_value_in_every_slot_round_trips() {
        // Exhaustive: 2048 values × 16 slots, neighbours set to distinct patterns so a bit
        // leaking into the next channel shows.
        for slot in 0..CHANNELS {
            for v in 0..2048u16 {
                let mut raw = [0u16; CHANNELS];
                for (i, r) in raw.iter_mut().enumerate() {
                    *r = ((i as u16) * 137 + 5) & 0x7FF;
                }
                raw[slot] = v;
                assert_eq!(unpack(&pack(&raw)), raw, "slot {slot} value {v}");
            }
        }
    }

    #[test]
    fn packing_is_little_endian_lsb_first() {
        let mut raw = [0u16; CHANNELS];
        raw[0] = 0x7FF;
        let p = pack(&raw);
        assert_eq!((p[0], p[1], p[2]), (0xFF, 0x07, 0x00));
        let mut raw = [0u16; CHANNELS];
        raw[1] = 1;
        assert_eq!(pack(&raw)[1], 0x08, "channel 2 starts at bit 11");
        // Bits above 11 are ignored.
        let mut raw = [0u16; CHANNELS];
        raw[0] = 0xF800;
        assert_eq!(pack(&raw), [0; PACKED_LEN]);
    }
}
