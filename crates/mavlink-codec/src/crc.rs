//! CRC-16/MCRF4XX ("X.25"), the MAVLink frame checksum.

/// Feed `bytes` into the running checksum `crc` and return the new value.
pub fn accumulate(mut crc: u16, bytes: &[u8]) -> u16 {
    for &b in bytes {
        let mut tmp = b ^ (crc & 0xFF) as u8;
        tmp ^= tmp << 4;
        let tmp = u16::from(tmp);
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }
    crc
}

/// Checksum of a frame: `header_and_payload` is everything after the start byte,
/// followed by the message's CRC_EXTRA seed.
pub fn frame_crc(header_and_payload: &[u8], crc_extra: u8) -> u16 {
    accumulate(accumulate(0xFFFF, header_and_payload), &[crc_extra])
}

#[cfg(test)]
mod tests {
    #[test]
    fn check_value() {
        assert_eq!(super::accumulate(0xFFFF, b"123456789"), 0x6F91);
    }
}
