//! CRC-16/MCRF4XX ("X.25" in MAVLink terminology), as used for MAVLink checksums
//! and CRC_EXTRA. The runtime codec carries its own copy; both are tested against
//! the same known values.

/// Incremental MAVLink X.25 checksum.
#[derive(Debug, Clone, Copy)]
pub struct X25(u16);

impl Default for X25 {
    fn default() -> Self {
        Self::new()
    }
}

impl X25 {
    /// Start a new checksum (initial value `0xFFFF`).
    pub fn new() -> Self {
        X25(0xFFFF)
    }

    /// Feed bytes into the checksum.
    pub fn accumulate(&mut self, bytes: &[u8]) {
        for &b in bytes {
            let mut tmp = b ^ (self.0 & 0xFF) as u8;
            tmp ^= tmp << 4;
            let tmp = u16::from(tmp);
            self.0 = (self.0 >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
        }
    }

    /// Current checksum value.
    pub fn get(&self) -> u16 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_crc16_mcrf4xx_check_value() {
        // Standard check value for CRC-16/MCRF4XX over ASCII "123456789".
        let mut c = X25::new();
        c.accumulate(b"123456789");
        assert_eq!(c.get(), 0x6F91);
    }
}
