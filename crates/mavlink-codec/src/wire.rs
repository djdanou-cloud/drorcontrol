//! Little-endian payload writer/reader used by the generated message code.
//!
//! MAVLink payloads are at most 255 bytes, and the generator rejects any message whose
//! fields would exceed that, so these helpers index a fixed 255-byte buffer.

/// Sequential little-endian writer into a 255-byte payload buffer.
pub struct Writer<'a> {
    buf: &'a mut [u8; 255],
    pos: usize,
}

/// Sequential little-endian reader over a payload, zero-extended to 255 bytes
/// (MAVLink 2 truncates trailing zero bytes on the wire).
pub struct Reader {
    buf: [u8; 255],
    pos: usize,
}

impl<'a> Writer<'a> {
    /// Start writing at offset 0.
    pub fn new(buf: &'a mut [u8; 255]) -> Self {
        Writer { buf, pos: 0 }
    }

    /// Bytes written so far.
    pub fn len(&self) -> usize {
        self.pos
    }

    /// True if nothing has been written.
    pub fn is_empty(&self) -> bool {
        self.pos == 0
    }

    /// Write raw bytes (used for `char[N]` and `uint8_t[N]` fields).
    pub fn put_bytes(&mut self, b: &[u8]) {
        self.buf[self.pos..self.pos + b.len()].copy_from_slice(b);
        self.pos += b.len();
    }
}

impl Reader {
    /// Copy `payload` (up to 255 bytes) into a zeroed buffer.
    pub fn new(payload: &[u8]) -> Self {
        let mut buf = [0u8; 255];
        let n = payload.len().min(255);
        buf[..n].copy_from_slice(&payload[..n]);
        Reader { buf, pos: 0 }
    }

    /// Read raw bytes into `out`.
    pub fn get_bytes(&mut self, out: &mut [u8]) {
        out.copy_from_slice(&self.buf[self.pos..self.pos + out.len()]);
        self.pos += out.len();
    }
}

macro_rules! prims {
    ($($put:ident $get:ident $t:ty),* $(,)?) => {
        impl Writer<'_> {
            $(
                #[doc = concat!("Write one little-endian `", stringify!($t), "`.")]
                pub fn $put(&mut self, v: $t) {
                    self.put_bytes(&v.to_le_bytes());
                }
            )*
        }
        impl Reader {
            $(
                #[doc = concat!("Read one little-endian `", stringify!($t), "`.")]
                pub fn $get(&mut self) -> $t {
                    let mut b = [0u8; size_of::<$t>()];
                    self.get_bytes(&mut b);
                    <$t>::from_le_bytes(b)
                }
            )*
        }
    };
}

prims!(
    put_u8 get_u8 u8, put_i8 get_i8 i8,
    put_u16 get_u16 u16, put_i16 get_i16 i16,
    put_u32 get_u32 u32, put_i32 get_i32 i32,
    put_u64 get_u64 u64, put_i64 get_i64 i64,
    put_f32 get_f32 f32, put_f64 get_f64 f64,
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_and_zero_extends() {
        let mut buf = [0u8; 255];
        let mut w = Writer::new(&mut buf);
        w.put_u32(0xDEAD_BEEF);
        w.put_i16(-2);
        w.put_f32(1.5);
        let n = w.len();
        assert_eq!(n, 10);
        assert_eq!(&buf[..4], &[0xEF, 0xBE, 0xAD, 0xDE]);

        let mut r = Reader::new(&buf[..n]);
        assert_eq!(r.get_u32(), 0xDEAD_BEEF);
        assert_eq!(r.get_i16(), -2);
        assert_eq!(r.get_f32(), 1.5);
        assert_eq!(r.get_u64(), 0, "past the payload reads zeros");
    }
}
