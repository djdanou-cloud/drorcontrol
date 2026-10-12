//! Test support: build a small dataflash log in memory, write it to a temporary file and open
//! it. Values are given in real units and encoded with the format's scale, as ArduPilot does.

use std::sync::atomic::{AtomicU32, Ordering};

use crate::dataflash::{FMT_TYPE, Log};

/// A field value.
pub enum V<'a> {
    /// Any numeric field.
    N(f64),
    /// A text field (`n`, `N`, `Z`).
    T(&'a str),
}

/// Builds a log record by record.
#[derive(Default)]
pub struct LogBuilder {
    bytes: Vec<u8>,
    formats: Vec<(String, u8, String)>,
}

fn size_and_scale(c: char) -> (usize, f64) {
    match c {
        'b' | 'B' | 'M' => (1, 1.0),
        'h' | 'H' | 'g' => (2, 1.0),
        'c' | 'C' => (2, 0.01),
        'i' | 'I' | 'f' => (4, 1.0),
        'e' | 'E' => (4, 0.01),
        'L' => (4, 1e-7),
        'q' | 'Q' | 'd' => (8, 1.0),
        'n' => (4, 1.0),
        'N' => (16, 1.0),
        'Z' | 'a' => (64, 1.0),
        _ => panic!("format code {c}"),
    }
}

impl LogBuilder {
    /// Declare a message (writes its `FMT` record).
    pub fn format(&mut self, name: &str, format: &str, cols: &str) -> &mut Self {
        let t = 10 + self.formats.len() as u8;
        let len = 3 + format.chars().map(|c| size_and_scale(c).0).sum::<usize>();
        self.bytes.extend([0xA3, 0x95, FMT_TYPE, t, len as u8]);
        for (s, n) in [(name, 4), (format, 16), (cols, 64)] {
            let mut b = s.as_bytes().to_vec();
            b.resize(n, 0);
            self.bytes.extend(b);
        }
        self.formats.push((name.into(), t, format.into()));
        self
    }

    /// Append one record of `name`, values in column order.
    pub fn rec(&mut self, name: &str, values: &[V]) -> &mut Self {
        let (_, t, format) = self
            .formats
            .iter()
            .find(|(n, _, _)| n == name)
            .unwrap_or_else(|| panic!("no FMT for {name}"));
        assert_eq!(values.len(), format.len(), "{name}: value count");
        self.bytes.extend([0xA3, 0x95, *t]);
        for (c, v) in format.chars().zip(values) {
            let (size, scale) = size_and_scale(c);
            match v {
                V::T(s) => {
                    let mut b = s.as_bytes().to_vec();
                    b.resize(size, 0);
                    self.bytes.extend(b);
                }
                V::N(x) => {
                    let raw = x / scale;
                    match c {
                        'b' => self.bytes.extend((raw.round() as i8).to_le_bytes()),
                        'B' | 'M' => self.bytes.push(raw.round() as u8),
                        'h' | 'c' => self.bytes.extend((raw.round() as i16).to_le_bytes()),
                        'H' | 'C' => self.bytes.extend((raw.round() as u16).to_le_bytes()),
                        'i' | 'e' | 'L' => self.bytes.extend((raw.round() as i32).to_le_bytes()),
                        'I' | 'E' => self.bytes.extend((raw.round() as u32).to_le_bytes()),
                        'q' => self.bytes.extend((raw.round() as i64).to_le_bytes()),
                        'Q' => self.bytes.extend((raw.round() as u64).to_le_bytes()),
                        'f' => self.bytes.extend((raw as f32).to_le_bytes()),
                        'd' => self.bytes.extend(raw.to_le_bytes()),
                        _ => panic!("numeric value for code {c}"),
                    }
                }
            }
        }
        self
    }

    /// Write to a unique temporary file and open it.
    pub fn open(&self) -> Log {
        static N: AtomicU32 = AtomicU32::new(0);
        let path = std::env::temp_dir().join(format!(
            "dc-testlog-{}-{}.bin",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::write(&path, &self.bytes).unwrap();
        let log = Log::open(&path).unwrap();
        let _ = std::fs::remove_file(&path); // the mapping stays valid (Unix); best effort elsewhere
        log
    }
}
