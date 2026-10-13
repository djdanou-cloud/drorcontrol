//! ArduPilot dataflash (`.bin`) logs: format discovery, parallel indexing, lazy columns.
//!
//! Records are `0xA3 0x95 <type> <payload>`; the record length per type comes from `FMT`
//! records (type 128, 89 bytes). Field encoding follows pymavlink `DFReader`
//! (`FORMAT_TO_STRUCT`), which is the reference this module is tested against.
//!
//! Indexing (see plan 05 §1.2):
//! 1. FMT pre-scan: find every `A3 95 80` and keep those that validate as FMT records. This
//!    is independent of record boundaries, so FMTs written mid-log are found too.
//! 2. Block walk, in parallel: the file is cut into [`BLOCK`]-sized blocks; each block is
//!    walked from the first offset that starts a chain of valid records. A record belongs
//!    to the block where it starts. Corrupt bytes are skipped (and counted).
//! 3. Per message type, record offsets are stored block-relative as `u32`
//!    (4 bytes per record: ~400 MB for 100 M records).
//!
//! Columns are materialised on demand from those offsets.

#![allow(unsafe_code)] // memory mapping (see `Log::open`)

use std::collections::BTreeMap;
use std::fs::File;
use std::io;
use std::path::Path;

use memmap2::Mmap;
use rayon::prelude::*;

/// Record header bytes.
pub const HEAD: [u8; 2] = [0xA3, 0x95];
/// Message type of FMT records.
pub const FMT_TYPE: u8 = 128;
/// Length of FMT records (header included).
pub const FMT_LEN: usize = 89;
/// Block size for parallel indexing.
pub const BLOCK: usize = 64 << 20;
/// Consecutive valid records required to trust a resynchronisation point.
const CHAIN: usize = 4;

/// How one field is stored.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// int8 (`b`)
    I8,
    /// uint8 (`B`, `M`)
    U8,
    /// int16 (`h`, and ×0.01 for `c`)
    I16,
    /// uint16 (`H`, and ×0.01 for `C`)
    U16,
    /// int32 (`i`, ×0.01 for `e`, ×1e-7 for `L`)
    I32,
    /// uint32 (`I`, ×0.01 for `E`)
    U32,
    /// int64 (`q`)
    I64,
    /// uint64 (`Q`)
    U64,
    /// IEEE half (`g`)
    F16,
    /// float (`f`)
    F32,
    /// double (`d`)
    F64,
    /// Fixed text (`n` 4, `N` 16, `Z` 64)
    Text,
    /// int16[32] (`a`)
    Array,
}

/// One field: where it is and how to read it.
#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    /// Column name from the FMT record.
    pub name: String,
    /// Format character.
    pub code: char,
    /// Storage kind.
    pub kind: Kind,
    /// Byte offset from the start of the record (header included).
    pub offset: usize,
    /// Bytes.
    pub size: usize,
    /// Scale applied by the format character (`c`, `C`, `e`, `E`: 0.01; `L`: 1e-7).
    pub mul: Option<f64>,
}

/// A message definition from an FMT record.
#[derive(Debug, Clone, PartialEq)]
pub struct Format {
    /// Message type id.
    pub msg_type: u8,
    /// Record length including the 3-byte header.
    pub len: usize,
    /// Message name, e.g. `ATT`.
    pub name: String,
    /// Format string, e.g. `QccccCCCC`.
    pub format: String,
    /// Fields in order.
    pub fields: Vec<Field>,
}

fn code_info(c: char) -> Option<(Kind, usize, Option<f64>)> {
    Some(match c {
        'b' => (Kind::I8, 1, None),
        'B' | 'M' => (Kind::U8, 1, None),
        'h' => (Kind::I16, 2, None),
        'c' => (Kind::I16, 2, Some(0.01)),
        'H' => (Kind::U16, 2, None),
        'C' => (Kind::U16, 2, Some(0.01)),
        'i' => (Kind::I32, 4, None),
        'e' => (Kind::I32, 4, Some(0.01)),
        'L' => (Kind::I32, 4, Some(1e-7)),
        'I' => (Kind::U32, 4, None),
        'E' => (Kind::U32, 4, Some(0.01)),
        'q' => (Kind::I64, 8, None),
        'Q' => (Kind::U64, 8, None),
        'g' => (Kind::F16, 2, None),
        'f' => (Kind::F32, 4, None),
        'd' => (Kind::F64, 8, None),
        'n' => (Kind::Text, 4, None),
        'N' => (Kind::Text, 16, None),
        'Z' => (Kind::Text, 64, None),
        'a' => (Kind::Array, 64, None),
        _ => return None,
    })
}

fn text(b: &[u8]) -> Option<String> {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    std::str::from_utf8(&b[..end]).ok().map(str::to_string)
}

impl Format {
    /// Parse an FMT record (`rec` starts at the header). `None` unless fully consistent.
    pub fn parse(rec: &[u8]) -> Option<Format> {
        if rec.len() < FMT_LEN || rec[..2] != HEAD || rec[2] != FMT_TYPE {
            return None;
        }
        let (msg_type, len) = (rec[3], usize::from(rec[4]));
        let name = text(&rec[5..9])?;
        let format = text(&rec[9..25])?;
        let columns = text(&rec[25..89])?;
        if name.is_empty() || !name.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_') {
            return None;
        }
        let names: Vec<&str> = if columns.is_empty() {
            Vec::new()
        } else {
            columns.split(',').collect()
        };
        if names.len() != format.chars().count() {
            return None;
        }
        let mut offset = 3;
        let mut fields = Vec::with_capacity(names.len());
        for (code, n) in format.chars().zip(names) {
            let (kind, size, mul) = code_info(code)?;
            fields.push(Field {
                name: n.to_string(),
                code,
                kind,
                offset,
                size,
                mul,
            });
            offset += size;
        }
        (offset == len).then_some(Format {
            msg_type,
            len,
            name,
            format,
            fields,
        })
    }

    /// Index of field `name`.
    pub fn field(&self, name: &str) -> Option<usize> {
        self.fields.iter().position(|f| f.name == name)
    }

    /// Numeric value of field `i` of record `rec`, scaled like pymavlink; `None` for text
    /// and arrays.
    pub fn value(&self, rec: &[u8], i: usize) -> Option<f64> {
        let f = &self.fields[i];
        let b = rec.get(f.offset..f.offset + f.size)?;
        let raw = match f.kind {
            Kind::I8 => f64::from(b[0] as i8),
            Kind::U8 => f64::from(b[0]),
            Kind::I16 => f64::from(i16::from_le_bytes([b[0], b[1]])),
            Kind::U16 => f64::from(u16::from_le_bytes([b[0], b[1]])),
            Kind::I32 => f64::from(i32::from_le_bytes(b.try_into().ok()?)),
            Kind::U32 => f64::from(u32::from_le_bytes(b.try_into().ok()?)),
            Kind::I64 => i64::from_le_bytes(b.try_into().ok()?) as f64,
            Kind::U64 => u64::from_le_bytes(b.try_into().ok()?) as f64,
            Kind::F16 => f64::from(f16_to_f32(u16::from_le_bytes([b[0], b[1]]))),
            Kind::F32 => f64::from(f32::from_le_bytes(b.try_into().ok()?)),
            Kind::F64 => f64::from_le_bytes(b.try_into().ok()?),
            Kind::Text | Kind::Array => return None,
        };
        Some(match f.mul {
            Some(m) => raw * m,
            None => raw,
        })
    }

    /// Exact integer value of a `Q`/`q`/smaller integer field (timestamps exceed f64 precision
    /// only after ~285 years of µs, but keep them integral anyway).
    pub fn u64_value(&self, rec: &[u8], i: usize) -> Option<u64> {
        let f = &self.fields[i];
        let b = rec.get(f.offset..f.offset + f.size)?;
        match f.kind {
            Kind::U64 => Some(u64::from_le_bytes(b.try_into().ok()?)),
            Kind::U32 => Some(u64::from(u32::from_le_bytes(b.try_into().ok()?))),
            _ => None,
        }
    }
}

/// IEEE 754 half → single.
pub fn f16_to_f32(h: u16) -> f32 {
    let sign = u32::from(h >> 15) << 31;
    let exp = u32::from((h >> 10) & 0x1F);
    let man = u32::from(h & 0x3FF);
    let bits = match (exp, man) {
        (0, 0) => sign,
        (0, m) => {
            // Subnormal: normalise.
            let shift = m.leading_zeros() - 21;
            sign | ((113 - shift) << 23) | (((m << shift) & 0x3FF) << 13)
        }
        (0x1F, m) => sign | 0x7F80_0000 | (m << 13),
        (e, m) => sign | ((e + 112) << 23) | (m << 13),
    };
    f32::from_bits(bits)
}

/// Record offsets of one message type, block-relative.
#[derive(Debug, Clone, Default)]
pub struct TypeIndex {
    /// `(block number, first index into rel)` for each block holding records of this type.
    pub blocks: Vec<(u32, usize)>,
    /// Offsets relative to the block start.
    pub rel: Vec<u32>,
}

impl TypeIndex {
    /// Number of records.
    pub fn len(&self) -> usize {
        self.rel.len()
    }

    /// True if there are no records.
    pub fn is_empty(&self) -> bool {
        self.rel.is_empty()
    }

    /// Absolute offsets, block by block (each slice paired with its block base).
    pub(crate) fn block_slices(&self) -> impl Iterator<Item = (usize, &[u32])> {
        self.blocks.iter().enumerate().map(move |(i, &(b, start))| {
            let end = self.blocks.get(i + 1).map_or(self.rel.len(), |n| n.1);
            (b as usize * BLOCK, &self.rel[start..end])
        })
    }
}

/// Counters from indexing.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct IndexStats {
    /// Records indexed.
    pub records: u64,
    /// Bytes skipped while resynchronising (corruption, unknown types).
    pub skipped_bytes: u64,
    /// FMT records found by the pre-scan.
    pub fmt_records: usize,
    /// Conflicting FMT definitions for the same type (first one wins).
    pub fmt_conflicts: usize,
}

/// An opened, indexed log.
pub struct Log {
    map: Mmap,
    /// Formats by message type.
    pub formats: BTreeMap<u8, Format>,
    /// Offsets by message type.
    pub index: BTreeMap<u8, TypeIndex>,
    /// Indexing counters.
    pub stats: IndexStats,
}

/// A numeric column with its time axis.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Column {
    /// `TimeUS` of each sample (empty if the message has no `TimeUS`).
    pub time_us: Vec<u64>,
    /// Values.
    pub values: Vec<f64>,
}

impl Log {
    /// Memory-map and index `path`.
    ///
    /// The file must not be truncated while open (a mapped page that disappears raises
    /// SIGBUS). Download logs to a temporary name and rename when complete.
    pub fn open(path: &Path) -> io::Result<Log> {
        let file = File::open(path)?;
        // SAFETY: read-only mapping; see the truncation caveat above.
        let map = unsafe { Mmap::map(&file)? };
        Ok(Self::index_map(map))
    }

    fn index_map(map: Mmap) -> Log {
        let data: &[u8] = &map;
        let (formats, fmt_records, fmt_conflicts) = scan_formats(&map);
        let mut lens = [0usize; 256];
        for (t, f) in &formats {
            lens[usize::from(*t)] = f.len;
        }
        lens[usize::from(FMT_TYPE)] = FMT_LEN;

        let nblocks = data.len().div_ceil(BLOCK).max(1);
        let per_block: Vec<(BTreeMap<u8, Vec<u32>>, u64, u64)> = (0..nblocks)
            .into_par_iter()
            .map(|b| {
                let r = walk_block(data, b, &lens);
                release(
                    &map,
                    b * BLOCK,
                    BLOCK.min(data.len().saturating_sub(b * BLOCK)),
                );
                r
            })
            .collect();

        let mut index: BTreeMap<u8, TypeIndex> = BTreeMap::new();
        let mut stats = IndexStats {
            fmt_records,
            fmt_conflicts,
            ..Default::default()
        };
        for (b, (types, records, skipped)) in per_block.into_iter().enumerate() {
            stats.records += records;
            stats.skipped_bytes += skipped;
            for (t, rel) in types {
                let ti = index.entry(t).or_default();
                ti.blocks.push((b as u32, ti.rel.len()));
                ti.rel.extend(rel);
            }
        }
        Log {
            map,
            formats,
            index,
            stats,
        }
    }

    /// Raw file bytes.
    pub fn bytes(&self) -> &[u8] {
        &self.map
    }

    /// Format by message name.
    pub fn format_by_name(&self, name: &str) -> Option<&Format> {
        self.formats.values().find(|f| f.name == name)
    }

    /// Record count by message name.
    pub fn counts(&self) -> BTreeMap<String, usize> {
        self.index
            .iter()
            .filter_map(|(t, ti)| self.formats.get(t).map(|f| (f.name.clone(), ti.len())))
            .collect()
    }

    /// The `i`-th record of message `msg` (header included).
    pub fn record(&self, msg: &str, i: usize) -> Option<&[u8]> {
        let fmt = self.format_by_name(msg)?;
        let ti = self.index.get(&fmt.msg_type)?;
        let rel = *ti.rel.get(i)?;
        let k = ti.blocks.partition_point(|&(_, start)| start <= i) - 1;
        let off = ti.blocks[k].0 as usize * BLOCK + rel as usize;
        self.bytes().get(off..off + fmt.len)
    }

    /// Materialise `msg.field` (optionally only records whose `inst_field` equals `inst`).
    pub fn column(&self, msg: &str, field: &str, instance: Option<(&str, f64)>) -> Option<Column> {
        let fmt = self.format_by_name(msg)?;
        let fi = fmt.field(field)?;
        let ti = fmt.field("TimeUS");
        let inst = match instance {
            Some((name, v)) => Some((fmt.field(name)?, v)),
            None => None,
        };
        let idx = self.index.get(&fmt.msg_type)?;
        let data = self.bytes();
        let parts: Vec<Column> = idx
            .block_slices()
            .collect::<Vec<_>>()
            .into_par_iter()
            .map(|(base, rels)| {
                let mut c = Column {
                    time_us: Vec::with_capacity(if ti.is_some() { rels.len() } else { 0 }),
                    values: Vec::with_capacity(rels.len()),
                };
                for &r in rels {
                    let off = base + r as usize;
                    let rec = &data[off..off + fmt.len];
                    if let Some((ii, want)) = inst
                        && fmt.value(rec, ii) != Some(want)
                    {
                        continue;
                    }
                    if let Some(v) = fmt.value(rec, fi) {
                        c.values.push(v);
                        if let Some(t) = ti {
                            c.time_us.push(fmt.u64_value(rec, t).unwrap_or(0));
                        }
                    }
                }
                release(&self.map, base, BLOCK.min(data.len() - base));
                c
            })
            .collect();
        let mut out = Column::default();
        for p in parts {
            out.time_us.extend(p.time_us);
            out.values.extend(p.values);
        }
        Some(out)
    }
}

/// Drop already-processed file pages from this process's resident set (they stay in the OS
/// page cache). Keeps memory use bounded on multi-gigabyte logs.
fn release(map: &Mmap, offset: usize, len: usize) {
    #[cfg(unix)]
    if len > 0 {
        // SAFETY: MADV_DONTNEED on a read-only file mapping only discards our page-table
        // entries; later reads fault the (unchanged) file contents back in.
        let _ =
            unsafe { map.unchecked_advise_range(memmap2::UncheckedAdvice::DontNeed, offset, len) };
    }
    #[cfg(not(unix))]
    let _ = (map, offset, len);
}

/// Find and validate every FMT record anywhere in the file.
fn scan_formats(map: &Mmap) -> (BTreeMap<u8, Format>, usize, usize) {
    let data: &[u8] = map;
    let nblocks = data.len().div_ceil(BLOCK).max(1);
    let found: Vec<Vec<(usize, Format)>> = (0..nblocks)
        .into_par_iter()
        .map(|b| {
            let start = b * BLOCK;
            let end = (start + BLOCK + FMT_LEN).min(data.len());
            let found = memchr::memmem::find_iter(&data[start..end], &[0xA3, 0x95, FMT_TYPE])
                .map(|i| start + i)
                .filter(|&p| p < start + BLOCK)
                .filter_map(|p| {
                    data.get(p..p + FMT_LEN)
                        .and_then(Format::parse)
                        .map(|f| (p, f))
                })
                .collect();
            release(map, start, end - start);
            found
        })
        .collect();
    let mut formats = BTreeMap::new();
    let (mut count, mut conflicts) = (0, 0);
    for (_, f) in found.into_iter().flatten() {
        count += 1;
        match formats.get(&f.msg_type) {
            Some(prev) if prev != &f => conflicts += 1,
            Some(_) => {}
            None => {
                formats.insert(f.msg_type, f);
            }
        }
    }
    (formats, count, conflicts)
}

fn valid_at(data: &[u8], p: usize, lens: &[usize; 256]) -> Option<usize> {
    let h = data.get(p..p + 3)?;
    if h[..2] != HEAD {
        return None;
    }
    let len = lens[usize::from(h[2])];
    (len > 0 && p + len <= data.len()).then_some(len)
}

/// First offset at or after `from` that starts a chain of valid records.
fn resync(data: &[u8], from: usize, lens: &[usize; 256]) -> Option<usize> {
    let mut p = from;
    while p < data.len() {
        let i = memchr::memchr(0xA3, &data[p..])?;
        p += i;
        let mut q = p;
        let mut ok = 0;
        while ok < CHAIN {
            match valid_at(data, q, lens) {
                Some(len) => {
                    q += len;
                    ok += 1;
                    if q == data.len() {
                        ok = CHAIN;
                    }
                }
                None => break,
            }
        }
        if ok == CHAIN {
            return Some(p);
        }
        p += 1;
    }
    None
}

/// Walk the records that start inside block `b`.
fn walk_block(data: &[u8], b: usize, lens: &[usize; 256]) -> (BTreeMap<u8, Vec<u32>>, u64, u64) {
    let start = b * BLOCK;
    let end = (start + BLOCK).min(data.len());
    let mut types: BTreeMap<u8, Vec<u32>> = BTreeMap::new();
    let (mut records, mut skipped) = (0u64, 0u64);
    let Some(mut p) = resync(data, start, lens) else {
        return (types, 0, (end - start) as u64);
    };
    if b == 0 {
        skipped += (p - start) as u64;
    }
    while p < end {
        match valid_at(data, p, lens) {
            Some(len) => {
                types
                    .entry(data[p + 2])
                    .or_default()
                    .push((p - start) as u32);
                records += 1;
                p += len;
            }
            None => match resync(data, p + 1, lens) {
                Some(q) => {
                    skipped += (q.min(end) - p) as u64;
                    p = q;
                }
                None => {
                    skipped += (end - p) as u64;
                    break;
                }
            },
        }
    }
    (types, records, skipped)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fmt_record(t: u8, name: &str, format: &str, cols: &str) -> Vec<u8> {
        let len: usize = 3 + format
            .chars()
            .map(|c| code_info(c).map_or(1, |i| i.1))
            .sum::<usize>();
        let mut r = vec![0xA3, 0x95, FMT_TYPE, t, len as u8];
        let mut put = |s: &str, n: usize| {
            let mut b = s.as_bytes().to_vec();
            b.resize(n, 0);
            r.extend(b);
        };
        put(name, 4);
        put(format, 16);
        put(cols, 64);
        r
    }

    #[test]
    fn parses_fmt_and_scales_like_pymavlink() {
        let f = Format::parse(&fmt_record(5, "TST", "QcLgf", "TimeUS,C,Lat,H,F")).unwrap();
        assert_eq!((f.len, f.name.as_str()), (3 + 8 + 2 + 4 + 2 + 4, "TST"));
        let mut rec = vec![0xA3, 0x95, 5];
        rec.extend(123_456u64.to_le_bytes());
        rec.extend((-1234i16).to_le_bytes());
        rec.extend((-353_632_610i32).to_le_bytes());
        rec.extend(0x3C00u16.to_le_bytes()); // half 1.0
        rec.extend(2.5f32.to_le_bytes());
        assert_eq!(f.u64_value(&rec, 0), Some(123_456));
        assert_eq!(f.value(&rec, 1), Some(-1234.0 * 0.01));
        assert_eq!(f.value(&rec, 2), Some(-353_632_610.0 * 1e-7));
        assert_eq!(f.value(&rec, 3), Some(1.0));
        assert_eq!(f.value(&rec, 4), Some(2.5));
    }

    #[test]
    fn rejects_inconsistent_fmt() {
        let mut r = fmt_record(5, "TST", "Qf", "TimeUS,F");
        r[4] = 99; // length disagrees with the format string
        assert!(Format::parse(&r).is_none());
        assert!(
            Format::parse(&fmt_record(5, "TST", "Qf", "TimeUS")).is_none(),
            "column count"
        );
        assert!(
            Format::parse(&fmt_record(5, "TST", "Qx", "TimeUS,X")).is_none(),
            "bad code"
        );
    }

    #[test]
    fn half_floats() {
        assert_eq!(f16_to_f32(0x0000), 0.0);
        assert_eq!(f16_to_f32(0xC000), -2.0);
        assert_eq!(f16_to_f32(0x7BFF), 65504.0);
        assert_eq!(f16_to_f32(0x0001), 5.960_464_5e-8, "smallest subnormal");
        assert!(f16_to_f32(0x7E00).is_nan());
    }
}
