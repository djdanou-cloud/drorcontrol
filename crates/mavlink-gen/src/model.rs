//! In-memory model of a MAVLink dialect, plus the wire rules derived from it
//! (field ordering, payload lengths, CRC_EXTRA).

use crate::crc::X25;

/// A MAVLink primitive field type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Prim {
    /// `uint8_t`
    U8,
    /// `int8_t`
    I8,
    /// `uint16_t`
    U16,
    /// `int16_t`
    I16,
    /// `uint32_t`
    U32,
    /// `int32_t`
    I32,
    /// `uint64_t`
    U64,
    /// `int64_t`
    I64,
    /// `float`
    F32,
    /// `double`
    F64,
    /// `char` (one byte, used for fixed-length strings)
    Char,
    /// `uint8_t_mavlink_version`: a `uint8_t` filled in by the sender
    U8MavlinkVersion,
}

impl Prim {
    /// Parse the XML type name (without any `[N]` suffix).
    pub fn from_xml(name: &str) -> Option<Self> {
        Some(match name {
            "uint8_t" => Prim::U8,
            "int8_t" => Prim::I8,
            "uint16_t" => Prim::U16,
            "int16_t" => Prim::I16,
            "uint32_t" => Prim::U32,
            "int32_t" => Prim::I32,
            "uint64_t" => Prim::U64,
            "int64_t" => Prim::I64,
            "float" => Prim::F32,
            "double" => Prim::F64,
            "char" => Prim::Char,
            "uint8_t_mavlink_version" => Prim::U8MavlinkVersion,
            _ => return None,
        })
    }

    /// Size of one element on the wire, in bytes.
    pub fn size(self) -> usize {
        match self {
            Prim::U8 | Prim::I8 | Prim::Char | Prim::U8MavlinkVersion => 1,
            Prim::U16 | Prim::I16 => 2,
            Prim::U32 | Prim::I32 | Prim::F32 => 4,
            Prim::U64 | Prim::I64 | Prim::F64 => 8,
        }
    }

    /// Type name as it enters the CRC_EXTRA calculation.
    pub fn crc_name(self) -> &'static str {
        match self {
            Prim::U8 | Prim::U8MavlinkVersion => "uint8_t",
            Prim::I8 => "int8_t",
            Prim::U16 => "uint16_t",
            Prim::I16 => "int16_t",
            Prim::U32 => "uint32_t",
            Prim::I32 => "int32_t",
            Prim::U64 => "uint64_t",
            Prim::I64 => "int64_t",
            Prim::F32 => "float",
            Prim::F64 => "double",
            Prim::Char => "char",
        }
    }

    /// Rust type used to hold one element.
    pub fn rust(self) -> &'static str {
        match self {
            Prim::U8 | Prim::Char | Prim::U8MavlinkVersion => "u8",
            Prim::I8 => "i8",
            Prim::U16 => "u16",
            Prim::I16 => "i16",
            Prim::U32 => "u32",
            Prim::I32 => "i32",
            Prim::U64 => "u64",
            Prim::I64 => "i64",
            Prim::F32 => "f32",
            Prim::F64 => "f64",
        }
    }
}

/// A field's type: a primitive, optionally a fixed-length array of it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FieldType {
    /// Element type.
    pub prim: Prim,
    /// `Some(n)` for `type[n]` arrays.
    pub array_len: Option<u8>,
}

impl FieldType {
    /// Parse an XML `type` attribute such as `float`, `char[16]` or `uint8_t_mavlink_version`.
    pub fn parse(s: &str) -> Option<Self> {
        let (base, len) = match s.split_once('[') {
            Some((base, rest)) => {
                let n: u8 = rest.strip_suffix(']')?.parse().ok()?;
                if n == 0 {
                    return None;
                }
                (base, Some(n))
            }
            None => (s, None),
        };
        Some(FieldType {
            prim: Prim::from_xml(base)?,
            array_len: len,
        })
    }

    /// Total bytes on the wire.
    pub fn wire_size(&self) -> usize {
        self.prim.size() * usize::from(self.array_len.unwrap_or(1))
    }
}

/// One field of a message.
#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    /// Name as written in the XML (snake_case).
    pub name: String,
    /// Wire type.
    pub ty: FieldType,
    /// Associated enum, if any.
    pub enum_name: Option<String>,
    /// Units (e.g. `cm/s`), if any.
    pub units: Option<String>,
    /// Human description.
    pub description: String,
    /// True if declared after `<extensions/>` (MAVLink 2 only).
    pub extension: bool,
}

/// One message definition.
#[derive(Debug, Clone, PartialEq)]
pub struct Message {
    /// Message ID (0..=16_777_215).
    pub id: u32,
    /// Name, e.g. `HEARTBEAT`.
    pub name: String,
    /// Human description.
    pub description: String,
    /// Fields in declaration order.
    pub fields: Vec<Field>,
    /// Marked `<deprecated>` or `<superseded>`.
    pub deprecated: bool,
    /// Marked `<wip/>` (work in progress, may change).
    pub wip: bool,
    /// Source XML file name (for diagnostics and docs).
    pub source: String,
}

impl Message {
    /// Fields in wire order: base fields stably sorted by element size (largest first),
    /// followed by extension fields in declaration order.
    pub fn wire_fields(&self) -> Vec<&Field> {
        let mut base: Vec<&Field> = self.fields.iter().filter(|f| !f.extension).collect();
        base.sort_by_key(|f| std::cmp::Reverse(f.ty.prim.size()));
        base.extend(self.fields.iter().filter(|f| f.extension));
        base
    }

    /// Payload length of the base (MAVLink 1 compatible) fields.
    pub fn base_len(&self) -> usize {
        self.fields
            .iter()
            .filter(|f| !f.extension)
            .map(|f| f.ty.wire_size())
            .sum()
    }

    /// Payload length including extension fields.
    pub fn max_len(&self) -> usize {
        self.fields.iter().map(|f| f.ty.wire_size()).sum()
    }

    /// The CRC_EXTRA seed byte: a hash of the message name and base-field layout,
    /// so that sender and receiver disagreeing on the layout fail the checksum.
    pub fn crc_extra(&self) -> u8 {
        let mut crc = X25::new();
        crc.accumulate(self.name.as_bytes());
        crc.accumulate(b" ");
        for f in self.wire_fields().into_iter().filter(|f| !f.extension) {
            crc.accumulate(f.ty.prim.crc_name().as_bytes());
            crc.accumulate(b" ");
            crc.accumulate(f.name.as_bytes());
            crc.accumulate(b" ");
            if let Some(n) = f.ty.array_len {
                crc.accumulate(&[n]);
            }
        }
        let c = crc.get();
        ((c & 0xFF) ^ (c >> 8)) as u8
    }
}

/// One enum entry.
#[derive(Debug, Clone, PartialEq)]
pub struct Entry {
    /// Full name, e.g. `MAV_CMD_NAV_WAYPOINT`.
    pub name: String,
    /// Numeric value.
    pub value: u64,
    /// Human description.
    pub description: String,
    /// Marked `<deprecated>` or `<superseded>`.
    pub deprecated: bool,
}

/// One enum definition (possibly merged from several XML files).
#[derive(Debug, Clone, PartialEq)]
pub struct Enum {
    /// Name, e.g. `MAV_CMD`.
    pub name: String,
    /// `bitmask="true"` in the XML.
    pub bitmask: bool,
    /// Human description.
    pub description: String,
    /// Entries in declaration order.
    pub entries: Vec<Entry>,
}

/// A fully resolved dialect (root file plus all includes).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Dialect {
    /// Messages sorted by ID.
    pub messages: Vec<Message>,
    /// Enums sorted by name.
    pub enums: Vec<Enum>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn field(name: &str, ty: &str, extension: bool) -> Field {
        Field {
            name: name.into(),
            ty: FieldType::parse(ty).unwrap(),
            enum_name: None,
            units: None,
            description: String::new(),
            extension,
        }
    }

    fn heartbeat() -> Message {
        Message {
            id: 0,
            name: "HEARTBEAT".into(),
            description: String::new(),
            fields: vec![
                field("type", "uint8_t", false),
                field("autopilot", "uint8_t", false),
                field("base_mode", "uint8_t", false),
                field("custom_mode", "uint32_t", false),
                field("system_status", "uint8_t", false),
                field("mavlink_version", "uint8_t_mavlink_version", false),
            ],
            deprecated: false,
            wip: false,
            source: "minimal.xml".into(),
        }
    }

    #[test]
    fn heartbeat_crc_extra_is_50() {
        // Well-known constant from every MAVLink implementation.
        assert_eq!(heartbeat().crc_extra(), 50);
    }

    #[test]
    fn wire_order_sorts_by_size_stably_and_keeps_extensions_last() {
        let mut m = heartbeat();
        m.fields.push(field("ext_a", "uint8_t", true));
        m.fields.push(field("ext_b", "uint64_t", true));
        let order: Vec<&str> = m.wire_fields().iter().map(|f| f.name.as_str()).collect();
        assert_eq!(
            order,
            [
                "custom_mode",
                "type",
                "autopilot",
                "base_mode",
                "system_status",
                "mavlink_version",
                "ext_a",
                "ext_b"
            ]
        );
        assert_eq!(m.base_len(), 9);
        assert_eq!(m.max_len(), 18);
        assert_eq!(m.crc_extra(), 50, "extensions must not change CRC_EXTRA");
    }

    #[test]
    fn parses_field_types() {
        let t = FieldType::parse("char[16]").unwrap();
        assert_eq!(
            (t.prim, t.array_len, t.wire_size()),
            (Prim::Char, Some(16), 16)
        );
        assert_eq!(FieldType::parse("float[4]").unwrap().wire_size(), 16);
        assert!(FieldType::parse("bool").is_none());
        assert!(FieldType::parse("uint8_t[0]").is_none());
        assert!(FieldType::parse("uint8_t[x]").is_none());
    }
}
