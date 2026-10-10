//! MAVLink XML parsing and include resolution.

use std::collections::{BTreeMap, HashSet};
use std::fmt;
use std::path::{Path, PathBuf};

use quick_xml::events::{BytesStart, Event};
use quick_xml::{Reader, XmlVersion};

use crate::model::{Dialect, Entry, Enum, Field, FieldType, Message};

/// Anything that makes a dialect unusable. Messages name the file involved.
#[derive(Debug)]
pub struct GenError(pub String);

impl fmt::Display for GenError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for GenError {}

fn err<T>(msg: impl Into<String>) -> Result<T, GenError> {
    Err(GenError(msg.into()))
}

/// Contents of a single XML file, before include resolution.
#[derive(Debug, Default)]
struct FileDefs {
    includes: Vec<String>,
    messages: Vec<Message>,
    enums: Vec<Enum>,
}

/// Load `root` and every file it (transitively) includes, and merge them into one dialect.
///
/// Rules: each file is read once; enums with the same name are merged (entries de-duplicated
/// by name); the same message ID defined twice is an error unless the definitions are identical.
pub fn load_dialect(root: &Path) -> Result<Dialect, GenError> {
    let mut seen = HashSet::new();
    let mut order = Vec::new();
    collect(root, &mut seen, &mut order)?;

    let mut messages: BTreeMap<u32, Message> = BTreeMap::new();
    let mut enums: BTreeMap<String, Enum> = BTreeMap::new();
    for defs in order {
        for m in defs.messages {
            match messages.get(&m.id) {
                Some(prev) if prev.name != m.name || prev.fields != m.fields => {
                    return err(format!(
                        "message id {} defined as {} ({}) and {} ({})",
                        m.id, prev.name, prev.source, m.name, m.source
                    ));
                }
                Some(_) => {}
                None => {
                    messages.insert(m.id, m);
                }
            }
        }
        for e in defs.enums {
            match enums.get_mut(&e.name) {
                Some(prev) => {
                    prev.bitmask |= e.bitmask;
                    if prev.description.is_empty() {
                        prev.description = e.description;
                    }
                    for entry in e.entries {
                        if !prev.entries.iter().any(|x| x.name == entry.name) {
                            prev.entries.push(entry);
                        }
                    }
                }
                None => {
                    enums.insert(e.name.clone(), e);
                }
            }
        }
    }

    let mut names = HashSet::new();
    for m in messages.values() {
        if !names.insert(m.name.as_str()) {
            return err(format!("message name {} used by two different ids", m.name));
        }
    }

    Ok(Dialect {
        messages: messages.into_values().collect(),
        enums: enums.into_values().collect(),
    })
}

/// Depth-first include walk; included files come before the file that includes them.
fn collect(
    path: &Path,
    seen: &mut HashSet<PathBuf>,
    order: &mut Vec<FileDefs>,
) -> Result<(), GenError> {
    let canonical = path
        .canonicalize()
        .map_err(|e| GenError(format!("{}: {e}", path.display())))?;
    if !seen.insert(canonical.clone()) {
        return Ok(());
    }
    let text = std::fs::read_to_string(&canonical)
        .map_err(|e| GenError(format!("{}: {e}", path.display())))?;
    let file_name = canonical
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    let defs = parse_file(&text, &file_name).map_err(|e| GenError(format!("{file_name}: {e}")))?;
    let dir = canonical.parent().unwrap_or(Path::new("."));
    for inc in &defs.includes {
        collect(&dir.join(inc), seen, order)?;
    }
    order.push(defs);
    Ok(())
}

/// Parse an enum entry value: decimal, `0x` hex, `0b` binary, or `2**N`.
pub fn parse_value(s: &str) -> Option<u64> {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).ok()
    } else if let Some(bin) = s.strip_prefix("0b") {
        u64::from_str_radix(bin, 2).ok()
    } else if let Some(exp) = s.strip_prefix("2**") {
        1u64.checked_shl(exp.parse().ok()?)
    } else {
        s.parse().ok()
    }
}

fn attr(e: &BytesStart<'_>, key: &str) -> Result<Option<String>, GenError> {
    for a in e.attributes() {
        let a = a.map_err(|e| GenError(format!("bad attribute: {e}")))?;
        if a.key.as_ref() == key {
            let v = a
                .normalized_value(XmlVersion::Implicit1_0)
                .map_err(|e| GenError(format!("bad attribute value: {e}")))?;
            return Ok(Some(v.into_owned()));
        }
    }
    Ok(None)
}

fn required(e: &BytesStart<'_>, key: &str) -> Result<String, GenError> {
    let tag = e.name().as_ref().to_string();
    attr(e, key)?.ok_or_else(|| GenError(format!("<{tag}> without `{key}`")))
}

/// Collapse runs of whitespace into single spaces.
fn squash(s: &str) -> String {
    s.split_whitespace().collect::<Vec<_>>().join(" ")
}

/// Where accumulated text should go when the current element closes.
#[derive(Debug, Clone, Copy, PartialEq)]
enum TextTarget {
    None,
    Include,
    MessageDesc,
    EnumDesc,
    EntryDesc,
    Field,
}

fn parse_file(text: &str, source: &str) -> Result<FileDefs, GenError> {
    let mut reader = Reader::from_str(text);
    reader.config_mut().trim_text(false); // keep spaces around entities; text is squashed later

    let mut defs = FileDefs::default();
    let mut stack: Vec<String> = Vec::new();
    let mut message: Option<Message> = None;
    let mut in_extensions = false;
    let mut enum_: Option<Enum> = None;
    let mut entry: Option<Entry> = None;
    let mut field: Option<Field> = None;
    let mut target = TextTarget::None;
    let mut buf = String::new();

    loop {
        let ev = reader.read_event().map_err(|e| {
            GenError(format!(
                "XML error at byte {}: {e}",
                reader.buffer_position()
            ))
        })?;
        match ev {
            Event::Start(ref e) | Event::Empty(ref e) => {
                let empty = matches!(ev, Event::Empty(_));
                let tag = e.name().as_ref().to_string();
                let parent = stack.last().map(String::as_str);
                match (tag.as_str(), parent) {
                    ("include", _) => target = TextTarget::Include,
                    ("message", _) => {
                        let id: u32 = required(e, "id")?
                            .parse()
                            .map_err(|_| GenError("message id is not a number".into()))?;
                        if id > 0xFF_FFFF {
                            return err(format!("message id {id} exceeds 24 bits"));
                        }
                        message = Some(Message {
                            id,
                            name: required(e, "name")?,
                            description: String::new(),
                            fields: Vec::new(),
                            deprecated: false,
                            wip: false,
                            source: source.to_string(),
                        });
                        in_extensions = false;
                    }
                    ("extensions", Some("message")) => in_extensions = true,
                    ("field", Some("message")) => {
                        let ty_s = required(e, "type")?;
                        let ty = FieldType::parse(&ty_s)
                            .ok_or_else(|| GenError(format!("unsupported field type `{ty_s}`")))?;
                        field = Some(Field {
                            name: required(e, "name")?,
                            ty,
                            enum_name: attr(e, "enum")?,
                            units: attr(e, "units")?,
                            description: String::new(),
                            extension: in_extensions,
                        });
                        target = TextTarget::Field;
                    }
                    ("enum", Some("enums")) => {
                        enum_ = Some(Enum {
                            name: required(e, "name")?,
                            bitmask: attr(e, "bitmask")?.as_deref() == Some("true"),
                            description: String::new(),
                            entries: Vec::new(),
                        });
                    }
                    ("entry", Some("enum")) => {
                        let v = required(e, "value")?;
                        let value = parse_value(&v)
                            .ok_or_else(|| GenError(format!("bad enum value `{v}`")))?;
                        entry = Some(Entry {
                            name: required(e, "name")?,
                            value,
                            description: String::new(),
                            deprecated: false,
                        });
                    }
                    ("description", Some("message")) => target = TextTarget::MessageDesc,
                    ("description", Some("enum")) => target = TextTarget::EnumDesc,
                    ("description", Some("entry")) => target = TextTarget::EntryDesc,
                    ("deprecated" | "superseded", Some("message")) => {
                        if let Some(m) = message.as_mut() {
                            m.deprecated = true;
                        }
                    }
                    ("deprecated" | "superseded", Some("entry")) => {
                        if let Some(en) = entry.as_mut() {
                            en.deprecated = true;
                        }
                    }
                    ("wip", Some("message")) => {
                        if let Some(m) = message.as_mut() {
                            m.wip = true;
                        }
                    }
                    _ => {}
                }
                if empty {
                    close(
                        &tag,
                        &mut defs,
                        &mut message,
                        &mut enum_,
                        &mut entry,
                        &mut field,
                        &mut target,
                        &mut buf,
                    )?;
                } else {
                    stack.push(tag);
                }
            }
            Event::Text(t) => {
                if target != TextTarget::None {
                    buf.push_str(&t);
                }
            }
            Event::CData(t) => {
                if target != TextTarget::None {
                    buf.push_str(&t);
                }
            }
            Event::GeneralRef(r) => {
                if target != TextTarget::None {
                    let resolved = match r.resolve_char_ref() {
                        Ok(Some(c)) => Some(c),
                        _ => match &*r {
                            "lt" => Some('<'),
                            "gt" => Some('>'),
                            "amp" => Some('&'),
                            "quot" => Some('"'),
                            "apos" => Some('\''),
                            _ => None,
                        },
                    };
                    match resolved {
                        Some(c) => buf.push(c),
                        None => return err(format!("unknown entity &{};", &*r)),
                    }
                }
            }
            Event::End(e) => {
                let tag = e.name().as_ref().to_string();
                stack.pop();
                close(
                    &tag,
                    &mut defs,
                    &mut message,
                    &mut enum_,
                    &mut entry,
                    &mut field,
                    &mut target,
                    &mut buf,
                )?;
            }
            Event::Eof => break,
            _ => {}
        }
    }
    Ok(defs)
}

/// Finish the element `tag`: move accumulated text and completed items to their owners.
#[allow(clippy::too_many_arguments)]
fn close(
    tag: &str,
    defs: &mut FileDefs,
    message: &mut Option<Message>,
    enum_: &mut Option<Enum>,
    entry: &mut Option<Entry>,
    field: &mut Option<Field>,
    target: &mut TextTarget,
    buf: &mut String,
) -> Result<(), GenError> {
    let text = squash(buf);
    match (tag, *target) {
        ("include", TextTarget::Include) => defs.includes.push(text),
        ("description", TextTarget::MessageDesc) => {
            if let Some(m) = message.as_mut() {
                m.description = text;
            }
        }
        ("description", TextTarget::EnumDesc) => {
            if let Some(e) = enum_.as_mut() {
                e.description = text;
            }
        }
        ("description", TextTarget::EntryDesc) => {
            if let Some(e) = entry.as_mut() {
                e.description = text;
            }
        }
        ("field", TextTarget::Field) => {
            if let (Some(mut f), Some(m)) = (field.take(), message.as_mut()) {
                f.description = text;
                if m.fields.iter().any(|x| x.name == f.name) {
                    return err(format!("{}: duplicate field {}", m.name, f.name));
                }
                m.fields.push(f);
            }
        }
        _ => {}
    }
    if matches!(tag, "include" | "description" | "field") {
        buf.clear();
        *target = TextTarget::None;
    }
    match tag {
        "message" => {
            if let Some(m) = message.take() {
                if m.max_len() > 255 {
                    return err(format!(
                        "{}: payload {} bytes exceeds 255",
                        m.name,
                        m.max_len()
                    ));
                }
                defs.messages.push(m);
            }
        }
        "entry" => {
            if let (Some(en), Some(e)) = (entry.take(), enum_.as_mut()) {
                e.entries.push(en);
            }
        }
        "enum" => {
            if let Some(e) = enum_.take() {
                defs.enums.push(e);
            }
        }
        _ => {}
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"<?xml version="1.0"?>
<mavlink>
  <enums>
    <enum name="MY_FLAGS" bitmask="true">
      <description>Flags &amp; things.</description>
      <entry value="1" name="MY_FLAGS_A"><description>A</description></entry>
      <entry value="0x10" name="MY_FLAGS_B"><deprecated since="2020-01" replaced_by="X"/></entry>
    </enum>
  </enums>
  <messages>
    <message id="42" name="DEMO">
      <description>Demo   message.</description>
      <field type="uint8_t" name="a" enum="MY_FLAGS">First &lt;field&gt;.</field>
      <field type="float[3]" name="v" units="m">Vector.</field>
      <extensions/>
      <field type="uint16_t" name="x">Extension.</field>
    </message>
  </messages>
</mavlink>"#;

    #[test]
    fn parses_messages_enums_and_extensions() {
        let d = parse_file(SAMPLE, "sample.xml").unwrap();
        assert_eq!(d.messages.len(), 1);
        let m = &d.messages[0];
        assert_eq!(
            (m.id, m.name.as_str(), m.description.as_str()),
            (42, "DEMO", "Demo message.")
        );
        assert_eq!(m.fields.len(), 3);
        assert_eq!(m.fields[0].description, "First <field>.");
        assert_eq!(m.fields[0].enum_name.as_deref(), Some("MY_FLAGS"));
        assert_eq!(m.fields[1].units.as_deref(), Some("m"));
        assert!(!m.fields[1].extension && m.fields[2].extension);
        assert_eq!((m.base_len(), m.max_len()), (13, 15));

        let e = &d.enums[0];
        assert!(e.bitmask);
        assert_eq!(e.description, "Flags & things.");
        assert_eq!(
            e.entries.iter().map(|x| x.value).collect::<Vec<_>>(),
            [1, 16]
        );
        assert!(e.entries[1].deprecated);
    }

    #[test]
    fn parses_values() {
        assert_eq!(parse_value("42"), Some(42));
        assert_eq!(parse_value("0x2A"), Some(42));
        assert_eq!(parse_value("0b101"), Some(5));
        assert_eq!(parse_value("2**10"), Some(1024));
        assert_eq!(parse_value("-1"), None);
        assert_eq!(parse_value("abc"), None);
    }

    #[test]
    fn rejects_oversized_payload() {
        let xml = r#"<mavlink><messages><message id="1" name="BIG">
            <field type="uint8_t[200]" name="a">a</field><field type="uint8_t[100]" name="b">b</field>
            </message></messages></mavlink>"#;
        assert!(parse_file(xml, "big.xml").is_err());
    }
}
