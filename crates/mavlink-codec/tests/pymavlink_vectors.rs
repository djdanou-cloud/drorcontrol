//! Cross-implementation test against pymavlink (the MAVLink reference implementation).
//!
//! Vectors come from `scripts/mavlink_vectors.py`, which builds pymavlink's dialect from the
//! same vendored XML. For every vector the Rust codec must:
//! 1. accept the frame (checksum, length, signature layout),
//! 2. agree on CRC_EXTRA and header fields,
//! 3. decode exactly the field values pymavlink decoded,
//! 4. re-encode (and re-sign) to byte-identical output,
//! 5. verify the signature with the same key.
//!
//! It also checks that the vectors cover every message in the compiled dialect, so XML
//! updates without regenerated vectors fail loudly.

#![cfg(not(feature = "dialect-all"))]

use std::collections::BTreeSet;

use mavlink_codec::signing::{self, SecretKey};
use mavlink_codec::{FieldValue, Frame, Header, Version, encode, message_info, message_infos};
use serde_json::Value;

const VECTORS: &str = include_str!("vectors/pymavlink.jsonl");

fn hex(s: &str) -> Vec<u8> {
    hex::decode(s).expect("valid hex")
}

/// Compare one decoded Rust field with pymavlink's JSON value; returns a mismatch description.
fn mismatch(name: &str, got: &FieldValue, want: &Value) -> Option<String> {
    let ok = match (got, want) {
        (FieldValue::UInt(g), w) => w.as_u64() == Some(*g),
        (FieldValue::Int(g), w) => w.as_i64() == Some(*g),
        (FieldValue::Float(g), w) => w.as_f64() == Some(*g),
        (FieldValue::Chars(g), w) => {
            let mut g = g.clone();
            while g.last() == Some(&0) {
                g.pop();
            }
            w["chars"].as_str().map(hex) == Some(g)
        }
        (FieldValue::UIntArray(g), Value::Array(w)) => {
            g.len() == w.len() && g.iter().zip(w).all(|(g, w)| w.as_u64() == Some(*g))
        }
        (FieldValue::IntArray(g), Value::Array(w)) => {
            g.len() == w.len() && g.iter().zip(w).all(|(g, w)| w.as_i64() == Some(*g))
        }
        (FieldValue::FloatArray(g), Value::Array(w)) => {
            g.len() == w.len() && g.iter().zip(w).all(|(g, w)| w.as_f64() == Some(*g))
        }
        _ => false,
    };
    (!ok).then(|| format!("{name}: rust {got:?} vs pymavlink {want}"))
}

#[test]
fn matches_pymavlink_reference_vectors() {
    let mut lines = VECTORS.lines();
    let header: Value = serde_json::from_str(lines.next().expect("header line")).unwrap();
    let mut covered = BTreeSet::new();
    let mut failures = Vec::new();
    let mut count = 0;

    for line in lines {
        let rec: Value = serde_json::from_str(line).unwrap();
        count += 1;
        let tag = format!("{} #{} {}", rec["name"], rec["id"], rec["variant"]);
        let id = rec["id"].as_u64().unwrap() as u32;
        covered.insert(id);
        let bytes = hex(rec["frame"].as_str().unwrap());

        let Some(info) = message_info(id) else {
            failures.push(format!("{tag}: id missing from Rust dialect"));
            continue;
        };
        if u64::from(info.crc_extra) != rec["crc_extra"].as_u64().unwrap() {
            failures.push(format!(
                "{tag}: CRC_EXTRA rust {} vs pymavlink {}",
                info.crc_extra, rec["crc_extra"]
            ));
        }

        let frame = match Frame::from_bytes(&bytes) {
            Ok(f) => f,
            Err(e) => {
                failures.push(format!("{tag}: frame rejected: {e:?}"));
                continue;
            }
        };
        let hdr = Header {
            seq: rec["seq"].as_u64().unwrap() as u8,
            sysid: rec["sysid"].as_u64().unwrap() as u8,
            compid: rec["compid"].as_u64().unwrap() as u8,
        };
        if (frame.seq(), frame.sysid(), frame.compid(), frame.msgid())
            != (hdr.seq, hdr.sysid, hdr.compid, id)
        {
            failures.push(format!("{tag}: header mismatch {frame:?}"));
        }

        let msg = frame.decode().expect("known id decodes");
        for (name, got) in msg.fields() {
            if let Some(m) = mismatch(name, &got, &rec["fields"][name]) {
                failures.push(format!("{tag}: {m}"));
            }
        }

        let version = if rec["variant"] == "v1" {
            Version::V1
        } else {
            Version::V2
        };
        let mut again = encode(version, hdr, &msg).unwrap();
        if let Some(key) = rec["key"].as_str() {
            let key = SecretKey(hex(key).try_into().unwrap());
            let link_id = rec["link_id"].as_u64().unwrap() as u8;
            let ts = rec["timestamp"].as_u64().unwrap();
            signing::sign(&mut again, &key, link_id, ts).unwrap();
            if let Err(e) = signing::verify(&frame, &key) {
                failures.push(format!("{tag}: pymavlink signature rejected: {e:?}"));
            }
        }
        if again.as_bytes() != bytes.as_slice() {
            failures.push(format!(
                "{tag}: re-encoded bytes differ\n  rust      {}\n  pymavlink {}",
                hex::encode(again.as_bytes()),
                hex::encode(&bytes)
            ));
        }
    }

    let all: BTreeSet<u32> = message_infos().iter().map(|m| m.id).collect();
    let missing: Vec<_> = all.difference(&covered).collect();
    assert!(
        missing.is_empty(),
        "vectors do not cover message ids {missing:?}: regenerate with scripts/mavlink_vectors.py"
    );
    assert_eq!(
        count,
        header["header"]["records"].as_u64().unwrap() as usize
    );
    assert!(
        failures.is_empty(),
        "{} of {count} vectors failed:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
