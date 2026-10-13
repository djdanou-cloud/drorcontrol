//! The generated field layout agrees with the serializer.

use mavlink_codec::messages::GlobalPositionInt;
use mavlink_codec::{Message, message_fields, message_infos};

#[test]
fn layout_tiles_every_payload() {
    for info in message_infos() {
        let fields = message_fields(info.id);
        assert!(!fields.is_empty(), "{}", info.name);
        let mut end = 0;
        for f in fields {
            assert_eq!(f.offset, end, "{}.{} offset", info.name, f.name);
            let size = match f.ty {
                "uint8_t" | "int8_t" | "char" => 1,
                "uint16_t" | "int16_t" => 2,
                "uint32_t" | "int32_t" | "float" => 4,
                _ => 8,
            };
            end += size * usize::from(f.array_len.max(1));
            if !f.extension {
                assert!(end <= info.base_len, "{}.{}", info.name, f.name);
            }
        }
        assert_eq!(end, info.max_len, "{}", info.name);
    }
    assert!(message_fields(16_000_000).is_empty());
}

#[test]
fn offsets_match_serialized_bytes() {
    let m = Message::GlobalPositionInt(GlobalPositionInt {
        lat: -353_632_610,
        lon: 1_491_652_300,
        ..Default::default()
    });
    let mut buf = [0u8; 255];
    m.serialize(&mut buf);
    let at = |name: &str| {
        let f = message_fields(m.id())
            .iter()
            .find(|f| f.name == name)
            .unwrap();
        assert_eq!((f.ty, f.units), ("int32_t", Some("degE7")));
        i32::from_le_bytes(buf[f.offset..f.offset + 4].try_into().unwrap())
    };
    assert_eq!((at("lat"), at("lon")), (-353_632_610, 1_491_652_300));
}
