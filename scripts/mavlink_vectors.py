#!/usr/bin/env python3
"""Generate MAVLink cross-implementation test vectors with pymavlink (the reference).

pymavlink generates its Python dialect from OUR vendored XML (not its bundled copy), packs
deterministic pseudo-random messages, and decodes them again; the decoded field values and
the exact frame bytes are written as JSON Lines. `crates/mavlink-codec/tests/pymavlink_vectors.rs`
checks that the Rust codec parses every frame, decodes identical field values, and re-encodes
byte-for-byte identical frames (including signatures).

Usage (pymavlink is a dev-only tool, LGPLv3, never shipped):
    python3 -m venv .venv && .venv/bin/pip install pymavlink
    .venv/bin/python scripts/mavlink_vectors.py            # regenerate the committed vectors

Output is deterministic for a given seed, XML and pymavlink version.
"""
import argparse
import importlib.util
import json
import random
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFS = ROOT / "crates/mavlink-codec/definitions"
OUT = ROOT / "crates/mavlink-codec/tests/vectors/pymavlink.jsonl"

INT_RANGES = {
    "int8_t": (-2**7, 2**7 - 1), "uint8_t": (0, 2**8 - 1),
    "int16_t": (-2**15, 2**15 - 1), "uint16_t": (0, 2**16 - 1),
    "int32_t": (-2**31, 2**31 - 1), "uint32_t": (0, 2**32 - 1),
    "int64_t": (-2**63, 2**63 - 1), "uint64_t": (0, 2**64 - 1),
}


def extension_fields(root_xml: Path) -> dict:
    """{message name: set of extension field names}, following <include>s."""
    result, seen, todo = {}, set(), [root_xml.resolve()]
    while todo:
        path = todo.pop()
        if path in seen:
            continue
        seen.add(path)
        tree = ET.parse(path)
        for inc in tree.iter("include"):
            todo.append((path.parent / inc.text.strip()).resolve())
        for msg in tree.iter("message"):
            ext, names = False, set()
            for child in msg:
                if child.tag == "extensions":
                    ext = True
                elif child.tag == "field" and ext:
                    names.add(child.get("name"))
            result[msg.get("name")] = names
    return result


def target_offsets(root_xml: Path) -> dict:
    """{message name: [target_system_ofs|None, target_component_ofs|None]} from pymavlink's parser."""
    from pymavlink.generator import mavparse
    result, seen, todo = {}, set(), [root_xml.resolve()]
    while todo:
        path = todo.pop()
        if path in seen:
            continue
        seen.add(path)
        xml = mavparse.MAVXML(str(path), "2.0")
        todo.extend((path.parent / inc).resolve() for inc in xml.include)
        for m in xml.message:
            sys_ofs = m.target_system_ofs if m.message_flags & mavparse.FLAG_HAVE_TARGET_SYSTEM else None
            comp_ofs = m.target_component_ofs if m.message_flags & mavparse.FLAG_HAVE_TARGET_COMPONENT else None
            result[m.name] = [sys_ofs, comp_ofs]
    return result


def load_dialect(root_xml: Path, workdir: Path):
    from pymavlink.generator import mavgen
    out = workdir / "dialect"
    opts = mavgen.Opts(str(out), wire_protocol="2.0", language="Python3", validate=False)
    if not mavgen.mavgen(opts, [str(root_xml)]):
        sys.exit("mavgen failed")
    spec = importlib.util.spec_from_file_location("dialect", str(out) + ".py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


def random_scalar(rng, ftype, zero):
    if zero:
        return 0.0 if ftype in ("float", "double") else 0
    if ftype == "float":
        return f32(rng.uniform(-1e6, 1e6))
    if ftype == "double":
        return rng.uniform(-1e12, 1e12)
    lo, hi = INT_RANGES[ftype.replace("_mavlink_version", "")]
    return rng.randint(lo, hi)


def random_args(rng, cls, sparse, zero_fields):
    args = []
    for name, ftype in zip(cls.fieldnames, cls.fieldtypes):
        alen = cls.array_lengths[cls.ordered_fieldnames.index(name)]
        zero = name in zero_fields or (sparse and rng.random() < 0.5)
        if ftype == "char":
            n = alen or 1
            text = "" if zero else "".join(rng.choice("ABCXYZabcxyz0189 _-") for _ in range(rng.randint(1, n)))
            args.append(text.encode("ascii") if alen or text else b"\0")
        elif alen:
            args.append([random_scalar(rng, ftype, zero) for _ in range(alen)])
        else:
            args.append(random_scalar(rng, ftype, zero))
    return args


def field_values(msg, cls):
    """Decoded values keyed by name, in a JSON shape the Rust test understands."""
    out = {}
    for name, ftype in zip(cls.fieldnames, cls.fieldtypes):
        v = getattr(msg, "_" + name + "_raw", None) if ftype == "char" else None
        if v is None:
            v = getattr(msg, name)
        if ftype == "char":
            raw = v if isinstance(v, (bytes, bytearray)) else str(v).encode("latin-1")
            out[name] = {"chars": raw.rstrip(b"\0").hex()}
        elif isinstance(v, (bytes, bytearray)):
            out[name] = list(v)
        elif isinstance(v, (list, tuple)):
            out[name] = list(v)
        else:
            out[name] = v
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xml", default=str(DEFS / "ardupilotmega.xml"))
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--seed", type=int, default=20261010)
    a = ap.parse_args()

    import pymavlink
    rng = random.Random(a.seed)
    root_xml = Path(a.xml)
    exts = extension_fields(root_xml)
    targets = target_offsets(root_xml)
    with tempfile.TemporaryDirectory() as tmp:
        d = load_dialect(root_xml, Path(tmp))
        records = []
        for msgid in sorted(d.mavlink_map):
            cls = d.mavlink_map[msgid]
            ext = exts.get(cls.msgname, set())
            variants = [("v2", False, set(), False), ("v2sparse", True, set(ext), False)]
            if msgid < 256 and not ext:
                variants.append(("v1", False, set(), False))
            if msgid % 7 == 0:
                variants.append(("v2signed", False, set(), True))
            for variant, sparse, zero_fields, signed in variants:
                seq, sysid, compid = rng.randint(0, 255), rng.randint(1, 255), rng.randint(0, 255)
                mav = d.MAVLink(None, srcSystem=sysid, srcComponent=compid)
                mav.seq = seq
                rec = {"name": cls.msgname, "id": msgid, "crc_extra": cls.crc_extra, "variant": variant,
                       "target_ofs": targets[cls.msgname],
                       "seq": seq, "sysid": sysid, "compid": compid}
                if signed:
                    key = bytes(rng.randint(0, 255) for _ in range(32))
                    mav.signing.secret_key, mav.signing.sign_outgoing = key, True
                    mav.signing.link_id = rng.randint(0, 255)
                    mav.signing.timestamp = rng.randint(1, 2**48 - 2)
                    rec.update(key=key.hex(), link_id=mav.signing.link_id, timestamp=mav.signing.timestamp)
                msg = cls(*random_args(rng, cls, sparse, zero_fields))
                frame = msg.pack(mav, force_mavlink1=(variant == "v1"))
                rx = d.MAVLink(None)
                decoded = rx.decode(bytearray(frame))
                rec["frame"] = bytes(frame).hex()
                rec["fields"] = field_values(decoded, cls)
                records.append(rec)

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    header = {"generator": "scripts/mavlink_vectors.py", "pymavlink": pymavlink.__version__ if hasattr(pymavlink, "__version__") else "unknown",
              "xml": root_xml.name, "seed": a.seed, "messages": len(d.mavlink_map), "records": len(records)}
    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps({"header": header}) + "\n")
        for r in records:
            fh.write(json.dumps(r, separators=(",", ":")) + "\n")
    print(f"wrote {out.relative_to(ROOT)}: {len(records)} records, {len(d.mavlink_map)} messages, "
          f"{out.stat().st_size} bytes")


if __name__ == "__main__":
    main()
