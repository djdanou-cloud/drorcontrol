#!/usr/bin/env python3
"""Make a dataflash (.bin) test fixture and pymavlink DFReader reference values.

Takes a real ArduPilot log (e.g. from SITL with LOG_DISARMED=1), keeps the head up to
--max-bytes cut at a record boundary, and records what pymavlink's DFReader (the reference)
decodes from it: per-message counts, per-field count/min/max/first/last, and the full numeric
field values of sampled records. `crates/logs/tests/pymavlink_dataflash.rs` checks the Rust
parser against these.

Usage (pymavlink is dev-only, LGPLv3, never shipped):
    .venv/bin/python scripts/dataflash_vectors.py path/to/00000001.BIN --name sitl-copter-4.7.1

For a whole log that stays out of git (checked by `cargo test -p logs -- --ignored`):
    .venv/bin/python scripts/dataflash_vectors.py big.BIN --name big --max-bytes 0 --out ~/dc-corpus
    DC_LOG_CORPUS=~/dc-corpus cargo test -p logs -- --ignored
"""
import argparse
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "crates/logs/tests/data"


def head_at_record_boundary(data: bytes, max_bytes: int) -> bytes:
    """Walk records (lengths from FMT) and cut after the last one that fits."""
    lens = {128: 89}
    pos = 0
    while pos + 3 <= len(data):
        t = data[pos + 2]
        if data[pos:pos + 2] != b"\xa3\x95" or t not in lens:
            raise SystemExit(f"unexpected bytes at {pos}; use a clean log")
        if t == 128:
            lens[data[pos + 3]] = data[pos + 4]
        if pos + lens[t] > max_bytes:
            break
        pos += lens[t]
    return data[:pos]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--name", required=True)
    ap.add_argument("--max-bytes", type=int, default=900_000, help="0 = whole log")
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--sample-every", type=int, default=97)
    a = ap.parse_args()

    from pymavlink import DFReader
    import pymavlink

    a.out.mkdir(parents=True, exist_ok=True)
    data = Path(a.log).read_bytes()
    fixture = a.out / f"{a.name}-head.bin"
    fixture.write_bytes(head_at_record_boundary(data, a.max_bytes or len(data)))

    reader = DFReader.DFReader_binary(str(fixture), zero_time_base=False)
    counts, stats, samples = {}, {}, []
    while True:
        m = reader.recv_msg()
        if m is None:
            break
        name = m.fmt.name
        i = counts.get(name, 0)
        counts[name] = i + 1
        numeric = {}
        for k in m.fmt.columns:
            v = getattr(m, k, None)
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                continue
            if isinstance(v, float) and not math.isfinite(v):
                continue
            numeric[k] = v
            s = stats.setdefault(f"{name}.{k}", {"count": 0, "min": v, "max": v, "first": v, "last": v})
            s["count"] += 1
            s["min"] = min(s["min"], v)
            s["max"] = max(s["max"], v)
            s["last"] = v
        if i % a.sample_every == 0:
            samples.append({"msg": name, "index": i, "fields": numeric})

    ref = {
        "generator": "scripts/dataflash_vectors.py",
        "pymavlink": getattr(pymavlink, "__version__", "unknown"),
        "fixture": fixture.name,
        "bytes": fixture.stat().st_size,
        "counts": counts,
        "stats": stats,
        "samples": samples,
    }
    out = a.out / f"{a.name}-head.json"
    out.write_text(json.dumps(ref, separators=(",", ":")) + "\n", encoding="utf-8")
    print(f"wrote {fixture} ({fixture.stat().st_size} B) and {out} "
          f"({out.stat().st_size} B): {sum(counts.values())} records, {len(counts)} types, {len(samples)} samples")


if __name__ == "__main__":
    main()
