# Vendored MAVLink message definitions

| | |
|---|---|
| Source | https://github.com/ArduPilot/mavlink (`message_definitions/v1.0/`) |
| Commit | `0d734a416f1836e0840c3bb9ab9e4daffc6330d1` (2026-10-01) |
| Why ArduPilot's fork | It is the dialect ArduPilot firmware is built from; `ardupilotmega.xml` there is authoritative. |
| Licence | See `COPYING` (copied from the same commit): generator LGPLv3 with an MIT exception for generated output. Licence posture is an open owner decision — see `docs/coordination/issues/`. |
| Local changes | None. Files are byte-identical to upstream. |

## Updating

1. Copy `message_definitions/v1.0/*.xml` and `COPYING` from a new upstream commit; update the table above.
2. `cargo test -p mavlink-codec` — the pymavlink vector test fails if any message lacks vectors.
3. Regenerate vectors with pymavlink (dev-only tool): `python3 -m venv .venv && .venv/bin/pip install pymavlink && .venv/bin/python scripts/mavlink_vectors.py`.
4. Review the diff of `tests/vectors/pymavlink.jsonl` and commit XML + vectors together.
