---
title: Licence confirmation for vendored MAVLink XML and pymavlink test tool
severity: S3
state: resolved
owner: -
reporter: mavlink-1
needs_human: yes
created: 2026-10-10T21:39:05Z
updated: 2026-10-10T21:39:05Z
---

## Description
T0-02 vendors `crates/mavlink-codec/definitions/*.xml` from ArduPilot/mavlink @ 0d734a4 (unmodified), with its `COPYING`: the generator is LGPLv3; generated output is MIT under a stated exception. Our generator is our own code (`crates/mavlink-gen`), not upstream mavgen, so whether that exception wording covers our generated code is a legal question. The XML is the de-facto MAVLink spec and is vendored the same way by QGroundControl, rust-mavlink and others.

pymavlink (LGPLv3) is used only as a dev-time reference tool to generate test vectors (`scripts/mavlink_vectors.py`); it is not a dependency of any shipped artefact. The generated vectors (`tests/vectors/pymavlink.jsonl`) are data produced from the XML.

## Evidence

## Proposed resolution
Owner/legal confirms (a) vendoring the XML in this repository and shipping code generated from it is acceptable under the intended product licence (docs/plan/README.md §5, open decision 1), and (b) dev-only use of pymavlink is acceptable. Nothing has been distributed; the vendoring is reversible.

## Updates
* 2026-10-12 (owner, ADR-0012 item 9): approved. The MAVLink XML and generated code stay in the repository; pymavlink is a dev/test tool. Resolved.
