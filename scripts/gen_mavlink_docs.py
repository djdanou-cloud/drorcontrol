#!/usr/bin/env python3
"""Generate the MAVLink reference (Markdown) from the vendored XML definitions.

    python3 scripts/gen_mavlink_docs.py OUT_DIR

Covers the `ardupilotmega` dialect and everything it includes, from
crates/mavlink-codec/definitions/ (the same files the codec is generated from). One page per
XML file (messages with field tables, enums, MAV_CMD parameters) plus an index with the pinned
upstream commit and how DrorControl handles each message today. The docs build runs this through
scripts/mkdocs_hooks.py; the output is not committed.
"""
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFS = ROOT / "crates/mavlink-codec/definitions"
ROOT_DIALECT = "ardupilotmega.xml"

# What the core does with a message today (keep in step with crates/vehicle and dc-core).
HANDLING = {
    "HEARTBEAT": "Received: vehicle detection, mode, armed state, link watchdog (CONNECTED/STALE/LOST). Sent: GCS heartbeat at 1 Hz.",
    "ATTITUDE": "Received into `VehicleState.attitude`; requested at 10 Hz.",
    "GLOBAL_POSITION_INT": "Received into `VehicleState.position`; requested at 5 Hz.",
    "VFR_HUD": "Received into `VehicleState.hud`; requested at 5 Hz.",
    "SYS_STATUS": "Received into `VehicleState.power`; requested at 2 Hz.",
    "GPS_RAW_INT": "Received into `VehicleState.gps`; requested at 2 Hz.",
    "STATUSTEXT": "Received and raised as `CoreEvent::StatusText`.",
    "PARAM_REQUEST_LIST": "Sent to start a parameter download.",
    "PARAM_REQUEST_READ": "Sent to re-request missing parameter indices.",
    "PARAM_VALUE": "Received during parameter download and as write read-back.",
    "PARAM_SET": "Parameter write with read-back verification (`protocols::params`, not yet exposed).",
    "COMMAND_LONG": "Sent for `MAV_CMD_SET_MESSAGE_INTERVAL` (stream rates) when a vehicle appears.",
}


def esc(text) -> str:
    """One-line, Markdown-safe text for tables and paragraphs."""
    if not text:
        return ""
    t = re.sub(r"\s+", " ", text).strip()
    t = t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    return re.sub(r"([|*\[\]_`\\])", r"\\\1", t)


def includes(name: str, seen=None) -> list:
    """`name` and its includes, depth-first, each once (includes first)."""
    seen = [] if seen is None else seen
    root = ET.parse(DEFS / name).getroot()
    for inc in root.findall("include"):
        if inc.text and inc.text.strip() not in seen:
            includes(inc.text.strip(), seen)
    if name not in seen:
        seen.append(name)
    return seen


def page_of(f: str) -> str:
    return f[:-4].lower() + ".md"


def generate(out: Path) -> dict:
    """Write the pages into `out`; returns {relative path: text}."""
    files = includes(ROOT_DIALECT)
    trees = {f: ET.parse(DEFS / f).getroot() for f in files}
    enum_home, msg_home = {}, {}
    for f in files:
        for e in trees[f].findall("enums/enum"):
            enum_home.setdefault(e.get("name"), f)
        for m in trees[f].findall("messages/message"):
            msg_home[m.get("name")] = f
    missing = sorted(set(HANDLING) - set(msg_home))
    if missing:
        sys.exit(f"HANDLING names unknown messages: {missing}")

    def enum_link(name, here):
        if not name or name not in enum_home:
            return esc(name or "")
        target = "" if enum_home[name] == here else page_of(enum_home[name])
        return f"[{esc(name)}]({target}#{name.lower()})"

    pages = {}
    for f in files:
        r = trees[f]
        msgs = sorted(r.findall("messages/message"), key=lambda m: int(m.get("id")))
        enums = r.findall("enums/enum")
        n = lambda k, w: f"{k} {w}" + ("" if k == 1 else "s")
        lines = [f"# {f}", "", f"Source: `crates/mavlink-codec/definitions/{f}`. "
                 f"{n(len(msgs), 'message')}, {n(len(enums), 'enum')}.", ""]
        incs = [i.text.strip() for i in r.findall("include") if i.text]
        if incs:
            lines += ["Includes: " + ", ".join(f"[{i}]({page_of(i)})" for i in incs) + ".", ""]
        if msgs:
            lines += ["## Messages", ""]
        for m in msgs:
            name, mid = m.get("name"), m.get("id")
            lines += [f"### {name} ({mid}) {{#{name.lower()}}}", ""]
            if m.find("deprecated") is not None:
                d = m.find("deprecated")
                lines += [f"!!! warning \"Deprecated\"\n    Since {esc(d.get('since'))}; replaced by {esc(d.get('replaced_by'))}. {esc(d.text)}", ""]
            if m.find("wip") is not None:
                lines += ["!!! note \"Work in progress\"\n    May change without notice.", ""]
            lines += [esc(m.findtext("description")), ""]
            if name in HANDLING:
                lines += [f"**DrorControl:** {HANDLING[name]}", ""]
            lines += ["| Field | Type | Units | Values | Description |", "|---|---|---|---|---|"]
            ext = False
            for c in m:
                if c.tag == "extensions":
                    ext = True
                if c.tag != "field":
                    continue
                fname = esc(c.get("name")) + (" *(ext)*" if ext else "")
                lines.append(f"| {fname} | `{c.get('type')}` | {esc(c.get('units'))} | "
                             f"{enum_link(c.get('enum'), f)} | {esc(c.text)} |")
            lines.append("")
        if enums:
            lines += ["## Enums", ""]
        for e in enums:
            name = e.get("name")
            heading = f"### {name} {{#{name.lower()}}}" if enum_home[name] == f else f"### {name} (extension)"
            lines += [heading, ""]
            if enum_home[name] != f:
                lines += [f"Adds entries to [{esc(name)}]({page_of(enum_home[name])}#{name.lower()}).", ""]
            if e.get("bitmask") == "true":
                lines += ["Bitmask.", ""]
            lines += [esc(e.findtext("description")), "", "| Value | Name | Description |", "|---|---|---|"]
            for en in e.findall("entry"):
                desc = esc(en.findtext("description"))
                params = [p for p in en.findall("param") if (p.text or "").strip() or p.get("label")]
                if params:
                    desc += "<br>" + "<br>".join(
                        f"param{p.get('index')}: {esc(p.get('label') or p.text)}"
                        + (f" ({esc(p.get('units'))})" if p.get("units") else "") for p in params)
                lines.append(f"| {en.get('value', '')} | {esc(en.get('name'))} | {desc} |")
            lines.append("")
        pages[page_of(f)] = "\n".join(lines) + "\n"

    upstream = (DEFS / "UPSTREAM.md").read_text(encoding="utf-8")
    commit = re.search(r"`([0-9a-f]{40})`", upstream)
    idx = ["# MAVLink reference", "",
           "Generated by `scripts/gen_mavlink_docs.py` from the vendored definitions "
           f"(ArduPilot/mavlink `{commit.group(1)[:7] if commit else '?'}`, see "
           "[`UPSTREAM.md`](https://github.com/djdanou-cloud/drorcontrol/blob/main/crates/mavlink-codec/definitions/UPSTREAM.md)). "
           "The generated Rust code is the source of truth for wire formats; this is the readable view.", "",
           f"Dialect `{ROOT_DIALECT[:-4]}` and its includes: "
           f"{sum(len(t.findall('messages/message')) for t in trees.values())} messages, "
           f"{len(enum_home)} enums.", "",
           "| File | Messages | Enums |", "|---|---|---|"]
    for f in files:
        idx.append(f"| [{f}]({page_of(f)}) | {len(trees[f].findall('messages/message'))} | "
                   f"{len(trees[f].findall('enums/enum'))} |")
    idx += ["", "## Handled by DrorControl today", "", "| Message | Handling |", "|---|---|"]
    for name, how in HANDLING.items():
        idx.append(f"| [{name}]({page_of(msg_home[name])}#{name.lower()}) | {how} |")
    pages["index.md"] = "\n".join(idx) + "\n"

    out.mkdir(parents=True, exist_ok=True)
    for rel, text in pages.items():
        p = out / rel
        if not p.exists() or p.read_text(encoding="utf-8") != text:  # unchanged files keep mtime
            p.write_text(text, encoding="utf-8", newline="\n")
    return pages


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    pages = generate(Path(sys.argv[1]))
    print(f"wrote {len(pages)} pages to {sys.argv[1]}")
