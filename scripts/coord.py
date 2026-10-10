#!/usr/bin/env python3
"""DrorControl agent-coordination helper (stdlib only). See docs/coordination/README.md.

Commands:
  register <agent> <workstream> [role]      create/refresh your status file
  status <agent> <state> [--task T] [--note TEXT]   update state: active|idle|blocked|done
  log <agent> <type> <subject> [--to X] [--ref R] [--body TEXT]   append a message
  claim <agent> <task> <path,...> [--hours N]      claim paths (fails on overlap)
  release <task>                                   remove a claim
  issue <agent> <severity S1-S4> <title> [--human] open an issue
  board                                            regenerate docs/coordination/BOARD.md
  check                                            detect overlaps / expired claims / stale agents
Every write command prints the files it touched; commit and push them yourself.
"""
import argparse, datetime as dt, pathlib, re, sys

ROOT = pathlib.Path(__file__).resolve().parent.parent / "docs" / "coordination"
STATE = {"active", "idle", "blocked", "done"}
TYPES = {"progress", "handoff", "question", "decision-request", "blocker", "fyi", "conflict", "done"}
STALE_H = 24

def now(): return dt.datetime.now(dt.timezone.utc).replace(microsecond=0)
def iso(t): return t.strftime("%Y-%m-%dT%H:%M:%SZ")
def parse_ts(s): return dt.datetime.strptime(s, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=dt.timezone.utc)
def slug(s): return re.sub(r"[^a-z0-9]+", "-", s.lower()).strip("-")[:48] or "x"

def read(p):
    t = p.read_text()
    meta, body = {}, t
    if t.startswith("---\n"):
        head, _, body = t[4:].partition("\n---\n")
        for line in head.splitlines():
            if ":" in line:
                k, _, v = line.partition(":"); meta[k.strip()] = v.strip()
    return meta, body

def write(p, meta, body=""):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text("---\n" + "".join(f"{k}: {v}\n" for k, v in meta.items()) + "---\n" + body.rstrip() + "\n")
    print("wrote", p.relative_to(ROOT.parent.parent))

def files(sub): return sorted(p for p in (ROOT / sub).glob("*.md") if not p.name.startswith("_"))

def cmd_register(a):
    p = ROOT / "status" / f"{slug(a.agent)}.md"
    old = read(p)[1] if p.exists() else "\n## Current focus\n(none)\n\n## Done\n\n## Blockers\n(none)\n"
    write(p, {"agent": a.agent, "role": a.role, "workstream": a.workstream, "state": "idle",
              "task": "-", "updated": iso(now())}, old)

def cmd_status(a):
    if a.state not in STATE: sys.exit(f"state must be one of {sorted(STATE)}")
    p = ROOT / "status" / f"{slug(a.agent)}.md"
    if not p.exists(): sys.exit("not registered; run register first")
    m, b = read(p)
    m.update(state=a.state, updated=iso(now()))
    if a.task: m["task"] = a.task
    if a.note: b = b.rstrip() + f"\n\n- {iso(now())} — {a.note}\n"
    write(p, m, b)

def cmd_log(a):
    if a.type not in TYPES: sys.exit(f"type must be one of {sorted(TYPES)}")
    t = now()
    p = ROOT / "log" / f"{t.strftime('%Y%m%dT%H%M%SZ')}-{slug(a.agent)}-{slug(a.subject)}.md"
    write(p, {"from": a.agent, "to": a.to, "type": a.type, "subject": a.subject, "ref": a.ref, "created": iso(t)}, a.body)

def overlap(x, y):
    x, y = x.rstrip("*/"), y.rstrip("*/")
    return x.startswith(y) or y.startswith(x)

def cmd_claim(a):
    paths = [s.strip() for s in a.paths.split(",") if s.strip()]
    for p in files("claims"):
        m, _ = read(p)
        if parse_ts(m["expires"]) < now() or (m["owner"] == a.agent and m["task"] == a.task): continue  # expired, or renewing own claim
        for mine in paths:
            for theirs in m["paths"].split(","):
                if overlap(mine, theirs.strip()):
                    sys.exit(f"CONFLICT: '{mine}' overlaps '{theirs.strip()}' claimed by {m['owner']} (task {m['task']}, expires {m['expires']}). Log a 'question' or pick another path.")
    t = now()
    write(ROOT / "claims" / f"{slug(a.task)}.md", {"owner": a.agent, "task": a.task, "paths": ", ".join(paths),
          "claimed": iso(t), "expires": iso(t + dt.timedelta(hours=a.hours))})

def cmd_release(a):
    p = ROOT / "claims" / f"{slug(a.task)}.md"
    if p.exists(): p.unlink(); print("removed", p.name)
    else: print("no such claim")

def cmd_issue(a):
    t = now()
    p = ROOT / "issues" / f"{t.strftime('%Y%m%dT%H%M%SZ')}-{slug(a.title)}.md"
    write(p, {"title": a.title, "severity": a.severity, "state": "open", "owner": "-", "reporter": a.agent,
              "needs_human": "yes" if a.human else "no", "created": iso(t), "updated": iso(t)},
          "\n## Description\n\n## Evidence\n\n## Proposed resolution\n\n## Updates\n")

def stale(m): return parse_ts(m["updated"]) < now() - dt.timedelta(hours=STALE_H)

def cmd_check(a):
    bad = 0
    cl = [(p, read(p)[0]) for p in files("claims")]
    for i, (p, m) in enumerate(cl):
        if parse_ts(m["expires"]) < now(): print("EXPIRED claim:", p.name, "owner", m["owner"]); bad += 1
        for q, n in cl[i + 1:]:
            if any(overlap(x.strip(), y.strip()) for x in m["paths"].split(",") for y in n["paths"].split(",")):
                print("OVERLAP:", p.name, "<->", q.name); bad += 1
    for p in files("status"):
        m, _ = read(p)
        if m["state"] in ("active", "blocked") and stale(m): print(f"STALE agent: {m['agent']} (last update {m['updated']})"); bad += 1
    print("check:", "issues found" if bad else "ok"); return 1 if bad else 0

def cmd_board(a):
    out = ["# Coordination Board", "", "> **Generated by `python3 scripts/coord.py board` — do not edit by hand.** "
           "On a merge conflict in this file, run the command again and `git add` the result.",
           f"> Generated: {iso(now())}", "", "## Agents", "", "| Agent | Role | Workstream | State | Task | Updated |", "|---|---|---|---|---|---|"]
    for p in files("status"):
        m, _ = read(p); flag = " ⚠ stale" if m["state"] in ("active", "blocked") and stale(m) else ""
        out.append(f"| {m['agent']} | {m['role']} | {m['workstream']} | {m['state']}{flag} | {m['task']} | {m['updated']} |")
    out += ["", "## Active claims", "", "| Task | Owner | Paths | Expires |", "|---|---|---|---|"]
    for p in files("claims"):
        m, _ = read(p); out.append(f"| {m['task']} | {m['owner']} | `{m['paths']}` | {m['expires']}{' (expired)' if parse_ts(m['expires']) < now() else ''} |")
    out += ["", "## Open issues", "", "| Severity | Title | State | Owner | Human needed | File |", "|---|---|---|---|---|---|"]
    iss = [(p, read(p)[0]) for p in files("issues")]
    iss = [x for x in iss if x[1]["state"] not in ("resolved", "wontfix")]
    for p, m in sorted(iss, key=lambda x: x[1]["severity"]):
        out.append(f"| {m['severity']} | {m['title']} | {m['state']} | {m['owner']} | {m['needs_human']} | [{p.name}](issues/{p.name}) |")
    out += ["", "## Recent messages (latest 15)", "", "| When | From → To | Type | Subject |", "|---|---|---|---|"]
    for p in files("log")[-15:][::-1]:
        m, _ = read(p); out.append(f"| {m['created']} | {m['from']} → {m['to']} | {m['type']} | [{m['subject']}](log/{p.name}) |")
    (ROOT / "BOARD.md").write_text("\n".join(out) + "\n"); print("wrote docs/coordination/BOARD.md")

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    s = ap.add_subparsers(dest="c", required=True)
    def add(n, f, *args, **kw):
        sp = s.add_parser(n); sp.set_defaults(f=f)
        for x in args: sp.add_argument(x)
        for k, v in kw.items(): sp.add_argument("--" + k, **v)
        return sp
    add("register", cmd_register, "agent", "workstream").add_argument("role", nargs="?", default="workstream-agent")
    add("status", cmd_status, "agent", "state", task={"default": ""}, note={"default": ""})
    add("log", cmd_log, "agent", "type", "subject", to={"default": "all"}, ref={"default": "-"}, body={"default": ""})
    cl = add("claim", cmd_claim, "agent", "task", "paths"); cl.add_argument("--hours", type=int, default=24)
    add("release", cmd_release, "task")
    add("issue", cmd_issue, "agent", "severity", "title").add_argument("--human", action="store_true")
    add("board", cmd_board); add("check", cmd_check)
    a = ap.parse_args(); sys.exit(a.f(a) or 0)

if __name__ == "__main__": main()
