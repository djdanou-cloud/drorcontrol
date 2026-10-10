#!/usr/bin/env python3
"""Generate Cursor / GitHub Copilot / Gemini variants of the drorcontrol-ui skill.

Single source of truth: .claude/skills/drorcontrol-ui/SKILL.md  (edit ONLY that file).
Usage:  python3 scripts/sync_skill.py          # write generated files
        python3 scripts/sync_skill.py --check  # exit 1 if any generated file is stale (CI)
"""
import os, pathlib, re, sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC = ROOT / ".claude/skills/drorcontrol-ui/SKILL.md"
DESC = "Architecture, UI/UX, and HMI standards for DrorControl (ArduPilot ground control station): design tokens, alert colors, telemetry ergonomics, safety interlocks, PFD, mission planner, parameters, calibration."
GLOBS = ["ui/**", "**/*.qml", "docs/design/**", "docs/plan/04-ux-design-system.md"]
NOTE = ("<!-- GENERATED from .claude/skills/drorcontrol-ui/SKILL.md by scripts/sync_skill.py "
        "- do not edit; edit the source and re-run the script. -->")

def body():
    t = SRC.read_text()
    assert t.startswith("---\n")
    return t.split("\n---\n", 1)[1].lstrip("\n")

def relink(text, target):
    """Rewrite links that are relative to SRC so they resolve from `target`."""
    def fix(m):
        link = m.group(2)
        if re.match(r"^(https?:|#|mailto:)", link): return m.group(0)
        path, _, frag = link.partition("#")
        abs_ = (SRC.parent / path).resolve()
        rel = os.path.relpath(abs_, target.parent.resolve()).replace(os.sep, "/")  # POSIX links on Windows too
        return f"{m.group(1)}({rel}{'#' + frag if frag else ''})"
    return re.sub(r"(\[[^\]]*\])\(([^)]+)\)", fix, text)

def cursor():
    t = ROOT / ".cursor/rules/drorcontrol-ui.mdc"
    head = f'---\ndescription: "{DESC}"\nglobs: {",".join(GLOBS)}\nalwaysApply: false\n---\n'
    return t, head + NOTE + "\n\n" + relink(body(), t)

def copilot_path():
    t = ROOT / ".github/instructions/drorcontrol-ui.instructions.md"
    head = f'---\napplyTo: "{",".join(GLOBS)}"\n---\n'
    return t, head + NOTE + "\n\n" + relink(body(), t)

def copilot_repo():
    t = ROOT / ".github/copilot-instructions.md"
    txt = f"""{NOTE}
# DrorControl — Copilot repository instructions

DrorControl: cross-platform (macOS-first) ArduPilot ground control station. Rust core + Qt 6/QML UI. Plan: `docs/plan/README.md`. Coordination protocol for agents and humans: `docs/coordination/README.md` (read first; claim paths, log progress, work on `main`, never force-push).

Hard rules (full UI spec: `.github/instructions/drorcontrol-ui.instructions.md`):
- Never use bare single-click buttons for Arm, Disarm-in-flight, RTL, Land, Kill, Motor test, Param reset, firmware flash. Use slide/hold + confirmation card; kill = 1.5 s hold + typed confirm; RTL = 0.8 s hold + <=5 s confirm. The core enforces this too.
- Status colors are reserved (warning #FF3B30, caution #FF9400, advisory #22D3EE, normal #2ECC71, guidance #E040FB, standby #757575). Never color-only; never decorative. Use design tokens, no literal colors.
- Telemetry numerals: tabular/monospace, fixed-width boxes, no layout shift. Alt/speed 1 dp, voltage 2 dp, lat/lon 7 dp, heading integer. One central unit formatter.
- UI pulls one telemetry snapshot per frame; never bind UI properties per MAVLink message. Keep HUD/GPU items isolated from form-heavy views.
- Touch targets >= 48x48 px (56 px gloved). Stale data is dimmed/invalidated, never shown as live.
- Mode lists, MAVLink IDs and parameter metadata come from pinned upstream (ArduPilot/MAVLink XML), not hand-typed.
- Never commit secrets. Ask a human before changing the command path, interlocks, failsafes, licences, or anything irreversible.
"""
    return t, txt

def gemini_body():
    t = ROOT / ".gemini/drorcontrol-ui.md"
    return t, NOTE + "\n\n" + relink(body(), t)

def gemini_root():
    t = ROOT / "GEMINI.md"
    txt = f"""{NOTE}
# DrorControl — Gemini context

Imported project context (Gemini CLI `@file` imports):

@./AGENTS.md
@./.gemini/drorcontrol-ui.md

Coordination protocol (claims, status, logs, conflicts): `docs/coordination/README.md`.
"""
    return t, txt

def main():
    outs = [cursor(), copilot_path(), copilot_repo(), gemini_body(), gemini_root()]
    stale = []
    for p, text in outs:
        if not text.endswith("\n"): text += "\n"
        if "--check" in sys.argv:
            if not p.exists() or p.read_text() != text: stale.append(p)
        else:
            p.parent.mkdir(parents=True, exist_ok=True); p.write_text(text); print("wrote", p.relative_to(ROOT))
    if stale:
        print("STALE (run scripts/sync_skill.py):", *[str(s.relative_to(ROOT)) for s in stale], sep="\n  "); sys.exit(1)
    if "--check" in sys.argv: print("generated skill files up to date")

if __name__ == "__main__": main()
