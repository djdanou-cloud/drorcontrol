#!/usr/bin/env python3
"""Inventory Mission Planner's user-visible features from its source and write parity.yaml.

Reads (never copies) the WinForms designer files and .resx string tables of an
ArduPilot/MissionPlanner checkout and enumerates: main navigation, Flight Data tabs and their
buttons, every context menu, Flight Plan controls, every Initial Setup and Config/Tuning page,
and the hidden Ctrl-F tools form. Each item becomes a parity row with a stable id, a primary
owner (workstream), a link to the plan's parity matrix (docs/plan/02-parity-matrix.md) when a
rule matches, and placeholders for test and handbook ids. Unmatched items get the area's
default row and `triage: true` for a human to decide. See docs/parity/README.md.

Usage:
    git clone --depth 1 https://github.com/ArduPilot/MissionPlanner /tmp/mp
    python3 scripts/mp_inventory.py /tmp/mp            # writes parity.yaml
    python3 scripts/mp_inventory.py /tmp/mp --check    # exit 1 if parity.yaml would change

Only labels and structure are recorded (as facts about the reference product); no Mission
Planner code is reproduced.
"""
import argparse
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "parity.yaml"
PLAN = ROOT / "docs/plan/02-parity-matrix.md"


# --- source readers ---------------------------------------------------------------------

def resx(path: Path) -> dict:
    if not path.exists():
        return {}
    out = {}
    for d in ET.parse(path).getroot().iter("data"):
        v = d.find("value")
        if v is not None and v.text is not None:
            out[d.get("name")] = v.text.strip()
    return out


class Designer:
    """Controls, their types, text and children from a WinForms *.Designer.cs + .resx."""

    def __init__(self, designer: Path, strings: dict):
        src = designer.read_text(encoding="utf-8-sig", errors="replace")
        self.res = resx(designer.with_name(designer.name.replace(".Designer.cs", ".resx")))
        self.strings = strings
        self.types = dict(re.findall(r"this\.(\w+)\s*=\s*new\s+([\w.]+)\([^()]*\)\s*;", src))
        self.text = {}
        for name, expr in re.findall(r"this\.(\w+)\.Text\s*=\s*([^;]+);", src):
            self.text[name] = self._expr(expr)
        self.children = {}
        for parent, child in re.findall(r"this\.(\w+)\.(?:Controls|Items|DropDownItems)\.Add\(this\.(\w+)\s*[,)]", src):
            self.children.setdefault(parent, []).append(child)
        for parent, body in re.findall(
            r"this\.(\w+)\.(?:Items|DropDownItems|Controls)\.AddRange\(new [\w.\[\]]+\s*\{([^}]*)\}", src
        ):
            self.children.setdefault(parent, []).extend(re.findall(r"this\.(\w+)", body))

    def _expr(self, expr: str):
        expr = expr.strip()
        m = re.fullmatch(r'"((?:[^"\\]|\\.)*)"', expr)
        if m:
            return m.group(1)
        m = re.fullmatch(r"(?:global::)?(?:MissionPlanner\.)?Strings\.(\w+)", expr)
        if m:
            return self.strings.get(m.group(1))
        return None

    def label(self, name: str):
        t = self.text.get(name) or self.res.get(f"{name}.Text") or self.res.get(f"{name}.ToolTipText")
        if t:
            t = re.sub(r"\s+", " ", t.replace("&&", "&").replace("&", "")).strip()
        # Ignore glyph-only labels (arrows, "...") - not features.
        return t if t and re.search(r"[A-Za-z0-9]", t) else None

    def walk(self, root: str, depth=0):
        """(depth, name, type, label) for root's subtree, in declaration order."""
        for c in self.children.get(root, []):
            yield depth, c, self.types.get(c, ""), self.label(c)
            yield from self.walk(c, depth + 1)


def is_button(t: str) -> bool:
    return t.endswith("Button") or t.endswith("MyButton") or t.endswith("CheckBox")


def is_menu_item(t: str) -> bool:
    return t.endswith("ToolStripMenuItem")


# --- plan mapping -----------------------------------------------------------------------

# (regex, plan id, owner[, areas]). Matched against "label | control/class name" in the order
# classify() describes, falling back to the full path (so "Telemetry Logs tab > 10x" -> its tab). First match wins; a
# rule with `areas` applies only there. Labels exclude the surface title ("Flight Plan" != "Plan").
RULES = [
    # Safety-critical commands first so nothing shadows them.
    (r"\bArm\b|Disarm|Safety Switch", "PAR-FD-010", "WS-SAFETY"),
    (r"Auto Analysis|loganalysis", "PAR-SA-005", "WS-LOGS"),
    # Flight Data actions are live vehicle commands, not mission items.
    (r"\bMode\b|Joystick Mode", "PAR-FD-012", "WS-UI", {"flight-data"}),
    (r"Actions|Loiter|\bRTL\b|\bLand\b|Take ?off|\bAuto\b|Jump To Tag|Restart Mission|Set WP|"
     r"Change (Speed|Alt)|Resume|Set EKF Origin|Set Home Alt", "PAR-FD-013", "WS-UI", {"flight-data"}),
    # Firmware / bootloader
    (r"\bDFU\b|Bootloader|Install Firmware|ConfigFirmware|CubeID|APJ Tool|rip all fw|driver clean", "PAR-IS-001", "WS-PLAT"),
    # Setup and calibration
    (r"Frame Type|ConfigFrame|TradHeli|Heli Setup", "PAR-IS-002", "WS-UI"),
    (r"Accel ?Cal|ConfigAccelerometer", "PAR-IS-003", "WS-UI"),
    (r"CompassMot|Compass/Motor", "PAR-IS-004", "WS-CORE"),
    (r"\bCompass|ConfigHWCompass|MagCal|mag calb", "PAR-IS-004", "WS-UI"),
    (r"Radio Calibration|ConfigRadioInput", "PAR-IS-005", "WS-UI"),
    (r"Flight Modes|ConfigFlightModes", "PAR-IS-006", "WS-UI"),
    (r"FailSafe|ConfigFailSafe", "PAR-IS-007", "WS-SAFETY"),
    (r"Servo Output|ConfigServo|ConfigRadioOutput", "PAR-IS-008", "WS-UI"),
    (r"Motor Test|ConfigMotorTest|ESC Calibration|ConfigESCCalibration", "PAR-IS-009", "WS-SAFETY"),
    (r"Battery Monitor|ConfigBatteryMonitoring", "PAR-IS-010", "WS-UI"),
    (r"Range ?Finder|Sonar|Airspeed|Optical ?Flow|OptFlow|PX4Flow|GPS Order|ConfigHWRangeFinder|ConfigHWAirspeed", "PAR-IS-011", "WS-UI"),
    (r"\bRTK\b|GPS ?Inject|ConfigSerialInjectGPS|NTRIP|\bNMEA\b|Moving Base|gps_inject", "PAR-IS-012", "WS-CORE"),
    (r"Sik ?radio|\bSiK\b", "PAR-IS-013", "WS-PLAT"),
    (r"Antenna|Tracker", "PAR-IS-014", "WS-CORE"),
    (r"Onboard OSD|ConfigOSD", "PAR-CT-009", "WS-UI"),
    (r"Gimbal|ConfigMount|\bOSD\b|Parachute|Bluetooth|ConfigHWBT|ESP8266|DroneCAN|UAVCAN|\bCAN\b|Serial Ports|ConfigSerial\b",
     "PAR-IS-015", "WS-UI", {"initial-setup", "config-tuning", "tools"}),
    (r"Joystick", "PAR-IS-016", "WS-PLAT"),
    (r"Terminal|\bREPL\b|Shell|Console|reboot", "PAR-IS-017", "WS-UI"),
    (r"HW ?ID|Hardware ID|decode HWIDs|\bSecure\b", "PAR-IS-018", "WS-UI"),
    (r"Prearm|PreArm", "PAR-IS-019", "WS-UI"),
    (r"Pre-?Flight|Checklist", "PAR-FD-016", "WS-UI"),
    # Parameters and tuning
    (r"Full Parameter Tree|ConfigRawParamsTree", "PAR-CT-002", "WS-UI"),
    (r"Full Parameter List|ConfigRawParams", "PAR-CT-001", "WS-UI"),
    (r"Standard Params|Advanced Params|ConfigFriendlyParams", "PAR-CT-003", "WS-UI"),
    (r"\bCompare\b", "PAR-CT-005", "WS-UI"),
    (r"Param (Restore|gen)|Initial (Tune )?Param|ConfigInitialParams", "PAR-CT-004", "WS-UI"),
    (r"Basic Tuning|ConfigSimplePids|ConfigArduplane|ConfigArdurover", "PAR-CT-006", "WS-UI"),
    (r"Extended Tuning|ConfigArducopter|Ateryx", "PAR-CT-007", "WS-UI"),
    (r"\bFFT\b|ConfigFFT", "PAR-SA-006", "WS-LOGS"),
    (r"User Params|ConfigUserDefined", "PAR-CT-010", "WS-UI"),
    (r"MAV ?FTP|MavFTPUI", "PAR-CT-011", "WS-CORE"),
    (r"^Planner$|ConfigPlanner|Lang Edit", "PAR-CT-013", "WS-UI"),
    # Map view (before mission rules: "Zoom to Home" is a view action, not a home edit)
    (r"\bZoom\b|Custom Maps?\b|Rotate Map|Measure|Prefetch|\bUTM\b|Age Map|3D Map|Inject GE|Docking|Clear Track", "PAR-FD-003", "WS-GFX"),
    # Logs, video, comms
    (r"Matlab|\bgpx\b|Kml or Graph|Convert \.Bin|bintolog|log2kml|dflogtokml", "PAR-SA-008", "WS-LOGS"),
    (r"Telemetry Logs|\bTlog|Playback|logindex|loadtelem", "PAR-SA-004", "WS-LOGS"),
    (r"Remote df logger|Download.*Log|logdownload", "PAR-SA-002", "WS-LOGS"),
    (r"DataFlash|DFLog|Log Browse|Review a Log|Anon Log|map logs|DashWare|ReSort All logs", "PAR-SA-003", "WS-LOGS"),
    (r"Geo ?Ref|Geotag", "PAR-SA-007", "WS-LOGS"),
    (r"Video|GStreamer|MJPEG|Record Hud|Stop Record|\bvlc\b|Aspect Ratio", "PAR-SA-009", "WS-VIDEO"),
    (r"ADSB|Transponder", "PAR-FD-020", "WS-CORE"),
    (r"Follow ?Me", "PAR-FD-018", "WS-CORE"),
    (r"Inspector|Mavlink|packet bytes|Message Interval|Lockup MAV", "PAR-CON-008", "WS-UI"),
    (r"MAVSerial pass", "PAR-CON-009", "WS-CORE"),
    (r"Signing", "PAR-CON-004", "WS-CORE"),
    # Mission planning
    (r"Geo-?Fence|ConfigAC_Fence|\bFence\b", "PAR-FP-006", "WS-GEO"),
    (r"\bRally\b", "PAR-FP-007", "WS-GEO"),
    (r"Survey|\bGrid\b|\bArea\b", "PAR-FP-008", "WS-GEO"),
    (r"Spline", "PAR-FP-003", "WS-GEO"),
    (r"Terrain|Elevation|\bSRTM\b|Verify Height|\bDEM\b|GDAL", "PAR-FP-005", "WS-GEO"),
    (r"Polygon|From SHP|Shp to Poly|\bCircle\b|Auto WP|Create.*WP|\bText\b|\bPOI\b", "PAR-FP-011", "WS-GEO"),
    (r"\bKML\b|Shape ?file|\bSHP\b|\b(Load|Save)\b", "PAR-FP-012", "WS-UI"),
    (r"\bHome\b", "PAR-FP-013", "WS-UI"),
    (r"\bRead\b|\bWrite\b|Upload|Clear Mission|Write Fast", "PAR-FP-002", "WS-CORE"),
    (r"\bDO_\w+|Loiter|\bJump\b|\bDelay\b|\bLand\b|Take ?off|\bRTL\b", "PAR-FP-004", "WS-UI"),
    (r"Delete WP|Insert WP|Add Below|Waypoint|Modify Alt|Command List|Reverse WPs", "PAR-FP-001", "WS-UI"),
    # Flight data
    (r"Fly To|Go Here|Guided|Point Camera|\bROI\b|Orbit|Follow the leader", "PAR-FD-014", "WS-UI"),
    (r"QNH|baro height", "PAR-FD-002", "WS-CORE"),
    (r"Relay|Servo|Aux Function", "PAR-FD-015", "WS-UI"),
    (r"Messages", "PAR-FD-006", "WS-UI"),
    (r"\bQuick\b", "PAR-FD-004", "WS-UI"),
    (r"Gauges", "PAR-FD-005", "WS-UI"),
    (r"\bStatus\b|Raw Sensor", "PAR-FD-007", "WS-UI"),
    (r"Payload|Camera|Gimbal|\bMount\b", "PAR-FD-019", "WS-VIDEO"),
    (r"Scripts?\b", "PAR-FD-021", "WS-CORE"),
    (r"Warning Manager|Speech|Voice", "PAR-FD-009", "WS-UI"),
    (r"Proximity", "PAR-FD-001", "WS-GFX"),
    (r"\bHUD\b", "PAR-FD-001", "WS-GFX"),
    (r"\bMap\b|Flight Planner|Coords", "PAR-FD-003", "WS-GFX"),
    (r"Simulation|\bSITL\b|Multirotor|\bPlane\b|\bRover\b|\bHeli\b", "PAR-SA-001", "WS-QA"),
]


# Suggested plan row when no rule matches (always flagged for triage).
AREA_DEFAULT = {
    "navigation": ("PAR-XC-007", "WS-UI"),
    "flight-data": ("PAR-FD-001", "WS-UI"),
    "flight-plan": ("PAR-FP-001", "WS-UI"),
    "initial-setup": ("PAR-IS-015", "WS-UI"),
    "config-tuning": ("PAR-CT-001", "WS-UI"),
    "tools": (None, "WS-ARCH"),
    "simulation": ("PAR-SA-001", "WS-QA"),
}

# Friendlier names for container controls that appear in paths.
CONTAINERS = {
    "contextMenuStripMap": "Map menu",
    "contextMenuStripHud": "HUD menu",
    "contextMenuStripactionstab": "Actions-tab menu",
    "contextMenuStripQuickView": "Quick-view menu",
    "contextMenuStrip1": "Map menu",
    "contextMenuStripPoly": "Polygon menu",
    "contextMenuStripZoom": "Zoom menu",
}


# Product-owner decisions on rows no rule could map, or that a decision overrides
# (2026-10-11, on the T0-11 triage list). row id -> (plan id, owner, status, basis).
OWNER = "owner 2026-10-11"
NAV = "navigation entry; contents covered by their own rows"
SWARM = "swarm control deferred to after 1.0"
DECISIONS = {
    "PAR-MP-NAV-MenuFlightData": ("PAR-XC-007", "WS-UI", "not-started", f"{OWNER}: Fly workspace ({NAV})"),
    "PAR-MP-NAV-MenuFlightPlanner": ("PAR-XC-007", "WS-UI", "not-started", f"{OWNER}: Plan workspace ({NAV})"),
    "PAR-MP-NAV-MenuInitConfig": ("PAR-XC-007", "WS-UI", "not-started", f"{OWNER}: Setup workspace ({NAV})"),
    "PAR-MP-NAV-MenuConfigTune": ("PAR-XC-007", "WS-UI", "not-started", f"{OWNER}: Tune workspace ({NAV})"),
    "PAR-MP-NAV-MenuHelp": ("PAR-XC-008", "WS-DOCS", "not-started", f"{OWNER}: in-app help and about"),
    "PAR-MP-TOOL-CoT": ("PAR-CON-010", "WS-CORE", "not-started", f"{OWNER}: CoT/TAK output in scope"),
    "PAR-MP-TOOL-grab-threads-txt": ("PAR-CT-011", "WS-CORE", "not-started", f"{OWNER}: fetched with the MAVFTP browser"),
    "PAR-MP-TOOL-MicroDrone": (None, "WS-ARCH", "wont", f"{OWNER}: out of scope, ADR-0008"),
    "PAR-MP-TOOL-structtest": (None, "WS-ARCH", "wont", f"{OWNER}: out of scope, ADR-0008"),
    "PAR-MP-TOOL-Swarm": ("PAR-FD-024", "WS-CORE", "deferred", f"{OWNER}: {SWARM}"),
    "PAR-MP-TOOL-Follow-Swarm": ("PAR-FD-024", "WS-CORE", "deferred", f"{OWNER}: {SWARM}"),
    "PAR-MP-TOOL-Sequence-Swarm": ("PAR-FD-024", "WS-CORE", "deferred", f"{OWNER}: {SWARM}"),
    "PAR-MP-TOOL-Follow-the-leader": ("PAR-FD-024", "WS-CORE", "deferred", f"{OWNER}: {SWARM} (Swarm FollowPathControl)"),
    "PAR-MP-SIM-Copter-Swarm-Single-link": ("PAR-FD-024", "WS-QA", "deferred", f"{OWNER}: {SWARM}"),
    "PAR-MP-SIM-Copter-Swarm-Multilink": ("PAR-FD-024", "WS-QA", "deferred", f"{OWNER}: {SWARM}"),
    "PAR-MP-SIM-Rover-Swarm-Multilink": ("PAR-FD-024", "WS-QA", "deferred", f"{OWNER}: {SWARM}"),
    "PAR-MP-SIM-Plane-Swarm-Multilink": ("PAR-FD-024", "WS-QA", "deferred", f"{OWNER}: {SWARM}"),
    "PAR-MP-SIM-Wipe": ("PAR-SA-001", "WS-QA", "not-started", f"{OWNER}: SITL launch option"),
}
FROZEN = "2026-10-11"  # product owner signed off the triage list; new triage rows need a decision


def plan_ids() -> set:
    return set(re.findall(r"^\|\s*([A-Z]+-\d{3})\s*\|", PLAN.read_text(encoding="utf-8"), re.M))


def classify(area: str, labels: str, key: str):
    """(plan id, owner, triage). Rules see only the item's own labels and control/class name."""
    # Submenu items ("Rally Points > Upload") take their meaning from the parent; buttons on a
    # tab or in a flat menu ("Actions tab > Joystick") from their own label.
    ctx = [p for p in labels.split(" > ") if not p.endswith((" tab", " menu"))]
    hays = [f"{labels.rsplit(' > ', 1)[-1]} | {key}", f"{labels} | {key}"]
    if len(ctx) > 1:
        hays.insert(0, f"{' > '.join(ctx)} | {key}")
    for hay in hays:
        for rx, pid, owner, *areas in RULES:
            if (not areas or area in areas[0]) and re.search(rx, hay, re.I):
                return pid, owner, False
    pid, owner = AREA_DEFAULT[area]
    return pid, owner, True


# --- extraction -------------------------------------------------------------------------

def slug(s: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "-", s).strip("-")[:60]


def page_label(expr: str, res: dict, strings: dict):
    """Resolve a page title expression: "lit" + Strings.X + rm.GetString("Y"), in order."""
    parts = []
    for lit, skey, rkey in re.findall(r'"((?:[^"\\]|\\.)*)"|Strings\.(\w+)|GetString\("([^"]+)"\)', expr):
        parts.append(lit or (strings.get(skey, skey) if skey else res.get(rkey, rkey)))
    return re.sub(r"\s+", " ", "".join(parts)).strip() or None


def extract(mp: Path):
    strings = resx(mp / "Strings.resx") or resx(next(iter(mp.rglob("Strings.resx")), Path("/nonexistent")))
    items = []  # (area, path, labels, key, source); path = surface title > labels

    def add(area, title, labels, key, source):
        items.append((area, f"{title} > {labels}", labels, key, source))

    # Main navigation.
    main = Designer(mp / "MainV2.Designer.cs", strings)
    for name in ["MenuFlightData", "MenuFlightPlanner", "MenuInitConfig", "MenuConfigTune", "MenuSimulation", "MenuHelp"]:
        add("navigation", "Main menu", main.label(name) or name, name, "MainV2.Designer.cs")

    # Flight Data: tabs, their buttons, context menus.
    src = "GCSViews/FlightData.Designer.cs"
    fd = Designer(mp / src, strings)
    for _, tab, t, label in fd.walk("tabControlactions"):
        if t.endswith("TabPage") and tab in fd.children.get("tabControlactions", []):
            tab_label = label or tab
            add("flight-data", "Flight Data", f"{tab_label} tab", tab, src)
            for _, c, ct, cl in fd.walk(tab):
                if is_button(ct) and cl:
                    add("flight-data", "Flight Data", f"{tab_label} tab > {cl}", c, src)
    for root, t in fd.types.items():
        if t.endswith("ContextMenuStrip"):
            for _, c, ct, cl in fd.walk(root):
                if is_menu_item(ct) and cl:
                    add("flight-data", "Flight Data", f"{CONTAINERS.get(root, root)} > {cl}", c, src)

    # Flight Plan: context menus and buttons.
    src = "GCSViews/FlightPlanner.Designer.cs"
    fp = Designer(mp / src, strings)
    for root, t in fp.types.items():
        if t.endswith("ContextMenuStrip"):
            stack = []
            for depth, c, ct, cl in fp.walk(root):
                if not is_menu_item(ct) or not cl:
                    continue
                stack = stack[:depth] + [cl]
                add("flight-plan", "Flight Plan", " > ".join([CONTAINERS.get(root, root)] + stack), c, src)
    for name, t in fp.types.items():
        label = fp.label(name)
        if is_button(t) and label:
            add("flight-plan", "Flight Plan", label, name, src)

    # Initial Setup and Config/Tuning pages.
    for view, area, title in [("InitialSetup", "initial-setup", "Initial Setup"), ("SoftwareConfig", "config-tuning", "Config/Tuning")]:
        text = (mp / f"GCSViews/{view}.cs").read_text(encoding="utf-8-sig", errors="replace")
        res = resx(mp / f"GCSViews/{view}.resx")
        for cls, label_expr in re.findall(r"AddBackstageViewPage\(typeof\(([\w.]+)\),\s*([^,\n)]+(?:\)[^,\n)]*)?)", text):
            label = page_label(label_expr, res, strings) or cls
            if cls in ("ConfigParamLoading", "ConfigMandatory", "ConfigOptional", "ConfigAdvanced", "ConfigFirmwareDisabled"):
                continue
            add(area, title, label, cls, f"GCSViews/{view}.cs")

    # Ctrl-F tools form.
    tmp = Designer(mp / "temp.Designer.cs", strings)
    for name, t in tmp.types.items():
        label = tmp.label(name)
        if is_button(t) and label:
            add("tools", "Ctrl-F Tools", label, name, "temp.Designer.cs")

    # Simulation view.
    sitl = mp / "GCSViews/SITL.Designer.cs"
    if sitl.exists():
        sd = Designer(sitl, strings)
        for name, t in sd.types.items():
            label = sd.label(name)
            if (is_button(t) or t.endswith("PictureBox")) and label:
                add("simulation", "Simulation", label, name, "GCSViews/SITL.Designer.cs")

    # De-duplicate by (area, path).
    seen, out = set(), []
    for it in items:
        if (it[0], it[1]) not in seen:
            seen.add((it[0], it[1]))
            out.append(it)
    return out


def yaml_str(s) -> str:
    if s is None:
        return "null"
    return '"' + str(s).replace("\\", "\\\\").replace('"', '\\"') + '"'


def render(mp: Path, items) -> str:
    commit = subprocess.run(["git", "-C", str(mp), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    date = subprocess.run(["git", "-C", str(mp), "log", "-1", "--format=%cs"], capture_output=True, text=True).stdout.strip()
    known = plan_ids()
    area_code = {"navigation": "NAV", "flight-data": "FD", "flight-plan": "FP", "initial-setup": "IS",
                 "config-tuning": "CT", "tools": "TOOL", "simulation": "SIM"}
    lines = [
        "# DrorControl parity checklist vs. ArduPilot Mission Planner.",
        "# GENERATED by scripts/mp_inventory.py from the Mission Planner source (labels and",
        "# structure only). Do not hand-edit: change the rules and re-run. See docs/parity/README.md.",
        "#",
        "# plan: matching row of docs/plan/02-parity-matrix.md (null = out of scope). basis: rule, or",
        "# the product-owner decision (DECISIONS in the script). triage: true = no rule or decision;",
        "# plan and owner are the area default and need a human decision.",
        "# status: not-started | in-progress | implemented | tested | documented | deferred (after 1.0)",
        "#   | wont (waiver ADR)",
        "version: 1",
        "source:",
        "  repo: ArduPilot/MissionPlanner",
        f"  commit: {commit}",
        f"  date: {date}",
        f"frozen: {FROZEN}",
        f"rows: {len(items)}",
        "items:",
    ]
    ids = set()
    triage = 0
    for area, path, labels, key, src in items:
        base = f"PAR-MP-{area_code[area]}-{slug(key if area in ('initial-setup', 'config-tuning', 'navigation') else labels)}"
        rid, n = base, 2
        while rid in ids:
            rid, n = f"{base}-{n}", n + 1
        ids.add(rid)
        pid, owner, tri = classify(area, labels, key)
        status, basis = "not-started", "rule" if not tri else "area default"
        if rid in DECISIONS:
            pid, owner, status, basis = DECISIONS[rid]
            tri = False
        if pid and pid.removeprefix("PAR-") not in known:
            raise SystemExit(f"{rid} maps to unknown plan id {pid}")
        triage += tri
        lines += [
            f"  - id: {rid}",
            f"    area: {area}",
            f"    mp: {yaml_str(path)}",
            f"    source: {yaml_str(src + ': ' + key)}",
            f"    plan: {pid or 'null'}",
            f"    owner: {owner}",
            f"    basis: {yaml_str(basis)}",
            f"    triage: {'true' if tri else 'false'}",
            f"    status: {status}",
            f"    test: T-{rid[4:]}",
            "    handbook: TBD",
        ]
    missing = set(DECISIONS) - ids
    if missing:
        raise SystemExit(f"decisions for rows that no longer exist: {sorted(missing)}")
    lines.insert(next(i for i, l in enumerate(lines) if l.startswith("rows: ")) + 1, f"triage: {triage}")
    return "\n".join(lines) + "\n"


def uncovered(text: str) -> list:
    """Plan rows with no Mission Planner item mapped to them (DrorControl-only or missed)."""
    used = set(re.findall(r"^    plan: PAR-(\S+)$", text, re.M))
    return sorted(plan_ids() - used)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mission_planner")
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    mp = Path(a.mission_planner)
    text = render(mp, extract(mp))
    if a.check:
        if not OUT.exists() or OUT.read_text(encoding="utf-8") != text:
            sys.exit("parity.yaml is out of date: run scripts/mp_inventory.py")
        print("parity.yaml up to date")
        return
    OUT.write_text(text, encoding="utf-8", newline="\n")
    rows = text.count("\n  - id:")
    print(f"wrote parity.yaml: {rows} rows, {text.split(chr(10) + 'triage: ')[1].split(chr(10))[0]} need triage")
    print("plan rows with no Mission Planner item:", " ".join(uncovered(text)))


if __name__ == "__main__":
    main()
