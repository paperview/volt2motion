"""Turn a KiCad motor driver design into simulator parameters.

Reads the .kicad_pcb (parts, nets, copper geometry, stackup) and, if present,
the .kicad_sch (symbol fields), then works out:

  * the bridge topology: which MOSFETs are high side and low side, which nets
    are the motor phases, the DC bus and ground
  * conduction resistance: MOSFETs (from the parts table), shunts and the
    copper between the bridge and the motor connector
  * switching time: MOSFET gate charge, gate resistors and gate driver strength
  * current sensing: shunt value, amplifier gain and ADC resolution
  * bus capacitance (with DC-bias derating) and the brake resistor
  * MOSFET cooling: copper area around the MOSFETs and how far heat spreads

Datasheet values that KiCad files do not hold (MOSFET resistance, gate
charge) come from a JSON parts table. Values that only the firmware knows
(PWM frequency, current limit) stay as they are in the base design.

Optional symbol fields the importer understands (add them in KiCad's symbol
properties):
  SIM_ROLE         shunt | bus_cap | brake_resistor | gate_resistor (overrides guessing)
  SIM_GAIN         current sense amplifier gain (V/V)
  SIM_ADC_BITS     ADC resolution on the MCU
  SIM_ADC_VREF     ADC reference voltage
  SIM_VDRIVE       gate drive voltage
  SIM_ISOURCE      gate driver peak source current (A)
  SIM_ISINK        gate driver peak sink current (A)
  SIM_DEADTIME_NS  dead time set in the gate driver or MCU
  SIM_DERATE       capacitance left at the bus voltage (MLCC DC bias), 0-1
  SIM_HEATSINK_RTH heatsink thermal resistance to air for the MOSFETs (K/W)
  Voltage          capacitor voltage rating (or put it in Value, e.g. "220u 63V")
"""
from __future__ import annotations

import heapq
import json
import math
import re
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from matplotlib.path import Path as Polygon

from . import sexpr as S
from .links import DOCS_URL
from .params import JointDesign

RHO_CU_20C = 1.72e-8     # ohm m
CU_TEMPCO = 0.00393      # per K
K_CU = 390.0             # W/m/K
CP_CU = 385.0 * 8960.0   # J/m^3/K
CP_FR4 = 1100.0 * 1850.0
FR4_THICKNESS_M = 1.6e-3
GROUND_NET = re.compile(r"^(P|A|D)?GND|^GND|^0$|^VSS", re.IGNORECASE)
PART_NUMBER_FIELDS = ("MPN", "Manufacturer Part Number", "MFR_PN", "Part Number", "PN")
# Pin names that mark a transistor's gate, drain and source. Exact names only, so
# an MCU pin such as GPIO, DAC or SWDIO is not mistaken for one.
TRANSISTOR_PINS = {"G": "G", "GATE": "G", "D": "D", "DRAIN": "D", "S": "S", "SOURCE": "S"}
MOSFET_FIELDS = ("vds_max_v", "id_max_a", "rds_on_25c_ohm", "rds_tempco_per_k", "qgs_c", "qgd_c",
                 "v_plateau_v", "rg_internal_ohm", "rth_jc_k_per_w")


# -- board model ------------------------------------------------------------------
@dataclass
class Pad:
    number: str
    net: str
    function: str     # "G", "D", "S" for transistors, else ""
    x: float          # mm, board coordinates
    y: float
    layers: list[str]


@dataclass
class Part:
    ref: str
    value: str
    footprint: str
    x: float
    y: float
    layer: str
    fields: dict[str, str] = field(default_factory=dict)
    pads: list[Pad] = field(default_factory=list)

    def nets(self) -> set[str]:
        return {p.net for p in self.pads if p.net}

    def field(self, *names: str) -> str | None:
        lower = {k.lower(): v for k, v in self.fields.items()}
        for n in names:
            v = lower.get(n.lower())
            if v not in (None, "", "~"):
                return v
        return None


@dataclass
class Zone:
    net: str
    layer: str
    pts: np.ndarray   # (N, 2) mm

    @property
    def area(self) -> float:
        x, y = self.pts[:, 0], self.pts[:, 1]
        return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


@dataclass
class Segment:
    net: str
    layer: str
    x1: float
    y1: float
    x2: float
    y2: float
    width: float


@dataclass
class Via:
    net: str
    x: float
    y: float


@dataclass
class Board:
    parts: dict[str, Part]
    zones: list[Zone]
    segments: list[Segment]
    vias: list[Via]
    copper_mm: dict[str, float]               # copper layer -> thickness in mm
    outline: tuple[float, float, float, float] | None  # xmin, ymin, xmax, ymax


# -- reading files ------------------------------------------------------------------
def _net_name(node: list | None, table: dict[str, str]) -> str:
    """Handles both (net 3 "NAME") and (net "NAME") and bare (net 3)."""
    if node is None:
        return ""
    atoms = [a for a in node[1:] if not isinstance(a, list)]
    if not atoms:
        return ""
    if len(atoms) >= 2:
        return atoms[1]
    a = atoms[0]
    return table.get(a, a) if a.lstrip("-").isdigit() else a


def _rotate(dx: float, dy: float, deg: float) -> tuple[float, float]:
    """KiCad rotation: degrees counterclockwise on screen, with y pointing down."""
    t = math.radians(deg)
    return dx * math.cos(t) + dy * math.sin(t), -dx * math.sin(t) + dy * math.cos(t)


def _fields(node: list) -> dict[str, str]:
    out = {}
    for p in S.children(node, "property"):
        if len(p) >= 3 and not isinstance(p[1], list) and not isinstance(p[2], list):
            out[p[1]] = p[2]
    for t in S.children(node, "fp_text"):   # KiCad 6 and 7 footprints
        if len(t) >= 3 and t[1] in ("reference", "value"):
            out[t[1].capitalize()] = t[2]
    return out


def read_pcb(path: str | Path) -> Board:
    root = S.parse(Path(path).read_text())
    if root[0] != "kicad_pcb":
        raise ValueError(f"{path} is not a KiCad PCB file")
    table = {a[1]: a[2] for a in S.children(root, "net") if len(a) >= 3}

    copper: dict[str, float] = {}
    setup = S.child(root, "setup")
    stackup = S.child(setup, "stackup") if setup else None
    if stackup:
        for layer in S.children(stackup, "layer"):
            if S.atom(S.child(layer, "type")) == "copper":
                copper[layer[1]] = float(S.atom(S.child(layer, "thickness"), default=0.035))
    if not copper:   # no stackup saved: assume a plain 2-layer, 1 oz board
        copper = {"F.Cu": 0.035, "B.Cu": 0.035}

    parts: dict[str, Part] = {}
    for fp in S.children(root, "footprint") + S.children(root, "module"):
        fx, fy, frot = S.floats(S.child(fp, "at"), 3)
        flds = _fields(fp)
        ref = flds.get("Reference", "?")
        part = Part(ref, flds.get("Value", ""), str(fp[1]), fx, fy,
                    S.atom(S.child(fp, "layer"), default="F.Cu"), flds)
        for pad in S.children(fp, "pad"):
            px, py, _ = S.floats(S.child(pad, "at"), 3)
            dx, dy = _rotate(px, py, frot)
            func = TRANSISTOR_PINS.get((S.atom(S.child(pad, "pinfunction"), default="") or "").strip().upper(), "")
            layers = [a for a in (S.child(pad, "layers") or [])[1:] if not isinstance(a, list)]
            part.pads.append(Pad(str(pad[1]), _net_name(S.child(pad, "net"), table), func,
                                 fx + dx, fy + dy, layers))
        parts[ref] = part

    zones = []
    for z in S.children(root, "zone"):
        net = S.atom(S.child(z, "net_name")) or _net_name(S.child(z, "net"), table)
        layers = [S.atom(S.child(z, "layer"))] if S.child(z, "layer") else \
            [a for a in (S.child(z, "layers") or [])[1:] if not isinstance(a, list)]
        polys = S.children(z, "filled_polygon") or S.children(z, "polygon")
        for poly in polys:
            layer = S.atom(S.child(poly, "layer")) or (layers[0] if layers else "")
            pts = np.array([S.floats(xy, 2) for xy in S.children(S.child(poly, "pts") or [], "xy")])
            if len(pts) >= 3:
                for lyr in ([layer] if S.child(poly, "layer") else layers):
                    zones.append(Zone(net, lyr, pts))

    segments = []
    for s in S.children(root, "segment"):
        x1, y1 = S.floats(S.child(s, "start"), 2)
        x2, y2 = S.floats(S.child(s, "end"), 2)
        segments.append(Segment(_net_name(S.child(s, "net"), table), S.atom(S.child(s, "layer"), default=""),
                                x1, y1, x2, y2, float(S.atom(S.child(s, "width"), default=0.25))))
    vias = []
    for v in S.children(root, "via"):
        x, y = S.floats(S.child(v, "at"), 2)
        vias.append(Via(_net_name(S.child(v, "net"), table), x, y))

    outline = None
    for r in S.children(root, "gr_rect"):
        if S.atom(S.child(r, "layer")) == "Edge.Cuts":
            (x1, y1), (x2, y2) = S.floats(S.child(r, "start"), 2), S.floats(S.child(r, "end"), 2)
            outline = (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))
    if outline is None:
        xs, ys = [], []
        for ln in S.children(root, "gr_line"):
            if S.atom(S.child(ln, "layer")) == "Edge.Cuts":
                for k in ("start", "end"):
                    x, y = S.floats(S.child(ln, k), 2)
                    xs.append(x)
                    ys.append(y)
        if xs:
            outline = (min(xs), min(ys), max(xs), max(ys))
    return Board(parts, zones, segments, vias, copper, outline)


def read_schematic_fields(path: str | Path) -> dict[str, dict[str, str]]:
    """Symbol fields by reference designator, from a .kicad_sch file."""
    root = S.parse(Path(path).read_text())
    out: dict[str, dict[str, str]] = {}
    for sym in S.children(root, "symbol"):   # top-level placed symbols only
        flds = _fields(sym)
        ref = flds.get("Reference")
        inst = S.child(sym, "instances")
        if inst:   # KiCad 7+: the annotated reference lives under instances
            for proj in S.children(inst, "project"):
                for p in S.children(proj, "path"):
                    ref = S.atom(S.child(p, "reference")) or ref
        if ref:
            out.setdefault(ref, {}).update(flds)
    return out


def load_project(path: str | Path) -> Board:
    """Load a board from a KiCad project folder or a .kicad_pcb file."""
    path = Path(path)
    if path.is_file():
        folder, pcbs = path.parent, [path]
    else:
        folder, pcbs = path, sorted(path.glob("*.kicad_pcb"))
    if not pcbs:
        raise FileNotFoundError(f"no .kicad_pcb file in {folder}")
    if len(pcbs) > 1:
        raise ValueError(f"{folder} has several boards ({', '.join(p.name for p in pcbs)}); pass the .kicad_pcb file")
    board = read_pcb(pcbs[0])
    for sch in sorted(folder.glob("*.kicad_sch")):
        for ref, flds in read_schematic_fields(sch).items():
            if ref in board.parts:
                board.parts[ref].fields.update(flds)
    return board


# -- value parsing ---------------------------------------------------------------------
_MULT = {"p": 1e-12, "n": 1e-9, "u": 1e-6, "µ": 1e-6, "m": 1e-3, "k": 1e3, "K": 1e3, "M": 1e6, "G": 1e9}


def parse_value(text: str) -> float | None:
    """Parse component values: '1m' = 1 milliohm, '4k7', '0R001', '220uF', '10R', '4.7u 100V'."""
    if not text:
        return None
    t = text.strip().split()[0].replace("Ω", "").replace("ohm", "").replace("Ohm", "")
    t = re.sub(r"(?i)meg", "M", t)
    t = re.sub(r"[FH]$", "", t)
    m = re.fullmatch(r"(\d*)([pnuµmkKMGR])(\d+)", t)   # letter as decimal point: 4k7, 0R001
    if m:
        whole, letter, frac = m.groups()
        return float(f"{whole or 0}.{frac}") * (1.0 if letter == "R" else _MULT[letter])
    m = re.fullmatch(r"(\d+\.?\d*|\.\d+)([pnuµmkKMGR]?)", t)
    if not m:
        return None
    num, letter = m.groups()
    return float(num) * (1.0 if letter in ("", "R") else _MULT[letter])


def parse_voltage(part: Part) -> float | None:
    v = part.field("Voltage", "SIM_VOLTAGE", "Rated Voltage")
    text = v or part.value
    m = re.search(r"(\d+\.?\d*)\s*V\b", text or "")
    return float(m.group(1)) if m else (parse_value(v) if v else None)


# -- analysis --------------------------------------------------------------------------
@dataclass
class Derived:
    key: str          # design parameter, e.g. "driver.rds_on"
    value: float
    base: float
    source: str
    kind: str         # "from board", "estimate" or "kept"


@dataclass
class ImportReport:
    derived: list[Derived] = field(default_factory=list)
    checks: list[tuple[str, bool, str]] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    topology: dict = field(default_factory=dict)


def _is_resistor(p: Part) -> bool:
    return p.ref.upper().startswith("R")


def _is_capacitor(p: Part) -> bool:
    return p.ref.upper().startswith("C")


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _trace_resistance(board: Board, net: str, start: tuple[float, float], end: tuple[float, float],
                      copper: dict[str, float]) -> float | None:
    """Lowest-resistance path along tracks of one net (Dijkstra), or None if not connected."""
    segs = [s for s in board.segments if s.net == net]
    if not segs:
        return None
    key = lambda x, y: (round(x, 2), round(y, 2))  # noqa: E731
    graph: dict[tuple, list[tuple[tuple, float]]] = {}
    for s in segs:
        t = copper.get(s.layer, 0.035) * 1e-3
        r = RHO_CU_20C * (math.hypot(s.x2 - s.x1, s.y2 - s.y1) * 1e-3) / (s.width * 1e-3 * t)
        a, b = key(s.x1, s.y1), key(s.x2, s.y2)
        graph.setdefault(a, []).append((b, r))
        graph.setdefault(b, []).append((a, r))
    for v in board.vias:   # vias join layers at the same point
        if v.net == net:
            graph.setdefault(key(v.x, v.y), [])
    nodes = list(graph)
    src = min(nodes, key=lambda n: _dist(n, start))
    dst = min(nodes, key=lambda n: _dist(n, end))
    if _dist(src, start) > 3.0 or _dist(dst, end) > 3.0:
        return None
    best = {src: 0.0}
    heap = [(0.0, src)]
    while heap:
        d, n = heapq.heappop(heap)
        if n == dst:
            return d
        if d > best.get(n, math.inf):
            continue
        for m, r in graph[n]:
            if d + r < best.get(m, math.inf):
                best[m] = d + r
                heapq.heappush(heap, (d + r, m))
    return None


def _zone_resistance(board: Board, net: str, start, end, copper: dict[str, float]) -> float | None:
    """Rough resistance of copper pours on one net: length over average width, layers in parallel.

    Layers only count in parallel when vias on the net join them.
    """
    zones = [z for z in board.zones if z.net == net]
    if not zones:
        return None
    length = max(_dist(start, end), 1.0) * 1e-3
    has_vias = sum(1 for v in board.vias if v.net == net) >= 2
    by_layer: dict[str, float] = {}
    for z in zones:
        by_layer[z.layer] = by_layer.get(z.layer, 0.0) + z.area
    if not has_vias:
        top = min(by_layer, key=lambda l: (l != "F.Cu", l))  # noqa: E741
        by_layer = {top: by_layer[top]}
    g = 0.0
    for layer, area_mm2 in by_layer.items():
        width = min(area_mm2 * 1e-6 / length, length * 4)
        t = copper.get(layer, 0.035) * 1e-3
        g += width * t / (RHO_CU_20C * length)
    return 1.0 / g if g > 0 else None


def _parallel(*rs: float | None) -> float | None:
    g = sum(1.0 / r for r in rs if r)
    return 1.0 / g if g else None


def analyze(
    board: Board,
    parts_table: dict,
    base: JointDesign,
    convection_h: float = 15.0,
    name: str = "imported_driver",
) -> tuple[JointDesign, ImportReport]:
    """Derive driver, bus and MOSFET thermal parameters from the board."""
    rep = ImportReport()
    mosfet_lib = parts_table.get("mosfets", {})
    changes: dict[str, float] = {}

    def derive(key: str, value: float, source: str, kind: str = "from board") -> None:
        sec, leaf = key.split(".")
        base_v = getattr(getattr(base, sec), leaf)
        changes[key] = value
        rep.derived.append(Derived(key, value, base_v, source, kind))

    def keep(key: str, why: str) -> None:
        sec, leaf = key.split(".")
        v = getattr(getattr(base, sec), leaf)
        rep.derived.append(Derived(key, v, v, why, "kept"))

    # Topology ------------------------------------------------------------------
    fets = {}
    for p in board.parts.values():
        funcs = {pad.function: pad.net for pad in p.pads if pad.function in ("G", "D", "S")}
        if {"G", "D", "S"} <= set(funcs):
            fets[p.ref] = funcs
    drains = Counter()
    sources = Counter()
    for f in fets.values():
        drains[f["D"]] += 1
        sources[f["S"]] += 1
    phases = sorted(n for n in drains if n in sources and n)
    high = sorted(r for r, f in fets.items() if f["S"] in phases)
    low = sorted(r for r, f in fets.items() if f["D"] in phases)
    bridge = high + low
    other_fets = sorted(r for r in fets if r not in bridge)
    bus_net = Counter(fets[r]["D"] for r in high).most_common(1)[0][0] if high else ""
    gnd_candidates = [n for p in board.parts.values() for n in p.nets() if GROUND_NET.match(n)]
    gnd_net = Counter(gnd_candidates).most_common(1)[0][0] if gnd_candidates else ""
    rep.topology = {"phases": phases, "high_side": high, "low_side": low, "bus_net": bus_net,
                    "ground_net": gnd_net, "other_transistors": other_fets}
    if not phases:
        rep.notes.append("No half-bridge found: the MOSFET symbols need pins named G, D and S (or GATE, "
                         f"DRAIN, SOURCE). Nothing was changed. See {DOCS_URL}/kicad-board.md")
        return base.updated({}, name=name), rep
    if len(phases) != 3:
        rep.notes.append(f"Found {len(phases)} phase(s). The simulator models a 3-phase motor.")

    # MOSFET specs ------------------------------------------------------------------
    specs = []
    for r in bridge:
        pn = board.parts[r].field(*PART_NUMBER_FIELDS) or board.parts[r].value
        spec = mosfet_lib.get(pn)
        if spec is None:
            rep.notes.append(f"{r}: part '{pn}' is not in the parts table, so its datasheet values are unknown. Add it to parts.json "
                             f"({DOCS_URL}/parts-table.md).")
        else:
            specs.append((r, pn, spec))
    part_numbers = sorted({pn for _, pn, _ in specs})
    fet = specs[0][2] if specs else None
    if len(part_numbers) > 1:
        rep.notes.append(f"Bridge uses several MOSFET types {part_numbers}; using {specs[0][1]} for all.")

    # Shunts and gate resistors ---------------------------------------------------------
    gate_nets = {fets[r]["G"] for r in bridge}
    low_sources = {fets[r]["S"] for r in low}
    shunts, gate_res, bus_caps, brake = [], [], [], []
    for p in board.parts.values():
        role = (p.field("SIM_ROLE") or "").lower()
        v = parse_value(p.value)
        nets = p.nets()
        if _is_resistor(p) or role in ("shunt", "gate_resistor", "brake_resistor"):
            if role == "shunt" or (not role and v is not None and v <= 0.05):
                where = "low side" if nets & low_sources else ("inline" if nets & set(phases) else "bus")
                shunts.append((p, v, where))
            elif role == "gate_resistor" or (not role and nets & gate_nets and v is not None and v < 1000):
                gate_res.append((p, v))
            elif role == "brake_resistor" or (
                not role and bus_net in nets and v is not None and 1 <= v <= 100
                and any(fets[o]["D"] in nets for o in other_fets)
            ):
                brake.append((p, v))
        elif _is_capacitor(p) or role == "bus_cap":
            if role == "bus_cap" or ({bus_net, gnd_net} <= nets):
                bus_caps.append(p)
    rep.topology.update(shunts=[s[0].ref for s in shunts], gate_resistors=[g[0].ref for g in gate_res],
                        bus_caps=[c.ref for c in bus_caps], brake_resistor=[b[0].ref for b in brake])

    # Conduction resistance -------------------------------------------------------------
    copper = board.copper_mm
    r_phase = []
    for ph in phases:
        bridge_pads = [(pd.x, pd.y) for r in bridge for pd in board.parts[r].pads if pd.net == ph]
        start = tuple(np.mean(bridge_pads, axis=0))
        conn = [(pd.x, pd.y) for p in board.parts.values() if p.ref not in fets and not _is_resistor(p)
                and not _is_capacitor(p) for pd in p.pads if pd.net == ph]
        if not conn:
            rep.notes.append(f"{ph}: no connector pad found, so its copper resistance is unknown.")
            continue
        end = max(conn, key=lambda c: _dist(c, start))
        r = _parallel(_trace_resistance(board, ph, start, end, copper),
                      _zone_resistance(board, ph, start, end, copper))
        if r is None:
            rep.notes.append(f"{ph}: no tracks or pours found on this net.")
        else:
            r_phase.append(r)
    r_cu = float(np.mean(r_phase)) if r_phase else 0.0
    r_shunt, shunt_share = 0.0, 0.0
    if shunts:
        r_shunt = float(np.mean([s[1] for s in shunts if s[1]]))
        where = shunts[0][2]
        # A low-side shunt only carries phase current while the low-side FET conducts.
        shunt_share = {"low side": 0.5, "inline": 1.0, "bus": 0.0}[where]
    if fet:
        rds25 = fet["rds_on_25c_ohm"]
        r_tot = rds25 + r_cu + shunt_share * r_shunt
        tempco = (rds25 * fet["rds_tempco_per_k"] + r_cu * CU_TEMPCO) / r_tot
        derive("driver.rds_on", r_tot,
               f"MOSFET {rds25 * 1e3:.2f} + copper {r_cu * 1e3:.2f} + shunt share "
               f"{shunt_share * r_shunt * 1e3:.2f} mOhm at 25 C")
        derive("driver.rds_tempco", tempco, "weighted MOSFET and copper temperature coefficients")
    else:
        keep("driver.rds_on", "no MOSFET datasheet values in the parts table")

    # Switching time -----------------------------------------------------------------------
    drv = next((p for p in board.parts.values() if p.field("SIM_VDRIVE", "SIM_ISOURCE", "SIM_ISINK")), None)
    vdrive = float(drv.field("SIM_VDRIVE")) if drv and drv.field("SIM_VDRIVE") else 10.0
    isrc = float(drv.field("SIM_ISOURCE")) if drv and drv.field("SIM_ISOURCE") else 1.0
    isnk = float(drv.field("SIM_ISINK")) if drv and drv.field("SIM_ISINK") else 2.0
    if drv is None:
        rep.notes.append("No gate driver fields (SIM_VDRIVE, SIM_ISOURCE, SIM_ISINK); assumed 10 V, 1 A, 2 A.")
    if fet:
        rg_vals = [g[1] for g in gate_res if g[1] is not None]
        rg = float(np.mean(rg_vals)) if rg_vals else 0.0
        r_gate = rg + fet["rg_internal_ohm"]
        q_sw = fet["qgs_c"] / 2 + fet["qgd_c"]   # charge moved while voltage and current cross
        vp = fet["v_plateau_v"]
        i_on = min(isrc, (vdrive - vp) / r_gate)
        i_off = min(isnk, vp / r_gate)
        derive("driver.switch_time", q_sw / i_on + q_sw / i_off,
               f"gate charge {q_sw * 1e9:.0f} nC, gate resistance {r_gate:.1f} Ohm, "
               f"{i_on:.2f} A on / {i_off:.2f} A off", "estimate")
    dt_part = next((p for p in board.parts.values() if p.field("SIM_DEADTIME_NS")), None)
    if dt_part:
        derive("driver.dead_time", float(dt_part.field("SIM_DEADTIME_NS")) * 1e-9, f"SIM_DEADTIME_NS on {dt_part.ref}")
    else:
        keep("driver.dead_time", "set in firmware or gate driver registers, not visible in KiCad")

    # Current sensing ----------------------------------------------------------------------
    amp = next((p for p in board.parts.values() if p.field("SIM_GAIN")), None)
    mcu = next((p for p in board.parts.values() if p.field("SIM_ADC_BITS")), None)
    i_range = None
    if shunts and amp and r_shunt > 0:
        gain = float(amp.field("SIM_GAIN"))
        bits = int(float(mcu.field("SIM_ADC_BITS"))) if mcu else 12
        vref = float(mcu.field("SIM_ADC_VREF")) if mcu and mcu.field("SIM_ADC_VREF") else 3.3
        if not mcu:
            rep.notes.append("No SIM_ADC_BITS field on the MCU; assumed a 12-bit ADC with a 3.3 V reference.")
        per_amp = r_shunt * gain                      # V per A at the ADC
        i_range = (vref / 2) / per_amp                # bidirectional, biased at mid-supply
        lsb = vref / 2**bits / per_amp
        derive("driver.current_sense_lsb", lsb, f"{r_shunt * 1e3:.2f} mOhm x gain {gain:g}, {bits}-bit ADC at {vref:g} V")
        derive("driver.current_sense_noise", lsb, "about one ADC count rms", "estimate")
    else:
        keep("driver.current_sense_lsb", "needs a shunt and a SIM_GAIN field on the amplifier")

    # Bus capacitance and brake --------------------------------------------------------------
    cap_total, cap_volts = 0.0, []
    for c in bus_caps:
        v = parse_value(c.value) or 0.0
        cap_total += v * float(c.field("SIM_DERATE") or 1.0)
        cv = parse_voltage(c)
        if cv:
            cap_volts.append((c.ref, cv))
    if bus_caps:
        derive("battery.bus_capacitance", cap_total,
               f"{len(bus_caps)} capacitors between {bus_net} and {gnd_net}, after SIM_DERATE")
    if brake and brake[0][1]:
        derive("battery.brake_resistance", brake[0][1], f"{brake[0][0].ref}")
    elif not brake:
        rep.notes.append("No brake resistor found. Regen must go back into the battery.")

    # MOSFET cooling --------------------------------------------------------------------------
    hs = next((p for p in board.parts.values() if p.field("SIM_HEATSINK_RTH")), None)
    if fet:
        r_board, c_board, a_eff = _board_cooling(board, bridge, convection_h)
        r_jc = fet["rth_jc_k_per_w"] / len(bridge)
        r_amb = r_board
        src = f"{a_eff * 1e4:.0f} cm^2 effective copper at h = {convection_h:g} W/m^2K"
        if hs:
            r_amb = 1.0 / (1.0 / r_board + 1.0 / float(hs.field("SIM_HEATSINK_RTH")))
            src += f", plus heatsink {hs.field('SIM_HEATSINK_RTH')} K/W"
        derive("thermal.r_fet_ambient", r_jc + r_amb, src, "estimate")
        derive("thermal.fet_capacity", c_board + 0.5 * len(bridge), "copper and FR4 within the spreading area", "estimate")
        rep.notes.append("MOSFET temperature is an average over the bridge. At stall one MOSFET can carry "
                         "peak current continuously and run hotter than this.")

    # Checks ------------------------------------------------------------------------------------
    ov = base.battery.overvoltage_v
    lim = base.driver.current_limit
    if fet:
        rep.checks.append(("MOSFET voltage rating >= 1.25x overvoltage trip", fet["vds_max_v"] >= 1.25 * ov,
                           f'{fet["vds_max_v"]:g} V vs {1.25 * ov:g} V'))
        rep.checks.append(("MOSFET current rating >= 2x current limit", fet["id_max_a"] >= 2 * lim,
                           f'{fet["id_max_a"]:g} A vs {2 * lim:g} A'))
    rep.checks.append(("Every bridge MOSFET has a datasheet entry", len(specs) == len(bridge),
                       f"{len(specs)} of {len(bridge)}"))
    rep.checks.append(("Every bridge MOSFET has a gate resistor", len(gate_res) >= len(bridge),
                       f"{len(gate_res)} of {len(bridge)}"))
    if i_range is not None:
        rep.checks.append(("Current sensing range >= 1.2x current limit", i_range >= 1.2 * lim,
                           f"+/-{i_range:.1f} A vs {1.2 * lim:.0f} A"))
    for ref, cv in cap_volts:
        rep.checks.append((f"{ref} voltage rating >= 1.2x overvoltage trip", cv >= 1.2 * ov, f"{cv:g} V vs {1.2 * ov:g} V"))
    if fet and r_cu:
        rep.checks.append(("Phase copper under half the MOSFET resistance", r_cu < 0.5 * fet["rds_on_25c_ohm"],
                           f"{r_cu * 1e3:.2f} vs {fet['rds_on_25c_ohm'] * 1e3:.2f} mOhm"))

    return base.updated(changes, name=name), rep


def _board_cooling(board: Board, bridge: list[str], h: float) -> tuple[float, float, float]:
    """Thermal resistance from the MOSFETs to air through the board, and its heat capacity.

    Heat spreads through all copper layers like a fin. Copper far from the
    MOSFETs counts less, weighted by exp(-r / Lc) where Lc = sqrt(k t / 2h) is
    the fin length scale for total copper thickness t. Both board faces convect.
    """
    if board.outline:
        x0, y0, x1, y1 = board.outline
    else:
        allpts = np.vstack([z.pts for z in board.zones]) if board.zones else np.zeros((1, 2))
        x0, y0 = allpts.min(axis=0)
        x1, y1 = allpts.max(axis=0)
    step = 0.5
    gx, gy = np.meshgrid(np.arange(x0 + step / 2, x1, step), np.arange(y0 + step / 2, y1, step))
    pts = np.column_stack([gx.ravel(), gy.ravel()])
    covered = np.zeros(len(pts), bool)
    layers_used = set()
    for z in board.zones:
        inside = Polygon(z.pts).contains_points(pts)
        if inside.any():
            layers_used.add(z.layer)
        covered |= inside
    t_total = sum(board.copper_mm.get(l, 0.035) for l in layers_used) * 1e-3 or 0.035e-3  # noqa: E741
    lc = math.sqrt(K_CU * t_total / (2 * h))
    fet_xy = np.array([[board.parts[r].x, board.parts[r].y] for r in bridge])
    dist = np.min(np.linalg.norm(pts[:, None, :] - fet_xy[None, :, :], axis=2), axis=1) * 1e-3
    weight = np.exp(-dist / lc) * covered
    cell = (step * 1e-3) ** 2
    a_eff = 2.0 * float(weight.sum()) * cell    # both faces
    r = 1.0 / (h * a_eff) if a_eff > 0 else 1e3
    c = float(weight.sum()) * cell * (t_total * CP_CU + FR4_THICKNESS_M * CP_FR4)
    return r, c, a_eff


EXAMPLE_PARTS = Path(__file__).parent / "data" / "parts.json"


def load_parts_table(path: str | Path = EXAMPLE_PARTS) -> dict:
    """Load a parts table and check every MOSFET entry is complete.

    The default is the bundled example (made-up parts).
    """
    try:
        table = json.loads(Path(path).read_text())
    except json.JSONDecodeError as e:
        raise ValueError(f"{path}: not valid JSON (line {e.lineno}, column {e.colno}: {e.msg})") from None
    for name, spec in table.get("mosfets", {}).items():
        if name.startswith("_"):
            continue
        missing = [k for k in MOSFET_FIELDS if not isinstance(spec.get(k), (int, float))]
        if missing:
            raise ValueError(f"{path}: MOSFET '{name}' is missing or has non-numeric {', '.join(missing)}")
        unknown = [k for k in spec if k not in MOSFET_FIELDS and not k.startswith("_")]
        if unknown:
            raise ValueError(f"{path}: MOSFET '{name}' has unknown field(s) {', '.join(unknown)}. "
                             f"Expected: {', '.join(MOSFET_FIELDS)}")
    table["mosfets"] = {k: v for k, v in table.get("mosfets", {}).items() if not k.startswith("_")}
    return table


def print_report(rep: ImportReport) -> None:
    t = rep.topology
    print("Board topology")
    print(f"  Phases: {', '.join(t.get('phases', []))}   bus: {t.get('bus_net')}   ground: {t.get('ground_net')}")
    print(f"  High side: {' '.join(t.get('high_side', []))}   low side: {' '.join(t.get('low_side', []))}")
    for k, label in (("shunts", "Shunts"), ("gate_resistors", "Gate resistors"),
                     ("bus_caps", "Bus capacitors"), ("brake_resistor", "Brake resistor"),
                     ("other_transistors", "Other transistors")):
        if t.get(k):
            print(f"  {label}: {' '.join(t[k])}")
    print("\nParameters from the board")
    w = max(len(d.key) for d in rep.derived)
    print(f"  {'parameter':<{w}}  {'base':>11}  {'board':>11}  kind        source")
    for d in rep.derived:
        print(f"  {d.key:<{w}}  {_fmt(d.base):>11}  {_fmt(d.value):>11}  {d.kind:<10}  {d.source}")
    if rep.checks:
        print("\nBoard checks")
        for name, ok, detail in rep.checks:
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    if rep.notes:
        print("\nNotes")
        for n in rep.notes:
            print(f"  - {n}")


def _fmt(v: float) -> str:
    if v == 0:
        return "0"
    return f"{v:.4g}" if 1e-3 <= abs(v) < 1e4 else f"{v:.3e}"
