"""Write a sample 3-phase motor driver as KiCad files, for trying out the importer.

The board: 6 MOSFETs in three half-bridges, a 1 mOhm low-side shunt per phase,
a current sense amplifier, a gate driver with 10 Ohm gate resistors, two 220 uF
electrolytics plus four 4.7 uF ceramics on the bus, a brake chopper, and power
pours on a 4-layer board with 2 oz outer copper.

It is only for trying the importer, not a working layout. The files open in
KiCad, but pours ignore clearances and only the nets the importer uses are drawn, so
KiCad's design rule check reports overlaps and unrouted signals.

  volt2motion init     writes it to sample_driver/ along with a starter design
"""
from __future__ import annotations

import uuid
from pathlib import Path

from .sexpr import quote as q

FET = "EXAMPLE-NFET-80V-2m5"
BRIDGE_X = {"A": 14.0, "B": 30.0, "C": 46.0}


def uid() -> str:
    return str(uuid.uuid4())


class Part:
    def __init__(self, ref, value, footprint, x, y, pads, mpn=None, sim=None, lib_id="Device:R"):
        self.ref, self.value, self.footprint = ref, value, footprint
        self.x, self.y, self.pads = x, y, pads
        self.mpn, self.sim, self.lib_id = mpn, sim or {}, lib_id


def pad(num, dx, dy, w, h, net, func="", tht=False):
    return dict(num=num, dx=dx, dy=dy, w=w, h=h, net=net, func=func, tht=tht)


def fet_pads(d, s, g):
    return [pad("4", -1.905, 2.2, 0.7, 1.0, g, "G"),
            pad("1", -0.635, 2.2, 0.7, 1.0, s, "S"),
            pad("2", 0.635, 2.2, 0.7, 1.0, s, "S"),
            pad("3", 1.905, 2.2, 0.7, 1.0, s, "S"),
            pad("5", 0.0, -1.0, 4.2, 3.2, d, "D")]


def build_parts() -> list[Part]:
    parts = [
        Part("J1", "PWR_IN", "Connector:Lug_2pin", 4, 4.5,
             [pad("1", 0, 0, 3.5, 3.5, "VBUS", tht=True), pad("2", 0, 8, 3.5, 3.5, "GND", tht=True)],
             lib_id="Connector:Conn_01x02"),
    ]
    n = 2
    for i, (ph, x) in enumerate(BRIDGE_X.items()):
        hi, lo = f"Q{2 * i + 1}", f"Q{2 * i + 2}"
        parts.append(Part(hi, FET, "Package_TO_SOT_SMD:TDSON-8-1", x, 6.0,
                          fet_pads("VBUS", f"PHASE_{ph}", f"GH_{ph}"), mpn=FET, lib_id="Transistor_FET:Q_NMOS"))
        parts.append(Part(lo, FET, "Package_TO_SOT_SMD:TDSON-8-1", x, 13.0,
                          fet_pads(f"PHASE_{ph}", f"SHUNT_{ph}", f"GL_{ph}"), mpn=FET, lib_id="Transistor_FET:Q_NMOS"))
        parts.append(Part(f"R{i + 1}", "1m", "Resistor_SMD:R_2512_6332Metric", x - 1, 19.0,
                          [pad("1", -2.9, 0, 1.6, 3.3, f"SHUNT_{ph}"), pad("2", 2.9, 0, 1.6, 3.3, "GND")],
                          mpn="EXAMPLE-SHUNT-1m-2512"))
        parts.append(Part(f"R{4 + 2 * i}", "10R", "Resistor_SMD:R_0603_1608Metric", x - 6, 8.2,
                          [pad("1", -0.8, 0, 0.8, 0.9, f"GH_{ph}_DRV"), pad("2", 0.8, 0, 0.8, 0.9, f"GH_{ph}")]))
        parts.append(Part(f"R{5 + 2 * i}", "10R", "Resistor_SMD:R_0603_1608Metric", x - 6, 15.2,
                          [pad("1", -0.8, 0, 0.8, 0.9, f"GL_{ph}_DRV"), pad("2", 0.8, 0, 0.8, 0.9, f"GL_{ph}")]))
        parts.append(Part(f"J{n}", f"MOTOR_{ph}", "Connector:Lug_1pin", x + 5, 39.0,
                          [pad("1", 0, 0, 4.0, 4.0, f"PHASE_{ph}", tht=True)], lib_id="Connector:Conn_01x01"))
        n += 1
    # Bus capacitors: electrolytics between bridges, ceramics near each high-side FET
    for ref, x in (("C1", 23.5), ("C2", 39.5)):
        parts.append(Part(ref, "220u 63V", "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm", x, 4.5,
                          [pad("1", 0, 0, 2.0, 2.0, "VBUS", tht=True), pad("2", 0, 5, 2.0, 2.0, "GND", tht=True)],
                          lib_id="Device:C_Polarized"))
    for ref, x in (("C3", 9.0), ("C4", 25.5), ("C5", 41.5), ("C6", 57.0)):
        parts.append(Part(ref, "4.7u", "Capacitor_SMD:C_1210_3225Metric", x, 5.5,
                          [pad("1", 0, 0, 2.7, 1.0, "VBUS"), pad("2", 0, 2.8, 2.7, 1.0, "GND")],
                          sim={"Voltage": "100V", "SIM_DERATE": "0.45"}, lib_id="Device:C"))
    # Brake chopper: resistor from the bus to a low-side switch
    parts.append(Part("R10", "10R", "Resistor_THT:R_Power_Axial", 56, 26,
                      [pad("1", -5, 0, 2.5, 2.5, "VBUS", tht=True), pad("2", 5, 0, 2.5, 2.5, "BRAKE", tht=True)],
                      sim={"SIM_ROLE": "brake_resistor"}))
    parts.append(Part("Q7", FET, "Package_TO_SOT_SMD:TDSON-8-1", 56, 33,
                      fet_pads("BRAKE", "GND", "G_BRK"), mpn=FET, lib_id="Transistor_FET:Q_NMOS"))
    # Control side
    parts.append(Part("U1", "STM32G474", "Package_QFP:LQFP-48", 10, 38,
                      [pad(str(k + 1), -3 + k, 0, 0.3, 1.2, net) for k, net in
                       enumerate(["+3V3", "GND", "CSA_A", "CSA_B", "CSA_C"])],
                      sim={"SIM_ADC_BITS": "12", "SIM_ADC_VREF": "3.3"}, lib_id="MCU_ST_STM32G4:STM32G474CETx"))
    parts.append(Part("U2", "CSA_x3", "Package_SO:TSSOP-16", 22, 30,
                      [pad(str(k + 1), -3 + k, 0, 0.3, 1.2, net) for k, net in
                       enumerate(["SHUNT_A", "SHUNT_B", "SHUNT_C", "CSA_A", "CSA_B", "CSA_C", "GND", "+3V3"])],
                      sim={"SIM_GAIN": "20"}, lib_id="Amplifier_Current:INA240A1"))
    parts.append(Part("U3", "GATE_DRV_3PH", "Package_QFN:QFN-40", 8, 24,
                      [pad(str(k + 1), -4 + k, 0, 0.3, 1.0, net) for k, net in
                       enumerate([f"G{s}_{p}_DRV" for p in "ABC" for s in "HL"] + ["GND", "VDRV"])],
                      sim={"SIM_VDRIVE": "12", "SIM_ISOURCE": "1.0", "SIM_ISINK": "2.0",
                           "SIM_DEADTIME_NS": "250"}, lib_id="Driver_FET:DRV8353"))
    return parts


def zones() -> list[tuple[str, str, list[tuple[float, float]]]]:
    out = []
    bus = [(1.5, 1.5), (60, 1.5), (60, 7.0), (1.5, 7.0)]
    for layer in ("F.Cu", "B.Cu"):
        out.append(("VBUS", layer, bus))
        for ph, x in BRIDGE_X.items():
            out.append((f"PHASE_{ph}", layer, [(x - 3, 7.7), (x + 7, 7.7), (x + 7, 42), (x + 3, 42),
                                               (x + 3, 13.8), (x - 3, 13.8)]))
    for ph, x in BRIDGE_X.items():
        out.append((f"SHUNT_{ph}", "F.Cu", [(x - 5, 14.7), (x + 1.2, 14.7), (x + 1.2, 16.2), (x - 2.5, 16.2),
                                            (x - 2.5, 20.6), (x - 5, 20.6)]))
    out.append(("GND", "In1.Cu", [(1, 1), (61, 1), (61, 44), (1, 44)]))
    return out


def vias() -> list[tuple[str, float, float]]:
    out = []
    for ph, x in BRIDGE_X.items():
        for dx in (-1.4, 0.0, 1.4):
            for dy in (-0.6, 0.6):
                out.append(("VBUS", x + dx, 5.0 + dy))      # under high-side drain tabs
                out.append((f"PHASE_{ph}", x + dx, 12.0 + dy))  # under low-side drain tabs
        out.append(("GND", x + 1.9, 21.5))
    return out


def write_pcb(parts: list[Part], path: Path) -> None:
    nets = [""] + sorted({p["net"] for part in parts for p in part.pads} | {z[0] for z in zones()})
    num = {n: i for i, n in enumerate(nets)}
    L = ['(kicad_pcb (version 20240108) (generator "make_sample_board") (generator_version "8.0")',
         '  (general (thickness 1.6) (legacy_teardrops no))', '  (paper "A4")',
         '  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal)'
         ' (44 "Edge.Cuts" user) (37 "F.SilkS" user "F.Silkscreen"))',
         '  (setup (stackup',
         '      (layer "F.Cu" (type "copper") (thickness 0.07))',
         '      (layer "dielectric 1" (type "prepreg") (thickness 0.2))',
         '      (layer "In1.Cu" (type "copper") (thickness 0.035))',
         '      (layer "dielectric 2" (type "core") (thickness 1.0))',
         '      (layer "In2.Cu" (type "copper") (thickness 0.035))',
         '      (layer "dielectric 3" (type "prepreg") (thickness 0.2))',
         '      (layer "B.Cu" (type "copper") (thickness 0.07)))',
         '    (pad_to_mask_clearance 0))']
    L += [f"  (net {i} {q(n)})" for i, n in enumerate(nets)]
    for part in parts:
        L.append(f'  (footprint {q(part.footprint)} (layer "F.Cu") (uuid {q(uid())}) (at {part.x} {part.y})')
        for key, val in (("Reference", part.ref), ("Value", part.value), ("Footprint", part.footprint)):
            L.append(f'    (property {q(key)} {q(val)} (at 0 0 0) (layer "F.SilkS") (uuid {q(uid())})'
                     ' (effects (font (size 1 1) (thickness 0.15))))')
        if part.mpn:
            L.append(f'    (property "MPN" {q(part.mpn)} (at 0 0 0) (layer "F.Fab") (hide yes) (uuid {q(uid())})'
                     ' (effects (font (size 1 1) (thickness 0.15))))')
        for p in part.pads:
            kind = 'thru_hole circle' if p["tht"] else 'smd rect'
            layers = '"*.Cu" "*.Mask"' if p["tht"] else '"F.Cu" "F.Paste" "F.Mask"'
            drill = f' (drill {min(p["w"], p["h"]) / 2:.2f})' if p["tht"] else ""
            func = f' (pinfunction {q(p["func"])}) (pintype "passive")' if p["func"] else ""
            L.append(f'    (pad {q(p["num"])} {kind} (at {p["dx"]} {p["dy"]}) (size {p["w"]} {p["h"]}){drill}'
                     f' (layers {layers}) (net {num[p["net"]]} {q(p["net"])}){func} (uuid {q(uid())}))')
        L.append("  )")
    for net, layer, pts in zones():
        xy = " ".join(f"(xy {x} {y})" for x, y in pts)
        L.append(f'  (zone (net {num[net]}) (net_name {q(net)}) (layer {q(layer)}) (uuid {q(uid())})'
                 ' (hatch edge 0.5) (connect_pads (clearance 0.3)) (min_thickness 0.25)'
                 f' (filled_areas_thickness no) (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))'
                 f' (polygon (pts {xy})))')
    for net, x, y in vias():
        L.append(f'  (via (at {x:.2f} {y:.2f}) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu")'
                 f' (net {num[net]}) (uuid {q(uid())}))')
    L.append(f'  (gr_rect (start 0 0) (end 62 45) (stroke (width 0.1) (type default)) (fill none)'
             f' (layer "Edge.Cuts") (uuid {q(uid())}))')
    L.append(")")
    path.write_text("\n".join(L) + "\n")


def write_schematic(parts: list[Part], path: Path) -> None:
    root = uid()
    L = [f'(kicad_sch (version 20231120) (generator "make_sample_board") (generator_version "8.0")'
         f' (uuid {q(root)}) (paper "A3")', "  (lib_symbols)"]
    for i, part in enumerate(parts):
        x, y = 25 + 20 * (i % 10), 25 + 20 * (i // 10)
        L.append(f'  (symbol (lib_id {q(part.lib_id)}) (at {x} {y} 0) (unit 1) (exclude_from_sim no)'
                 f' (in_bom yes) (on_board yes) (dnp no) (uuid {q(uid())})')
        props = {"Reference": part.ref, "Value": part.value, "Footprint": part.footprint}
        if part.mpn:
            props["MPN"] = part.mpn
        props.update(part.sim)
        for k, v in props.items():
            hide = "" if k in ("Reference", "Value") else " (hide yes)"
            L.append(f'    (property {q(k)} {q(v)} (at {x} {y} 0) (effects (font (size 1.27 1.27)){hide}))')
        L.append(f'    (instances (project "sample_driver" (path {q("/" + root)} (reference {q(part.ref)}) (unit 1))))')
        L.append("  )")
    L.append(f'  (sheet_instances (path "/" (page "1")))')
    L.append(")")
    path.write_text("\n".join(L) + "\n")


def write(folder: str | Path) -> tuple[Path, Path]:
    """Write sample_driver.kicad_pcb and sample_driver.kicad_sch into folder."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    parts = build_parts()
    pcb, sch = folder / "sample_driver.kicad_pcb", folder / "sample_driver.kicad_sch"
    write_pcb(parts, pcb)
    write_schematic(parts, sch)
    return pcb, sch
