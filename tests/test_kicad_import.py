import json

import pytest

from volt2motion import JointDesign, sample_board
from volt2motion import sexpr as S
from volt2motion.kicad_import import (
    Board, Segment, _trace_resistance, analyze, load_parts_table, load_project, parse_value,
)

PARTS = load_parts_table()


@pytest.fixture(scope="module")
def sample(tmp_path_factory):
    folder = tmp_path_factory.mktemp("sample_driver")
    sample_board.write(folder)
    board = load_project(folder)
    design, report = analyze(board, PARTS, JointDesign(), name="sample")
    return board, design, report


def test_sexpr_handles_nesting_and_escapes():
    node = S.parse('(a (b "x \\"y\\" z") (c 1 2.5) d)')
    assert node[0] == "a"
    assert S.atom(S.child(node, "b")) == 'x "y" z'
    assert S.floats(S.child(node, "c"), 2) == [1.0, 2.5]
    with pytest.raises(ValueError):
        S.parse("(a (b)")


@pytest.mark.parametrize("text, value", [
    ("1m", 1e-3), ("4k7", 4700), ("0R001", 1e-3), ("220u 63V", 220e-6), ("10R", 10),
    ("4.7uF", 4.7e-6), ("100n", 1e-7), ("1M", 1e6), ("2meg", 2e6), ("47", 47),
])
def test_parse_value(text, value):
    assert parse_value(text) == pytest.approx(value)


def test_sample_topology(sample):
    _, _, rep = sample
    t = rep.topology
    assert t["phases"] == ["PHASE_A", "PHASE_B", "PHASE_C"]
    assert t["bus_net"] == "VBUS" and t["ground_net"] == "GND"
    assert t["high_side"] == ["Q1", "Q3", "Q5"] and t["low_side"] == ["Q2", "Q4", "Q6"]
    assert sorted(t["shunts"]) == ["R1", "R2", "R3"]
    assert len(t["gate_resistors"]) == 6
    assert t["brake_resistor"] == ["R10"]
    assert t["other_transistors"] == ["Q7"]


def test_sample_values(sample):
    _, d, _ = sample
    # 2 x 220 uF + 4 x 4.7 uF derated to 45 %
    assert d.battery.bus_capacitance == pytest.approx(440e-6 + 4 * 4.7e-6 * 0.45)
    # MOSFET 2.5 mOhm + half the 1 mOhm shunt + some copper
    assert 3.0e-3 + 0.1e-3 < d.driver.rds_on < 4.5e-3
    assert d.driver.dead_time == pytest.approx(250e-9)
    assert d.driver.current_sense_lsb == pytest.approx(3.3 / 4096 / (1e-3 * 20))
    assert d.battery.brake_resistance == 10
    assert 5 < d.thermal.r_fet_ambient < 40


def test_sample_flags_undersized_electrolytics(sample):
    _, _, rep = sample
    failed = {name for name, ok, _ in rep.checks if not ok}
    assert "C1 voltage rating >= 1.2x overvoltage trip" in failed
    assert "C3 voltage rating >= 1.2x overvoltage trip" not in failed


def test_airflow_lowers_mosfet_thermal_resistance(sample):
    board, still, _ = sample
    fan, _ = analyze(board, PARTS, JointDesign(), convection_h=60.0)
    assert fan.thermal.r_fet_ambient < 0.4 * still.thermal.r_fet_ambient


def test_unknown_mosfet_keeps_base_resistance(sample):
    board, _, _ = sample
    d, rep = analyze(board, {"mosfets": {}}, JointDesign())
    assert d.driver.rds_on == JointDesign().driver.rds_on
    assert any("not in the parts table" in n for n in rep.notes)


def test_trace_resistance_of_a_straight_track():
    board = Board({}, [], [Segment("N", "F.Cu", 0, 0, 10, 0, 1.0)], [], {"F.Cu": 0.035}, None)
    r = _trace_resistance(board, "N", (0, 0), (10, 0), board.copper_mm)
    assert r == pytest.approx(1.72e-8 * 0.01 / (0.001 * 35e-6))


def test_mcu_pins_are_not_mistaken_for_a_transistor(tmp_path):
    from volt2motion.kicad_import import read_pcb
    pcb = tmp_path / "b.kicad_pcb"
    pads = "".join(f'(pad "{i}" smd rect (at {i} 0) (size 1 1) (layers "F.Cu") (net {i} "N{i}") (pinfunction "{f}"))'
                   for i, f in enumerate(["GPIO1", "DAC", "SWDIO", "G"], start=1))
    pcb.write_text(f'(kicad_pcb (version 20240108) (footprint "QFN" (layer "F.Cu") (at 0 0) '
                   f'(property "Reference" "U1") {pads}))')
    funcs = [p.function for p in read_pcb(pcb).parts["U1"].pads]
    assert funcs == ["", "", "", "G"]


@pytest.mark.parametrize("entry, message", [
    ({"vds_max_v": 80}, "missing or has non-numeric id_max_a"),
    ({**{k: 1.0 for k in ("vds_max_v", "id_max_a", "rds_on_25c_ohm", "rds_tempco_per_k", "qgs_c", "qgd_c",
                         "v_plateau_v", "rg_internal_ohm", "rth_jc_k_per_w")}, "rdson": 1}, "unknown field"),
])
def test_incomplete_parts_entry_is_a_clear_error(tmp_path, entry, message):
    path = tmp_path / "parts.json"
    path.write_text(json.dumps({"mosfets": {"MY-FET": entry}}))
    with pytest.raises(ValueError, match=message):
        load_parts_table(path)
