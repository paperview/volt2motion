import math
import warnings

import numpy as np
import pytest

from volt2motion import JointDesign, calibration_routine, pick_and_place, simulate, summarize
from volt2motion import calibrate as C
from volt2motion import virtual_hardware as V
from volt2motion.actuator import Actuator
from volt2motion.power import PowerBus
from volt2motion.thermal import ThermalModel

warnings.filterwarnings("ignore")


def make_actuator(design: JointDesign) -> Actuator:
    bus = PowerBus(design.battery)
    th = ThermalModel(design.thermal)
    return Actuator(design, bus, th, np.random.default_rng(0))


@pytest.mark.parametrize("fidelity", ["fast", "full"])
def test_stall_torque_matches_torque_constant(fidelity):
    d = JointDesign()
    act = make_actuator(d)
    for _ in range(50):   # let the current loop settle
        tau = act.step(10.0, 0.0, 0.0, 1e-3, fidelity)
    expected = 1.5 * d.motor.pole_pairs * act.flux() * 10.0
    assert tau == pytest.approx(expected, rel=0.02)


def test_full_and_fast_agree_on_duty_cycle():
    d = JointDesign()
    tr = pick_and_place(cycles=2)
    full = summarize(simulate(d, tr, fidelity="full"), d)
    fast = summarize(simulate(d, tr, fidelity="fast"), d)
    assert fast["avg_power_w"] == pytest.approx(full["avg_power_w"], rel=0.05)
    assert fast["iq_rms_a"] == pytest.approx(full["iq_rms_a"], rel=0.05)
    assert fast["t_winding_steady_c"] == pytest.approx(full["t_winding_steady_c"], abs=2.0)


@pytest.mark.parametrize("fidelity", ["fast", "full"])
def test_back_emf_limits_current_at_high_speed(fidelity):
    d = JointDesign()
    act = make_actuator(d)
    vmax = d.driver.max_modulation * act.bus.vbus / math.sqrt(3)
    omega = 1.02 * vmax / act.flux()  / d.motor.pole_pairs   # just past no-load speed
    for _ in range(20):
        act.step(20.0, 0.0, omega, 1e-3, fidelity)
    assert act.v_sat > 0.5
    assert act.iq < 1.0


def test_regen_into_supply_that_cannot_sink_uses_brake():
    d = JointDesign().updated({"battery.accepts_regen": False, "arm.payload_mass": 15.0,
                               "gearbox.rated_peak_torque": 1e9})
    # Fast lowering of a heavy payload pushes energy back into the bus.
    log = simulate(d, pick_and_place(q_pick=1.2, q_place=-0.6, move_time=0.4, cycles=1), fidelity="full")
    m = summarize(log, d)
    assert m["vbus_max_v"] > d.battery.brake_off_v
    assert m["brake_energy_j"] > 0.0


def test_design_round_trip(tmp_path):
    d = JointDesign().updated({"gearbox.ratio": 80, "motor.kt_curve_current": [0, 40],
                               "motor.kt_curve_scale": [1, 0.9]}, name="test")
    path = tmp_path / "d.json"
    d.save(path)
    loaded = JointDesign.load(path)
    assert loaded == d
    with pytest.raises(KeyError):
        d.updated({"gearbox.not_a_parameter": 1})


def test_calibration_recovers_parameters_without_unmodeled_effects():
    d = JointDesign()
    truth = d.updated({
        "motor.resistance": d.motor.resistance * 1.2,
        "motor.flux_linkage": d.motor.flux_linkage * 0.9,
        "gearbox.coulomb": 3.5,
        "gearbox.viscous": 2.2,
        "gearbox.efficiency": 0.68,
    })
    log = simulate(truth, calibration_routine(40), firmware=d, fidelity="full", seed=3)
    elec = C.fit_electrical(log, d)
    mech = C.fit_mechanical(log, d, elec)
    assert elec.resistance_20c == pytest.approx(truth.motor.resistance, rel=0.03)
    assert elec.flux_linkage_20c == pytest.approx(truth.motor.flux_linkage, rel=0.03)
    assert mech.coulomb == pytest.approx(3.5, rel=0.15)
    assert mech.viscous == pytest.approx(2.2, rel=0.15)
    assert mech.efficiency == pytest.approx(0.68, abs=0.03)


def test_residual_improves_torque_prediction_on_held_out_data():
    d = JointDesign()
    cal = V.record(d, calibration_routine(60), seed=10)
    val = V.record(d, pick_and_place(cycles=2), seed=11)
    elec = C.fit_electrical(cal, d)
    calibrated = C.apply_calibration(d, elec, C.fit_mechanical(cal, d, elec))
    residual = C.ResidualModel().fit([cal], calibrated)
    e_design = C.torque_prediction_rmse(val, d)
    e_cal = C.torque_prediction_rmse(val, calibrated)
    e_res = C.torque_prediction_rmse(val, calibrated, residual)
    assert e_cal < 0.5 * e_design
    assert e_res < e_cal
