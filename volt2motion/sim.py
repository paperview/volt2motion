"""Run one joint from battery voltage to arm motion and report design checks."""
from __future__ import annotations

import time
from collections import deque
from typing import Protocol

import numpy as np

from .actuator import Actuator
from .controller import JointController
from .joint import JointModel, gearbox_friction
from .params import JointDesign
from .power import PowerBus
from .thermal import ThermalModel
from .trajectory import Trajectory

# Columns a real robot or bench rig can log. Phase 2 calibration only uses these.
HARDWARE_COLUMNS = [
    "t", "q_ref", "q", "qd", "payload", "iq_ref", "iq", "id", "vd", "vq", "vbus", "t_winding",
]

# Extra columns only a simulation knows.
SIM_COLUMNS = HARDWARE_COLUMNS + [
    "qd_ref", "q_true", "qd_true", "iq_true", "v_mag", "v_max", "v_sat", "i_limit",
    "limited", "i_bus", "i_batt", "i_brake", "soc", "tau_motor", "tau_in", "tau_friction",
    "tau_residual", "tau_cmd", "t_housing", "t_fet", "p_copper", "p_iron", "p_fet", "p_bus",
    "fault",
]

RESIDUAL_LAGS = (5, 10)   # ms of current history the residual model sees


class Residual(Protocol):
    def predict_one(self, q: float, qd: float, iq: float, iq_lags: tuple[float, ...]) -> float: ...


def simulate(
    plant: JointDesign,
    trajectory: Trajectory,
    duration: float | None = None,
    fidelity: str = "full",
    firmware: JointDesign | None = None,
    residual: Residual | None = None,
    seed: int = 0,
    log_every: int = 1,
    verbose: bool = False,
) -> dict[str, np.ndarray]:
    """Simulate the joint and return a log with one row per logged 1 ms step.

    plant     the physical joint (what is really built)
    firmware  what the controller believes; defaults to the plant
    residual  learned correction from phase 2, added as joint torque
    """
    firmware = firmware or plant
    duration = trajectory.duration if duration is None else duration
    rng = np.random.default_rng(seed)
    ctrl = JointController(firmware)
    dt = ctrl.dt
    joint = JointModel(plant, dt)
    bus = PowerBus(plant.battery)
    thermal = ThermalModel(plant.thermal)
    act = Actuator(plant, bus, thermal, rng, firmware=firmware)
    ratio = plant.gearbox.ratio

    q0, _, _, payload0 = trajectory(0.0)
    joint.set_state(q0)
    payload_state = None
    n_steps = int(round(duration / dt))
    n_rows = (n_steps + log_every - 1) // log_every
    log = {c: np.zeros(n_rows) for c in SIM_COLUMNS}
    iq_hist: deque[float] = deque([0.0] * (max(RESIDUAL_LAGS) + 1), maxlen=max(RESIDUAL_LAGS) + 1)
    t_start = time.perf_counter()
    row = 0

    for k in range(n_steps):
        t = k * dt
        q_ref, qd_ref, qdd_ref, payload = trajectory(t)
        if payload != payload_state:
            joint.set_payload(payload)
            ctrl.set_payload(payload)
            payload_state = payload

        q, qd = joint.q, joint.qd
        iq_ref = ctrl.update(q * ratio, q_ref, qd_ref, qdd_ref)
        tau_m = act.step(iq_ref, q * ratio, qd * ratio, dt, fidelity)
        tau_in = ratio * tau_m
        coulomb, viscous = gearbox_friction(plant.gearbox, q, qd, tau_in)
        joint.set_friction(coulomb, viscous)
        iq_hist.append(act.iq_meas)
        tau_res = 0.0
        if residual is not None:
            lags = tuple(iq_hist[-1 - lag] for lag in RESIDUAL_LAGS)
            tau_res = residual.predict_one(ctrl.q_meas, ctrl.qd_filt, act.iq_meas, lags)
        joint.step(tau_in + tau_res)

        if k % log_every == 0:
            tau_f = joint.friction_torque()
            vals = {
                "t": t, "q_ref": q_ref, "q": ctrl.q_meas, "qd": ctrl.qd_filt,
                "payload": float(payload), "iq_ref": iq_ref, "iq": act.iq_meas, "id": act.id_meas,
                "vd": act.vd, "vq": act.vq, "vbus": bus.vbus, "t_winding": thermal.t_winding,
                "qd_ref": qd_ref, "q_true": q, "qd_true": qd, "iq_true": act.iq,
                "v_mag": act.v_mag, "v_max": act.v_max, "v_sat": act.v_sat, "i_limit": act.limit,
                "limited": float(act.limited), "i_bus": act.i_bus, "i_batt": bus.i_batt,
                "i_brake": bus.i_brake, "soc": bus.soc, "tau_motor": tau_m, "tau_in": tau_in,
                "tau_friction": tau_f, "tau_residual": tau_res, "tau_cmd": ctrl.tau_cmd,
                "t_housing": thermal.t_housing, "t_fet": thermal.t_fet,
                "p_copper": act.p_copper, "p_iron": act.p_iron, "p_fet": act.p_fet,
                "p_bus": act.p_bus, "fault": float(bool(bus.fault)),
            }
            for c in SIM_COLUMNS:
                log[c][row] = vals[c]
            row += 1

        if verbose and k and k % 60_000 == 0:
            el = time.perf_counter() - t_start
            print(f"  simulated {t / 60:5.1f} min in {el:5.1f} s")

    log["_meta_wall_s"] = np.array([time.perf_counter() - t_start])
    log["_meta_brake_j"] = np.array([bus.brake_energy])
    log["_meta_batt_j"] = np.array([bus.batt_energy])
    log["_meta_fault"] = np.array([bus.fault or "none"], dtype=object)
    return log


def hardware_view(log: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Keep only the channels real hardware can measure."""
    return {c: log[c] for c in HARDWARE_COLUMNS}


def summarize(log: dict[str, np.ndarray], design: JointDesign) -> dict:
    """Design checks for one run. Returns metrics plus a list of pass/fail checks."""
    dt = float(log["t"][1] - log["t"][0])
    duration = float(log["t"][-1] + dt)
    err = np.degrees(log["q_ref"] - log["q_true"])
    moving = np.abs(log["qd_ref"]) > 1e-3
    th = ThermalModel(design.thermal)
    # Average losses over the run predict where temperatures settle.
    tw_ss, tf_ss = th.steady_state(log["p_copper"].mean(), log["p_iron"].mean(), log["p_fet"].mean())
    energy_j = float(np.sum(log["p_bus"]) * dt)
    tau_out = log["tau_in"] - log["tau_friction"]
    m = {
        "duration_s": duration,
        "tracking_err_max_deg": float(np.max(np.abs(err))),
        "tracking_err_rms_moving_deg": float(np.sqrt(np.mean(err[moving] ** 2))) if moving.any() else 0.0,
        "iq_peak_a": float(np.max(np.abs(log["iq_true"]))),
        "iq_rms_a": float(np.sqrt(np.mean(log["iq_true"] ** 2))),
        "current_limited_pct": 100.0 * float(np.mean(log["limited"])),
        "voltage_saturated_pct": 100.0 * float(np.mean(log["v_sat"])),
        "joint_torque_peak_nm": float(np.max(np.abs(tau_out))),
        "vbus_min_v": float(np.min(log["vbus"])),
        "vbus_max_v": float(np.max(log["vbus"])),
        "brake_energy_j": float(log["_meta_brake_j"][0]),
        "energy_from_bus_wh": energy_j / 3600.0,
        "avg_power_w": energy_j / duration,
        "t_winding_max_c": float(np.max(log["t_winding"])),
        "t_fet_max_c": float(np.max(log["t_fet"])),
        "t_winding_steady_c": float(tw_ss),
        "t_fet_steady_c": float(tf_ss),
        "fault": str(log["_meta_fault"][0]),
        "sim_wall_s": float(log["_meta_wall_s"][0]),
    }
    t, b, g = design.thermal, design.battery, design.gearbox
    m["checks"] = [
        ("No driver fault", m["fault"] == "none", m["fault"]),
        ("Tracking error under 1 deg", m["tracking_err_max_deg"] < 1.0, f'{m["tracking_err_max_deg"]:.2f} deg'),
        ("Current limit rarely hit (<1% of time)", m["current_limited_pct"] < 1.0, f'{m["current_limited_pct"]:.1f}%'),
        ("Voltage headroom (<1% saturated)", m["voltage_saturated_pct"] < 1.0, f'{m["voltage_saturated_pct"]:.1f}%'),
        ("Gearbox peak torque within rating", m["joint_torque_peak_nm"] < g.rated_peak_torque,
         f'{m["joint_torque_peak_nm"]:.0f} of {g.rated_peak_torque:.0f} Nm'),
        ("Bus stays below brake threshold", m["vbus_max_v"] < b.brake_on_v, f'{m["vbus_max_v"]:.1f} V'),
        ("Bus stays above undervoltage", m["vbus_min_v"] > b.undervoltage_v, f'{m["vbus_min_v"]:.1f} V'),
        ("Winding steady state under limit", m["t_winding_steady_c"] < t.winding_limit_c,
         f'{m["t_winding_steady_c"]:.0f} C (limit {t.winding_limit_c:.0f})'),
        ("MOSFET steady state under limit", m["t_fet_steady_c"] < t.fet_limit_c,
         f'{m["t_fet_steady_c"]:.0f} C (limit {t.fet_limit_c:.0f})'),
    ]
    return m


def print_summary(m: dict, title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))
    rows = [
        ("Simulated", f'{m["duration_s"]:.1f} s in {m["sim_wall_s"]:.1f} s wall time'),
        ("Tracking error, max / rms while moving", f'{m["tracking_err_max_deg"]:.3f} / {m["tracking_err_rms_moving_deg"]:.3f} deg'),
        ("Phase current, peak / rms", f'{m["iq_peak_a"]:.1f} / {m["iq_rms_a"]:.1f} A'),
        ("Joint torque peak", f'{m["joint_torque_peak_nm"]:.1f} Nm'),
        ("Bus voltage, min / max", f'{m["vbus_min_v"]:.2f} / {m["vbus_max_v"]:.2f} V'),
        ("Average power from bus", f'{m["avg_power_w"]:.1f} W'),
        ("Winding temp, max seen / steady state", f'{m["t_winding_max_c"]:.1f} / {m["t_winding_steady_c"]:.1f} C'),
        ("MOSFET temp, max seen / steady state", f'{m["t_fet_max_c"]:.1f} / {m["t_fet_steady_c"]:.1f} C'),
    ]
    w = max(len(r[0]) for r in rows)
    for k, v in rows:
        print(f"  {k:<{w}}  {v}")
    print("  Checks:")
    for name, ok, detail in m["checks"]:
        print(f"    [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
