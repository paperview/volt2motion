"""A synthetic stand-in for a built machine, for trying phase 2 without hardware.

It is the same simulator with hidden parameters that differ from the design,
plus effects the design model does not include. Calibration only sees its
hardware-loggable channels, exactly as it would with a real robot.

With real hardware, use CSV logs instead (volt2motion calibrate --log, and
logio.py for the format).
"""
from __future__ import annotations

from .params import JointDesign
from .sim import hardware_view, simulate
from .trajectory import Trajectory


def hidden_truth(design: JointDesign) -> JointDesign:
    """How the built joint might differ from the design. Calibration never reads this."""
    m, g, t = design.motor, design.gearbox, design.thermal
    return design.updated(
        {
            # Typical part-to-datasheet differences
            "motor.flux_linkage": m.flux_linkage * 0.94,
            "motor.resistance": m.resistance * 1.12,
            "motor.ld": m.ld * 1.08,
            "motor.lq": m.lq * 1.08,
            "gearbox.coulomb": g.coulomb * 1.6,
            "gearbox.viscous": g.viscous * 1.3,
            "gearbox.efficiency": g.efficiency - 0.05,
            "thermal.r_housing_ambient": t.r_housing_ambient * 1.25,
            # Effects the design model leaves out
            "motor.cogging_amp": 0.02,
            "gearbox.stribeck_amp": 2.5,
            "gearbox.position_friction_amp": 1.2,
        },
        name="virtual_hardware",
    )


def record(
    design: JointDesign,
    trajectory: Trajectory,
    firmware: JointDesign | None = None,
    seed: int = 100,
    fidelity: str = "full",
) -> dict:
    """Run the virtual machine and return only what real logging would capture."""
    truth = hidden_truth(design)
    log = simulate(truth, trajectory, firmware=firmware or design, fidelity=fidelity, seed=seed)
    return hardware_view(log)
