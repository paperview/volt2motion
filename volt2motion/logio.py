"""CSV logs: the contract between the robot's logging firmware and calibration.

Log at a fixed 1 kHz with these columns (SI units, radians, amps, volts, C):

  t          time, s
  q_ref      commanded joint angle, rad
  q          measured joint angle from the encoder, rad
  qd         the controller's filtered joint velocity, rad/s
  payload    1 while the gripper holds the payload, else 0
  iq_ref     q-axis current command, A
  iq, id     measured dq currents, averaged over the 1 ms step, A
  vd, vq     dq voltage commands applied by the driver, averaged, V
  vbus       DC bus voltage, V
  t_winding  winding thermistor, C
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .sim import HARDWARE_COLUMNS


def save_csv(log: dict[str, np.ndarray], path: str | Path, columns: list[str] | None = None) -> None:
    columns = columns or HARDWARE_COLUMNS
    data = np.column_stack([log[c] for c in columns])
    np.savetxt(path, data, delimiter=",", header=",".join(columns), comments="", fmt="%.7g")


def load_csv(path: str | Path) -> dict[str, np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", names=True)
    log = {name: np.asarray(data[name], float) for name in data.dtype.names}
    missing = [c for c in HARDWARE_COLUMNS if c not in log]
    if missing:
        raise ValueError(f"{path}: missing columns {missing}")
    dt = np.diff(log["t"])
    if np.max(np.abs(dt - dt[0])) > 1e-6:
        raise ValueError(f"{path}: rows must be evenly spaced in time (resample to 1 kHz first)")
    return log
