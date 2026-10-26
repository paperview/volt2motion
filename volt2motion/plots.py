"""Plots for design runs and calibration results."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .params import JointDesign  # noqa: E402


def design_plot(log: dict[str, np.ndarray], design: JointDesign, path: str | Path, title: str) -> None:
    t = log["t"]
    minutes = t[-1] > 120
    tx = t / 60 if minutes else t
    xlabel = "time (min)" if minutes else "time (s)"
    fig, ax = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
    fig.suptitle(title)

    a = ax[0, 0]
    a.plot(tx, np.degrees(log["q_ref"]), label="command", lw=1)
    a.plot(tx, np.degrees(log["q_true"]), label="actual", lw=1, ls="--")
    a2 = a.twinx()
    a2.plot(tx, np.degrees(log["q_ref"] - log["q_true"]), color="tab:red", lw=0.6, alpha=0.6)
    a2.set_ylabel("error (deg)", color="tab:red")
    a.set_ylabel("joint angle (deg)")
    a.legend(loc="upper left")

    a = ax[0, 1]
    a.plot(tx, log["iq_ref"], label="iq command", lw=0.8)
    a.plot(tx, log["iq_true"], label="iq actual", lw=0.8)
    a.plot(tx, log["i_limit"], "k:", lw=1, label="current limit")
    a.plot(tx, -log["i_limit"], "k:", lw=1)
    a.set_ylabel("phase current (A)")
    a.legend(loc="upper right")

    a = ax[1, 0]
    a.plot(tx, log["v_mag"], label="|v| applied", lw=0.8)
    a.plot(tx, log["v_max"], "k:", label="limit (Vbus / sqrt 3)", lw=1)
    a.set_ylabel("phase voltage (V)")
    a.legend(loc="upper right")

    a = ax[1, 1]
    a.plot(tx, log["vbus"], lw=0.8, color="tab:purple")
    a.axhline(design.battery.brake_on_v, color="k", ls=":", lw=1)
    a.set_ylabel("bus voltage (V)", color="tab:purple")
    a2 = a.twinx()
    a2.plot(tx, log["i_bus"], lw=0.6, color="tab:green", alpha=0.7)
    a2.set_ylabel("bus current (A)", color="tab:green")

    a = ax[2, 0]
    a.plot(tx, log["tau_in"], label="motor x ratio", lw=0.8)
    a.plot(tx, log["tau_in"] - log["tau_friction"], label="at joint", lw=0.8)
    a.plot(tx, log["tau_friction"], label="gearbox friction", lw=0.8)
    a.set_ylabel("torque (Nm)")
    a.set_xlabel(xlabel)
    a.legend(loc="upper right")

    a = ax[2, 1]
    a.plot(tx, log["t_winding"], label="winding")
    a.plot(tx, log["t_housing"], label="housing")
    a.plot(tx, log["t_fet"], label="MOSFETs")
    a.axhline(design.thermal.winding_limit_c, color="tab:blue", ls=":", lw=1)
    a.axhline(design.thermal.fet_limit_c, color="tab:green", ls=":", lw=1)
    a.set_ylabel("temperature (C)")
    a.set_xlabel(xlabel)
    a.legend(loc="upper left")

    for a in ax.flat:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def calibration_plot(real: dict, sims: dict[str, dict], path: str | Path, title: str) -> None:
    """Overlay the real log and each model's simulation of the same trajectory."""
    fig, ax = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle(title)
    t = real["t"]
    ax[0].plot(t, np.degrees(real["q_ref"] - real["q"]), "k", lw=1.6, label="hardware log")
    ax[1].plot(t, real["iq"], "k", lw=1.6, label="hardware log")
    for name, log in sims.items():
        n = min(len(t), len(log["t"]))
        ax[0].plot(t[:n], np.degrees(log["q_ref"][:n] - log["q"][:n]), lw=0.9, label=name)
        ax[1].plot(t[:n], log["iq"][:n], lw=0.9, label=name)
    ax[0].set_ylabel("tracking error (deg)")
    ax[1].set_ylabel("phase current iq (A)")
    ax[1].set_xlabel("time (s)")
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
