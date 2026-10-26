"""Phase 2: calibrate the design model against logs from the built machine.

Two steps, in this order:

1. Physics fit. Least-squares fits of the parameters the model already has:
   winding resistance, magnet flux, inductance, gearbox friction and
   efficiency. These stay interpretable, so you can compare them to the
   datasheet and feed them back into the next hardware revision.
2. Learned residual. A small neural network learns whatever torque the
   calibrated physics still gets wrong (breakaway friction, position-dependent
   gear friction, unmodeled effects). It is the ANYmal "actuator network"
   idea, but as a correction on top of physics instead of a replacement, so
   the model still extrapolates sensibly outside the training data.

Inputs are only the channels in sim.HARDWARE_COLUMNS, which real firmware can log.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler

from .joint import JointModel
from .links import DOCS_URL
from .params import JointDesign
from .sim import RESIDUAL_LAGS

SLIDING_SPEED = 0.03     # rad/s; below this the joint may be stuck in static friction
# The residual is off below GATE_LOW and fully on above GATE_HIGH (rad/s).
# Near standstill it has little data and guesses badly, so MuJoCo's static
# friction handles that regime instead.
GATE_LOW, GATE_HIGH = 0.02, 0.05


def _gate(qd):
    return np.clip((np.abs(qd) - GATE_LOW) / (GATE_HIGH - GATE_LOW), 0.0, 1.0)


# -- signal processing ---------------------------------------------------------
def derivatives(log: dict[str, np.ndarray], window_ms: int = 31) -> tuple[np.ndarray, np.ndarray]:
    """Joint velocity and acceleration from logged position (non-causal, offline only)."""
    dt = float(log["t"][1] - log["t"][0])
    w = window_ms if window_ms % 2 else window_ms + 1
    qd = savgol_filter(log["q"], w, 3, deriv=1, delta=dt)
    qdd = savgol_filter(log["q"], w, 3, deriv=2, delta=dt)
    return qd, qdd


def required_joint_torque(log: dict[str, np.ndarray], design: JointDesign) -> np.ndarray:
    """Torque the gearbox must have delivered, from the measured motion.

    Uses the friction-free rigid-body model (link and payload masses from CAD
    or a scale, rotor inertia from the datasheet).
    """
    qd, qdd = derivatives(log)
    dt = float(log["t"][1] - log["t"][0])
    model = JointModel(design, dt)
    tau = np.empty_like(qd)
    state = None
    for i, (q, v, a, p) in enumerate(zip(log["q"], qd, qdd, log["payload"])):
        on = p > 0.5
        if on != state:
            model.set_payload(on)
            state = on
        tau[i] = model.inverse_dynamics(q, v, a)
    return tau


# -- step 1: physics fit ---------------------------------------------------------
@dataclass
class ElectricalFit:
    resistance_20c: float
    flux_linkage_20c: float
    inductance: float
    deadtime_gain: float
    vq_rmse: float
    mean_temp_c: float


@dataclass
class MechanicalFit:
    coulomb: float
    viscous: float
    efficiency: float
    kt_free_fit: float      # torque constant fitted from motion alone (cross-check)
    kt_electrical: float    # torque constant implied by the back-EMF fit
    torque_rmse: float
    n_samples: int


def _lstsq(X: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, float]:
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    rmse = float(np.sqrt(np.mean((X @ coef - y) ** 2)))
    return coef, rmse


def fit_electrical(log: dict[str, np.ndarray], design: JointDesign) -> ElectricalFit:
    """Fit R, flux and L from the q- and d-axis voltage equations.

    vq = R_eff iq + L diq/dt + flux we + dead-time error
    vd = R_eff id - L we iq + dead-time error
    R_eff includes the MOSFET on-resistance, which is subtracted afterwards.
    """
    m, d, g = design.motor, design.driver, design.gearbox
    dt = float(log["t"][1] - log["t"][0])
    qd, _ = derivatives(log)
    we = m.pole_pairs * g.ratio * qd
    iq, i_d, vbus = log["iq"], log["id"], log["vbus"]
    diq = savgol_filter(iq, 11, 2, deriv=1, delta=dt)
    norm = np.sqrt(iq**2 + i_d**2 + 0.25)
    mask = np.abs(iq) > 0.5

    X = np.column_stack([iq, diq, we, vbus * iq / norm])[mask]
    coef, rmse = _lstsq(X, log["vq"][mask])
    r_eff, l_fit, flux, k_dt = coef

    t_mean = float(np.mean(log["t_winding"][mask]))
    r20 = (r_eff - d.rds_on) / (1.0 + m.copper_tempco * (t_mean - 20.0))
    # The magnets are not measured directly; winding temperature is the proxy.
    flux20 = flux / (1.0 + m.magnet_tempco * (t_mean - 20.0))
    return ElectricalFit(float(r20), float(flux20), float(l_fit), float(k_dt), rmse, t_mean)


def fit_mechanical(log: dict[str, np.ndarray], design: JointDesign, elec: ElectricalFit) -> MechanicalFit:
    """Fit gearbox Coulomb friction, viscous friction and efficiency.

    Only samples where the joint is clearly sliding are used, because static
    friction while stuck can take any value up to its limit.
    """
    m, g = design.motor, design.gearbox
    qd, _ = derivatives(log)
    tau_dyn = required_joint_torque(log, design)
    kt = 1.5 * m.pole_pairs * elec.flux_linkage_20c * (1.0 + m.magnet_tempco * (log["t_winding"] - 20.0))
    tau_in = g.ratio * kt * log["iq"]
    s = np.sign(qd)
    mask = np.abs(qd) > SLIDING_SPEED

    X = np.column_stack([s, qd, np.abs(tau_in) * s])[mask]
    coef, rmse = _lstsq(X, (tau_in - tau_dyn)[mask])
    coulomb, viscous, loss = coef

    # Cross-check: fit Kt from motion alone. A large mismatch with the
    # electrical value points at wrong link masses or a current sensor gain error.
    n_iq = g.ratio * log["iq"]
    X2 = np.column_stack([n_iq, -s, -qd, -np.abs(n_iq) * s])[mask]
    coef2, _ = _lstsq(X2, tau_dyn[mask])
    kt_free = coef2[0]
    return MechanicalFit(
        float(coulomb), float(viscous), float(1.0 - loss), float(kt_free),
        float(1.5 * m.pole_pairs * elec.flux_linkage_20c), rmse, int(mask.sum()),
    )


def fit_problems(elec: ElectricalFit, mech: MechanicalFit) -> list[str]:
    """Signs of a log convention mistake: fitted values that cannot be physical.

    A sign error in the log does not make the fit fail. It makes it return
    negative flux, negative resistance or impossible friction, so check before
    using the result.
    """
    problems = []
    if elec.flux_linkage_20c <= 0 and mech.kt_free_fit < 0:
        problems.append("Current and voltage have the opposite sign to the joint angle. Flip iq, iq_ref, "
                        "vd and vq in the log, or flip q, q_ref and qd.")
    elif elec.flux_linkage_20c <= 0:
        problems.append("The angle direction looks reversed. Positive q must lift the arm against "
                        "gravity. Flip q, q_ref and qd in the log.")
    elif mech.kt_free_fit <= 0 or elec.resistance_20c <= 0:
        problems.append("The current has the opposite sign to the voltage. Flip iq and iq_ref in the "
                        "log, or flip vd and vq.")
    bad = []
    if elec.resistance_20c <= 0:
        bad.append(f"resistance {elec.resistance_20c * 1e3:.1f} mOhm")
    if elec.inductance <= 0:
        bad.append(f"inductance {elec.inductance * 1e6:.0f} uH")
    if mech.coulomb < 0:
        bad.append(f"Coulomb friction {mech.coulomb:.2f} Nm")
    if mech.viscous < 0:
        bad.append(f"viscous friction {mech.viscous:.2f} Nm s/rad")
    if not 0 < mech.efficiency <= 1:
        bad.append(f"gearbox efficiency {mech.efficiency:.2f}")
    if bad and not problems:
        problems.append(f"Some fitted values are not physical, so check the log's units and signs "
                        f"({DOCS_URL}/logging.md#conventions).")
    if bad:
        problems.append("Not physical: " + ", ".join(bad) + ".")
    return problems


def apply_calibration(design: JointDesign, elec: ElectricalFit, mech: MechanicalFit) -> JointDesign:
    return design.updated(
        {
            "motor.resistance": elec.resistance_20c,
            "motor.flux_linkage": elec.flux_linkage_20c,
            "motor.ld": elec.inductance,
            "motor.lq": elec.inductance,
            "gearbox.coulomb": mech.coulomb,
            "gearbox.viscous": mech.viscous,
            "gearbox.efficiency": mech.efficiency,
        },
        name=f"{design.name}_calibrated",
    )


def physics_joint_torque(log: dict[str, np.ndarray], design: JointDesign, qd: np.ndarray | None = None) -> np.ndarray:
    """Joint torque the physics model predicts from the logged current."""
    m, g = design.motor, design.gearbox
    if qd is None:
        qd, _ = derivatives(log)
    kt = 1.5 * m.pole_pairs * m.flux_linkage * (1.0 + m.magnet_tempco * (log["t_winding"] - 20.0))
    tau_in = g.ratio * kt * log["iq"]
    coulomb = g.coulomb + (1.0 - g.efficiency) * np.abs(tau_in)
    return tau_in - coulomb * np.sign(qd) - g.viscous * qd


# -- step 2: learned residual -------------------------------------------------
def _features(q, qd, iq, lags) -> np.ndarray:
    q, qd, iq = np.asarray(q, float), np.asarray(qd, float), np.asarray(iq, float)
    cols = [np.sin(q), np.cos(q), qd, np.tanh(qd / 0.05), iq, np.abs(iq) * np.tanh(qd / 0.05)]
    cols += [np.asarray(l, float) for l in lags]
    return np.column_stack(cols) if q.ndim else np.array(cols, float)


def _lagged(x: np.ndarray, lag: int) -> np.ndarray:
    out = np.empty_like(x)
    out[lag:] = x[:-lag]
    out[:lag] = x[0]
    return out


class ResidualModel:
    """Learned torque correction on top of the calibrated physics model.

    It is off near standstill, where MuJoCo's static friction model takes
    over, and its output is clipped to the range seen in training so it cannot
    run away on inputs it has never seen.
    """

    def __init__(self, hidden: tuple[int, ...] = (64, 64), seed: int = 0):
        self.hidden = hidden
        self.seed = seed
        self.scaler = StandardScaler()
        self.net: MLPRegressor | None = None
        self.clip = 0.0

    def fit(self, logs: list[dict[str, np.ndarray]], design: JointDesign) -> "ResidualModel":
        X_all, y_all = [], []
        for log in logs:
            qd_s, _ = derivatives(log)
            target = required_joint_torque(log, design) - physics_joint_torque(log, design, qd_s)
            lags = [_lagged(log["iq"], lag) for lag in RESIDUAL_LAGS]
            X = _features(log["q"], log["qd"], log["iq"], lags)
            mask = (np.abs(qd_s) > GATE_LOW) & (np.abs(log["qd"]) > GATE_LOW)
            X_all.append(X[mask])
            y_all.append(target[mask])
        X, y = np.vstack(X_all), np.concatenate(y_all)
        # Percentile, not max: payload grabs and ramp starts leave derivative spikes.
        self.clip = 1.5 * float(np.percentile(np.abs(y), 99.9))
        Xs = self.scaler.fit_transform(X)
        self.net = MLPRegressor(
            hidden_layer_sizes=self.hidden, activation="tanh", alpha=1e-4,
            early_stopping=True, max_iter=400, random_state=self.seed,
        ).fit(Xs, y)
        self._cache()
        return self

    def _cache(self) -> None:
        self._w = [w.copy() for w in self.net.coefs_]
        self._b = [b.copy() for b in self.net.intercepts_]
        self._mu = self.scaler.mean_.copy()
        self._sd = self.scaler.scale_.copy()

    def predict(self, log: dict[str, np.ndarray]) -> np.ndarray:
        lags = [_lagged(log["iq"], lag) for lag in RESIDUAL_LAGS]
        h = (_features(log["q"], log["qd"], log["iq"], lags) - self._mu) / self._sd
        for w, b in zip(self._w[:-1], self._b[:-1]):
            h = np.tanh(h @ w + b)
        y = np.clip((h @ self._w[-1] + self._b[-1])[:, 0], -self.clip, self.clip)
        return y * _gate(log["qd"])

    def predict_one(self, q: float, qd: float, iq: float, iq_lags: tuple[float, ...]) -> float:
        """Fast single-sample forward pass for use inside the simulation loop."""
        g = float(_gate(qd))
        if g == 0.0:
            return 0.0
        h = (_features(q, qd, iq, iq_lags) - self._mu) / self._sd
        for w, b in zip(self._w[:-1], self._b[:-1]):
            h = np.tanh(h @ w + b)
        y = float((h @ self._w[-1] + self._b[-1])[0])
        y = max(-self.clip, min(self.clip, y))
        return y * g

    def save(self, path: str | Path) -> None:
        """Save the weights as plain arrays (.npz). Loading needs no pickle."""
        arrays = {f"w{i}": w for i, w in enumerate(self._w)}
        arrays.update({f"b{i}": b for i, b in enumerate(self._b)})
        np.savez(path, mu=self._mu, sd=self._sd, clip=self.clip,
                 hidden=np.array(self.hidden), seed=self.seed, **arrays)

    @staticmethod
    def load(path: str | Path) -> "ResidualModel":
        with np.load(path, allow_pickle=False) as f:
            model = ResidualModel(tuple(int(n) for n in f["hidden"]), int(f["seed"]))
            layers = len(model.hidden) + 1
            model._w = [f[f"w{i}"] for i in range(layers)]
            model._b = [f[f"b{i}"] for i in range(layers)]
            model._mu, model._sd = f["mu"], f["sd"]
            model.clip = float(f["clip"])
        return model


def torque_prediction_rmse(log: dict[str, np.ndarray], design: JointDesign, residual: ResidualModel | None = None) -> float:
    """How well a model predicts the joint torque in a log (sliding samples, Nm rms)."""
    qd_s, _ = derivatives(log)
    pred = physics_joint_torque(log, design, qd_s)
    if residual is not None:
        pred = pred + residual.predict(log)
    actual = required_joint_torque(log, design)
    mask = np.abs(qd_s) > SLIDING_SPEED
    return float(np.sqrt(np.mean((pred - actual)[mask] ** 2)))


def compare_traces(a: dict[str, np.ndarray], b: dict[str, np.ndarray]) -> dict[str, float]:
    """RMS difference between two logs of the same trajectory."""
    n = min(len(a["t"]), len(b["t"]))
    err_a = np.degrees(a["q_ref"][:n] - a["q"][:n])
    err_b = np.degrees(b["q_ref"][:n] - b["q"][:n])
    return {
        "tracking_err_diff_deg": float(np.sqrt(np.mean((err_a - err_b) ** 2))),
        "current_diff_a": float(np.sqrt(np.mean((a["iq"][:n] - b["iq"][:n]) ** 2))),
        "winding_temp_diff_c": float(abs(a["t_winding"][n - 1] - b["t_winding"][n - 1])),
    }


def tracking_rms_deg(log: dict[str, np.ndarray]) -> float:
    return float(np.sqrt(np.mean(np.degrees(log["q_ref"] - log["q"]) ** 2)))
