"""Joint trajectories: a pick-and-place cycle, a calibration routine, and replay of logged setpoints."""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Segment:
    t0: float
    duration: float
    q0: float
    q1: float
    payload: bool
    kind: str = "minjerk"   # "minjerk", "hold" or "ramp" (constant velocity)


class Trajectory:
    """Piecewise trajectory. Call it with time to get (q, qd, qdd, payload_on)."""

    def __init__(self, segments: list[Segment]):
        self.segments = segments
        self.starts = [s.t0 for s in segments]
        self.duration = segments[-1].t0 + segments[-1].duration

    def __call__(self, t: float) -> tuple[float, float, float, bool]:
        i = max(0, bisect.bisect_right(self.starts, t) - 1)
        s = self.segments[i]
        tau = min(max((t - s.t0) / s.duration, 0.0), 1.0)
        dq = s.q1 - s.q0
        if s.kind == "hold":
            return s.q0, 0.0, 0.0, s.payload
        if s.kind == "ramp":
            return s.q0 + dq * tau, dq / s.duration if tau < 1.0 else 0.0, 0.0, s.payload
        # Minimum-jerk profile
        p = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        v = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / s.duration
        a = (60 * tau - 180 * tau**2 + 120 * tau**3) / s.duration**2
        return s.q0 + dq * p, dq * v, dq * a, s.payload


class _Builder:
    def __init__(self, q0: float):
        self.t = 0.0
        self.q = q0
        self.segments: list[Segment] = []

    def move(self, q1: float, duration: float, payload: bool) -> None:
        self.segments.append(Segment(self.t, duration, self.q, q1, payload))
        self.t += duration
        self.q = q1

    def ramp(self, q1: float, speed: float, payload: bool) -> None:
        duration = abs(q1 - self.q) / speed
        self.segments.append(Segment(self.t, duration, self.q, q1, payload, "ramp"))
        self.t += duration
        self.q = q1

    def hold(self, duration: float, payload: bool) -> None:
        self.segments.append(Segment(self.t, duration, self.q, self.q, payload, "hold"))
        self.t += duration


def pick_and_place(
    q_pick: float = -0.5,
    q_place: float = 1.0,
    move_time: float = 0.9,
    dwell: float = 0.4,
    cycles: int = 3,
) -> Trajectory:
    """Grab at q_pick, carry the payload to q_place, release, return empty."""
    b = _Builder(q_pick)
    for _ in range(cycles):
        b.hold(dwell, payload=False)      # gripper closing
        b.move(q_place, move_time, payload=True)
        b.hold(dwell, payload=True)       # gripper opening
        b.move(q_pick, move_time, payload=False)
    return Trajectory(b.segments)


def calibration_routine(duration: float = 90.0, seed: int = 1, q_min: float = -0.8, q_max: float = 1.3) -> Trajectory:
    """A system identification routine to run on the built machine.

    Mixes slow constant-speed sweeps (to separate Coulomb from viscous
    friction), fast random moves (to load the motor and gearbox) and holds,
    with and without the payload.
    """
    rng = np.random.default_rng(seed)
    b = _Builder(0.0)
    b.hold(0.5, payload=False)
    while b.t < duration:
        payload = bool(rng.random() < 0.5)
        choice = rng.random()
        if choice < 0.35:
            speed = float(rng.choice([0.05, 0.1, 0.2, 0.4, 0.8]))
            target = float(rng.uniform(q_min, q_max))
            if abs(target - b.q) > 0.2:
                b.ramp(target, speed, payload)
        elif choice < 0.85:
            target = float(rng.uniform(q_min, q_max))
            dq = abs(target - b.q)
            if dq > 0.05:
                b.move(target, float(rng.uniform(max(0.35, 0.5 * math.sqrt(dq)), 2.0)), payload)
        else:
            b.hold(float(rng.uniform(0.2, 0.8)), payload)
    return Trajectory(b.segments)


class SampledTrajectory:
    """A trajectory replayed from logged setpoints, so a simulation can rerun a real job.

    Velocity and acceleration setpoints are used when the log has them
    (qd_ref, qdd_ref columns). Otherwise they come from smoothed derivatives
    of the logged angle, which is exact for smooth moves but rounds off
    sudden velocity steps.
    """

    def __init__(self, t, q_ref, payload, qd_ref=None, qdd_ref=None, smooth_s: float = 0.02):
        from scipy.signal import savgol_filter

        t, q = np.asarray(t, float), np.asarray(q_ref, float)
        self.dt = float(t[1] - t[0])
        self.q = q
        window = max(5, int(round(smooth_s / self.dt)) | 1)
        if len(q) > window:
            self.qd = savgol_filter(q, window, 3, deriv=1, delta=self.dt)
            self.qdd = savgol_filter(q, window, 3, deriv=2, delta=self.dt)
        else:
            self.qd = np.gradient(q, self.dt)
            self.qdd = np.gradient(self.qd, self.dt)
        if qd_ref is not None:
            self.qd = np.asarray(qd_ref, float)
        if qdd_ref is not None:
            self.qdd = np.asarray(qdd_ref, float)
        self.payload = np.asarray(payload, float) > 0.5
        self.duration = len(q) * self.dt

    @classmethod
    def from_log(cls, log: dict[str, np.ndarray]) -> "SampledTrajectory":
        return cls(log["t"], log["q_ref"], log["payload"], log.get("qd_ref"), log.get("qdd_ref"))

    def __call__(self, t: float) -> tuple[float, float, float, bool]:
        x = min(max(t / self.dt, 0.0), len(self.q) - 1.0)
        i = min(int(x), len(self.q) - 2)
        w = x - i
        lerp = lambda a: float(a[i] * (1 - w) + a[i + 1] * w)  # noqa: E731
        return lerp(self.q), lerp(self.qd), lerp(self.qdd), bool(self.payload[i if w < 0.5 else i + 1])


def to_samples(trajectory, rate_hz: float = 1000.0) -> dict[str, np.ndarray]:
    """Sample a trajectory into setpoint columns for firmware to play back."""
    t = np.arange(int(round(trajectory.duration * rate_hz)) + 1) / rate_hz
    rows = np.array([trajectory(float(ti))[:3] for ti in t])
    payload = np.array([float(trajectory(float(ti))[3]) for ti in t])
    return {"t": t, "q_ref": rows[:, 0], "qd_ref": rows[:, 1], "qdd_ref": rows[:, 2], "payload": payload}
