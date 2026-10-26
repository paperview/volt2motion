"""Joint position controller: encoder, velocity filter, PD plus model feedforward.

The controller only knows the firmware's design values. On real hardware those
are the numbers you flash. In phase 2 you reflash with calibrated values.
"""
from __future__ import annotations

import math

from .joint import JointModel
from .params import JointDesign


class JointController:
    def __init__(self, firmware: JointDesign):
        self.fw = firmware
        c = firmware.control
        self.dt = 1.0 / c.rate_hz
        self.ratio = firmware.gearbox.ratio
        self.kt = firmware.motor.kt
        self.enc_res = firmware.encoder_resolution
        self.alpha = 1.0 - math.exp(-2 * math.pi * c.vel_filter_hz * self.dt)
        self.model = JointModel(firmware, self.dt)   # friction-free, for feedforward
        self.model.set_payload(True)
        j = self.model.joint_inertia()   # worst case: payload on
        wn = 2 * math.pi * c.bandwidth_hz
        self.kp = j * wn * wn
        self.kd = 2.0 * c.damping_ratio * j * wn
        self.q_meas = None
        self.q_prev = None
        self.qd_filt = 0.0
        self.tau_cmd = 0.0

    def set_payload(self, on: bool) -> None:
        self.model.set_payload(on)

    def read_encoder(self, theta_m: float) -> float:
        """Joint angle as the motor-side encoder reports it."""
        return round(theta_m / self.enc_res) * self.enc_res / self.ratio

    def update(self, theta_m: float, q_ref: float, qd_ref: float, qdd_ref: float) -> float:
        """Return the q-axis current command in amps."""
        q = self.read_encoder(theta_m)
        if self.q_prev is None:
            self.q_prev = q
        raw = (q - self.q_prev) / self.dt
        self.qd_filt += self.alpha * (raw - self.qd_filt)
        self.q_prev = q
        self.q_meas = q

        tau = self.kp * (q_ref - q) + self.kd * (qd_ref - self.qd_filt)
        if self.fw.control.use_feedforward:
            tau += self.feedforward(q_ref, qd_ref, qdd_ref)
        self.tau_cmd = tau
        return tau / (self.ratio * self.kt)

    def feedforward(self, q: float, qd: float, qdd: float) -> float:
        g = self.fw.gearbox
        tau = self.model.inverse_dynamics(q, qd, qdd)
        s = math.tanh(qd / 0.02)
        # Friction the gearbox will eat, including its load-dependent loss.
        loss_frac = 1.0 - g.efficiency
        tau_needed = tau + (g.coulomb * s + g.viscous * qd)
        return tau_needed + loss_frac * abs(tau_needed) * s
