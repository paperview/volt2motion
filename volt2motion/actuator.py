"""Motor plus driver PCB: FOC current loop, inverter losses, PMSM electrical model.

Two fidelity levels share the same parameters:

* "full" runs the current loop and the motor's electrical equations every PWM
  period (50 us at 20 kHz). It captures current loop dynamics, the one-period
  computation delay, sensor noise, dead time and voltage saturation.
* "fast" assumes the current loop tracks perfectly within its voltage limit and
  steps once per mechanics step (1 ms). It runs roughly 8x faster and is meant
  for long duty cycles, such as a full work shift for thermal checks.

The inverter is an averaged model: it does not resolve individual switching
edges, but it keeps their effects (voltage limit, conduction, switching and
dead-time losses). Resolve edges in LTspice when you need the transients.
"""
from __future__ import annotations

import math

import numpy as np

from .params import JointDesign
from .power import PowerBus
from .thermal import ThermalModel

SQRT3 = math.sqrt(3.0)
_DEADTIME_SMOOTH_A = 0.5   # current where the dead-time error reaches full size
_DEADTIME_GAIN = 4.0 / math.pi  # fundamental of a square-wave voltage error


class Actuator:
    def __init__(
        self,
        plant: JointDesign,
        bus: PowerBus,
        thermal: ThermalModel,
        rng: np.random.Generator,
        firmware: JointDesign | None = None,
    ):
        self.m = plant.motor
        self.d = plant.driver
        self.bus = bus
        self.th = thermal
        self.rng = rng
        fw = (firmware or plant).motor
        fd = (firmware or plant).driver
        # Current loop gains come from what the firmware believes about the motor.
        wc = 2 * math.pi * fd.current_loop_bw_hz
        self.kp_d = fw.ld * wc
        self.kp_q = fw.lq * wc
        self.ki = fw.resistance * wc
        self.fw_ld, self.fw_lq, self.fw_flux = fw.ld, fw.lq, fw.flux_linkage
        self.dt_e = 1.0 / self.d.pwm_freq
        self._kt_curve = (
            (np.asarray(self.m.kt_curve_current), np.asarray(self.m.kt_curve_scale))
            if len(self.m.kt_curve_current) >= 2 else None
        )

        # Electrical state
        self.id = 0.0
        self.iq = 0.0
        self.int_d = 0.0
        self.int_q = 0.0
        self.vd_next = 0.0   # command computed last period, applied this period
        self.vq_next = 0.0

        # Diagnostics for the last mechanics step (averages over the step)
        self.iq_meas = 0.0
        self.id_meas = 0.0
        self.vd = 0.0
        self.vq = 0.0
        self.v_mag = 0.0
        self.v_max = 0.0
        self.v_sat = 0.0       # fraction of the step spent at the voltage limit
        self.limit = self.d.current_limit
        self.limited = False
        self.p_copper = 0.0
        self.p_iron = 0.0
        self.p_fet = 0.0
        self.p_bus = 0.0
        self.i_bus = 0.0

    # -- temperature-dependent motor properties --------------------------------
    def resistance(self) -> float:
        return self.m.resistance * (1.0 + self.m.copper_tempco * (self.th.t_winding - 20.0))

    def flux(self) -> float:
        return self.m.flux_linkage * (1.0 + self.m.magnet_tempco * (self.th.t_housing - 20.0))

    def rds(self) -> float:
        """Driver series resistance per phase at the current MOSFET temperature."""
        return self.d.rds_on * (1.0 + self.d.rds_tempco * (self.th.t_fet - 25.0))

    def kt_scale(self, iq_abs: float) -> float:
        if self._kt_curve is None:
            return 1.0
        return float(np.interp(iq_abs, *self._kt_curve))

    def current_limit(self) -> float:
        """Firmware current limit after thermal derating."""
        d, th = self.d, self.th
        f_w = (d.derate_end_c - th.t_winding) / (d.derate_end_c - d.derate_start_c)
        f_f = (d.fet_derate_end_c - th.t_fet) / (d.fet_derate_end_c - d.fet_derate_start_c)
        return d.current_limit * min(1.0, max(0.0, min(f_w, f_f)))

    def _iron_drag(self, omega_m: float) -> tuple[float, float]:
        """Iron loss in W and the matching drag torque on the rotor."""
        m = self.m
        p = m.iron_loss_hyst * abs(omega_m) + m.iron_loss_eddy * omega_m * omega_m
        drag = (m.iron_loss_hyst + m.iron_loss_eddy * abs(omega_m)) * math.tanh(omega_m)
        return p, drag

    # -- stepping ----------------------------------------------------------------
    def step(self, iq_ref: float, theta_m: float, omega_m: float, dt: float, fidelity: str) -> float:
        """Advance one mechanics step. Returns average motor shaft torque in Nm."""
        self.limit = self.current_limit()
        lim = self.limit
        self.limited = abs(iq_ref) > lim
        iq_ref = max(-lim, min(lim, iq_ref))
        if fidelity == "full":
            tau = self._step_full(iq_ref, theta_m, omega_m, dt)
        elif fidelity == "fast":
            tau = self._step_fast(iq_ref, theta_m, omega_m, dt)
        else:
            raise ValueError(f"unknown fidelity {fidelity!r}")
        self.th.step(self.p_copper, self.p_iron, self.p_fet, dt)
        return tau

    def _step_full(self, iq_ref: float, theta_m: float, omega_m: float, dt: float) -> float:
        m, d, bus = self.m, self.d, self.bus
        n = max(1, int(round(dt / self.dt_e)))
        h = dt / n
        r = self.resistance()
        lam = self.flux()
        ld, lq = m.ld, m.lq
        pp = m.pole_pairs
        we = pp * omega_m
        ad = math.exp(-r * h / ld)
        aq = math.exp(-r * h / lq)
        kp_d, kp_q, ki = self.kp_d, self.kp_q, self.ki
        fw_ld, fw_lq, fw_flux = self.fw_ld, self.fw_lq, self.fw_flux
        rds = self.rds()
        dt_frac = d.dead_time * d.pwm_freq
        sw_k = 1.5 * (2.0 / math.pi) * d.switch_time * d.pwm_freq
        lsb = d.current_sense_lsb
        noise = self.rng.standard_normal(2 * n) * d.current_sense_noise
        cog_a, cog_n = m.cogging_amp, m.cogging_periods
        aux = bus.p.aux_load_w
        p_iron, drag = self._iron_drag(omega_m)
        faulted = bool(bus.fault)

        i_d, i_q = self.id, self.iq
        int_d, int_q = self.int_d, self.int_q
        vd_next, vq_next = self.vd_next, self.vq_next
        th = theta_m
        s_tau = s_cu = s_fet = s_bus = s_ibus = 0.0
        s_idm = s_iqm = s_vd = s_vq = s_vmag = s_vmax = 0.0
        sat = 0

        for k in range(n):
            vbus = bus.vbus
            vmax = d.max_modulation * vbus / SQRT3
            # Current measurement (ADC noise and quantization)
            idm = round((i_d + noise[2 * k]) / lsb) * lsb
            iqm = round((i_q + noise[2 * k + 1]) / lsb) * lsb
            # PI current control with cross-coupling and back-EMF decoupling
            ed = -idm
            eq = iq_ref - iqm
            ud = kp_d * ed + int_d - we * fw_lq * iqm
            uq = kp_q * eq + int_q + we * (fw_ld * idm + fw_flux)
            umag = math.hypot(ud, uq)
            if umag > vmax:
                scale = vmax / umag
                ud *= scale
                uq *= scale
                sat += 1
            else:
                int_d += ki * ed * h
                int_q += ki * eq * h
            if faulted:
                ud = uq = 0.0
            # The command computed now is applied next period (computation delay).
            va_d, va_q = vd_next, vq_next
            vd_next, vq_next = ud, uq

            # Inverter: MOSFET resistance and dead-time voltage error oppose the current.
            imag = math.sqrt(i_d * i_d + i_q * i_q + _DEADTIME_SMOOTH_A ** 2)
            r_dt = _DEADTIME_GAIN * dt_frac * vbus / imag
            vo_d = va_d - (rds + r_dt) * i_d
            vo_q = va_q - (rds + r_dt) * i_q

            # PMSM in the rotor frame, exact RL update with back-EMF held over the step
            e_d = vo_d + we * lq * i_q
            e_q = vo_q - we * (ld * i_d + lam)
            id_new = i_d * ad + e_d / r * (1.0 - ad)
            iq_new = i_q * aq + e_q / r * (1.0 - aq)
            ida = 0.5 * (i_d + id_new)
            iqa = 0.5 * (i_q + iq_new)
            i2 = ida * ida + iqa * iqa

            lam_t = lam * self.kt_scale(abs(iqa)) if self._kt_curve is not None else lam
            tau = 1.5 * pp * (lam_t * iqa + (ld - lq) * ida * iqa)
            if cog_a:
                tau += cog_a * math.sin(cog_n * th)

            p_cu = 1.5 * r * i2
            p_cond = 1.5 * (rds + r_dt) * i2
            p_sw = sw_k * vbus * math.sqrt(i2)
            p_bus = 1.5 * (vo_d * ida + vo_q * iqa) + p_cond + p_sw
            i_load = (p_bus + aux) / vbus
            bus.step(i_load, h)

            i_d, i_q = id_new, iq_new
            th += omega_m * h
            s_tau += tau
            s_cu += p_cu
            s_fet += p_cond + p_sw
            s_bus += p_bus + aux
            s_ibus += i_load
            s_idm += idm
            s_iqm += iqm
            s_vd += va_d
            s_vq += va_q
            s_vmag += math.hypot(va_d, va_q)
            s_vmax += vmax

        self.id, self.iq = i_d, i_q
        self.int_d, self.int_q = int_d, int_q
        self.vd_next, self.vq_next = vd_next, vq_next
        inv = 1.0 / n
        self.id_meas, self.iq_meas = s_idm * inv, s_iqm * inv
        self.vd, self.vq = s_vd * inv, s_vq * inv
        self.v_mag, self.v_max = s_vmag * inv, s_vmax * inv
        self.v_sat = sat * inv
        self.p_copper, self.p_iron, self.p_fet = s_cu * inv, p_iron, s_fet * inv
        self.p_bus, self.i_bus = s_bus * inv, s_ibus * inv
        return s_tau * inv - drag

    def _step_fast(self, iq_ref: float, theta_m: float, omega_m: float, dt: float) -> float:
        m, d, bus = self.m, self.d, self.bus
        r = self.resistance()
        rds = self.rds()
        r_tot = r + rds
        lam = self.flux()
        lq = m.lq
        we = m.pole_pairs * omega_m
        vbus = bus.vbus
        vmax = d.max_modulation * vbus / SQRT3

        # Steady-state current range reachable within the voltage limit (id = 0):
        # (R^2 + (we L)^2) iq^2 + 2 R we lam iq + (we lam)^2 <= vmax^2
        a = r_tot * r_tot + (we * lq) ** 2
        b = 2.0 * r_tot * we * lam
        c = (we * lam) ** 2 - vmax * vmax
        disc = b * b - 4.0 * a * c
        sat = False
        if disc < 0.0:
            iq = -b / (2.0 * a)   # back-EMF exceeds the bus: best the driver can do
            sat = True
        else:
            root = math.sqrt(disc)
            lo, hi = (-b - root) / (2.0 * a), (-b + root) / (2.0 * a)
            iq = min(hi, max(lo, iq_ref))
            sat = iq != iq_ref
        if bus.fault:
            iq = 0.0
        i_d = 0.0

        imag = math.sqrt(iq * iq + _DEADTIME_SMOOTH_A ** 2)
        r_dt = _DEADTIME_GAIN * d.dead_time * d.pwm_freq * vbus / imag
        vo_d = -we * lq * iq
        vo_q = r * iq + we * lam
        va_d = vo_d + (rds + r_dt) * i_d
        va_q = vo_q + (rds + r_dt) * iq

        lam_t = lam * self.kt_scale(abs(iq))
        tau = 1.5 * m.pole_pairs * lam_t * iq
        if m.cogging_amp:
            tau += m.cogging_amp * math.sin(m.cogging_periods * theta_m)
        p_iron, drag = self._iron_drag(omega_m)

        i2 = iq * iq
        p_cu = 1.5 * r * i2
        p_cond = 1.5 * (rds + r_dt) * i2
        p_sw = 1.5 * (2.0 / math.pi) * d.switch_time * d.pwm_freq * vbus * abs(iq)
        p_bus = 1.5 * vo_q * iq + p_cond + p_sw + bus.p.aux_load_w
        i_load = p_bus / vbus
        bus.step(i_load, dt)

        self.id, self.iq = i_d, iq
        self.id_meas, self.iq_meas = i_d, iq
        self.vd, self.vq = va_d, va_q
        self.v_mag, self.v_max = math.hypot(va_d, va_q), vmax
        self.v_sat = 1.0 if sat else 0.0
        self.p_copper, self.p_iron, self.p_fet = p_cu, p_iron, p_cond + p_sw
        self.p_bus, self.i_bus = p_bus, i_load
        return tau - drag
