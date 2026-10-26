"""Battery, DC bus capacitor and brake chopper."""
from __future__ import annotations

import numpy as np

from .params import BatteryParams

# Open-circuit voltage per Li-ion cell versus state of charge.
_OCV_SOC = np.array([0.0, 0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 0.9, 1.0])
_OCV_V = np.array([3.00, 3.30, 3.45, 3.55, 3.65, 3.78, 3.95, 4.05, 4.18])


class PowerBus:
    """Battery behind an internal resistance, feeding the driver's bus capacitor.

    The update is implicit in the bus voltage, so it stays stable at any step
    size from PWM periods up to the 1 ms mechanics step.
    """

    def __init__(self, p: BatteryParams):
        self.p = p
        self.soc = p.soc_init
        self.vbus = self.ocv()
        self.i_batt = 0.0
        self.i_brake = 0.0
        self.brake_on = False
        self.fault = ""
        self.brake_energy = 0.0   # J burned in the brake resistor
        self.batt_energy = 0.0    # J drawn from the battery (negative means charged)

    def ocv(self) -> float:
        return self.p.cells_series * float(np.interp(self.soc, _OCV_SOC, _OCV_V))

    def step(self, i_load: float, dt: float) -> None:
        """Advance by dt with the driver drawing i_load amps (negative during regen)."""
        p = self.p
        ocv = self.ocv()
        a = p.bus_capacitance / dt
        g_brake = 1.0 / p.brake_resistance if self.brake_on else 0.0
        g_batt = 1.0 / p.internal_resistance
        v = (a * self.vbus + ocv * g_batt - i_load) / (a + g_batt + g_brake)
        i_batt = (ocv - v) * g_batt
        if i_batt < 0.0 and not p.accepts_regen:
            # A supply that cannot sink current: regen charges the capacitor only.
            v = (a * self.vbus - i_load) / (a + g_brake)
            i_batt = 0.0
        self.vbus = v
        self.i_batt = i_batt
        self.i_brake = v * g_brake
        self.brake_energy += v * self.i_brake * dt
        self.batt_energy += ocv * i_batt * dt
        self.soc -= i_batt * dt / (p.capacity_ah * 3600.0)

        if v > p.brake_on_v:
            self.brake_on = True
        elif v < p.brake_off_v:
            self.brake_on = False
        if not self.fault:
            if v > p.overvoltage_v:
                self.fault = "overvoltage"
            elif v < p.undervoltage_v:
                self.fault = "undervoltage"
