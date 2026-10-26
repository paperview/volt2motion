"""Lumped thermal model: motor winding, motor housing and driver MOSFETs."""
from __future__ import annotations

from .params import ThermalParams


class ThermalModel:
    """Winding -> housing -> ambient, plus MOSFETs -> ambient through the PCB.

    The rotor magnets are assumed to sit at housing temperature.
    """

    def __init__(self, p: ThermalParams):
        self.p = p
        self.t_winding = p.ambient_c
        self.t_housing = p.ambient_c
        self.t_fet = p.ambient_c

    def step(self, p_copper: float, p_iron: float, p_fet: float, dt: float) -> None:
        p = self.p
        q_wh = (self.t_winding - self.t_housing) / p.r_winding_housing
        q_ha = (self.t_housing - p.ambient_c) / p.r_housing_ambient
        q_fa = (self.t_fet - p.ambient_c) / p.r_fet_ambient
        self.t_winding += dt * (p_copper - q_wh) / p.winding_capacity
        self.t_housing += dt * (q_wh + p_iron - q_ha) / p.housing_capacity
        self.t_fet += dt * (p_fet - q_fa) / p.fet_capacity

    def steady_state(self, p_copper: float, p_iron: float, p_fet: float) -> tuple[float, float]:
        """Winding and MOSFET temperatures if these average losses ran forever."""
        p = self.p
        t_housing = p.ambient_c + (p_copper + p_iron) * p.r_housing_ambient
        t_winding = t_housing + p_copper * p.r_winding_housing
        t_fet = p.ambient_c + p_fet * p.r_fet_ambient
        return t_winding, t_fet
