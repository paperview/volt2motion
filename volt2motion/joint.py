"""MuJoCo model of one joint: a link with a gripper payload, driven through a gearbox.

Gearbox friction is applied through MuJoCo's own joint friction (frictionloss)
and damping. That gives true stick-slip at standstill and stays numerically
stable, which a hand-written friction force at a 1 ms step would not.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from .params import ArmParams, GearboxParams, JointDesign

_EMPTY_MASS = 1e-6


def joint_xml(arm: ArmParams, armature: float, timestep: float) -> str:
    L = arm.link_length
    return f"""
<mujoco model="single_joint">
  <option timestep="{timestep}" integrator="implicitfast" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="joint" type="hinge" axis="0 -1 0" armature="{armature}"/>
      <geom type="capsule" fromto="0 0 0 {L} 0 0" size="0.03" mass="{arm.link_mass}"
            contype="0" conaffinity="0"/>
      <body name="payload" pos="{L} 0 0">
        <inertial pos="0 0 0" mass="{arm.payload_mass}" diaginertia="1e-3 1e-3 1e-3"/>
        <geom type="box" size="0.05 0.05 0.05" contype="0" conaffinity="0" mass="0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


class JointModel:
    """Wraps a MuJoCo model and data for the single joint."""

    def __init__(self, design: JointDesign, timestep: float):
        self.design = design
        self.model = mujoco.MjModel.from_xml_string(
            joint_xml(design.arm, design.armature, timestep)
        )
        self.data = mujoco.MjData(self.model)
        self._payload_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "payload")
        self._payload_mass = design.arm.payload_mass
        self._payload_inertia = self.model.body_inertia[self._payload_id].copy()
        self.payload_on = True

    def set_payload(self, on: bool) -> None:
        scale = 1.0 if on else _EMPTY_MASS / max(self._payload_mass, _EMPTY_MASS)
        self.model.body_mass[self._payload_id] = max(self._payload_mass * scale, _EMPTY_MASS)
        self.model.body_inertia[self._payload_id] = self._payload_inertia * max(scale, 1e-6)
        self.payload_on = on

    def set_state(self, q: float, qd: float = 0.0) -> None:
        self.data.qpos[0] = q
        self.data.qvel[0] = qd
        mujoco.mj_forward(self.model, self.data)

    @property
    def q(self) -> float:
        return float(self.data.qpos[0])

    @property
    def qd(self) -> float:
        return float(self.data.qvel[0])

    def set_friction(self, coulomb: float, viscous: float) -> None:
        self.model.dof_frictionloss[0] = max(0.0, coulomb)
        self.model.dof_damping[0] = max(0.0, viscous)

    def step(self, tau_applied: float) -> None:
        self.data.qfrc_applied[0] = tau_applied
        mujoco.mj_step(self.model, self.data)

    def friction_torque(self) -> float:
        """Friction torque applied during the last step (positive opposes motion)."""
        return -float(self.data.qfrc_constraint[0] + self.data.qfrc_passive[0])

    def inverse_dynamics(self, q: float, qd: float, qdd: float) -> float:
        """Rigid-body torque needed for this motion, friction excluded."""
        d = self.data
        d.qpos[0], d.qvel[0], d.qacc[0] = q, qd, qdd
        mujoco.mj_inverse(self.model, d)
        return float(d.qfrc_inverse[0])

    def joint_inertia(self) -> float:
        """Total inertia at the joint (link, payload and reflected rotor)."""
        q = self.q
        return self.inverse_dynamics(q, 0.0, 1.0) - self.inverse_dynamics(q, 0.0, 0.0)


def gearbox_friction(g: GearboxParams, q: float, qd: float, tau_in: float) -> tuple[float, float]:
    """Coulomb friction magnitude and viscous coefficient at the output.

    The Coulomb part grows with transmitted torque, which is how gearbox
    efficiency shows up at the joint.
    """
    coulomb = (
        g.coulomb
        + g.stribeck_amp * math.exp(-((qd / g.stribeck_vel) ** 2))
        + g.position_friction_amp * math.sin(q)
        + (1.0 - g.efficiency) * abs(tau_in)
    )
    return coulomb, g.viscous


def smooth_sign(x: np.ndarray | float, eps: float = 0.02):
    return np.tanh(np.asarray(x) / eps)
