"""Design parameters for one robot joint.

Covers the motor, the driver PCB, the gearbox, the battery and power bus, the
thermal paths and the arm the joint moves.

Phase 1 (design) fills these from datasheets, CAD and FEA. Phase 2
(calibration) overwrites some of them with values fitted from real logs.
A design is plain JSON, so you can copy one, edit a number and rerun.

Each parameter carries its unit and a description. "volt2motion params" prints
them, and docs/parameters.md is generated from them.
"""
from __future__ import annotations

import difflib
import json
import math
from dataclasses import MISSING, asdict, dataclass, field, fields, is_dataclass
from pathlib import Path


def P(default, unit: str, doc: str):
    """A design parameter with its unit and a one-line description."""
    meta = {"unit": unit, "doc": doc}
    if isinstance(default, list):
        return field(default_factory=lambda: list(default), metadata=meta)
    return field(default=default, metadata=meta)


@dataclass
class MotorParams:
    """The motor. Per-phase values, star (wye) equivalent."""
    pole_pairs: int = P(7, "", "Magnet pole pairs, half the magnet count. A 12N14P motor has 7.")
    flux_linkage: float = P(0.008, "Wb", "Magnet flux linkage at 20 C, phase peak. Sets torque per amp "
                            "(Kt = 1.5 x pole_pairs x flux_linkage) and back-EMF. See the design file "
                            "guide for converting Kv or Ke.")
    resistance: float = P(0.15, "ohm", "Winding resistance per phase at 20 C. Half the line-to-line value.")
    ld: float = P(120e-6, "H", "d-axis inductance per phase. Half the line-to-line value. "
                  "Same as lq for surface-magnet motors.")
    lq: float = P(120e-6, "H", "q-axis inductance per phase. Half the line-to-line value.")
    rotor_inertia: float = P(3.0e-5, "kg m^2", "Rotor inertia, from the datasheet or CAD.")
    copper_tempco: float = P(0.00393, "1/K", "Winding resistance rise per K. 0.00393 for copper.")
    magnet_tempco: float = P(-0.0012, "1/K", "Flux change per K. About -0.0012 for NdFeB, -0.0003 for SmCo.")
    iron_loss_hyst: float = P(2e-3, "W/(rad/s)", "Hysteresis iron loss per rad/s of rotor speed.")
    iron_loss_eddy: float = P(1e-5, "W/(rad/s)^2", "Eddy current iron loss per (rad/s)^2 of rotor speed.")
    cogging_amp: float = P(0.0, "Nm", "Cogging torque amplitude at the motor shaft. 0 turns it off.")
    cogging_periods: int = P(84, "", "Cogging cycles per rotor turn: the least common multiple of the "
                             "slot and magnet counts (84 for 12N14P).")
    kt_curve_current: list[float] = P([], "A", "Optional FEA saturation curve: |iq| points. Leave empty "
                                      "for no saturation.")
    kt_curve_scale: list[float] = P([], "", "Torque constant scale at each kt_curve_current point, "
                                    "1.0 meaning no saturation.")

    @property
    def kt(self) -> float:
        """Torque constant at 20 C in Nm per A (amplitude-invariant dq)."""
        return 1.5 * self.pole_pairs * self.flux_linkage


@dataclass
class DriverParams:
    """The motor driver PCB and its firmware."""
    pwm_freq: float = P(20_000.0, "Hz", "PWM frequency, also the current loop rate.")
    rds_on: float = P(0.004, "ohm", "Series resistance per phase: MOSFET on-resistance plus copper plus "
                      "the shunt's share. With rds_tempco 0 this is a fixed hot value. import-kicad sets "
                      "the 25 C value and a tempco.")
    rds_tempco: float = P(0.0, "1/K", "Rise of rds_on per K of MOSFET temperature above 25 C. 0 keeps "
                          "rds_on fixed.")
    switch_time: float = P(60e-9, "s", "MOSFET rise plus fall time. Sets switching loss.")
    dead_time: float = P(300e-9, "s", "Dead time between high and low side switching.")
    max_modulation: float = P(0.95, "", "Usable fraction of the SVPWM voltage limit (bus voltage / sqrt 3).")
    current_limit: float = P(40.0, "A", "Peak phase current limit set in firmware.")
    current_loop_bw_hz: float = P(1500.0, "Hz", "Current loop bandwidth.")
    current_sense_noise: float = P(0.05, "A", "Current sensor noise, rms per sample.")
    current_sense_lsb: float = P(0.02, "A", "Current per ADC count.")
    encoder_bits: int = P(14, "", "Motor-side absolute encoder resolution in bits.")
    derate_start_c: float = P(100.0, "C", "Winding temperature where the current limit starts to drop.")
    derate_end_c: float = P(120.0, "C", "Winding temperature where the current limit reaches zero.")
    fet_derate_start_c: float = P(100.0, "C", "MOSFET temperature where the current limit starts to drop.")
    fet_derate_end_c: float = P(125.0, "C", "MOSFET temperature where the current limit reaches zero.")


@dataclass
class GearboxParams:
    """The gearbox. Friction values are at the output shaft."""
    ratio: float = P(50.0, "", "Reduction ratio: motor turns per output turn.")
    efficiency: float = P(0.75, "", "Efficiency at rated load. The loss grows with load: "
                          "(1 - efficiency) x |input torque|.")
    coulomb: float = P(2.0, "Nm", "Constant friction torque. Hard to know before building; "
                       "calibrate fits it.")
    viscous: float = P(1.5, "Nm s/rad", "Friction torque per rad/s of output speed. Calibrate fits it.")
    rated_peak_torque: float = P(150.0, "Nm", "Repeated peak torque rating from the gearbox datasheet.")
    stribeck_amp: float = P(0.0, "Nm", "Extra breakaway friction at standstill. Usually left at 0.")
    stribeck_vel: float = P(0.05, "rad/s", "Speed where breakaway friction fades out.")
    position_friction_amp: float = P(0.0, "Nm", "Friction that varies once per output turn. Usually left at 0.")


@dataclass
class BatteryParams:
    """The Li-ion pack, wiring and DC bus."""
    cells_series: int = P(13, "", "Li-ion cells in series. 13 is about 48 V nominal.")
    capacity_ah: float = P(20.0, "Ah", "Pack capacity.")
    internal_resistance: float = P(0.08, "ohm", "Pack plus wiring resistance.")
    soc_init: float = P(0.9, "", "State of charge at the start of a run, 0 to 1.")
    accepts_regen: bool = P(True, "", "false for a bench supply that cannot sink current.")
    bus_capacitance: float = P(660e-6, "F", "Capacitance on the driver board's DC bus. import-kicad sets it.")
    brake_on_v: float = P(56.0, "V", "Bus voltage where the brake chopper switches on.")
    brake_off_v: float = P(55.0, "V", "Bus voltage where the brake chopper switches off.")
    brake_resistance: float = P(10.0, "ohm", "Brake resistor. import-kicad sets it.")
    overvoltage_v: float = P(60.0, "V", "The driver faults above this bus voltage.")
    undervoltage_v: float = P(39.0, "V", "The driver faults below this bus voltage.")
    aux_load_w: float = P(3.0, "W", "Driver logic, encoder and CAN transceiver.")


@dataclass
class ThermalParams:
    """Winding, housing and MOSFET heat paths."""
    ambient_c: float = P(30.0, "C", "Air temperature around the joint.")
    winding_capacity: float = P(40.0, "J/K", "Heat capacity of the windings: copper mass in kg x 385.")
    housing_capacity: float = P(350.0, "J/K", "Heat capacity of stator and housing: mass in kg x about 460 "
                                "for steel or 900 for aluminium.")
    r_winding_housing: float = P(1.0, "K/W", "Thermal resistance from windings to housing.")
    r_housing_ambient: float = P(1.5, "K/W", "Thermal resistance from housing to air, including whatever "
                                 "the motor is bolted to.")
    fet_capacity: float = P(5.0, "J/K", "Heat capacity of the MOSFETs and nearby copper. import-kicad sets it.")
    r_fet_ambient: float = P(4.0, "K/W", "Thermal resistance from the MOSFETs to air. import-kicad sets it.")
    winding_limit_c: float = P(120.0, "C", "Highest allowed winding temperature: the insulation class "
                               "limit minus a margin.")
    fet_limit_c: float = P(125.0, "C", "Highest allowed MOSFET temperature.")


@dataclass
class ArmParams:
    """The link the joint moves. Angle 0 is horizontal, positive lifts."""
    link_length: float = P(0.5, "m", "Joint axis to the payload.")
    link_mass: float = P(3.0, "kg", "Link mass, modeled as a uniform rod from the joint to the payload.")
    payload_mass: float = P(5.0, "kg", "Heaviest item the arm carries, as a point mass at the link's end.")


@dataclass
class ControlParams:
    """Joint position loop running on the driver (or the main computer)."""
    rate_hz: float = P(1000.0, "Hz", "Position loop rate.")
    bandwidth_hz: float = P(8.0, "Hz", "Position loop bandwidth. Higher tracks tighter but needs more "
                            "current and a stiffer structure.")
    damping_ratio: float = P(0.8, "", "Position loop damping ratio.")
    vel_filter_hz: float = P(150.0, "Hz", "Cutoff of the velocity estimate's low-pass filter.")
    use_feedforward: bool = P(True, "", "Add model-based gravity, inertia and friction torque to the "
                              "loop output.")


@dataclass
class JointDesign:
    name: str = P("baseline", "", "Design name, used for output file names.")
    motor: MotorParams = field(default_factory=MotorParams)
    driver: DriverParams = field(default_factory=DriverParams)
    gearbox: GearboxParams = field(default_factory=GearboxParams)
    battery: BatteryParams = field(default_factory=BatteryParams)
    thermal: ThermalParams = field(default_factory=ThermalParams)
    arm: ArmParams = field(default_factory=ArmParams)
    control: ControlParams = field(default_factory=ControlParams)

    @property
    def armature(self) -> float:
        """Rotor inertia reflected through the gearbox, kg m^2 at the joint."""
        return self.motor.rotor_inertia * self.gearbox.ratio ** 2

    @property
    def encoder_resolution(self) -> float:
        """Motor-side encoder step in radians."""
        return 2 * math.pi / 2 ** self.driver.encoder_bits

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "JointDesign":
        """Build a design from a dict. Missing values take their defaults.

        Unknown keys raise ValueError, so a typo cannot be silently ignored.
        Keys starting with "_" are comments and are skipped.
        """
        return _build(cls, data, "")

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2) + "\n")

    @classmethod
    def load(cls, path: str | Path) -> "JointDesign":
        try:
            data = json.loads(Path(path).read_text())
        except json.JSONDecodeError as e:
            raise ValueError(f"{path}: not valid JSON (line {e.lineno}, column {e.colno}: {e.msg})") from None
        try:
            return cls.from_dict(data)
        except ValueError as e:
            raise ValueError(f"{path}: {e}") from None

    def updated(self, changes: dict[str, object], name: str | None = None) -> "JointDesign":
        """Return a copy with dotted-key changes, e.g. {"gearbox.ratio": 60}."""
        data = self.to_dict()
        for key, value in changes.items():
            node = data
            *parents, leaf = key.split(".")
            for p in parents:
                if not isinstance(node.get(p), dict):
                    raise KeyError(_unknown(key, parameter_keys()))
                node = node[p]
            if leaf not in node or isinstance(node[leaf], dict):
                raise KeyError(_unknown(key, parameter_keys()))
            node[leaf] = value
        if name is not None:
            data["name"] = name
        return JointDesign.from_dict(data)


def parameter_reference() -> list[tuple[str, str, object, str]]:
    """Every design parameter as (dotted key, unit, default, description)."""
    rows = []
    default = JointDesign()
    for f in fields(JointDesign):
        if f.metadata:
            rows.append((f.name, f.metadata["unit"], getattr(default, f.name), f.metadata["doc"]))
            continue
        section = getattr(default, f.name)
        for sf in fields(section):
            rows.append((f"{f.name}.{sf.name}", sf.metadata["unit"], getattr(section, sf.name), sf.metadata["doc"]))
    return rows


def section_docs() -> dict[str, str]:
    """One-line description of each section, from the class docstrings."""
    return {f.name: (f.default_factory.__doc__ or "").strip()
            for f in fields(JointDesign) if f.default_factory is not MISSING}


def parameter_keys() -> list[str]:
    return [row[0] for row in parameter_reference()]


def _unknown(key: str, known: list[str]) -> str:
    msg = f"unknown design parameter '{key}'"
    close = difflib.get_close_matches(key, known, n=1, cutoff=0.6)
    if close:
        msg += f". Did you mean '{close[0]}'?"
    return msg + " (volt2motion params lists them all)"


def _build(cls, data: dict, prefix: str):
    if not isinstance(data, dict):
        raise ValueError(f"'{prefix.rstrip('.') or 'design'}' should be a JSON object")
    known = {f.name: f for f in fields(cls)}
    for key in data:
        if not key.startswith("_") and key not in known:
            raise ValueError(_unknown(prefix + key, [prefix + k for k in known]))
    kwargs = {}
    for name, f in known.items():
        if name not in data:
            continue
        value = data[name]
        sub = f.default_factory
        if sub is not MISSING and is_dataclass(sub):
            value = _build(sub, value, f"{prefix}{name}.")
        kwargs[name] = value
    return cls(**kwargs)
