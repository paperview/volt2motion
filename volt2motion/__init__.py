"""Voltage-to-motion simulator for one robot joint, with a calibration path for real hardware."""
from .params import JointDesign
from .sim import simulate, summarize, print_summary, hardware_view
from .trajectory import pick_and_place, calibration_routine

__all__ = [
    "JointDesign", "simulate", "summarize", "print_summary", "hardware_view",
    "pick_and_place", "calibration_routine",
]
