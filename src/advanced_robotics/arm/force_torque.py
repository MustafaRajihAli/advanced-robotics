"""Force-torque compliance control.

Reads WrenchSample from the end-effector sensor and produces a velocity
correction so the arm backs off when it exceeds configured force/torque
limits (e.g. during contact-rich assembly tasks).
"""
from __future__ import annotations

import numpy as np

from advanced_robotics.core.errors import SafetyFaultError
from advanced_robotics.core.types import WrenchSample


class ComplianceController:
    def __init__(self, force_limit_n: float, torque_limit_nm: float, gain: float = 0.001) -> None:
        self.force_limit_n = force_limit_n
        self.torque_limit_nm = torque_limit_nm
        self.gain = gain

    def check_limits(self, sample: WrenchSample) -> None:
        force_mag = float(np.linalg.norm(sample.force_n))
        torque_mag = float(np.linalg.norm(sample.torque_nm))
        if force_mag > self.force_limit_n:
            raise SafetyFaultError(
                f"Force limit exceeded: {force_mag:.1f}N > {self.force_limit_n:.1f}N"
            )
        if torque_mag > self.torque_limit_nm:
            raise SafetyFaultError(
                f"Torque limit exceeded: {torque_mag:.1f}Nm > {self.torque_limit_nm:.1f}Nm"
            )

    def velocity_correction(self, sample: WrenchSample) -> tuple[float, float, float]:
        """Small Cartesian velocity correction (m/s) proportional to sensed force,
        used to comply with contact rather than fight it. Caller is responsible
        for calling check_limits() first and stopping on SafetyFaultError."""
        fx, fy, fz = sample.force_n
        return (-self.gain * fx, -self.gain * fy, -self.gain * fz)
