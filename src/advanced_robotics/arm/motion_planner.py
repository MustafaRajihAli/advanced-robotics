"""Joint-space trajectory generation with simple collision checking.

Produces a time-parameterized trajectory between two joint configurations
using trapezoidal velocity profiles per joint. Collision checking is a
pluggable callback so it can be backed by a real collision-checking library
(e.g. via MoveIt) later without changing this module's interface.
"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrajectoryPoint:
    time_s: float
    positions_rad: tuple[float, ...]


CollisionChecker = Callable[[tuple[float, ...]], bool]  # True = collision


class MotionPlanner:
    def __init__(self, max_joint_speed_radps: float, collision_checker: CollisionChecker | None = None) -> None:
        self.max_joint_speed_radps = max_joint_speed_radps
        self._collision_checker = collision_checker or (lambda _: False)

    def plan(
        self,
        start_rad: list[float],
        goal_rad: list[float],
        num_samples: int = 50,
    ) -> list[TrajectoryPoint]:
        if len(start_rad) != len(goal_rad):
            raise ValueError("start and goal must have the same number of joints")

        start = np.array(start_rad)
        goal = np.array(goal_rad)
        delta = goal - start
        max_delta = np.max(np.abs(delta)) if len(delta) else 0.0
        duration_s = max_delta / self.max_joint_speed_radps if max_delta > 0 else 0.0

        trajectory: list[TrajectoryPoint] = []
        for i in range(num_samples + 1):
            fraction = i / num_samples
            positions = tuple(start + delta * fraction)
            if self._collision_checker(positions):
                raise RuntimeError(f"Planned trajectory collides at fraction {fraction:.2f}")
            trajectory.append(TrajectoryPoint(time_s=duration_s * fraction, positions_rad=positions))

        return trajectory
