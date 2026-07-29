"""Path planning + dynamic obstacle avoidance for a single AMR.

Delegates localization to a SlamBackend. The planner here is intentionally
simple (straight-line + reactive avoidance) so it's testable without Nav2;
swap in a Nav2 client for the real deployment.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from advanced_robotics.amr.slam import SlamBackend
from advanced_robotics.core.types import Pose2D


@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    radius_m: float


class Navigator:
    def __init__(
        self,
        slam: SlamBackend,
        max_linear_speed_mps: float,
        max_angular_speed_radps: float,
        obstacle_safety_margin_m: float,
    ) -> None:
        self._slam = slam
        self._max_linear_speed_mps = max_linear_speed_mps
        self._max_angular_speed_radps = max_angular_speed_radps
        self._safety_margin_m = obstacle_safety_margin_m

    def compute_velocity_command(
        self, goal: Pose2D, obstacles: list[Obstacle]
    ) -> tuple[float, float]:
        """Return (linear_mps, angular_radps) toward `goal`, braking for obstacles."""
        pose = self._slam.get_pose()
        dx = goal.x - pose.x
        dy = goal.y - pose.y
        distance = math.hypot(dx, dy)
        if distance < 1e-3:
            return 0.0, 0.0

        heading_to_goal = math.atan2(dy, dx)
        heading_error = _wrap_angle(heading_to_goal - pose.yaw_rad)

        linear = min(self._max_linear_speed_mps, distance)
        angular = max(
            -self._max_angular_speed_radps,
            min(self._max_angular_speed_radps, heading_error),
        )

        closest = self._closest_obstacle_distance(pose, obstacles)
        if closest is not None and closest < self._safety_margin_m:
            linear = 0.0

        return linear, angular

    @staticmethod
    def _closest_obstacle_distance(pose: Pose2D, obstacles: list[Obstacle]) -> float | None:
        if not obstacles:
            return None
        return min(
            math.hypot(o.x - pose.x, o.y - pose.y) - o.radius_m for o in obstacles
        )


def _wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi
