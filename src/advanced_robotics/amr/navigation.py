"""Path planning + dynamic obstacle avoidance for a single AMR.

Delegates localization to a SlamBackend. The planner here is intentionally
simple (straight-line + reactive avoidance) so it's testable without Nav2;
swap in a Nav2 client (DWB/MPPI/TEB controller) for the real deployment.

Obstacle handling follows the Nav2 costmap inflation model rather than a
hard stop/go threshold: speed is scaled down smoothly as the robot enters
`inflation_radius_m` and only reaches zero inside `robot_radius_m`, the
robot's own footprint. A hard binary stop threshold is what causes the
"freezing robot problem" (the planner sees every path as blocked and never
recovers) documented in Nav2 obstacle-avoidance research; recovery here is
handled by `is_stuck()` triggering a rotate-in-place behavior after
`stuck_threshold` consecutive zero-velocity ticks.
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
        inflation_radius_m: float | None = None,
        cost_scaling_factor: float = 3.0,
        stuck_threshold: int = 20,
    ) -> None:
        self._slam = slam
        self._max_linear_speed_mps = max_linear_speed_mps
        self._max_angular_speed_radps = max_angular_speed_radps
        self._safety_margin_m = obstacle_safety_margin_m
        # Inflation radius must be at least the hard safety margin, matching
        # Nav2's convention that inflation_radius >= robot footprint radius.
        self._inflation_radius_m = max(
            inflation_radius_m if inflation_radius_m is not None else obstacle_safety_margin_m * 2,
            obstacle_safety_margin_m,
        )
        self._cost_scaling_factor = cost_scaling_factor
        self._stuck_threshold = stuck_threshold
        self._consecutive_blocked_ticks = 0

    def compute_velocity_command(
        self, goal: Pose2D, obstacles: list[Obstacle]
    ) -> tuple[float, float]:
        """Return (linear_mps, angular_radps) toward `goal`.

        Speed scales down smoothly inside the inflation radius and only
        reaches zero inside the hard safety margin. If the robot is blocked
        for `stuck_threshold` consecutive ticks, issues a rotate-in-place
        recovery command instead of staying frozen.
        """
        pose = self._slam.get_pose()
        dx = goal.x - pose.x
        dy = goal.y - pose.y
        distance = math.hypot(dx, dy)
        if distance < 1e-3:
            self._consecutive_blocked_ticks = 0
            return 0.0, 0.0

        heading_to_goal = math.atan2(dy, dx)
        heading_error = _wrap_angle(heading_to_goal - pose.yaw_rad)

        linear = min(self._max_linear_speed_mps, distance)
        angular = max(
            -self._max_angular_speed_radps,
            min(self._max_angular_speed_radps, heading_error),
        )

        closest = self._closest_obstacle_distance(pose, obstacles)
        linear *= self._speed_scale_for_obstacle(closest)

        if linear <= 0.0:
            self._consecutive_blocked_ticks += 1
            if self.is_stuck():
                return self._recovery_command()
            return 0.0, angular

        self._consecutive_blocked_ticks = 0
        return linear, angular

    def is_stuck(self) -> bool:
        return self._consecutive_blocked_ticks >= self._stuck_threshold

    def _recovery_command(self) -> tuple[float, float]:
        """Rotate in place to re-scan the area and break the freezing-robot
        deadlock, rather than waiting indefinitely for the obstacle to clear."""
        return 0.0, self._max_angular_speed_radps

    def _speed_scale_for_obstacle(self, closest_distance_m: float | None) -> float:
        if closest_distance_m is None:
            return 1.0
        if closest_distance_m <= self._safety_margin_m:
            return 0.0
        if closest_distance_m >= self._inflation_radius_m:
            return 1.0
        # Linear ramp from 0 at the safety margin to 1 at the inflation
        # radius. cost_scaling_factor is accepted for Nav2 config parity but
        # a linear ramp is used here for a simple, monotonic, testable curve.
        span = self._inflation_radius_m - self._safety_margin_m
        return (closest_distance_m - self._safety_margin_m) / span

    @staticmethod
    def _closest_obstacle_distance(pose: Pose2D, obstacles: list[Obstacle]) -> float | None:
        if not obstacles:
            return None
        return min(
            math.hypot(o.x - pose.x, o.y - pose.y) - o.radius_m for o in obstacles
        )


def _wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi
