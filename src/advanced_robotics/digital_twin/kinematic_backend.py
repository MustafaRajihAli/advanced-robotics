"""Dependency-free kinematic simulator backend.

`IsaacSimBackend` and `GazeboBackend` need a GPU stack and a ROS 2 install
respectively, so neither can run in CI or on a developer laptop. This backend
implements the same `SimulatorBackend` interface with an explicit differential-
drive integration, so the whole task pipeline (plant task -> allocation ->
safety-gated velocity commands -> inspection -> status report) executes for
real in tests and in `scripts/run_demo.py`.

It models kinematics only -- no dynamics, no wheel slip, no sensor noise.
That is enough to validate control flow and safety gating; use Gazebo for
physics fidelity and Isaac Sim for policy training.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from advanced_robotics.amr.navigation import Obstacle
from advanced_robotics.core.types import Pose2D
from advanced_robotics.vision.camera_fusion import CameraFrame, LidarScan


@dataclass
class SimRobot:
    pose: Pose2D
    linear_mps: float = 0.0
    angular_radps: float = 0.0


@dataclass
class KinematicSimulator:
    """Differential-drive world with static circular obstacles.

    `obstacle_sense_radius_m` bounds what `obstacles_near` reports, mirroring
    a real LiDAR's range rather than handing the navigator global knowledge.
    """

    world: str = "warehouse_basic"
    obstacles: list[Obstacle] = field(default_factory=list)
    obstacle_sense_radius_m: float = 5.0
    camera_resolution: tuple[int, int] = (64, 64)
    # Camera ids listed here render a dark patch, standing in for a physical
    # defect on the inspected part so the vision path has something to find.
    planted_defect_camera_ids: frozenset[str] = frozenset()

    _robots: dict[str, SimRobot] = field(default_factory=dict, init=False)
    _initial_poses: dict[str, Pose2D] = field(default_factory=dict, init=False)
    _elapsed_s: float = field(default=0.0, init=False)

    # -- world setup ------------------------------------------------------

    def add_robot(self, robot_id: str, pose: Pose2D) -> None:
        self._robots[robot_id] = SimRobot(pose=Pose2D(pose.x, pose.y, pose.yaw_rad))
        self._initial_poses[robot_id] = Pose2D(pose.x, pose.y, pose.yaw_rad)

    @property
    def elapsed_s(self) -> float:
        return self._elapsed_s

    @property
    def robot_ids(self) -> list[str]:
        return list(self._robots)

    # -- SimulatorBackend -------------------------------------------------

    def reset(self, world: str) -> None:
        self.world = world
        self._elapsed_s = 0.0
        for robot_id, pose in self._initial_poses.items():
            self._robots[robot_id] = SimRobot(pose=Pose2D(pose.x, pose.y, pose.yaw_rad))

    def step(self, dt_s: float) -> None:
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        for robot in self._robots.values():
            pose = robot.pose
            # Integrate at the midpoint heading: with a plain Euler step a
            # robot turning while driving drifts off its true arc, which shows
            # up as a navigator that never converges on its goal.
            mid_yaw = pose.yaw_rad + robot.angular_radps * dt_s / 2.0
            pose.x += robot.linear_mps * math.cos(mid_yaw) * dt_s
            pose.y += robot.linear_mps * math.sin(mid_yaw) * dt_s
            pose.yaw_rad = _wrap_angle(pose.yaw_rad + robot.angular_radps * dt_s)
        self._elapsed_s += dt_s

    def get_robot_pose(self, robot_id: str) -> Pose2D:
        return self._require(robot_id).pose

    def set_robot_velocity(self, robot_id: str, linear_mps: float, angular_radps: float) -> None:
        robot = self._require(robot_id)
        robot.linear_mps = linear_mps
        robot.angular_radps = angular_radps

    def get_camera_frame(self, camera_id: str) -> CameraFrame:
        height, width = self.camera_resolution
        image = np.full((height, width, 3), 160, dtype=np.uint8)
        if camera_id in self.planted_defect_camera_ids:
            h0, w0 = height // 3, width // 3
            image[h0 : h0 + height // 8, w0 : w0 + width // 8] = 20
        return CameraFrame(camera_id=camera_id, image=image, timestamp_s=self._elapsed_s)

    def get_lidar_scan(self, lidar_id: str) -> LidarScan:
        """Points sampled on each obstacle's perimeter -- enough for fusion
        timestamp checks, not a raytraced scan."""
        points: list[tuple[float, float, float]] = []
        for obstacle in self.obstacles:
            for i in range(16):
                angle = 2 * math.pi * i / 16
                points.append(
                    (
                        obstacle.x + obstacle.radius_m * math.cos(angle),
                        obstacle.y + obstacle.radius_m * math.sin(angle),
                        0.0,
                    )
                )
        array = np.array(points, dtype=np.float64) if points else np.empty((0, 3))
        return LidarScan(lidar_id=lidar_id, points_xyz=array, timestamp_s=self._elapsed_s)

    # -- extras used by the orchestrator ---------------------------------

    def obstacles_near(self, robot_id: str) -> list[Obstacle]:
        pose = self.get_robot_pose(robot_id)
        return [
            o
            for o in self.obstacles
            if math.hypot(o.x - pose.x, o.y - pose.y) - o.radius_m
            <= self.obstacle_sense_radius_m
        ]

    def _require(self, robot_id: str) -> SimRobot:
        try:
            return self._robots[robot_id]
        except KeyError:
            raise KeyError(f"unknown robot '{robot_id}' in world '{self.world}'") from None


def _wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi
