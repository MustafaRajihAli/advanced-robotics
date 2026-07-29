"""Digital twin simulation adapter.

Defines the interface AMR/arm/vision code talks to whether it's running
against real hardware or a simulated world, so the same Navigator/
MotionPlanner/DefectDetector code paths get exercised pre-deployment.
Concrete backends (Gazebo, Isaac Sim) implement SimulatorBackend.
"""
from __future__ import annotations

from typing import Protocol

from advanced_robotics.core.types import Pose2D
from advanced_robotics.vision.camera_fusion import CameraFrame, LidarScan


class SimulatorBackend(Protocol):
    def reset(self, world: str) -> None: ...

    def step(self, dt_s: float) -> None: ...

    def get_robot_pose(self, robot_id: str) -> Pose2D: ...

    def set_robot_velocity(self, robot_id: str, linear_mps: float, angular_radps: float) -> None: ...

    def get_camera_frame(self, camera_id: str) -> CameraFrame: ...

    def get_lidar_scan(self, lidar_id: str) -> LidarScan: ...


class NullSimulator:
    """No-op backend so digital_twin-dependent code imports cleanly before a
    real Gazebo/Isaac Sim adapter is wired in."""

    def reset(self, world: str) -> None:
        pass

    def step(self, dt_s: float) -> None:
        pass

    def get_robot_pose(self, robot_id: str) -> Pose2D:
        return Pose2D(0.0, 0.0, 0.0)

    def set_robot_velocity(self, robot_id: str, linear_mps: float, angular_radps: float) -> None:
        pass

    def get_camera_frame(self, camera_id: str) -> CameraFrame:
        raise NotImplementedError("NullSimulator has no camera feed; use a real backend")

    def get_lidar_scan(self, lidar_id: str) -> LidarScan:
        raise NotImplementedError("NullSimulator has no lidar feed; use a real backend")
