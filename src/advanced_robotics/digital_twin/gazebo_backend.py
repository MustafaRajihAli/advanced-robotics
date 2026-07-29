"""Gazebo backend for ROS 2-integrated digital twin validation.

Per current sim-to-real practice, Gazebo sits between Isaac Sim training
and real hardware: it runs the same ROS 2 topics/interfaces the physical
robot uses (via ros_gz bridges), so a policy or control loop validated here
needs no code changes to run on the real AMR/arm through
`ros2_ws/src/novus_robotics_bridge`. Talks to Gazebo over ROS 2 topics/
services rather than a native Gazebo API, so it only needs `rclpy` at
runtime (imported lazily, same reasoning as the ROS 2 bridge node).
"""
from __future__ import annotations

from advanced_robotics.core.types import Pose2D
from advanced_robotics.digital_twin.simulator import SimulatorBackend
from advanced_robotics.vision.camera_fusion import CameraFrame, LidarScan


class GazeboBackend:
    """SimulatorBackend implementation that drives a Gazebo world through
    the same ROS 2 topics (`/odom`, `/cmd_vel`, camera/lidar topics) the
    real robot bridge uses, for hardware-representative validation."""

    def __init__(self, robot_namespace: str = "") -> None:
        self.robot_namespace = robot_namespace
        self._node = None

    def reset(self, world: str) -> None:
        # import rclpy
        # from novus_robotics_bridge.bridge_node import BridgeNode
        # rclpy.init()
        # self._node = BridgeNode()
        # call the /reset_world or gz service to load `world`
        raise NotImplementedError(
            "Requires a running Gazebo + ROS 2 environment. "
            "Wire this up to novus_robotics_bridge.BridgeNode before use."
        )

    def step(self, dt_s: float) -> None:
        raise NotImplementedError

    def get_robot_pose(self, robot_id: str) -> Pose2D:
        raise NotImplementedError

    def set_robot_velocity(self, robot_id: str, linear_mps: float, angular_radps: float) -> None:
        raise NotImplementedError

    def get_camera_frame(self, camera_id: str) -> CameraFrame:
        raise NotImplementedError

    def get_lidar_scan(self, lidar_id: str) -> LidarScan:
        raise NotImplementedError


_: type[SimulatorBackend] = GazeboBackend
