"""NVIDIA Isaac Sim backend for RL training and high-fidelity digital twin work.

Isaac Sim (via Isaac Lab / Isaac Sim Gym) is the recommended training
environment for sim-to-real RL policies: it gives GPU-accelerated
parallel simulation and closer visual/physical fidelity than Gazebo,
which research on mobile-robot sim-to-real transfer uses specifically for
training before handing the trained policy to Gazebo and then real ROS 2
hardware for validation (see WORK_PLAN.md Phase 6).

This module defines the connection contract; the actual Isaac Sim Python
API (`omni.isaac.*` / `isaacsim.*`) is only importable inside an Isaac Sim
process, so it's imported lazily to keep this package installable without
Isaac Sim present.
"""
from __future__ import annotations

from advanced_robotics.core.types import Pose2D
from advanced_robotics.digital_twin.simulator import SimulatorBackend
from advanced_robotics.vision.camera_fusion import CameraFrame, LidarScan


class IsaacSimBackend:
    """SimulatorBackend implementation targeting NVIDIA Isaac Sim.

    Intended use: RL training for AMR navigation / arm manipulation
    policies at scale (many parallel envs), then export the trained policy
    for validation against GazeboBackend before real hardware deployment.
    """

    def __init__(self, headless: bool = True, num_envs: int = 1) -> None:
        self.headless = headless
        self.num_envs = num_envs
        self._world = None  # set in reset(); typed as Any to avoid a hard Isaac Sim import

    def reset(self, world: str) -> None:
        # from isaacsim.simulation_app import SimulationApp
        # self._simulation_app = SimulationApp({"headless": self.headless})
        # from omni.isaac.core import World
        # self._world = World()
        # self._world.scene.add_default_ground_plane()
        # ... load `world` USD stage here ...
        raise NotImplementedError(
            "Isaac Sim not installed in this environment. "
            "Install isaacsim and implement world loading before use."
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


# Type-check that this satisfies the shared interface without instantiating it.
_: type[SimulatorBackend] = IsaacSimBackend
