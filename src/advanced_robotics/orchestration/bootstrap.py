"""Assembles a runnable, fully-wired simulation stack from config.

One place builds the object graph so the API, the demo script, and the tests
all exercise the same wiring. Swapping the kinematic backend for Gazebo or the
in-memory plant client for a real MES happens here and nowhere else.
"""
from __future__ import annotations

from dataclasses import dataclass

from advanced_robotics.amr.fleet_coordinator import FleetCoordinator, RobotState
from advanced_robotics.amr.navigation import Navigator, Obstacle
from advanced_robotics.arm.motion_planner import MotionPlanner
from advanced_robotics.core.config import AppConfig, load_config
from advanced_robotics.core.types import Pose2D
from advanced_robotics.digital_twin.kinematic_backend import KinematicSimulator
from advanced_robotics.digital_twin.simulator import SimulatorSlam
from advanced_robotics.integrations.in_memory_plant import InMemoryPlantClient
from advanced_robotics.orchestration.task_executor import TaskExecutor
from advanced_robotics.safety.estop import SoftwareEstop
from advanced_robotics.safety.safety_monitor import SafetyMonitor
from advanced_robotics.safety.ssm import SsmParameters
from advanced_robotics.vision.inspector import IntensityThresholdInspector


@dataclass
class SimulationStack:
    config: AppConfig
    simulator: KinematicSimulator
    coordinator: FleetCoordinator
    estop: SoftwareEstop
    safety: SafetyMonitor
    plant_client: InMemoryPlantClient
    executor: TaskExecutor


def build_simulation_stack(
    config: AppConfig | None = None,
    *,
    obstacles: list[Obstacle] | None = None,
    defect_camera_ids: frozenset[str] = frozenset(),
    fleet_spacing_m: float = 2.0,
) -> SimulationStack:
    config = config or load_config()

    simulator = KinematicSimulator(
        world=config.digital_twin.world,
        obstacles=list(obstacles or []),
        planted_defect_camera_ids=defect_camera_ids,
    )
    coordinator = FleetCoordinator()
    for index in range(config.amr.fleet_size):
        robot_id = f"amr{index}"
        pose = Pose2D(0.0, index * fleet_spacing_m, 0.0)
        simulator.add_robot(robot_id, pose)
        coordinator.register_robot(RobotState(robot_id=robot_id, pose=pose))

    estop = SoftwareEstop()
    safety = SafetyMonitor(
        estop=estop,
        heartbeat_timeout_ms=config.safety.estop_heartbeat_timeout_ms,
        ssm_params=SsmParameters(
            human_speed_mps=config.safety.ssm_human_speed_mps,
            reaction_time_s=config.safety.ssm_reaction_time_s,
            stopping_time_s=config.safety.ssm_stopping_time_s,
            intrusion_distance_m=config.safety.ssm_intrusion_distance_m,
        ),
    )
    plant_client = InMemoryPlantClient()

    def navigator_factory(robot_id: str) -> Navigator:
        return Navigator(
            slam=SimulatorSlam(simulator, robot_id),
            max_linear_speed_mps=config.amr.max_linear_speed_mps,
            max_angular_speed_radps=config.amr.max_angular_speed_radps,
            obstacle_safety_margin_m=config.amr.obstacle_safety_margin_m,
            inflation_radius_m=config.amr.inflation_radius_m,
            cost_scaling_factor=config.amr.cost_scaling_factor,
            stuck_threshold=config.amr.stuck_recovery_ticks,
        )

    executor = TaskExecutor(
        coordinator=coordinator,
        safety=safety,
        simulator=simulator,
        plant_client=plant_client,
        navigator_factory=navigator_factory,
        obstacle_source=simulator.obstacles_near,
        inspector=IntensityThresholdInspector(),
        motion_planner=MotionPlanner(max_joint_speed_radps=1.0),
        # Sim time advances far faster than wall-clock, so the loop pumps the
        # heartbeat that a hardware e-stop source would supply on its own.
        heartbeat=estop.heartbeat,
    )

    return SimulationStack(
        config=config,
        simulator=simulator,
        coordinator=coordinator,
        estop=estop,
        safety=safety,
        plant_client=plant_client,
        executor=executor,
    )
