"""Phase 5 success criteria: a task received from a mock MES call is executed
end to end in simulation and the status is reported back."""
import math

import pytest

from advanced_robotics.amr.navigation import Obstacle
from advanced_robotics.core.config import load_config
from advanced_robotics.integrations.plant_client import PlantTask
from advanced_robotics.orchestration.bootstrap import build_simulation_stack
from advanced_robotics.orchestration.task_executor import (
    STATUS_BLOCKED,
    STATUS_COMPLETED,
    STATUS_HALTED,
    STATUS_REJECTED,
)

pytestmark = pytest.mark.asyncio


@pytest.fixture
def config():
    return load_config()


async def test_transport_task_runs_end_to_end_and_reports_back(config):
    stack = build_simulation_stack(config)
    stack.plant_client.enqueue(PlantTask("job-1", "transport", {"x": 5.0, "y": 0.0}))

    outcomes = await stack.executor.run_once()

    assert len(outcomes) == 1
    assert outcomes[0].status == STATUS_COMPLETED
    assert outcomes[0].robot_id == "amr0"  # nearest idle robot to the goal

    pose = stack.simulator.get_robot_pose("amr0")
    assert math.hypot(5.0 - pose.x, 0.0 - pose.y) <= stack.executor.goal_tolerance_m

    report = stack.plant_client.reports_for("job-1")[-1]
    assert report.status == STATUS_COMPLETED
    assert report.details["robot_id"] == "amr0"
    assert report.details["sim_time_s"] > 0


async def test_robot_is_released_after_task_completes(config):
    stack = build_simulation_stack(config)
    stack.plant_client.enqueue(PlantTask("job-1", "transport", {"x": 2.0, "y": 0.0}))

    await stack.executor.run_once()

    assert all(not r.busy for r in stack.coordinator.robots.values())
    assert stack.coordinator.pending_tasks == []


async def test_inspection_task_reports_planted_defect(config):
    stack = build_simulation_stack(config, defect_camera_ids=frozenset({"cam0"}))
    stack.plant_client.enqueue(
        PlantTask("job-2", "inspection", {"x": 3.0, "y": 0.0, "camera_ids": ["cam0", "cam1"]})
    )

    outcome = (await stack.executor.run_once())[0]

    assert outcome.status == STATUS_COMPLETED
    assert len(outcome.defects) == 2
    by_camera = {d.camera_id: d for d in outcome.defects}
    assert by_camera["cam0"].defect_found is True
    assert by_camera["cam0"].bbox_xyxy is not None
    assert by_camera["cam1"].defect_found is False

    reported = stack.plant_client.reports_for("job-2")[-1].details["defects"]
    assert [d["defect_found"] for d in reported] == [True, False]


async def test_estop_halts_the_task_and_reports_halted(config):
    stack = build_simulation_stack(config)
    stack.estop.trigger()
    stack.plant_client.enqueue(PlantTask("job-3", "transport", {"x": 5.0, "y": 0.0}))

    outcome = (await stack.executor.run_once())[0]

    assert outcome.status == STATUS_HALTED
    assert "e-stop" in outcome.detail
    # Halted on tick 1, before any motion command was issued.
    assert stack.simulator.get_robot_pose("amr0").x == 0.0
    assert stack.plant_client.reports_for("job-3")[-1].status == STATUS_HALTED
    # The robot is still released despite the fault.
    assert not stack.coordinator.robots["amr0"].busy


async def test_obstacle_blocking_the_goal_reports_blocked_not_a_hang(config):
    stack = build_simulation_stack(
        config,
        # Sitting on the goal, radius large enough that the whole approach is
        # inside the hard safety margin.
        obstacles=[Obstacle(x=3.0, y=0.0, radius_m=3.0)],
    )
    stack.executor.max_blocked_ticks = 60
    stack.plant_client.enqueue(PlantTask("job-4", "transport", {"x": 3.0, "y": 0.0}))

    outcome = (await stack.executor.run_once())[0]

    assert outcome.status == STATUS_BLOCKED
    assert outcome.ticks <= 60
    assert stack.plant_client.reports_for("job-4")[-1].status == STATUS_BLOCKED


async def test_pick_place_task_runs_on_the_arm_without_consuming_a_robot(config):
    stack = build_simulation_stack(config)
    stack.plant_client.enqueue(
        PlantTask("job-5", "pick_place", {"joint_goal_rad": [0.5, -0.3, 0.2, 0.0, 0.1, 0.0]})
    )

    outcome = (await stack.executor.run_once())[0]

    assert outcome.status == STATUS_COMPLETED
    assert outcome.robot_id == "arm0"
    assert all(not r.busy for r in stack.coordinator.robots.values())


async def test_malformed_task_is_rejected_and_reported_without_motion(config):
    stack = build_simulation_stack(config)
    stack.plant_client.enqueue(PlantTask("job-6", "teleport", {"x": 1, "y": 1}))

    outcomes = await stack.executor.run_once()

    assert outcomes == []  # nothing was executed
    report = stack.plant_client.reports_for("job-6")[-1]
    assert report.status == STATUS_REJECTED
    assert "unsupported task_type" in report.details["detail"]


async def test_multiple_tasks_go_to_different_robots(config):
    stack = build_simulation_stack(config)
    stack.plant_client.enqueue(PlantTask("job-a", "transport", {"x": 1.0, "y": 0.0}))
    stack.plant_client.enqueue(PlantTask("job-b", "transport", {"x": 1.0, "y": 6.0}))

    outcomes = await stack.executor.run_once()

    assert [o.status for o in outcomes] == [STATUS_COMPLETED, STATUS_COMPLETED]
    assert outcomes[0].robot_id != outcomes[1].robot_id


async def test_fetch_drains_the_queue_so_tasks_do_not_rerun(config):
    stack = build_simulation_stack(config)
    stack.plant_client.enqueue(PlantTask("job-1", "transport", {"x": 1.0, "y": 0.0}))

    await stack.executor.run_once()
    second_pass = await stack.executor.run_once()

    assert second_pass == []
    assert stack.plant_client.queued_task_ids == []
