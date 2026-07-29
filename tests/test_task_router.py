import pytest

from advanced_robotics.core.errors import TaskRoutingError
from advanced_robotics.integrations.plant_client import PlantTask
from advanced_robotics.orchestration.task_router import JobKind, route


def test_routes_mes_transport_job():
    task = PlantTask("job-1", "transport", {"goal": {"x": 4.0, "y": 2.0, "yaw_rad": 1.0}})

    job = route(task, source_system="mes")

    assert job.kind is JobKind.TRANSPORT
    assert job.needs_mobile_robot is True
    assert (job.goal.x, job.goal.y, job.goal.yaw_rad) == (4.0, 2.0, 1.0)
    assert job.source_system == "mes"


def test_erp_alias_and_flat_payload_are_accepted():
    job = route(PlantTask("wo-9", "move", {"x": 1, "y": 2}))

    assert job.kind is JobKind.TRANSPORT
    assert (job.goal.x, job.goal.y, job.goal.yaw_rad) == (1.0, 2.0, 0.0)


def test_inspect_requires_cameras():
    with pytest.raises(TaskRoutingError, match="camera_ids"):
        route(PlantTask("job-2", "inspection", {"x": 1, "y": 1}))


def test_inspect_accepts_single_camera_string():
    job = route(PlantTask("job-2", "inspection", {"x": 1, "y": 1, "camera_ids": "cam0"}))

    assert job.kind is JobKind.INSPECT
    assert job.camera_ids == ("cam0",)


def test_pick_place_needs_joint_goal_and_no_mobile_robot():
    job = route(PlantTask("job-3", "pick_and_place", {"joint_goal_rad": [0.1, 0.2, 0.3]}))

    assert job.kind is JobKind.PICK_PLACE
    assert job.needs_mobile_robot is False
    assert job.joint_goal_rad == (0.1, 0.2, 0.3)


def test_pick_place_rejects_missing_joint_goal():
    with pytest.raises(TaskRoutingError, match="joint_goal_rad"):
        route(PlantTask("job-3", "pick_place", {}))


def test_unknown_task_type_is_rejected():
    with pytest.raises(TaskRoutingError, match="unsupported task_type"):
        route(PlantTask("job-4", "teleport", {"x": 1, "y": 1}))


def test_non_numeric_goal_is_rejected_not_coerced():
    with pytest.raises(TaskRoutingError, match="numeric"):
        route(PlantTask("job-5", "transport", {"x": "over there", "y": 2}))
