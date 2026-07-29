from advanced_robotics.amr.fleet_coordinator import FleetCoordinator, RobotState, Task
from advanced_robotics.core.types import Pose2D


def test_allocates_nearest_idle_robot():
    coordinator = FleetCoordinator()
    coordinator.register_robot(RobotState("r1", Pose2D(0, 0, 0)))
    coordinator.register_robot(RobotState("r2", Pose2D(10, 10, 0)))
    coordinator.submit_task(Task("t1", goal=Pose2D(1, 1, 0)))

    assigned = coordinator.allocate()

    assert len(assigned) == 1
    assert assigned[0].assigned_robot_id == "r1"
    assert coordinator.robots["r1"].busy is True


def test_completing_task_frees_robot():
    coordinator = FleetCoordinator()
    coordinator.register_robot(RobotState("r1", Pose2D(0, 0, 0)))
    coordinator.submit_task(Task("t1", goal=Pose2D(1, 1, 0)))
    coordinator.allocate()

    coordinator.complete_task("t1")

    assert coordinator.robots["r1"].busy is False
    assert coordinator.pending_tasks == []
