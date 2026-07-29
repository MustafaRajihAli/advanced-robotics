import pytest

from advanced_robotics.digital_twin.gazebo_backend import GazeboBackend
from advanced_robotics.digital_twin.isaac_sim_backend import IsaacSimBackend
from advanced_robotics.digital_twin.simulator import NullSimulator


def test_null_simulator_returns_origin_pose():
    sim = NullSimulator()
    pose = sim.get_robot_pose("r1")
    assert (pose.x, pose.y, pose.yaw_rad) == (0.0, 0.0, 0.0)


def test_isaac_sim_backend_reset_not_implemented_without_isaac_sim():
    backend = IsaacSimBackend()
    with pytest.raises(NotImplementedError):
        backend.reset("warehouse_basic")


def test_gazebo_backend_reset_not_implemented_without_ros2():
    backend = GazeboBackend()
    with pytest.raises(NotImplementedError):
        backend.reset("warehouse_basic")
