import math

import pytest

from advanced_robotics.amr.navigation import Obstacle
from advanced_robotics.core.types import Pose2D
from advanced_robotics.digital_twin.kinematic_backend import KinematicSimulator
from advanced_robotics.digital_twin.simulator import SimulatorSlam


def test_straight_line_integration():
    sim = KinematicSimulator()
    sim.add_robot("amr0", Pose2D(0.0, 0.0, 0.0))
    sim.set_robot_velocity("amr0", 1.0, 0.0)

    for _ in range(10):
        sim.step(0.1)

    pose = sim.get_robot_pose("amr0")
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(0.0)
    assert sim.elapsed_s == pytest.approx(1.0)


def test_rotation_in_place_does_not_translate():
    sim = KinematicSimulator()
    sim.add_robot("amr0", Pose2D(0.0, 0.0, 0.0))
    sim.set_robot_velocity("amr0", 0.0, math.pi / 2)

    sim.step(1.0)

    pose = sim.get_robot_pose("amr0")
    assert pose.x == pytest.approx(0.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.yaw_rad == pytest.approx(math.pi / 2)


def test_reset_restores_initial_poses_and_clock():
    sim = KinematicSimulator()
    sim.add_robot("amr0", Pose2D(1.0, 2.0, 0.0))
    sim.set_robot_velocity("amr0", 1.0, 0.0)
    sim.step(1.0)

    sim.reset("warehouse_basic")

    pose = sim.get_robot_pose("amr0")
    assert (pose.x, pose.y) == (1.0, 2.0)
    assert sim.elapsed_s == 0.0


def test_obstacles_outside_sense_radius_are_hidden():
    sim = KinematicSimulator(
        obstacles=[Obstacle(1.0, 0.0, 0.2), Obstacle(50.0, 0.0, 0.2)],
        obstacle_sense_radius_m=5.0,
    )
    sim.add_robot("amr0", Pose2D(0.0, 0.0, 0.0))

    sensed = sim.obstacles_near("amr0")

    assert [o.x for o in sensed] == [1.0]


def test_camera_frame_carries_planted_defect_only_where_configured():
    sim = KinematicSimulator(planted_defect_camera_ids=frozenset({"cam1"}))

    clean = sim.get_camera_frame("cam0")
    defective = sim.get_camera_frame("cam1")

    assert clean.image.min() == 160
    assert defective.image.min() == 20


def test_unknown_robot_raises():
    sim = KinematicSimulator()
    with pytest.raises(KeyError):
        sim.get_robot_pose("nope")


def test_simulator_slam_reports_world_pose():
    sim = KinematicSimulator()
    sim.add_robot("amr0", Pose2D(3.0, 4.0, 0.5))
    slam = SimulatorSlam(sim, "amr0")

    assert slam.is_localized() is True
    assert (slam.get_pose().x, slam.get_pose().y) == (3.0, 4.0)
