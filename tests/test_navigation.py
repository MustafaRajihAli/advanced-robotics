from advanced_robotics.amr.navigation import Navigator, Obstacle
from advanced_robotics.amr.slam import StubSlamBackend
from advanced_robotics.core.types import Pose2D


def test_navigator_drives_toward_goal():
    slam = StubSlamBackend(pose=Pose2D(0.0, 0.0, 0.0))
    nav = Navigator(
        slam=slam,
        max_linear_speed_mps=1.0,
        max_angular_speed_radps=1.0,
        obstacle_safety_margin_m=0.3,
    )
    linear, angular = nav.compute_velocity_command(goal=Pose2D(5.0, 0.0, 0.0), obstacles=[])
    assert linear > 0
    assert angular == 0.0


def test_navigator_stops_for_close_obstacle():
    slam = StubSlamBackend(pose=Pose2D(0.0, 0.0, 0.0))
    nav = Navigator(
        slam=slam,
        max_linear_speed_mps=1.0,
        max_angular_speed_radps=1.0,
        obstacle_safety_margin_m=0.5,
    )
    obstacles = [Obstacle(x=0.2, y=0.0, radius_m=0.1)]
    linear, _ = nav.compute_velocity_command(goal=Pose2D(5.0, 0.0, 0.0), obstacles=obstacles)
    assert linear == 0.0


def test_navigator_reached_goal_returns_zero():
    slam = StubSlamBackend(pose=Pose2D(1.0, 1.0, 0.0))
    nav = Navigator(
        slam=slam,
        max_linear_speed_mps=1.0,
        max_angular_speed_radps=1.0,
        obstacle_safety_margin_m=0.3,
    )
    linear, angular = nav.compute_velocity_command(goal=Pose2D(1.0, 1.0, 0.0), obstacles=[])
    assert linear == 0.0
    assert angular == 0.0


def test_navigator_slows_proportionally_inside_inflation_radius():
    slam = StubSlamBackend(pose=Pose2D(0.0, 0.0, 0.0))
    nav = Navigator(
        slam=slam,
        max_linear_speed_mps=1.0,
        max_angular_speed_radps=1.0,
        obstacle_safety_margin_m=0.2,
        inflation_radius_m=1.0,
    )
    # obstacle 0.6m away (after radius) sits inside inflation but outside margin
    obstacles = [Obstacle(x=0.6, y=0.0, radius_m=0.0)]
    linear, _ = nav.compute_velocity_command(goal=Pose2D(5.0, 0.0, 0.0), obstacles=obstacles)
    assert 0.0 < linear < 1.0


def test_navigator_recovers_from_freezing_with_rotation():
    slam = StubSlamBackend(pose=Pose2D(0.0, 0.0, 0.0))
    nav = Navigator(
        slam=slam,
        max_linear_speed_mps=1.0,
        max_angular_speed_radps=0.8,
        obstacle_safety_margin_m=0.5,
        stuck_threshold=3,
    )
    obstacles = [Obstacle(x=0.2, y=0.0, radius_m=0.1)]

    for _ in range(3):
        linear, angular = nav.compute_velocity_command(goal=Pose2D(5.0, 0.0, 0.0), obstacles=obstacles)

    assert nav.is_stuck()
    assert linear == 0.0
    assert angular == 0.8  # rotate-in-place recovery, not a permanent freeze
