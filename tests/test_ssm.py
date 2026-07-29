import pytest

from advanced_robotics.core.errors import SafetyFaultError
from advanced_robotics.safety.estop import SoftwareEstop
from advanced_robotics.safety.safety_monitor import SafetyMonitor
from advanced_robotics.safety.ssm import (
    SsmParameters,
    is_separation_violated,
    protective_separation_distance_m,
)


def test_stationary_robot_still_requires_minimum_separation():
    # Even at zero robot speed, human contribution + intrusion distance apply.
    distance = protective_separation_distance_m(robot_speed_toward_human_mps=0.0)
    assert distance > 0.5


def test_faster_robot_requires_more_separation():
    slow = protective_separation_distance_m(robot_speed_toward_human_mps=0.1)
    fast = protective_separation_distance_m(robot_speed_toward_human_mps=1.0)
    assert fast > slow


def test_is_separation_violated_true_when_too_close():
    assert is_separation_violated(measured_distance_m=0.1, robot_speed_toward_human_mps=0.5)


def test_is_separation_violated_false_when_far_enough():
    params = SsmParameters()
    required = protective_separation_distance_m(0.1, params)
    assert not is_separation_violated(required + 1.0, robot_speed_toward_human_mps=0.1, params=params)


def test_safety_monitor_check_separation_raises_when_violated():
    monitor = SafetyMonitor(estop=SoftwareEstop(), heartbeat_timeout_ms=250)
    with pytest.raises(SafetyFaultError):
        monitor.check_separation(measured_distance_m=0.05, robot_speed_toward_human_mps=0.5)
