import pytest

from advanced_robotics.arm.force_torque import ComplianceController
from advanced_robotics.core.errors import SafetyFaultError
from advanced_robotics.core.types import WrenchSample


def test_within_limits_does_not_raise():
    controller = ComplianceController(force_limit_n=50.0, torque_limit_nm=20.0)
    sample = WrenchSample(force_n=(1.0, 0.0, 0.0), torque_nm=(0.0, 0.0, 0.0), timestamp_s=0.0)
    controller.check_limits(sample)  # should not raise


def test_force_over_limit_raises():
    controller = ComplianceController(force_limit_n=10.0, torque_limit_nm=20.0)
    sample = WrenchSample(force_n=(20.0, 0.0, 0.0), torque_nm=(0.0, 0.0, 0.0), timestamp_s=0.0)
    with pytest.raises(SafetyFaultError):
        controller.check_limits(sample)
