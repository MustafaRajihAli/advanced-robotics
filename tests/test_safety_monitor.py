import pytest

from advanced_robotics.core.errors import SafetyFaultError
from advanced_robotics.safety.estop import SoftwareEstop
from advanced_robotics.safety.safety_monitor import SafetyMonitor


def test_check_passes_with_recent_heartbeat():
    estop = SoftwareEstop()
    monitor = SafetyMonitor(estop=estop, heartbeat_timeout_ms=250)
    monitor.check()  # should not raise


def test_estop_trigger_raises_and_latches():
    estop = SoftwareEstop()
    monitor = SafetyMonitor(estop=estop, heartbeat_timeout_ms=250)
    estop.trigger()

    with pytest.raises(SafetyFaultError):
        monitor.check()

    estop.reset()
    with pytest.raises(SafetyFaultError):
        monitor.check()  # still latched until monitor.reset()

    monitor.reset()
    monitor.check()  # now clean
