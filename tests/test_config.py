from advanced_robotics.core.config import load_config


def test_loads_default_config():
    config = load_config()
    assert config.amr.fleet_size == 4
    assert config.arm.dof == 6
    assert config.safety.estop_heartbeat_timeout_ms == 250
