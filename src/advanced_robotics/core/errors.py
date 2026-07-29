class AdvancedRoboticsError(Exception):
    """Base error for the platform."""


class SafetyFaultError(AdvancedRoboticsError):
    """Raised when a safety watchdog trips (e-stop, heartbeat loss, limit breach)."""


class IntegrationError(AdvancedRoboticsError):
    """Raised when an ERP/MES/SCADA call fails."""


class ConfigError(AdvancedRoboticsError):
    """Raised on invalid or missing configuration."""
