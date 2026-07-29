"""Watchdog that halts AMR/arm actuation on fault and logs to an audit trail.

Every actuator-facing loop (navigation velocity commands, arm trajectory
execution) should call SafetyMonitor.check() before issuing a command and
treat SafetyFaultError as an immediate stop.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

from advanced_robotics.core.errors import SafetyFaultError
from advanced_robotics.core.types import RobotMode
from advanced_robotics.safety.estop import EstopSource

logger = logging.getLogger("advanced_robotics.safety")


@dataclass
class AuditEvent:
    timestamp_s: float
    reason: str


@dataclass
class SafetyMonitor:
    estop: EstopSource
    heartbeat_timeout_ms: int
    mode: RobotMode = RobotMode.IDLE
    audit_log: list[AuditEvent] = field(default_factory=list)

    def check(self) -> None:
        """Raise SafetyFaultError and flip to FAULT/ESTOPPED if unsafe.
        Callers must stop all actuation when this raises."""
        if self.estop.is_triggered():
            self._fault(RobotMode.ESTOPPED, "e-stop triggered")
            raise SafetyFaultError("e-stop triggered")

        age_ms = (time.monotonic() - self.estop.last_heartbeat_s()) * 1000
        if age_ms > self.heartbeat_timeout_ms:
            self._fault(RobotMode.FAULT, f"heartbeat lost ({age_ms:.0f}ms)")
            raise SafetyFaultError(f"heartbeat lost ({age_ms:.0f}ms)")

        if self.mode in (RobotMode.ESTOPPED, RobotMode.FAULT):
            # stays latched until an explicit reset, even if the trigger cleared
            raise SafetyFaultError(f"latched in {self.mode.value}, awaiting reset")

    def reset(self) -> None:
        self.mode = RobotMode.IDLE
        self.audit_log.append(AuditEvent(time.monotonic(), "reset"))

    def _fault(self, mode: RobotMode, reason: str) -> None:
        self.mode = mode
        self.audit_log.append(AuditEvent(time.monotonic(), reason))
        logger.warning("safety fault: %s", reason)
