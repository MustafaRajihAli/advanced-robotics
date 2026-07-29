"""Hardware e-stop integration contract.

Abstracts the physical e-stop signal (GPIO, fieldbus, or a ROS 2 topic in the
real deployment) behind a simple polled interface plus a heartbeat check, so
safety_monitor.py doesn't need to know the transport.
"""
from __future__ import annotations

import time
from typing import Protocol


class EstopSource(Protocol):
    def is_triggered(self) -> bool: ...

    def last_heartbeat_s(self) -> float: ...


class SoftwareEstop:
    """In-process e-stop for simulation/dev. A real deployment replaces this
    with a GPIO or fieldbus-backed implementation of EstopSource."""

    def __init__(self) -> None:
        self._triggered = False
        self._last_heartbeat_s = time.monotonic()

    def trigger(self) -> None:
        self._triggered = True

    def reset(self) -> None:
        self._triggered = False
        self._last_heartbeat_s = time.monotonic()

    def heartbeat(self) -> None:
        self._last_heartbeat_s = time.monotonic()

    def is_triggered(self) -> bool:
        return self._triggered

    def last_heartbeat_s(self) -> float:
        return self._last_heartbeat_s
