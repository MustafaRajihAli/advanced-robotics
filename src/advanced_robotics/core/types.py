"""Shared value types used across AMR, arm, vision, and safety modules."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Pose2D:
    __slots__ = ("x", "y", "yaw_rad")

    def __init__(self, x: float, y: float, yaw_rad: float) -> None:
        self.x = x
        self.y = y
        self.yaw_rad = yaw_rad


@dataclass(frozen=True)
class JointState:
    positions_rad: tuple[float, ...]
    velocities_radps: tuple[float, ...]
    efforts_nm: tuple[float, ...]


@dataclass(frozen=True)
class WrenchSample:
    """Force-torque sensor reading at the end effector."""

    force_n: tuple[float, float, float]
    torque_nm: tuple[float, float, float]
    timestamp_s: float


class RobotMode(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    ESTOPPED = "estopped"
    FAULT = "fault"


@dataclass(frozen=True)
class DefectReport:
    frame_id: str
    camera_id: str
    defect_found: bool
    confidence: float
    bbox_xyxy: tuple[int, int, int, int] | None = None
