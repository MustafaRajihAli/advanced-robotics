"""SLAM front-end abstraction.

Real localization/mapping runs in ROS 2 (e.g. slam_toolbox, Nav2). This module
defines the contract the rest of the app depends on so the AMR logic can be
tested without ROS 2 running, and swapped to a different SLAM stack later.
"""
from __future__ import annotations

from typing import Protocol

from advanced_robotics.core.types import Pose2D


class SlamBackend(Protocol):
    def get_pose(self) -> Pose2D: ...

    def get_occupancy_grid(self) -> "OccupancyGrid": ...

    def is_localized(self) -> bool: ...


class OccupancyGrid:
    def __init__(self, width: int, height: int, resolution_m: float, cells: bytes) -> None:
        self.width = width
        self.height = height
        self.resolution_m = resolution_m
        self.cells = cells

    def is_occupied(self, x: int, y: int) -> bool:
        idx = y * self.width + x
        return self.cells[idx] > 50


class StubSlamBackend:
    """In-memory backend for tests/dev before the ROS 2 bridge is wired up."""

    def __init__(self, pose: Pose2D | None = None) -> None:
        self._pose = pose or Pose2D(0.0, 0.0, 0.0)
        self._localized = True

    def get_pose(self) -> Pose2D:
        return self._pose

    def get_occupancy_grid(self) -> OccupancyGrid:
        return OccupancyGrid(width=1, height=1, resolution_m=0.05, cells=b"\x00")

    def is_localized(self) -> bool:
        return self._localized
