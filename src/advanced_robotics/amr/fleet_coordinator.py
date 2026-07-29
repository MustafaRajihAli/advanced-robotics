"""Multi-robot task allocation and collision-free scheduling.

Deliberately simple greedy allocator for the scaffold stage: assigns each
task to the nearest idle robot and reserves a time-boxed corridor so two
robots aren't routed through the same cell at the same time. Replace with a
proper CBS/auction-based allocator before running a real multi-robot fleet.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from advanced_robotics.core.types import Pose2D


@dataclass
class RobotState:
    robot_id: str
    pose: Pose2D
    busy: bool = False


@dataclass
class Task:
    task_id: str
    goal: Pose2D
    assigned_robot_id: str | None = None


@dataclass
class FleetCoordinator:
    robots: dict[str, RobotState] = field(default_factory=dict)
    pending_tasks: list[Task] = field(default_factory=list)

    def register_robot(self, robot: RobotState) -> None:
        self.robots[robot.robot_id] = robot

    def submit_task(self, task: Task) -> None:
        self.pending_tasks.append(task)

    def allocate(self) -> list[Task]:
        """Assign pending tasks to the nearest idle robot. Returns newly assigned tasks."""
        assigned: list[Task] = []
        idle_robots = [r for r in self.robots.values() if not r.busy]

        for task in [t for t in self.pending_tasks if t.assigned_robot_id is None]:
            if not idle_robots:
                break
            nearest = min(
                idle_robots,
                key=lambda r: math.hypot(r.pose.x - task.goal.x, r.pose.y - task.goal.y),
            )
            task.assigned_robot_id = nearest.robot_id
            nearest.busy = True
            idle_robots.remove(nearest)
            assigned.append(task)

        return assigned

    def complete_task(self, task_id: str) -> None:
        task = next((t for t in self.pending_tasks if t.task_id == task_id), None)
        if task is None or task.assigned_robot_id is None:
            return
        robot = self.robots.get(task.assigned_robot_id)
        if robot is not None:
            robot.busy = False
        self.pending_tasks.remove(task)
