"""End-to-end task execution: plant system -> robot -> status report.

This is the loop the whole platform exists to run:

    fetch task -> route -> allocate robot -> drive it (safety-gated every tick)
    -> inspect -> release robot -> report status upstream

Two invariants it must never break:

1. `SafetyMonitor.check()` runs before every velocity command and every
   trajectory point. A `SafetyFaultError` zeroes the robot's velocity, aborts
   the job, and is reported upstream as `halted` -- never retried silently,
   because the monitor latches until an operator reset.
2. The allocated robot is released in a `finally` block. A task that faults
   must not leave a robot marked busy forever, which would starve the fleet.

The executor talks to `SimulatorBackend`, so it is identical against the
kinematic sim, Gazebo, Isaac Sim, or the ROS 2 bridge to real hardware.
"""
from __future__ import annotations

import logging
import math
from collections.abc import Callable
from dataclasses import dataclass, field

from advanced_robotics.amr.fleet_coordinator import FleetCoordinator
from advanced_robotics.amr.fleet_coordinator import Task as FleetTask
from advanced_robotics.amr.navigation import Navigator, Obstacle
from advanced_robotics.arm.motion_planner import MotionPlanner
from advanced_robotics.core.errors import SafetyFaultError, TaskRoutingError
from advanced_robotics.core.types import DefectReport, Pose2D
from advanced_robotics.digital_twin.simulator import SimulatorBackend, SimulatorSlam
from advanced_robotics.integrations.plant_client import PlantSystemClient
from advanced_robotics.orchestration.task_router import JobKind, RobotJob, route
from advanced_robotics.safety.safety_monitor import SafetyMonitor
from advanced_robotics.vision.inspector import Inspector

logger = logging.getLogger("advanced_robotics.orchestration")

# Reported upstream as the task status; plant systems key their own state
# machines off these strings, so treat them as part of the integration
# contract, not internal labels.
STATUS_COMPLETED = "completed"
STATUS_HALTED = "halted"
STATUS_BLOCKED = "blocked"
STATUS_TIMEOUT = "timeout"
STATUS_REJECTED = "rejected"
STATUS_FAILED = "failed"


@dataclass(frozen=True)
class TaskOutcome:
    task_id: str
    status: str
    robot_id: str | None = None
    ticks: int = 0
    sim_time_s: float = 0.0
    detail: str = ""
    defects: tuple[DefectReport, ...] = ()

    @property
    def succeeded(self) -> bool:
        return self.status == STATUS_COMPLETED

    def as_details(self) -> dict:
        return {
            "robot_id": self.robot_id,
            "ticks": self.ticks,
            "sim_time_s": round(self.sim_time_s, 3),
            "detail": self.detail,
            "defects": [
                {
                    "frame_id": d.frame_id,
                    "camera_id": d.camera_id,
                    "defect_found": d.defect_found,
                    "confidence": round(d.confidence, 4),
                    "bbox_xyxy": list(d.bbox_xyxy) if d.bbox_xyxy else None,
                }
                for d in self.defects
            ],
        }


@dataclass
class TaskExecutor:
    coordinator: FleetCoordinator
    safety: SafetyMonitor
    simulator: SimulatorBackend
    plant_client: PlantSystemClient
    source_system: str = "mock-mes"

    navigator_factory: Callable[[str], Navigator] | None = None
    obstacle_source: Callable[[str], list[Obstacle]] = lambda _robot_id: []
    inspector: Inspector | None = None
    motion_planner: MotionPlanner | None = None
    arm_id: str = "arm0"

    dt_s: float = 0.05
    goal_tolerance_m: float = 0.15
    max_ticks: int = 4000
    # Consecutive blocked ticks tolerated before giving up. Must exceed the
    # navigator's own stuck_threshold so rotate-in-place recovery gets a
    # chance to run before the task is declared blocked.
    max_blocked_ticks: int = 200
    # On real hardware the e-stop source pumps its own heartbeat. In
    # simulation nothing does, so the monitor would fault on heartbeat age
    # partway through a long run; the sim loop pumps it here instead.
    heartbeat: Callable[[], None] | None = None

    outcomes: list[TaskOutcome] = field(default_factory=list)

    # -- public API -------------------------------------------------------

    async def run_once(self) -> list[TaskOutcome]:
        """Claim every pending plant task, execute it, and report back."""
        tasks = await self.plant_client.fetch_pending_tasks()
        jobs: list[RobotJob] = []

        for task in tasks:
            try:
                jobs.append(route(task, source_system=self.source_system))
            except TaskRoutingError as exc:
                outcome = TaskOutcome(
                    task_id=task.task_id, status=STATUS_REJECTED, detail=str(exc)
                )
                logger.warning("rejected task %s: %s", task.task_id, exc)
                await self._finish(outcome)

        results: list[TaskOutcome] = []
        for job in jobs:
            results.append(await self.execute(job))
        return results

    async def execute(self, job: RobotJob) -> TaskOutcome:
        if not job.needs_mobile_robot:
            return await self._finish(self._execute_arm_job(job))

        robot_id = self._allocate(job)
        if robot_id is None:
            outcome = TaskOutcome(
                task_id=job.task_id,
                status=STATUS_FAILED,
                detail="no idle robot available",
            )
            return await self._finish(outcome)

        try:
            outcome = self._execute_mobile_job(job, robot_id)
        finally:
            # Always release the robot -- a faulted task must not strand it.
            self._safe_stop(robot_id)
            self.coordinator.complete_task(job.task_id)
        return await self._finish(outcome)

    # -- execution --------------------------------------------------------

    def _allocate(self, job: RobotJob) -> str | None:
        goal = job.goal or Pose2D(0.0, 0.0, 0.0)
        self._sync_fleet_poses()
        self.coordinator.submit_task(FleetTask(task_id=job.task_id, goal=goal))
        assigned = self.coordinator.allocate()
        for task in assigned:
            if task.task_id == job.task_id:
                return task.assigned_robot_id
        # Nothing was assigned to us -- withdraw the task so it doesn't linger
        # in the pending list and get double-allocated on a later cycle.
        self.coordinator.pending_tasks = [
            t for t in self.coordinator.pending_tasks if t.task_id != job.task_id
        ]
        return None

    def _sync_fleet_poses(self) -> None:
        """Allocation is distance-based, so the coordinator's view of where the
        robots are must come from the world, not from stale registration data."""
        for robot_id, robot in self.coordinator.robots.items():
            try:
                robot.pose = self.simulator.get_robot_pose(robot_id)
            except KeyError:
                logger.warning("robot %s registered in fleet but absent from world", robot_id)

    def _execute_mobile_job(self, job: RobotJob, robot_id: str) -> TaskOutcome:
        assert job.goal is not None  # guaranteed by the router for mobile jobs
        navigator = self._navigator_for(robot_id)
        blocked_ticks = 0

        for tick in range(1, self.max_ticks + 1):
            if self.heartbeat is not None:
                self.heartbeat()

            try:
                self.safety.check()
            except SafetyFaultError as exc:
                self._safe_stop(robot_id)
                return self._outcome(job, STATUS_HALTED, robot_id, tick, str(exc))

            pose = self.simulator.get_robot_pose(robot_id)
            if math.hypot(job.goal.x - pose.x, job.goal.y - pose.y) <= self.goal_tolerance_m:
                self._safe_stop(robot_id)
                return self._complete_at_goal(job, robot_id, tick)

            linear, angular = navigator.compute_velocity_command(
                job.goal, self.obstacle_source(robot_id)
            )
            self.simulator.set_robot_velocity(robot_id, linear, angular)
            self.simulator.step(self.dt_s)

            if linear <= 0.0:
                blocked_ticks += 1
                if blocked_ticks >= self.max_blocked_ticks:
                    self._safe_stop(robot_id)
                    return self._outcome(
                        job,
                        STATUS_BLOCKED,
                        robot_id,
                        tick,
                        f"blocked for {blocked_ticks} ticks; recovery did not clear the path",
                    )
            else:
                blocked_ticks = 0

        self._safe_stop(robot_id)
        return self._outcome(
            job, STATUS_TIMEOUT, robot_id, self.max_ticks, f"goal not reached in {self.max_ticks} ticks"
        )

    def _complete_at_goal(self, job: RobotJob, robot_id: str, tick: int) -> TaskOutcome:
        if job.kind is not JobKind.INSPECT:
            return self._outcome(job, STATUS_COMPLETED, robot_id, tick, "goal reached")

        if self.inspector is None:
            return self._outcome(
                job, STATUS_FAILED, robot_id, tick, "inspect task but no inspector configured"
            )

        reports: list[DefectReport] = []
        for camera_id in job.camera_ids:
            frame = self.simulator.get_camera_frame(camera_id)
            reports.append(self.inspector.detect(frame))

        found = sum(1 for r in reports if r.defect_found)
        return self._outcome(
            job,
            STATUS_COMPLETED,
            robot_id,
            tick,
            f"inspected {len(reports)} camera(s), {found} with defects",
            defects=tuple(reports),
        )

    def _execute_arm_job(self, job: RobotJob) -> TaskOutcome:
        assert job.joint_goal_rad is not None  # guaranteed by the router
        if self.motion_planner is None:
            return self._outcome(
                job, STATUS_FAILED, None, 0, "pick_place task but no motion planner configured"
            )

        start = [0.0] * len(job.joint_goal_rad)
        try:
            trajectory = self.motion_planner.plan(start, list(job.joint_goal_rad))
        except (ValueError, RuntimeError) as exc:
            return self._outcome(job, STATUS_FAILED, self.arm_id, 0, f"planning failed: {exc}")

        for index, point in enumerate(trajectory, start=1):
            if self.heartbeat is not None:
                self.heartbeat()
            try:
                self.safety.check()
            except SafetyFaultError as exc:
                return self._outcome(job, STATUS_HALTED, self.arm_id, index, str(exc))

        final = trajectory[-1]
        return self._outcome(
            job,
            STATUS_COMPLETED,
            self.arm_id,
            len(trajectory),
            f"trajectory of {len(trajectory)} points executed in {final.time_s:.2f}s",
        )

    # -- helpers ----------------------------------------------------------

    def _navigator_for(self, robot_id: str) -> Navigator:
        if self.navigator_factory is not None:
            return self.navigator_factory(robot_id)
        return Navigator(
            slam=SimulatorSlam(self.simulator, robot_id),
            max_linear_speed_mps=1.0,
            max_angular_speed_radps=1.0,
            obstacle_safety_margin_m=0.3,
        )

    def _safe_stop(self, robot_id: str) -> None:
        try:
            self.simulator.set_robot_velocity(robot_id, 0.0, 0.0)
        except KeyError:
            pass

    def _outcome(
        self,
        job: RobotJob,
        status: str,
        robot_id: str | None,
        ticks: int,
        detail: str,
        defects: tuple[DefectReport, ...] = (),
    ) -> TaskOutcome:
        return TaskOutcome(
            task_id=job.task_id,
            status=status,
            robot_id=robot_id,
            ticks=ticks,
            sim_time_s=ticks * self.dt_s,
            detail=detail,
            defects=defects,
        )

    async def _finish(self, outcome: TaskOutcome) -> TaskOutcome:
        self.outcomes.append(outcome)
        await self.plant_client.report_status(
            outcome.task_id, outcome.status, outcome.as_details()
        )
        logger.info("task %s -> %s (%s)", outcome.task_id, outcome.status, outcome.detail)
        return outcome
