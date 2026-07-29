"""Maps inbound PlantTask payloads onto typed robot jobs.

Plant systems name the same operation differently -- an ERP work order says
`move`, an MES job says `transport`, a SCADA alarm says `inspect`. Routing
lives here rather than in the executor so that adding a plant system means
adding aliases and a payload shape, not touching the execution loop.

Validation is strict and fails loudly: a malformed payload is reported back to
the plant system as `rejected` rather than being coerced into a motion command
with a guessed goal.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from advanced_robotics.core.errors import TaskRoutingError
from advanced_robotics.core.types import Pose2D
from advanced_robotics.integrations.plant_client import PlantTask


class JobKind(str, Enum):
    TRANSPORT = "transport"
    INSPECT = "inspect"
    PICK_PLACE = "pick_place"


_ALIASES: dict[str, JobKind] = {
    "transport": JobKind.TRANSPORT,
    "move": JobKind.TRANSPORT,
    "delivery": JobKind.TRANSPORT,
    "goto": JobKind.TRANSPORT,
    "inspect": JobKind.INSPECT,
    "inspection": JobKind.INSPECT,
    "quality_check": JobKind.INSPECT,
    "alarm": JobKind.INSPECT,
    "pick_place": JobKind.PICK_PLACE,
    "pick_and_place": JobKind.PICK_PLACE,
    "assembly": JobKind.PICK_PLACE,
}


@dataclass(frozen=True)
class RobotJob:
    task_id: str
    kind: JobKind
    source_system: str
    goal: Pose2D | None = None
    camera_ids: tuple[str, ...] = ()
    joint_goal_rad: tuple[float, ...] | None = None

    @property
    def needs_mobile_robot(self) -> bool:
        return self.kind in (JobKind.TRANSPORT, JobKind.INSPECT)


def route(task: PlantTask, source_system: str = "unknown") -> RobotJob:
    """Translate a PlantTask into a RobotJob, or raise TaskRoutingError."""
    kind = _ALIASES.get(task.task_type.strip().lower())
    if kind is None:
        raise TaskRoutingError(
            f"task {task.task_id}: unsupported task_type '{task.task_type}' "
            f"(known: {', '.join(sorted(_ALIASES))})"
        )

    payload = task.payload or {}

    if kind is JobKind.PICK_PLACE:
        joints = payload.get("joint_goal_rad")
        if not isinstance(joints, (list, tuple)) or not joints:
            raise TaskRoutingError(
                f"task {task.task_id}: pick_place requires a non-empty 'joint_goal_rad' list"
            )
        try:
            joint_goal = tuple(float(j) for j in joints)
        except (TypeError, ValueError) as exc:
            raise TaskRoutingError(
                f"task {task.task_id}: 'joint_goal_rad' must be numeric ({exc})"
            ) from exc
        return RobotJob(
            task_id=task.task_id,
            kind=kind,
            source_system=source_system,
            joint_goal_rad=joint_goal,
        )

    goal = _parse_goal(task.task_id, payload)
    camera_ids: tuple[str, ...] = ()
    if kind is JobKind.INSPECT:
        raw_cameras = payload.get("camera_ids") or payload.get("cameras") or []
        if isinstance(raw_cameras, str):
            raw_cameras = [raw_cameras]
        if not raw_cameras:
            raise TaskRoutingError(
                f"task {task.task_id}: inspect requires 'camera_ids'"
            )
        camera_ids = tuple(str(c) for c in raw_cameras)

    return RobotJob(
        task_id=task.task_id,
        kind=kind,
        source_system=source_system,
        goal=goal,
        camera_ids=camera_ids,
    )


def _parse_goal(task_id: str, payload: dict) -> Pose2D:
    raw = payload.get("goal", payload)
    try:
        return Pose2D(
            x=float(raw["x"]),
            y=float(raw["y"]),
            yaw_rad=float(raw.get("yaw_rad", 0.0)),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise TaskRoutingError(
            f"task {task_id}: payload needs a goal with numeric 'x' and 'y' ({exc})"
        ) from exc
