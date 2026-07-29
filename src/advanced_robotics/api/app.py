"""Internal API exposing robot status + the task queue to the plant layer.

Backed by a real `SimulationStack`, not in-memory placeholders: a task POSTed
here is queued on the plant client, and `POST /tasks/run` drains the queue
through the same `TaskExecutor` path a real MES poll would take.

`create_app()` takes the stack explicitly so tests can inject a stack with
known obstacles or a triggered e-stop. The module-level `app` builds a default
simulation stack, so `uvicorn advanced_robotics.api.app:app` gives a working
end-to-end demo without hardware.
"""
from __future__ import annotations

from typing import Annotated

from fastapi import Depends, FastAPI, HTTPException, Request
from pydantic import BaseModel, Field

from advanced_robotics.integrations.plant_client import PlantTask
from advanced_robotics.orchestration.bootstrap import SimulationStack, build_simulation_stack


def get_stack(request: Request) -> SimulationStack:
    return request.app.state.stack


# Module level, not a closure inside create_app: with postponed annotation
# evaluation FastAPI resolves handler annotations against module globals, and a
# locally-defined alias would be invisible to it (and silently demoted to a
# query parameter).
Stack = Annotated[SimulationStack, Depends(get_stack)]


class RobotStatus(BaseModel):
    robot_id: str
    busy: bool
    pose: dict


class TaskRequest(BaseModel):
    task_id: str
    task_type: str
    payload: dict = Field(default_factory=dict)


class OutcomeResponse(BaseModel):
    task_id: str
    status: str
    robot_id: str | None = None
    ticks: int
    sim_time_s: float
    detail: str


class SafetyStatus(BaseModel):
    mode: str
    estop_triggered: bool
    audit_events: int


def create_app(stack: SimulationStack | None = None) -> FastAPI:
    app = FastAPI(title="Advanced Robotics API", version="0.2.0")
    app.state.stack = stack or build_simulation_stack()

    def _safety_status(stack: SimulationStack) -> SafetyStatus:
        return SafetyStatus(
            mode=stack.safety.mode.value,
            estop_triggered=stack.estop.is_triggered(),
            audit_events=len(stack.safety.audit_log),
        )

    def _outcome_response(outcome) -> OutcomeResponse:
        return OutcomeResponse(
            task_id=outcome.task_id,
            status=outcome.status,
            robot_id=outcome.robot_id,
            ticks=outcome.ticks,
            sim_time_s=outcome.sim_time_s,
            detail=outcome.detail,
        )

    @app.get("/health")
    def health() -> dict:
        return {"status": "ok"}

    @app.get("/robots", response_model=list[RobotStatus])
    def list_robots(stack: Stack) -> list[RobotStatus]:
        return [
            RobotStatus(
                robot_id=robot_id,
                busy=robot.busy,
                pose=_pose_dict(stack, robot_id),
            )
            for robot_id, robot in stack.coordinator.robots.items()
        ]

    @app.get("/safety", response_model=SafetyStatus)
    def safety_status(stack: Stack) -> SafetyStatus:
        return _safety_status(stack)

    @app.post("/safety/reset", response_model=SafetyStatus)
    def safety_reset(stack: Stack) -> SafetyStatus:
        """Explicit operator reset. The monitor latches faults, so this is the
        only way motion resumes -- audited, never automatic."""
        if stack.estop.is_triggered():
            raise HTTPException(
                status_code=409,
                detail="e-stop still engaged; release it before resetting the monitor",
            )
        stack.safety.reset()
        return _safety_status(stack)

    @app.post("/tasks", response_model=TaskRequest, status_code=202)
    def submit_task(task: TaskRequest, stack: Stack) -> TaskRequest:
        stack.plant_client.enqueue(
            PlantTask(task_id=task.task_id, task_type=task.task_type, payload=task.payload)
        )
        return task

    @app.get("/tasks")
    def list_tasks(stack: Stack) -> dict:
        return {
            "queued": stack.plant_client.queued_task_ids,
            "executed": [o.task_id for o in stack.executor.outcomes],
        }

    @app.post("/tasks/run", response_model=list[OutcomeResponse])
    async def run_queued_tasks(
        stack: Stack,
    ) -> list[OutcomeResponse]:
        outcomes = await stack.executor.run_once()
        return [_outcome_response(o) for o in outcomes]

    @app.get("/outcomes", response_model=list[OutcomeResponse])
    def list_outcomes(stack: Stack) -> list[OutcomeResponse]:
        return [_outcome_response(o) for o in stack.executor.outcomes]

    return app


def _pose_dict(stack: SimulationStack, robot_id: str) -> dict:
    pose = stack.simulator.get_robot_pose(robot_id)
    return {"x": pose.x, "y": pose.y, "yaw_rad": pose.yaw_rad}


app = create_app()
