"""Internal API exposing robot status + task queue to the plant layer.

Deliberately minimal for the scaffold stage: health check and status/task
endpoints backed by in-memory state. Wire this up to the real
FleetCoordinator/SafetyMonitor instances as those subsystems come online.
"""
from __future__ import annotations

from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI(title="Advanced Robotics API", version="0.1.0")


class RobotStatus(BaseModel):
    robot_id: str
    mode: str
    pose: dict | None = None


class TaskRequest(BaseModel):
    task_id: str
    task_type: str
    payload: dict


_robot_status: dict[str, RobotStatus] = {}
_task_queue: list[TaskRequest] = []


@app.get("/health")
def health() -> dict:
    return {"status": "ok"}


@app.get("/robots", response_model=list[RobotStatus])
def list_robots() -> list[RobotStatus]:
    return list(_robot_status.values())


@app.post("/tasks", response_model=TaskRequest)
def submit_task(task: TaskRequest) -> TaskRequest:
    _task_queue.append(task)
    return task


@app.get("/tasks", response_model=list[TaskRequest])
def list_tasks() -> list[TaskRequest]:
    return _task_queue
