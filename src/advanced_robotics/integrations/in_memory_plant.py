"""In-memory PlantSystemClient for simulation, tests, and the demo script.

Same protocol as `MesClient`/`ErpClient`/`ScadaClient`, minus the network. The
orchestrator is written against `PlantSystemClient`, so the end-to-end task
path exercised in tests is the same code that will run against a real MES --
only the client instance differs.

Tasks are handed out once: `fetch_pending_tasks` drains the queue, matching a
real MES where a claimed job leaves the `queued` state.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from advanced_robotics.integrations.plant_client import PlantTask


@dataclass(frozen=True)
class StatusReport:
    task_id: str
    status: str
    details: dict


@dataclass
class InMemoryPlantClient:
    system_name: str = "mock-mes"
    _queued: list[PlantTask] = field(default_factory=list, init=False)
    reports: list[StatusReport] = field(default_factory=list, init=False)

    def enqueue(self, task: PlantTask) -> None:
        self._queued.append(task)

    @property
    def queued_task_ids(self) -> list[str]:
        return [t.task_id for t in self._queued]

    async def fetch_pending_tasks(self) -> list[PlantTask]:
        claimed, self._queued = self._queued, []
        return claimed

    async def report_status(self, task_id: str, status: str, details: dict) -> None:
        self.reports.append(StatusReport(task_id=task_id, status=status, details=details))

    def reports_for(self, task_id: str) -> list[StatusReport]:
        return [r for r in self.reports if r.task_id == task_id]

    async def aclose(self) -> None:
        return None
