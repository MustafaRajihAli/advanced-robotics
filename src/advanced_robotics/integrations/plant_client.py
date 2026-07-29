"""Common interface for ERP/MES/SCADA clients.

Each system (erp_client, mes_client, scada_client) implements this Protocol
so the rest of the app can talk to "the plant" without caring which system
issued a task or wants a status update.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class PlantTask:
    task_id: str
    task_type: str
    payload: dict


class PlantSystemClient(Protocol):
    async def fetch_pending_tasks(self) -> list[PlantTask]: ...

    async def report_status(self, task_id: str, status: str, details: dict) -> None: ...
