"""SCADA integration client (e.g. plant-floor tag read/write).

SCADA systems are commonly OPC-UA/Modbus rather than REST; this client keeps
the same PlantSystemClient shape as erp_client/mes_client for the task-based
methods, and adds tag-level read/write for direct process I/O.
"""
from __future__ import annotations

import httpx

from advanced_robotics.core.errors import IntegrationError
from advanced_robotics.integrations.plant_client import PlantTask


class ScadaClient:
    def __init__(self, base_url: str, api_key: str, timeout_s: float = 5.0) -> None:
        self._client = httpx.AsyncClient(
            base_url=base_url,
            headers={"Authorization": f"Bearer {api_key}"},
            timeout=timeout_s,
        )

    async def fetch_pending_tasks(self) -> list[PlantTask]:
        try:
            response = await self._client.get("/alarms", params={"active": "true"})
            response.raise_for_status()
        except httpx.HTTPError as exc:
            raise IntegrationError(f"SCADA fetch_pending_tasks failed: {exc}") from exc

        return [
            PlantTask(task_id=item["alarm_id"], task_type="alarm", payload=item)
            for item in response.json()
        ]

    async def report_status(self, task_id: str, status: str, details: dict) -> None:
        try:
            response = await self._client.post(
                f"/alarms/{task_id}/ack", json={"status": status, "details": details}
            )
            response.raise_for_status()
        except httpx.HTTPError as exc:
            raise IntegrationError(f"SCADA report_status failed: {exc}") from exc

    async def read_tag(self, tag: str) -> float:
        try:
            response = await self._client.get(f"/tags/{tag}")
            response.raise_for_status()
        except httpx.HTTPError as exc:
            raise IntegrationError(f"SCADA read_tag({tag}) failed: {exc}") from exc
        return float(response.json()["value"])

    async def write_tag(self, tag: str, value: float) -> None:
        try:
            response = await self._client.put(f"/tags/{tag}", json={"value": value})
            response.raise_for_status()
        except httpx.HTTPError as exc:
            raise IntegrationError(f"SCADA write_tag({tag}) failed: {exc}") from exc

    async def aclose(self) -> None:
        await self._client.aclose()
