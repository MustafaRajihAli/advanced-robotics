"""MES integration client (e.g. production scheduling, batch tracking)."""
from __future__ import annotations

import httpx

from advanced_robotics.core.errors import IntegrationError
from advanced_robotics.integrations.plant_client import PlantTask


class MesClient:
    def __init__(self, base_url: str, api_key: str, timeout_s: float = 10.0) -> None:
        self._client = httpx.AsyncClient(
            base_url=base_url,
            headers={"Authorization": f"Bearer {api_key}"},
            timeout=timeout_s,
        )

    async def fetch_pending_tasks(self) -> list[PlantTask]:
        try:
            response = await self._client.get("/jobs", params={"status": "queued"})
            response.raise_for_status()
        except httpx.HTTPError as exc:
            raise IntegrationError(f"MES fetch_pending_tasks failed: {exc}") from exc

        return [
            PlantTask(task_id=item["job_id"], task_type=item["job_type"], payload=item)
            for item in response.json()
        ]

    async def report_status(self, task_id: str, status: str, details: dict) -> None:
        try:
            response = await self._client.post(
                f"/jobs/{task_id}/status", json={"status": status, "details": details}
            )
            response.raise_for_status()
        except httpx.HTTPError as exc:
            raise IntegrationError(f"MES report_status failed: {exc}") from exc

    async def aclose(self) -> None:
        await self._client.aclose()
