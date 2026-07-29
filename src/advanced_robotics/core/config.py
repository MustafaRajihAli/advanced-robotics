"""Config loading shared by every subsystem."""
from __future__ import annotations

import os
from pathlib import Path

import yaml
from pydantic import BaseModel


class AMRConfig(BaseModel):
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    obstacle_safety_margin_m: float
    fleet_size: int


class ArmConfig(BaseModel):
    dof: int
    force_limit_n: float
    torque_limit_nm: float
    positioning_tolerance_mm: float


class VisionConfig(BaseModel):
    camera_ids: list[str]
    lidar_id: str
    inference_backend: str
    defect_confidence_threshold: float


class SafetyConfig(BaseModel):
    estop_heartbeat_timeout_ms: int
    watchdog_interval_ms: int


class IntegrationsConfig(BaseModel):
    erp_enabled: bool
    mes_enabled: bool
    scada_enabled: bool


class DigitalTwinConfig(BaseModel):
    backend: str
    world: str


class AppConfig(BaseModel):
    amr: AMRConfig
    arm: ArmConfig
    vision: VisionConfig
    safety: SafetyConfig
    integrations: IntegrationsConfig
    digital_twin: DigitalTwinConfig


def load_config(path: str | Path | None = None) -> AppConfig:
    """Load YAML config, defaulting to config/default.yaml relative to the repo root."""
    if path is None:
        path = os.environ.get(
            "ADVANCED_ROBOTICS_CONFIG",
            Path(__file__).resolve().parents[3] / "config" / "default.yaml",
        )
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    return AppConfig.model_validate(raw)
