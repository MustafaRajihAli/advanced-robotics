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
    # Nav2 costmap-inflation-style tuning (see docs/ARCHITECTURE.md for source)
    inflation_radius_m: float = 0.55
    cost_scaling_factor: float = 3.0
    stuck_recovery_ticks: int = 20


class ArmConfig(BaseModel):
    dof: int
    force_limit_n: float
    torque_limit_nm: float
    positioning_tolerance_mm: float
    ik_max_restarts: int = 5


class VisionConfig(BaseModel):
    camera_ids: list[str]
    lidar_id: str
    inference_backend: str
    defect_confidence_threshold: float
    nms_iou_threshold: float = 0.45
    model_input_size: int = 640


class SafetyConfig(BaseModel):
    estop_heartbeat_timeout_ms: int
    watchdog_interval_ms: int
    # ISO/TS 15066 speed-and-separation-monitoring parameters
    ssm_human_speed_mps: float = 1.6
    ssm_reaction_time_s: float = 0.1
    ssm_stopping_time_s: float = 0.2
    ssm_intrusion_distance_m: float = 0.85


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
