"""ISO/TS 15066 Speed and Separation Monitoring (SSM).

Implements the protective separation distance formula from ISO/TS 15066
(now folded into ISO 10218-2:2025 for collaborative applications). Below
this distance the robot must be at a safety-rated controlled stop.

    S(t0) = Sh(t0) + Sr(t0) + Ss(t0) + C + Zd + Zr

Where:
    Sh  human contribution: distance the operator can cover during the
        combined response + stopping time, at the human's directed speed
    Sr  robot contribution while still moving toward the operator during
        the system's reaction time
    Ss  additional distance the robot travels while decelerating to a stop
    C   intrusion distance (expected reach of the operator into the
        sensing field before detection, e.g. per ISO 13855)
    Zd  position uncertainty of the human as measured by the sensing system
    Zr  position uncertainty of the robot (encoder/calibration tolerance)

This module computes the formula; it does not replace a certified
safety-rated sensing system or a formal risk assessment. Treat
`protective_separation_distance_m` as the minimum distance at which
SafetyMonitor must trip a controlled stop, tuned per-application.

Reference: ISO/TS 15066:2016 Annex A; folded into ISO 10218-2:2025.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SsmParameters:
    human_speed_mps: float = 1.6  # ISO/TS 15066 default assumed human speed
    reaction_time_s: float = 0.1  # sensing + control system reaction time (Tr)
    stopping_time_s: float = 0.2  # time for robot to reach a safety-rated stop (Ts)
    intrusion_distance_m: float = 0.85  # C, per ISO 13855 for the sensing modality used
    human_position_uncertainty_m: float = 0.05  # Zd
    robot_position_uncertainty_m: float = 0.02  # Zr


def protective_separation_distance_m(
    robot_speed_toward_human_mps: float,
    params: SsmParameters | None = None,
) -> float:
    """Minimum separation distance (m) below which the robot must stop.

    `robot_speed_toward_human_mps` is the robot's speed component directed
    at the operator (>= 0). The robot's stopping distance term assumes
    roughly linear deceleration over `stopping_time_s`.
    """
    if robot_speed_toward_human_mps < 0:
        raise ValueError("robot_speed_toward_human_mps must be >= 0")
    params = params or SsmParameters()

    human_contribution = params.human_speed_mps * (
        params.reaction_time_s + params.stopping_time_s
    )
    robot_reaction_contribution = robot_speed_toward_human_mps * params.reaction_time_s
    robot_stopping_contribution = 0.5 * robot_speed_toward_human_mps * params.stopping_time_s

    return (
        human_contribution
        + robot_reaction_contribution
        + robot_stopping_contribution
        + params.intrusion_distance_m
        + params.human_position_uncertainty_m
        + params.robot_position_uncertainty_m
    )


def is_separation_violated(
    measured_distance_m: float,
    robot_speed_toward_human_mps: float,
    params: SsmParameters | None = None,
) -> bool:
    """True if the measured human-robot distance is below the required
    protective separation distance and the robot must stop."""
    required = protective_separation_distance_m(robot_speed_toward_human_mps, params or SsmParameters())
    return measured_distance_m < required
