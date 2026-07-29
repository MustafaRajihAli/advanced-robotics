"""Forward/inverse kinematics for a configurable serial arm.

Uses standard Denavit-Hartenberg parameters supplied per-joint so this works
for any 6-DOF (or fewer) arm without hardcoding a specific model.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DHParam:
    a: float
    alpha: float
    d: float
    theta_offset: float = 0.0


def _dh_transform(param: DHParam, theta: float) -> np.ndarray:
    t = theta + param.theta_offset
    ct, st = np.cos(t), np.sin(t)
    ca, sa = np.cos(param.alpha), np.sin(param.alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, param.a * ct],
            [st, ct * ca, -ct * sa, param.a * st],
            [0, sa, ca, param.d],
            [0, 0, 0, 1],
        ]
    )


class ArmKinematics:
    def __init__(self, dh_params: list[DHParam]) -> None:
        self.dh_params = dh_params

    def forward(self, joint_angles_rad: list[float]) -> np.ndarray:
        """Return the 4x4 end-effector pose in the base frame."""
        if len(joint_angles_rad) != len(self.dh_params):
            raise ValueError("joint_angles_rad length must match dh_params length")
        pose = np.eye(4)
        for param, theta in zip(self.dh_params, joint_angles_rad):
            pose = pose @ _dh_transform(param, theta)
        return pose

    def inverse(
        self,
        target_pose: np.ndarray,
        initial_guess_rad: list[float],
        max_iters: int = 200,
        tol: float = 1e-4,
        step_scale: float = 0.5,
    ) -> list[float]:
        """Damped least-squares numerical IK. Good enough for a scaffold;
        replace with an analytic or MoveIt-based solver for production."""
        q = np.array(initial_guess_rad, dtype=float)
        target_pos = target_pose[:3, 3]

        for _ in range(max_iters):
            current = self.forward(list(q))
            current_pos = current[:3, 3]
            error = target_pos - current_pos
            if np.linalg.norm(error) < tol:
                break

            jacobian = self._numerical_jacobian(q)
            damped = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + 1e-6 * np.eye(3), error
            )
            q = q + step_scale * damped

        return list(q)

    def _numerical_jacobian(self, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        base_pos = self.forward(list(q))[:3, 3]
        jac = np.zeros((3, len(q)))
        for i in range(len(q)):
            dq = q.copy()
            dq[i] += eps
            perturbed_pos = self.forward(list(dq))[:3, 3]
            jac[:, i] = (perturbed_pos - base_pos) / eps
        return jac
