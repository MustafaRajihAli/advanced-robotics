"""Forward/inverse kinematics for a configurable serial arm.

Uses standard Denavit-Hartenberg parameters supplied per-joint so this works
for any 6-DOF (or fewer) arm without hardcoding a specific model.

The inverse solver follows TRAC-IK's core idea (Beeson & Ames, 2015): a
single damped-least-squares Newton run gets stuck in local minima at joint
limits, so on failure it retries from randomized joint configurations
within limits rather than giving up after one attempt. This is a numerical
approximation of TRAC-IK's dual-solver approach (KDL-style Newton + SQP run
concurrently); swap in `pick_ik` or a MoveIt 2 IK plugin for production use
where sub-millisecond solve time or exact analytic solutions are required.
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


@dataclass(frozen=True)
class JointLimit:
    lower_rad: float
    upper_rad: float

    def clamp(self, value: float) -> float:
        return min(max(value, self.lower_rad), self.upper_rad)


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
    def __init__(self, dh_params: list[DHParam], joint_limits: list[JointLimit] | None = None) -> None:
        self.dh_params = dh_params
        self.joint_limits = joint_limits

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
        max_restarts: int = 5,
        rng: np.random.Generator | None = None,
    ) -> list[float]:
        """Damped least-squares numerical IK with joint-limit clamping and
        randomized-restart local-minima escape (TRAC-IK's core idea).

        Tries `initial_guess_rad` first; if it doesn't converge within
        tolerance, retries from up to `max_restarts` randomized starting
        configurations sampled within joint_limits (or a fixed window around
        the guess if no limits are set) before returning the best attempt.
        """
        rng = rng or np.random.default_rng()
        target_pos = target_pose[:3, 3]

        best_q = np.array(initial_guess_rad, dtype=float)
        best_error = np.inf

        candidates = [np.array(initial_guess_rad, dtype=float)]
        for _ in range(max_restarts):
            candidates.append(self._random_configuration(initial_guess_rad, rng))

        for start in candidates:
            q = self._solve_from(start, target_pos, max_iters, tol, step_scale)
            error = float(np.linalg.norm(self.forward(list(q))[:3, 3] - target_pos))
            if error < best_error:
                best_error, best_q = error, q
            if error < tol:
                break

        return list(best_q)

    def _solve_from(
        self, start: np.ndarray, target_pos: np.ndarray, max_iters: int, tol: float, step_scale: float
    ) -> np.ndarray:
        q = start.copy()
        for _ in range(max_iters):
            current_pos = self.forward(list(q))[:3, 3]
            error = target_pos - current_pos
            if np.linalg.norm(error) < tol:
                break

            jacobian = self._numerical_jacobian(q)
            damped = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + 1e-6 * np.eye(3), error
            )
            q = q + step_scale * damped
            q = self._clamp_to_limits(q)

        return q

    def _clamp_to_limits(self, q: np.ndarray) -> np.ndarray:
        if self.joint_limits is None:
            return q
        return np.array([limit.clamp(v) for limit, v in zip(self.joint_limits, q)])

    def _random_configuration(self, initial_guess_rad: list[float], rng: np.random.Generator) -> np.ndarray:
        if self.joint_limits is not None:
            return np.array([rng.uniform(l.lower_rad, l.upper_rad) for l in self.joint_limits])
        # No limits configured: perturb around the initial guess within +/- pi.
        guess = np.array(initial_guess_rad, dtype=float)
        return guess + rng.uniform(-np.pi, np.pi, size=guess.shape)

    def _numerical_jacobian(self, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        base_pos = self.forward(list(q))[:3, 3]
        jac = np.zeros((3, len(q)))
        for i in range(len(q)):
            dq = q.copy()
            dq[i] += eps
            perturbed_pos = self.forward(list(dq))[:3, 3]
            jac[:, i] = (perturbed_pos - base_pos) / eps
        return jac
