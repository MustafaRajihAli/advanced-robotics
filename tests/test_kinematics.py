import numpy as np

from advanced_robotics.arm.kinematics import ArmKinematics, DHParam, JointLimit


def _simple_2dof_arm() -> ArmKinematics:
    # Two revolute joints, each with a 1m link, planar (alpha=0).
    return ArmKinematics(
        [
            DHParam(a=1.0, alpha=0.0, d=0.0),
            DHParam(a=1.0, alpha=0.0, d=0.0),
        ]
    )


def test_forward_kinematics_straight_arm():
    arm = _simple_2dof_arm()
    pose = arm.forward([0.0, 0.0])
    assert np.allclose(pose[:3, 3], [2.0, 0.0, 0.0], atol=1e-6)


def test_inverse_kinematics_converges():
    arm = _simple_2dof_arm()
    target = arm.forward([0.3, 0.5])
    solved = arm.inverse(target, initial_guess_rad=[0.0, 0.0])
    result_pose = arm.forward(solved)
    assert np.allclose(result_pose[:3, 3], target[:3, 3], atol=1e-3)


def test_inverse_kinematics_respects_joint_limits():
    arm = ArmKinematics(
        [
            DHParam(a=1.0, alpha=0.0, d=0.0),
            DHParam(a=1.0, alpha=0.0, d=0.0),
        ],
        joint_limits=[JointLimit(-1.0, 1.0), JointLimit(-1.0, 1.0)],
    )
    target = arm.forward([0.3, 0.5])
    solved = arm.inverse(target, initial_guess_rad=[0.0, 0.0], rng=np.random.default_rng(0))
    assert all(-1.0 <= q <= 1.0 for q in solved)


def test_inverse_kinematics_restarts_escape_bad_initial_guess():
    arm = _simple_2dof_arm()
    target = arm.forward([1.2, -0.8])
    # Deliberately poor initial guess; restarts should still find a solution.
    solved = arm.inverse(
        target, initial_guess_rad=[3.0, 3.0], max_restarts=8, rng=np.random.default_rng(42)
    )
    result_pose = arm.forward(solved)
    assert np.allclose(result_pose[:3, 3], target[:3, 3], atol=1e-2)
