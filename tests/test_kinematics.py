import numpy as np

from advanced_robotics.arm.kinematics import ArmKinematics, DHParam


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
