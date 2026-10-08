"""Pure-python tests (no ROS needed): python3 -m pytest test/"""
import math

import numpy as np

from rebel_demo.kinematics import HOME, LOWER, UPPER, fk, ik, quat_to_matrix


def test_home_pose():
    T = fk(HOME)                                   # all joints 0: rebel2 straight up, tool0 pointing up
    assert np.allclose(T[:3, 3], [-0.0013, 0, 0.9227], atol=1e-3)
    assert np.allclose(T[:3, 2], [0, 0, 1], atol=1e-6)


def test_spec_sheet_limits():
    assert math.isclose(LOWER[1], math.radians(-80), abs_tol=1e-5)    # joint2
    assert math.isclose(UPPER[2], math.radians(140), abs_tol=1e-5)    # joint3


def test_ik_round_trip():
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = HOME + rng.uniform(-0.4, 0.4, 6)
        _, pe, re = ik(fk(q), q + rng.uniform(-0.1, 0.1, 6))
        assert pe < 1e-3 and re < 0.5


def test_quat_to_matrix():
    a = math.radians(90)
    R = quat_to_matrix([0, 0, math.sin(a / 2), math.cos(a / 2)])
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-9)


def test_gripper_threshold_hysteresis():
    import importlib.util
    if importlib.util.find_spec("rclpy") is None:
        return
    from rebel_demo.gripper_node import next_state
    assert next_state(0.02, 1, 0.06, 0.01) == 0           # near -> close
    assert next_state(0.10, 0, 0.06, 0.01) == 1           # far -> open
    assert next_state(0.062, 0, 0.06, 0.01) == 0          # inside dead band -> keep
    assert next_state(0.062, 1, 0.06, 0.01) == 1
    assert next_state(0.02, 0, 0.06, 0.01, invert=True) == 1
