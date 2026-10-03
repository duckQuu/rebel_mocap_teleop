import math

import numpy as np

from rebel_demo.kinematics import HOME, STRAIGHT_UP, fk, ik, quat_to_matrix


def test_straight_up_pose():
    T = fk(STRAIGHT_UP)
    assert np.allclose(T[:3, 3], [0, 0, 0.8984], atol=1e-3)


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
