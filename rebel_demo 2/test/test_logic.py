"""Pure-python tests (no ROS needed): python3 -m pytest test/"""
import glob
import math
import os

import numpy as np

from rebel_demo import kinematics as K
from rebel_demo.trajectory_io import Trajectory, robot_program

HERE = os.path.dirname(os.path.abspath(__file__))
TRAJ = sorted(glob.glob(os.path.join(HERE, "..", "trajectories", "*.csv")))


def test_straight_up_pose():
    T = K.fk(K.STRAIGHT_UP)
    assert np.allclose(T[:3, 3], [0, 0, 0.8984], atol=1e-3)     # flange straight above the base
    assert np.allclose(T[:3, 0], [0, 0, 1], atol=1e-6)          # tool0 x axis points up


def test_trajectories_follow_their_flange_path():
    for f in TRAJ:
        tr = Trajectory(f)
        for k in range(0, len(tr.q), 25):
            assert np.linalg.norm(K.fk(tr.q[k])[:3, 3] - tr.path[k]) < 0.005, (tr.name, k)


def test_robot_program_respects_speed_and_starts_at_current_pose():
    for f in TRAJ:
        tr = Trajectory(f)
        t, q, info = robot_program(tr, K.HOME, speed_fraction=0.5)
        v = (np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None]).max()
        assert v <= 0.5 * K.VMAX * 1.02, (tr.name, math.degrees(v))
        assert np.allclose(q[0], K.HOME) and np.allclose(q[-1], tr.q[-1])
        assert np.all(np.diff(t) > 0)
        margin = np.minimum(q - K.LOWER, K.UPPER - q).min()
        assert info["within_limits"] == bool(margin >= math.radians(2.0))   # limit check is consistent


def test_ik_round_trip():
    """IK (used live by the teleop node) recovers FK poses from a nearby seed."""
    import numpy as np
    from rebel_demo.kinematics import HOME, fk, ik
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = HOME + rng.uniform(-0.4, 0.4, 6)
        q_sol, pe, re = ik(fk(q), q + rng.uniform(-0.1, 0.1, 6))
        assert pe < 1e-3 and re < 0.5
