"""Pure-python tests (no ROS needed): python3 -m pytest test/"""
import math

import numpy as np

from rebel_demo.kinematics import HOME, LOWER, UPPER, fk, ik, quat_to_matrix


def test_home_pose():
    T = fk(HOME)                                   # tool0 35 cm in front, 30 cm up, pointing straight down
    assert np.allclose(T[:3, 3], [0.35, 0, 0.30], atol=2e-3)
    assert np.allclose(T[:3, 2], [0, 0, -1], atol=1e-2)
    assert HOME[2] > 0                             # elbow bent the normal way


def test_analytic_jacobian():
    from rebel_demo.kinematics import fk_jac
    q = np.radians([10, 30, 60, -20, 40, 15])
    T, J = fk_jac(q)
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = 1e-6
        assert np.allclose((fk(q + dq)[:3, 3] - T[:3, 3]) / 1e-6, J[:3, i], atol=1e-5)


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


def _q(axis, deg):
    a = math.radians(deg) / 2
    n = math.sqrt(sum(c * c for c in axis))
    return tuple(c / n * math.sin(a) for c in axis) + (math.cos(a),)


def test_gripper_orientation_angle():
    import importlib.util
    if importlib.util.find_spec("rclpy") is None:
        return
    from rebel_demo.gripper_node import next_state_angle, quat_mul, rotation_angle_deg
    assert abs(rotation_angle_deg((0, 0, 0, 1))) < 1e-9
    for axis in [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0)]:
        assert abs(rotation_angle_deg(_q(axis, 180)) - 180) < 1e-6          # flipped about any axis
        assert abs(rotation_angle_deg(_q(axis, 60)) - 60) < 1e-6
    q = _q((0, 1, 0), 60)
    assert abs(rotation_angle_deg(tuple(-c for c in q)) - 60) < 1e-6        # q and -q are the same rotation
    # turning the whole hand (both bodies by the same world rotation) does not change the difference
    palm, finger = _q((0, 0, 1), 20), quat_mul(_q((0, 0, 1), 20), _q((1, 0, 0), 150))
    face = _q((1, 1, 0), 77)
    rel = lambda a, b: quat_mul((-a[0], -a[1], -a[2], a[3]), b)             # a^-1 * b
    assert abs(rotation_angle_deg(rel(palm, finger)) - rotation_angle_deg(rel(quat_mul(face, palm), quat_mul(face, finger)))) < 1e-6
    # calibration: the open pose counts as 0 deg
    q_open = _q((0, 0, 1), 40)
    assert abs(rotation_angle_deg(q_open, q_open)) < 1e-6
    assert abs(rotation_angle_deg(quat_mul(q_open, _q((1, 0, 0), 180)), q_open) - 180) < 1e-6


def test_gripper_angle_bands():
    import importlib.util
    if importlib.util.find_spec("rclpy") is None:
        return
    from rebel_demo.gripper_node import next_state_angle
    assert next_state_angle(10, 0, 30, 100) == 1           # same plane -> open
    assert next_state_angle(170, 1, 30, 100) == 0          # reversed -> closed
    assert next_state_angle(60, 1, 30, 100) == 1           # in between -> keep
    assert next_state_angle(60, 0, 30, 100) == 0
    assert next_state_angle(10, 1, 30, 100, invert=True) == 0


def test_keyboard_debounce():
    import importlib.util
    if importlib.util.find_spec("rclpy") is None:
        return
    from rebel_demo.keyboard_node import KeyDebounce
    d = KeyDebounce(0.3)
    assert d(10.0) is True
    assert d(10.1) is False                 # key repeat of a held SPACE
    assert d(10.35) is True


def test_limits_hit():
    from rebel_demo.kinematics import limits_hit
    assert limits_hit(HOME) == []                                   # HOME is well inside every limit
    q = HOME.copy()
    q[1], q[2] = UPPER[1], LOWER[2]                                 # shoulder at its upper limit, elbow at the bound
    assert limits_hit(q) == [(1, "upper"), (2, "lower")]
    q = HOME.copy()
    q[2] = LOWER[2] + math.radians(0.5)                             # inside the 1 deg margin
    assert limits_hit(q) == [(2, "lower")]
    assert limits_hit(q, margin_deg=0.1) == []                      # outside a tighter margin


def test_ik_unreachable_target_stays_inside_limits_and_gets_close():
    far = fk(HOME)
    far[:3, 3] = [0.3, 0.0, -1.5]                                    # far below anything the arm can reach
    q, pe, _ = ik(far, HOME, w_rot=0.0, iters=200, q_rest=HOME)
    assert np.all(q >= LOWER - 1e-9) and np.all(q <= UPPER + 1e-9)   # never outside the limits (incl. elbow bound)
    assert pe < np.linalg.norm(far[:3, 3] - fk(HOME)[:3, 3])         # moved towards the target
    # slightly beyond reach: pinning at the limit gives the closest pose, not a corner posture far away
    near = fk(HOME)
    near[:3, 3] = fk(HOME)[:3, 3] + [0.15, 0.0, -0.55]               # 55 cm down, 15 cm forward (just out of reach)
    q = HOME.copy()
    for _ in range(400):
        qi, pe, _ = ik(near, q, w_rot=0.0, iters=25, q_rest=HOME)
        q = np.clip(q + np.clip(qi - q, -0.0125, 0.0125), LOWER, UPPER)
    assert np.linalg.norm(fk(q)[:3, 3] - near[:3, 3]) < 0.14
