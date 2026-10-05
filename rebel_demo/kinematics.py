"""igus ReBeL 6-DoF kinematics (numbers identical to urdf/igus_rebel_6dof.urdf,
from CommonplaceRobotics/iRC_ROS irc_ros_description, rebel_version 00/01)."""
import math

import numpy as np

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
JOINT_LABELS = ["J1 base", "J2 shoulder", "J3 elbow", "J4 forearm roll", "J5 wrist", "J6 flange roll"]
JOINT_COLORS = [(0.90, 0.10, 0.10), (1.00, 0.55, 0.00), (0.95, 0.85, 0.10),
                (0.20, 0.80, 0.20), (0.10, 0.60, 1.00), (0.70, 0.30, 1.00)]

# (origin xyz [m], origin rpy [rad], axis, lower [rad], upper [rad])

# for rviz
# JOINTS = [
#     ((0, 0, 0.100), (0, 0, 0), (0, 0, -1), -math.pi * 179 / 180, math.pi * 179 / 180),
#     ((0, 0, 0.149), (0, math.pi / 6, 0), (0, 1, 0), -math.pi * 11 / 18, math.pi * 11 / 18),
#     ((0, 0, 0.2384), (0, math.pi / 6, 0), (0, 1, 0), -math.pi * 11 / 18, math.pi * 11 / 18),
#     ((0, 0, 0.149 - 0.03), (0, 0, 0), (0, 0, 1), -math.pi * 179 / 180, math.pi * 179 / 180),
#     ((0, 0, 0.14 + 0.03), (0, -math.pi / 24, 0), (0, 1, 0),
#      -math.pi * 19 / 36 + math.pi / 24, math.pi * 19 / 36 + math.pi / 24),
#     ((0, 0, 0.1208), (0, 0, 0), (0, 0, 1), -math.pi * 179 / 180, math.pi * 179 / 180),
# ]

# for isaac
JOINTS = [
    ((0, 0, 0.1462),     (0, 0, math.pi), (0, 0, 1),  -3.1241,  3.1241),   # joint1
    ((0, 0.0265, 0.106),  (0, 0, 0),       (0, -1, 0), -1.4835,  2.4435),  # joint2
    ((0, -0.0265, 0.24152), (0, 0, 0),     (0, -1, 0), -1.39626, 2.61799), # joint3
    ((0.001345, 0, 0.157479), (0, 0, 0),   (0, 0, 1),  -3.12414, 3.12414), # joint4
    ((0, 0, 0.142),       (0, 0, 0),       (0, -1, 0), -1.65806, 1.65806), # joint5
    ((0, 0, 0.0768),      (0, 0, 0),       (0, 0, 1),  -3.12414, 3.12414), # joint6
]

LOWER = np.array([j[3] for j in JOINTS])
UPPER = np.array([j[4] for j in JOINTS])
VMAX = math.radians(45.0)                       # rad/s, every joint (URDF velocity limit)

HOME = np.radians([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])        # "ready" pose, flange in front, pointing down
# STRAIGHT_UP = np.radians([0.0, -30.0, -30.0, 0.0, 7.5, 0.0])  # URDF has built-in 30/30/-7.5 deg offsets


def rot(axis, a):
    x, y, z = axis
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def rpy(r, p, y):
    return rot((0, 0, 1), y) @ rot((0, 1, 0), p) @ rot((1, 0, 0), r)


_FLANGE_R = rpy(0, 0, math.pi)            # link_8 -> flange (tool0): x axis points out of the flange
_FLANGE_P = np.array([0, 0, 0.0527])


def quat_to_matrix(q):
    """[x, y, z, w] -> 3x3 rotation matrix."""
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def joint_frames(q):
    """[(position, rotation axis)] of each joint in base_link for joint angles q [rad]."""
    R, p, out = np.eye(3), np.zeros(3), []
    for (xyz, r_p_y, ax, _, _), qi in zip(JOINTS, q):
        p = p + R @ np.array(xyz, float)
        R = R @ rpy(*r_p_y)
        out.append((p.copy(), R @ np.array(ax, float)))
        R = R @ rot(ax, qi)
    return out


def fk(q):
    """4x4 flange (tool0) pose in base_link."""
    R, p = np.eye(3), np.zeros(3)
    for (xyz, r_p_y, ax, _, _), qi in zip(JOINTS, q):
        p = p + R @ np.array(xyz, float)
        R = R @ rpy(*r_p_y) @ rot(ax, qi)
    T = np.eye(4)
    T[:3, :3] = R @ _FLANGE_R
    T[:3, 3] = p + R @ _FLANGE_P
    return T


# ---------------------------------------------------------------- IK (damped least squares)
def _log_so3(Rm):
    c = np.clip((np.trace(Rm) - 1) / 2, -1, 1)
    th = math.acos(c)
    v = np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])
    if th < 1e-6:
        return 0.5 * v
    if th > math.pi - 1e-4:                         # near 180 deg: robust fallback
        w, V = np.linalg.eigh((Rm + Rm.T) / 2)
        axis = V[:, np.argmax(w)]
        return th * axis
    return th / (2 * math.sin(th)) * v


def _err(T, Tg, w_rot):
    return np.r_[Tg[:3, 3] - T[:3, 3], w_rot * _log_so3(Tg[:3, :3] @ T[:3, :3].T)]


def ik(T_goal, q_seed, w_rot=0.3, iters=60, lam=0.02, q_rest=None, k_null=0.05):
    """Flange pose T_goal (4x4, base_link) -> joint angles [rad].
    w_rot = 0 -> position only (wrist free, pulled gently towards q_rest).
    Returns (q, position_error_m, rotation_error_deg)."""
    q = np.clip(np.asarray(q_seed, float).copy(), LOWER, UPPER)
    for _ in range(iters):
        T = fk(q)
        e = _err(T, T_goal, w_rot)
        J = np.empty((6, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = 1e-6
            J[:, i] = (e - _err(fk(q + dq), T_goal, w_rot)) / 1e-6
        Jp = J.T @ np.linalg.inv(J @ J.T + lam ** 2 * np.eye(6))
        step = Jp @ e
        if q_rest is not None:
            step = step + (np.eye(6) - Jp @ J) @ (k_null * (q_rest - q))
        if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) <= 1e-3 * w_rot and np.linalg.norm(step) < 1e-4:
            break
        q = np.clip(q + np.clip(step, -0.3, 0.3), LOWER, UPPER)
    T = fk(q)
    pe = float(np.linalg.norm(T_goal[:3, 3] - T[:3, 3]))
    re = float(math.degrees(np.linalg.norm(_log_so3(T_goal[:3, :3] @ T[:3, :3].T))))
    return q, pe, re
