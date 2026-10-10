"""igus ReBeL (rebel2) 6-DoF kinematics: numbers from igus_rebel_description's igus_rebel2_arm_macro.xacro
(the arm of dual_arm_rig_v2.urdf.xacro that the Isaac rig uses, same for arm_left_ / arm_right_). fk() / ik() work in tool0 (link6 + 52.7 mm)."""
import math

import numpy as np
# from moveit_msgs.srv import GetPositionIK   # unused for now; needs MoveIt installed (keep kinematics numpy-only)

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
JOINT_LABELS = ["J1 base", "J2 shoulder", "J3 elbow", "J4 forearm roll", "J5 wrist", "J6 flange roll"]
JOINT_COLORS = [(0.90, 0.10, 0.10), (1.00, 0.55, 0.00), (0.95, 0.85, 0.10),
                (0.20, 0.80, 0.20), (0.10, 0.60, 1.00), (0.70, 0.30, 1.00)]

# (origin xyz [m], origin rpy [rad], axis, lower [rad], upper [rad]
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

# for isaac: igus_rebel2_arm_macro.xacro (fixed motor joints merged), spec-sheet limits
JOINTS = [
    ((0, 0, 0.1462),     (0, 0, math.pi), (0, 0, 1),  -3.1241,  3.1241),   # joint1
    ((0, 0.0265, 0.106),  (0, 0, 0),       (0, -1, 0), -1.396263, 2.4435),  # joint2 (lower: igus spec, -80 deg)
    ((0, -0.0265, 0.24152), (0, 0, 0),     (0, -1, 0), math.radians(15.0), 2.443461), # joint3 (upper: igus spec, 140 deg, limit to 15 deg to prevent down elbow)
    ((0.001345, 0, 0.157479), (0, 0, 0),   (0, 0, 1),  -3.12414, 3.12414), # joint4
    ((0, 0, 0.142),       (0, 0, 0),       (0, -1, 0), -1.65806, 1.65806), # joint5
    ((0, 0, 0.0768),      (0, 0, 0),       (0, 0, 1),  -3.12414, 3.12414), # joint6
]

LOWER = np.array([j[3] for j in JOINTS])
UPPER = np.array([j[4] for j in JOINTS])
VMAX = math.radians(45.0)                       # rad/s, every joint (URDF velocity limit)

# "ready" pose: tool0 35 cm in front of the arm base and 30 cm above it, pointing straight down, elbow bent
# (joint3 +88 deg). All-zero would be the arm straight up: a wrist singularity with the gripper at the ceiling.
HOME = np.radians([0.0, 13.4, 87.9, 0.0, 78.7, 0.0])
# for straight up
# HOME = np.radians([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


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


_CHAIN = [(np.array(xyz, float), rpy(*r_p_y), np.array(ax, float), ax) for xyz, r_p_y, ax, _, _ in JOINTS]


def limits_hit(q, lower=None, upper=None, margin_deg=1.0):
    """Joints within margin_deg of a limit: [(joint_index, "lower" | "upper")] (default: LOWER / UPPER)."""
    lo = LOWER if lower is None else lower
    up = UPPER if upper is None else upper
    m = math.radians(margin_deg)
    out = []
    for i, qi in enumerate(np.asarray(q, float)):
        if qi <= lo[i] + m:
            out.append((i, "lower"))
        elif qi >= up[i] - m:
            out.append((i, "upper"))
    return out


def fk_jac(q):
    """Flange pose (4x4, base_link) and its geometric Jacobian (6x6: rows 0-2 linear, 3-5 angular) in one pass."""
    R, p = np.eye(3), np.zeros(3)
    origins, axes = [], []
    for (xyz, R0, axv, ax), qi in zip(_CHAIN, q):
        p = p + R @ xyz
        R = R @ R0
        a = R @ axv
        origins.append(p.copy())
        axes.append(a)
        R = R @ rot(ax, qi)
    T = np.eye(4)
    T[:3, :3] = R @ _FLANGE_R
    T[:3, 3] = p + R @ _FLANGE_P
    J = np.empty((6, 6))
    for i, (o, a) in enumerate(zip(origins, axes)):
        J[:3, i] = np.cross(a, T[:3, 3] - o)
        J[3:, i] = a
    return T, J


def _dls_step(J, e, free, lam, q, q_rest, k_null):
    """Damped least-squares joint step using only the `free` joints (pinned joints stay put)."""
    Jf = J[:, free]
    Jp = Jf.T @ np.linalg.inv(Jf @ Jf.T + lam ** 2 * np.eye(J.shape[0]))
    sf = Jp @ e
    if q_rest is not None:
        sf = sf + (np.eye(Jf.shape[1]) - Jp @ Jf) @ (k_null * (q_rest - q)[free])
    step = np.zeros(6)
    step[free] = sf
    return step


def ik(T_goal, q_seed, w_rot=0.3, iters=60, lam=0.02, q_rest=HOME, k_null=0.05):
    """Flange pose T_goal (4x4, base_link) -> joint angles [rad] (damped least squares, analytic Jacobian).
    w_rot = 0 -> position only (wrist free, pulled gently towards q_rest).
    Joint limits are handled by pinning: a joint whose step would cross LOWER / UPPER is held at the limit and the
    step is solved again for the remaining joints, so an unreachable target gives the closest reachable pose (the arm
    slides along the limit) instead of a corner posture far from it.
    Returns (q, position_error_m, rotation_error_deg)."""
    q = np.clip(np.asarray(q_seed, float).copy(), LOWER, UPPER)
    rows = 3 if w_rot == 0 else 6
    for _ in range(iters):
        T, J = fk_jac(q)
        e = _err(T, T_goal, w_rot)[:rows]
        J = J[:rows].copy()
        J[3:] *= w_rot
        free = np.ones(6, bool)
        fixed = np.zeros(6)                                   # joint moves that are pinned at a limit
        for _pass in range(4):
            step = fixed + _dls_step(J, e - J @ fixed, free, lam, q, q_rest, k_null)
            step = np.where(free, np.clip(step, -0.3, 0.3), fixed)
            bad = free & ((q + step < LOWER) | (q + step > UPPER))
            if not bad.any():
                break
            free &= ~bad                                      # pin them at the limit, re-solve the others
            fixed[bad] = np.clip(q + step, LOWER, UPPER)[bad] - q[bad]
        if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) <= 1e-3 * w_rot and np.linalg.norm(step) < 1e-4:
            break
        q = np.clip(q + step, LOWER, UPPER)
    T = fk(q)
    pe = float(np.linalg.norm(T_goal[:3, 3] - T[:3, 3]))
    re = float(math.degrees(np.linalg.norm(_log_so3(T_goal[:3, :3] @ T[:3, :3].T))))
    return q, pe, re
