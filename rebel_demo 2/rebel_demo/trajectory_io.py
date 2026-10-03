"""Reading demo trajectory CSVs and building smooth, speed-limited joint timelines (no ROS here)."""
import csv
import glob
import math
import os

import numpy as np

from .kinematics import LOWER, UPPER, VMAX


def read_csv(path):
    """Minimal CSV reader: {column: np.array(float)}; non-numeric columns are skipped."""
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    header, body = rows[0], [r for r in rows[1:] if r]
    cols = {}
    for j, name in enumerate(header):
        try:
            cols[name] = np.array([float(r[j]) if r[j] != "" else np.nan for r in body])
        except ValueError:
            pass
    return cols


def list_trajectories(directory, only="all"):
    files = sorted(glob.glob(os.path.join(directory, "*.csv")))
    if only and only.lower() not in ("all", "*"):
        files = [f for f in files if only in os.path.basename(f)]
    return files


def take_name(path):
    return os.path.basename(path).replace("_clean_30hz.csv", "").replace(".csv", "")


def quat_to_matrix(q):
    """[x, y, z, w] -> 3x3 rotation matrix."""
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _pose(d, prefix, k):
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix([d[f"{prefix}_q{a}"][k] for a in "xyzw"])
    T[:3, 3] = [d[f"{prefix}_{a}_m"][k] for a in "xyz"]
    return T


class Trajectory:
    """One demo: times t [s], joints q [N,6] rad, flange path [N,3] (base_link), reachable [N]."""

    def __init__(self, path):
        d = read_csv(path)
        if "q1_deg" not in d:
            raise ValueError(f"{os.path.basename(path)}: no q1_deg..q6_deg columns "
                             "(run motive2gr00t.py with --joints-pos-only or --calib)")
        self.name = take_name(path)
        self.q = np.radians(np.stack([d[f"q{i}_deg"] for i in range(1, 7)], axis=1))
        n = len(self.q)
        self.t = d["time_episode_s"] - d["time_episode_s"][0] if "time_episode_s" in d else np.arange(n) / 30.0
        self.path = (np.stack([d["tcp_robot_x_m"], d["tcp_robot_y_m"], d["tcp_robot_z_m"]], axis=1)
                     if "tcp_robot_x_m" in d else np.zeros((n, 3)))
        self.reachable = d["ik_reachable"].astype(int) if "ik_reachable" in d else np.ones(n, int)
        # box pose in the robot frame: T_robot_box = T_robot_tool @ inv(T_box_tool)  (None if columns missing)
        self.box_pose = None
        if all(c in d for c in ("tcp_robot_qw", "tool_in_box_qw")):
            Ts = [_pose(d, "tcp_robot", k) @ np.linalg.inv(_pose(d, "tool_in_box", k)) for k in range(0, n, max(1, n // 20))]
            T = Ts[0].copy()
            T[:3, 3] = np.median([t[:3, 3] for t in Ts], axis=0)
            self.box_pose = T

    def max_joint_speed(self):
        dt = np.diff(self.t)
        return float((np.abs(np.diff(self.q, axis=0)) / dt[:, None]).max()) if len(dt) else 0.0

    def jumps(self, max_step_deg=10.0):
        """indices k where q[k] -> q[k+1] changes a joint by more than max_step_deg (IK posture switch)."""
        return np.where(np.abs(np.diff(self.q, axis=0)).max(axis=1) > math.radians(max_step_deg))[0]


def quintic(q0, q1, duration, dt):
    """Joint-space move with zero velocity and acceleration at both ends. Returns (t, q)."""
    n = max(2, int(math.ceil(duration / dt)) + 1)
    tau = np.linspace(0.0, 1.0, n)
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    return tau * duration, q0 + s[:, None] * (q1 - q0)


def transit_duration(q0, q1, speed=math.radians(30.0), minimum=1.5):
    # peak speed of a quintic is 1.875 x the average speed
    return max(minimum, 1.875 * float(np.abs(q1 - q0).max()) / speed)


def time_scale(traj, speed_fraction=0.8, vmax=VMAX):
    """Factor >= 1 that slows the trajectory so no joint exceeds speed_fraction * vmax."""
    v = traj.max_joint_speed()
    return max(1.0, v / (speed_fraction * vmax))


def check_limits(q, margin_deg=2.0):
    m = math.radians(margin_deg)
    bad = (q < LOWER + m) | (q > UPPER - m)
    return not bad.any(), np.argwhere(bad)


def robot_program(traj, q_start, speed_fraction=0.8, dt=1.0 / 30.0, transit_speed=math.radians(20.0)):
    """Full executable timeline for the real/mock robot:
    q_start --(slow quintic)--> traj.q[0] --(traj, slowed if needed)--> end.
    Posture switches inside the trajectory are replaced by slow quintic moves.
    Returns t [K], q [K,6], info dict."""
    s = time_scale(traj, speed_fraction)
    T0, Q0 = quintic(q_start, traj.q[0], transit_duration(q_start, traj.q[0], transit_speed), dt)
    ts, qs = [T0], [Q0]
    t_now = T0[-1]
    start = 0
    for k in list(traj.jumps()) + [len(traj.q) - 1]:
        # demo segment traj.q[start..k]; its first point equals the previous end -> skip it
        seg_t = (traj.t[start:k + 1] - traj.t[start]) * s
        if k > start:
            ts.append(t_now + seg_t[1:])
            qs.append(traj.q[start + 1:k + 1])
            t_now = ts[-1][-1]
        if k < len(traj.q) - 1:                             # bridge a posture switch slowly
            Tb, Qb = quintic(traj.q[k], traj.q[k + 1], transit_duration(traj.q[k], traj.q[k + 1], transit_speed), dt)
            ts.append(t_now + Tb[1:])
            qs.append(Qb[1:])
            t_now = ts[-1][-1]
        start = k + 1
    t = np.concatenate(ts)
    q = np.concatenate(qs)
    ok, where = check_limits(q)
    v = (np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None]).max()
    info = dict(time_scale=round(s, 2), duration_s=round(float(t[-1]), 2), points=len(t),
                max_joint_speed_deg_s=round(math.degrees(v), 1), within_limits=bool(ok),
                posture_switches=int(len(traj.jumps())))
    return t, q, info
