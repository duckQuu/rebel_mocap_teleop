from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

UNIT_SCALE = {"Meters": 1.0, "Centimeters": 0.01, "Millimeters": 0.001}

# motive
def resolve_bodies(bodies, tool=None, base=None):
    """Find tool / box ("base") / flap rigid bodies by name (case-insensitive) unless given.
    NOTE: the "base" body is the BOX (its plate sits under the flaps), not the robot."""
    names = list(bodies)
    low = {n.lower(): n for n in names}
    def pick(explicit, keys, what):
        if explicit:
            if explicit not in bodies:
                sys.exit(f"rigid body '{explicit}' not in CSV (found {names})")
            return explicit
        hits = [n for n in names if any(k in n.lower() for k in keys)]
        if len(hits) != 1:
            sys.exit(f"cannot auto-detect the {what} rigid body among {names}; pass --{what}-body")  # noqa
        return hits[0]
    t = pick(tool, ("arm", "hand", "tool", "gripper"), "tool")
    b = pick(base, ("base", "box"), "box")
    flaps = {}
    for n in names:
        l = n.lower()
        if "flap" in l:
            side = "left" if "left" in l else "right" if "right" in l else l
            flaps[side] = n
    return t, b, flaps

def load_motive(path: Path):
    rows = list(csv.reader(open(path, newline="")))
    hdr = rows[0]
    meta = dict(zip(hdr[0::2], hdr[1::2]))
    typ, name, kind, axis = rows[2], rows[3], rows[5], rows[6]
    body_rows = [r for r in rows[7:] if r and r[0] != ""]
    width = len(axis)
    data = np.full((len(body_rows), width), np.nan)
    for i, r in enumerate(body_rows):
        for j, x in enumerate(r[:width]):
            if x != "":
                data[i, j] = float(x)
    cols: dict[tuple, list[int]] = {}
    for j in range(2, width):
        cols.setdefault((typ[j], name[j], kind[j]), []).append(j)
    if meta.get("Rotation Type") != "Quaternion":
        sys.exit("Export with Rotation Type = Quaternion")
    if meta.get("Length Units") not in UNIT_SCALE:
        sys.exit(f"unknown Length Units {meta.get('Length Units')}")
    scale = UNIT_SCALE[meta["Length Units"]]

    bodies = {}
    for (t, n, k), idx in cols.items():
        if t == "Rigid Body" and k == "Rotation":
            q = data[:, idx]                                   # X Y Z W (scipy order)
            p = data[:, cols[("Rigid Body", n, "Position")]] * scale
            n_rb = len({nm for (tt, nm, kk) in cols if tt == "Rigid Body Marker" and nm.startswith(n + ":")})
            raw = sorted({nm for (tt, nm, kk) in cols if tt == "Marker" and nm.startswith(n + ":")})
            vis = np.zeros(len(data), int)
            for nm in raw:
                vis += ~np.isnan(data[:, cols[("Marker", nm, "Position")][0]])
            rbm_names = sorted({nm for (tt, nm, kk) in cols if tt == "Rigid Body Marker" and nm.startswith(n + ":")})
            rbm = np.stack([data[:, cols[("Rigid Body Marker", nm, "Position")]] * scale for nm in rbm_names], 1) \
                if rbm_names else None
            bodies[n] = dict(pos=p, quat=q, n_markers=len(raw), n_rb_markers=n_rb, visible=vis, rbm=rbm)
    fps = float(meta["Capture Frame Rate"])
    return meta, data[:, 1], fps, bodies


def validity(body, min_markers: int | None):
    need = min(3 if min_markers is None else min_markers, body["n_markers"])
    ok = ~np.isnan(body["pos"][:, 0])
    if body["n_markers"] > 0:          # no labeled raw columns -> rely on NaN + frozen check only
        ok &= body["visible"] >= need
    same = np.r_[False, np.all(np.diff(np.c_[body["pos"], body["quat"]], axis=0) == 0, axis=1)]
    return ok & ~same


def runs(mask):
    """[(start, end_exclusive)]"""
    d = np.diff(np.r_[0, mask.astype(int), 0])
    return list(zip(np.where(d == 1)[0], np.where(d == -1)[0]))


# gap fill + filter
def continuous_quat(q):
    q = q.copy()
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]
    return q


def fill_gaps(t, pos, quat, valid, max_gap_frames):
    """Interpolate interior gaps <= max_gap_frames. Returns pos, quat, filled_mask, long_gaps."""
    pos, quat = pos.copy(), quat.copy()
    filled = np.zeros(len(t), bool)
    long_gaps = []
    for a, b in runs(~valid):
        if a == 0 or b == len(t):
            continue                       
        if b - a > max_gap_frames:
            long_gaps.append((a, b))
            continue
        tt = t[[a - 1, b]]
        for k in range(3):
            pos[a:b, k] = np.interp(t[a:b], tt, pos[[a - 1, b], k])
        sl = Slerp(tt, R.from_quat(quat[[a - 1, b]]))
        quat[a:b] = sl(t[a:b]).as_quat()
        filled[a:b] = True
    return pos, quat, filled, long_gaps


def lowpass(x, fs, fc):
    if fc <= 0 or len(x) < 30:
        return x
    b, a = butter(2, fc / (fs / 2))
    return filtfilt(b, a, x, axis=0)


# frames
def T_from(pos, rot: R):
    T = np.eye(4)
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = pos
    return T


def detect_up(base, base_valid, tool_pos, up_arg):
    if up_arg != "auto":
        return np.eye(3)["xyz".index(up_arg)], f"{up_arg} (given)"
    if base["rbm"] is None or base["rbm"].shape[1] < 3:
        sys.exit("--up auto needs >= 3 base markers; pass --up x|y|z")
    ok = base_valid & ~np.isnan(base["rbm"]).any(axis=(1, 2))
    P = base["rbm"][np.argmax(ok) if ok.any() else 0]
    n = np.linalg.svd(P - P.mean(0))[2][2]
    if np.dot(np.nanmedian(tool_pos, 0) - P.mean(0), n) < 0:
        n = -n
    ax = int(np.argmax(np.abs(n)))
    return n / np.linalg.norm(n), f"auto: base-plate normal {np.round(n, 3).tolist()} (~{'+-'[int(n[ax] < 0)]}{'XYZ'[ax]})"


def robot_frame_in_world(up, robot_origin, robot_yaw_deg):
    z = up
    x = np.eye(3)[1 if int(np.argmax(np.abs(up))) == 0 else 0]   
    x = x - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    Rw = np.c_[x, y, z] @ R.from_euler("z", robot_yaw_deg, degrees=True).as_matrix()
    return T_from(np.asarray(robot_origin, float), R.from_matrix(Rw))


def rot6d_rows(Rm):
    return Rm[..., :2, :].reshape(*Rm.shape[:-2], 6)



PI = np.pi
# (origin xyz, origin rpy, axis, lower, upper)
REBEL_JOINTS = [  
    ((0, 0, 0.100), (0, 0, 0), (0, 0, -1), -PI * 179 / 180, PI * 179 / 180),
    ((0, 0, 0.149), (0, PI / 6, 0), (0, 1, 0), -PI * 11 / 18, PI * 11 / 18),
    ((0, 0, 0.2384), (0, PI / 6, 0), (0, 1, 0), -PI * 11 / 18, PI * 11 / 18),
    ((0, 0, 0.149 - 0.03), (0, 0, 0), (0, 0, 1), -PI * 179 / 180, PI * 179 / 180),
    ((0, 0, 0.14 + 0.03), (0, -PI / 24, 0), (0, 1, 0), -PI * 19 / 36 + PI / 24, PI * 19 / 36 + PI / 24),
    ((0, 0, 0.1208), (0, 0, 0), (0, 0, 1), -PI * 179 / 180, PI * 179 / 180),
]
# link_8 -> flange (= tool0)
REBEL_FLANGE = T_from((0, 0, 0.0012), R.from_euler("xyz", (0, -PI / 2, 0)))  
REBEL_VMAX = PI * 45 / 180                                                   
LO = np.array([j[3] for j in REBEL_JOINTS])
HI = np.array([j[4] for j in REBEL_JOINTS])
_JO = [T_from(j[0], R.from_euler("xyz", j[1])) for j in REBEL_JOINTS]


def _axis_rot(ax, a):
    x, y, z = ax
    c, s_, C = np.cos(a), np.sin(a), 1 - np.cos(a)
    T = np.eye(4)
    T[:3, :3] = [[c + x * x * C, x * y * C - z * s_, x * z * C + y * s_],
                 [y * x * C + z * s_, c + y * y * C, y * z * C - x * s_],
                 [z * x * C - y * s_, z * y * C + x * s_, c + z * z * C]]
    return T


_AX = [np.asarray(j[2], float) for j in REBEL_JOINTS]


def rebel_fk(q):
    T = np.eye(4)
    for Tj, ax, qi in zip(_JO, _AX, q):
        T = T @ Tj @ _axis_rot(ax, qi)
    return T @ REBEL_FLANGE


def _log_so3(Rm):
    c = np.clip((np.trace(Rm) - 1) / 2, -1, 1)
    th = np.arccos(c)
    v = np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])
    if th < 1e-6:
        return 0.5 * v
    if th > np.pi - 1e-4:                      
        return R.from_matrix(Rm).as_rotvec()
    return th / (2 * np.sin(th)) * v


def _err(T, Tg, w_rot):
    return np.r_[Tg[:3, 3] - T[:3, 3], w_rot * _log_so3(Tg[:3, :3] @ T[:3, :3].T)]


REBEL_HOME = np.radians([0.0, 20.0, 60.0, 0.0, 60.0, 0.0])   


def rebel_ik(Tg, q0, w_rot=0.3, iters=100, lam=0.02, q_rest=None, k_null=0.05):
    """Damped least-squares IK. With q_rest, spare freedom (e.g. the wrist in position-only
    mode) is pulled gently towards q_rest, which keeps the posture natural and continuous."""
    q = np.clip(q0.copy(), LO, HI)
    for _ in range(iters):
        T = rebel_fk(q)
        e = _err(T, Tg, w_rot)
        if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) <= 1e-3 * w_rot and q_rest is None:
            break
        J = np.empty((6, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = 1e-6
            J[:, i] = (e - _err(rebel_fk(q + dq), Tg, w_rot)) / 1e-6
        Jp = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(6))
        dqs = Jp @ e
        if q_rest is not None:                       
            dn = (np.eye(6) - Jp @ J) @ (k_null * (q_rest - q))
            if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(dn) < 1e-3:
                break
            dqs += dn
        q = np.clip(q + np.clip(dqs, -0.3, 0.3), LO, HI)
    T = rebel_fk(q)
    pe = float(np.linalg.norm(Tg[:3, 3] - T[:3, 3]))
    re = float(np.degrees(np.linalg.norm(_log_so3(Tg[:3, :3] @ T[:3, :3].T))))
    return q, pe, re


def ik_check(T_flange_seq, fps, pos_tol=0.005, rot_tol_deg=3.0, w_rot=0.3, q_home=REBEL_HOME):
    rng = np.random.default_rng(0)
    q_rest = q_home if w_rot == 0 else None           
    ok_fn = lambda pe, re: pe < pos_tol and (w_rot == 0 or re < rot_tol_deg)

    def best_of(T, seeds, ref, iters):
        cands = [rebel_ik(T, s, w_rot=w_rot, iters=iters, q_rest=q_rest) for s in seeds]
        good = [c for c in cands if ok_fn(c[1], c[2])]
        if good:
            return min(good, key=lambda c: np.abs(c[0] - ref).sum())
        return min(cands, key=lambda c: c[1] + 0.002 * c[2])

    first = best_of(T_flange_seq[0], [q_home] + [rng.uniform(LO, HI) for _ in range(30)], q_home, 200)
    if q_rest is not None:                                
        first = rebel_ik(T_flange_seq[0], first[0], w_rot=w_rot, iters=2000, q_rest=q_rest)
    qs, pes, res = [first[0]], [first[1]], [first[2]]
    for T in T_flange_seq[1:]:
        q, pe, re = rebel_ik(T, qs[-1], w_rot=w_rot, q_rest=q_rest)
        if not ok_fn(pe, re):                             
            n = 12 if ok_fn(pes[-1], res[-1]) else 2
            q, pe, re = best_of(T, [qs[-1]] + [rng.uniform(LO, HI) for _ in range(n)], qs[-1], 150)
        qs.append(q), pes.append(pe), res.append(re)
    qs, pes, res = np.array(qs), np.array(pes), np.array(res)
    ok = (pes < pos_tol) & ((res < rot_tol_deg) | (w_rot == 0))
    qd = np.abs(np.diff(qs, axis=0)) * fps
    too_fast = np.r_[False, (qd > REBEL_VMAX).any(axis=1)]
    return dict(q=qs, pos_err=pes, rot_err=res, reachable=ok, too_fast=too_fast,
                max_joint_speed_deg_s=np.degrees(qd.max(axis=0)).round(1).tolist(),
                max_step_deg=float(np.degrees(np.abs(np.diff(qs, axis=0)).max())) if len(qs) > 1 else 0.0)


# video
def retime_video(src: Path, dst: Path, t_query, size=None, fps=30):
    """Write an MP4 whose frame k is the source frame nearest to t_query[k] (seconds, video clock)."""
    import cv2

    cap = cv2.VideoCapture(str(src))
    if not cap.isOpened():
        sys.exit(f"cannot open video {src}")
    frames, stamps = [], []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        stamps.append(cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0)
        frames.append(f)
    cap.release()
    stamps = np.asarray(stamps)
    if len(frames) == 0:
        sys.exit(f"no frames in {src}")
    if t_query[0] < stamps[0] - 0.05 or t_query[-1] > stamps[-1] + 0.05:
        sys.exit(f"{src}: needs video time {t_query[0]:.2f}-{t_query[-1]:.2f}s, "
                 f"video covers {stamps[0]:.2f}-{stamps[-1]:.2f}s (check --video-offset)")
    idx = np.clip(np.searchsorted(stamps, t_query), 1, len(stamps) - 1)
    idx -= (t_query - stamps[idx - 1]) < (stamps[idx] - t_query)
    h, w = frames[0].shape[:2]
    if size:
        w, h = size
    dst.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}",
           "-r", str(fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-g", "30",
           "-crf", "20", str(dst)]
    pr = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for i in idx:
        f = frames[i]
        if size:
            f = cv2.resize(f, (w, h), interpolation=cv2.INTER_AREA)
        pr.stdin.write(f.tobytes())
    pr.stdin.close()
    if pr.wait() != 0:
        sys.exit("ffmpeg failed")
    return (h, w), float(np.abs(stamps[idx] - t_query).max())


# write data
STATE_NAMES = ["x", "y", "z", "r00", "r01", "r02", "r10", "r11", "r12", "gripper"]


def load_or_init_meta(out: Path, fps, names):
    meta = out / "meta"
    if (meta / "info.json").exists():
        info = json.load(open(meta / "info.json"))
        if info["fps"] != fps:
            sys.exit(f"dataset fps {info['fps']} != --fps {fps}")
        eps = [json.loads(l) for l in open(meta / "episodes.jsonl")]
        tasks = [json.loads(l) for l in open(meta / "tasks.jsonl")]
        if info["features"]["observation.state"]["names"] != names:
            sys.exit("state layout differs from the existing dataset (--no-gripper mismatch?)")
        return info, eps, tasks
    vec = lambda: {"dtype": "float32", "shape": [len(names)], "names": names}
    scal = lambda dt: {"dtype": dt, "shape": [1], "names": None}
    info = {
        "codebase_version": "v2.1", "robot_type": "igus_rebel_6dof_handheld_mocap",
        "total_episodes": 0, "total_frames": 0, "total_tasks": 0, "total_videos": 0,
        "total_chunks": 1, "chunks_size": 1000, "fps": fps, "splits": {"train": "0:0"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": {
            "observation.state": vec(), "action": vec(),
            "timestamp": scal("float32"), "frame_index": scal("int64"),
            "episode_index": scal("int64"), "index": scal("int64"), "task_index": scal("int64"),
            "annotation.human.task_description": scal("int64"),
            "next.reward": scal("float32"), "next.done": scal("bool"),
        },
    }
    return info, [], []


def write_meta(out: Path, info, eps, tasks, cams):
    meta = out / "meta"
    meta.mkdir(parents=True, exist_ok=True)
    info["total_episodes"] = len(eps)
    info["total_frames"] = int(sum(e["length"] for e in eps))
    info["total_tasks"] = len(tasks)
    info["total_videos"] = len(eps) * len(cams)
    info["total_chunks"] = max(1, (len(eps) - 1) // info["chunks_size"] + 1)
    info["splits"] = {"train": f"0:{len(eps)}"}
    json.dump(info, open(meta / "info.json", "w"), indent=4)
    with open(meta / "episodes.jsonl", "w") as f:
        for e in eps:
            f.write(json.dumps(e) + "\n")
    with open(meta / "tasks.jsonl", "w") as f:
        for t in tasks:
            f.write(json.dumps(t) + "\n")
    groups = {"eef_9d": {"start": 0, "end": 9}}
    if info["features"]["observation.state"]["shape"][0] == 10:
        groups["gripper"] = {"start": 9, "end": 10}
    modality = {
        "state": dict(groups),
        "action": dict(groups),
        "video": {c: {"original_key": f"observation.images.{c}"} for c in cams},
        "annotation": {"human.task_description": {"original_key": "task_index"}},
    }
    json.dump(modality, open(meta / "modality.json", "w"), indent=4)
    for stale in ("stats.json", "relative_stats.json"):        
        (meta / stale).unlink(missing_ok=True)


def write_parquet(df, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    df.to_parquet(path, index=False)


# QC plot
def qc_plot(path, t, valid, filled, win, flap_ang, tk, xyz, speed, ik):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    for a, b in runs(~valid):
        for x in ax:
            x.axvspan(t[a], t[min(b, len(t) - 1)], color="#d62728", alpha=0.12, lw=0)
    for x in ax:
        x.axvspan(win[0], win[1], color="#2ca02c", alpha=0.07, lw=0)
    ax[0].plot(t, valid.astype(int), lw=1, label="tool tracked")
    ax[0].plot(t, filled.astype(int) * 0.5, lw=1, label="gap-filled")
    ax[0].set_ylabel("tracking")
    ax[0].legend(loc="upper right", fontsize=8)
    for n, a in flap_ang.items():
        ax[1].plot(t, a, label=n)
    ax[1].set_ylabel("flap angle [deg]")
    ax[1].legend(loc="upper left", fontsize=8)
    for k, c in enumerate("xyz"):
        ax[2].plot(tk, xyz[:, k], label=f"TCP {c} (robot frame)")
    ax[2].set_ylabel("m")
    ax[2].legend(loc="upper left", fontsize=8)
    ax[3].plot(tk, speed, label="TCP speed [m/s]")
    if ik is not None:
        bad = ~ik["reachable"]
        ax[3].scatter(tk[bad], np.zeros(bad.sum()), s=4, c="#d62728", label="ReBeL IK fails")
        fast = ik["too_fast"]
        ax[3].scatter(tk[fast], np.full(fast.sum(), 0.02), s=4, c="#ff7f0e", label="joint > 45 deg/s")
    ax[3].set_ylabel("m/s")
    ax[3].set_xlabel("take time [s]  (red = tool not tracked, green = exported window)")
    ax[3].legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--task", default=None, help="default: from file name (left/right/both)")
    ap.add_argument("--open-flaps", default="auto", choices=["auto", "all", "left", "right"],
                    help="which flap(s) must open to end the episode; auto = from file name")
    ap.add_argument("--tool-body", default=None, help="default: auto (name contains arm/hand/tool)")
    ap.add_argument("--box-body", default=None, help="default: auto (name contains base/box)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--video", action="append", default=[], help="cam=path.mp4 (repeatable)")
    ap.add_argument("--video-offset", action="append", default=[],
                    help="cam=seconds: video_time = mocap_take_time + offset")
    ap.add_argument("--video-size", type=int, nargs=2, default=None, metavar=("W", "H"))
    ap.add_argument("--no-video", action="store_true", help="debug only: dataset will NOT train")
    # geometry
    ap.add_argument("--up", default="auto", choices=["auto", "x", "y", "z"],
                    help="Motive world up axis; auto = normal of the base marker plate")
    ap.add_argument("--robot-origin", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                    help="where the ReBeL base_link will be, in Motive WORLD coords [m]. "
                         "Keep it identical for every take. Without it the box pivot is used "
                         "(provisional frame, IK check skipped).")
    ap.add_argument("--calib", type=Path, default=None,
                    help="calib.json from calibrate_rebel_mocap.py: replaces --robot-origin/--robot-yaw/--tcp-*; "
                         "tcp_robot_* becomes the ReBeL flange pose and q1..q6 are written")
    ap.add_argument("--robot-yaw", type=float, default=0.0,
                    help="deg about up; robot x = world axis most perpendicular to up, rotated by this")
    ap.add_argument("--tcp-offset", type=float, nargs=3, default=[0, 0, 0],
                    help="TCP position in the tool rigid-body frame [m]")
    ap.add_argument("--tcp-rpy", type=float, nargs=3, default=None,
                    help="TCP axes in tool rigid-body frame, xyz euler [deg]; TCP axes must match ReBeL tool0")
    ap.add_argument("--flange-to-tcp", type=float, nargs=3, default=[0.0, 0, 0],
                    help="TCP position in ReBeL tool0/flange frame [m] (tool0 x points out of the flange)")
    # cleaning / windowing
    ap.add_argument("--min-markers", type=int, default=None, help="markers that must be visible (default 3)")
    ap.add_argument("--max-gap", type=float, default=0.10, help="s, longest gap to interpolate")
    ap.add_argument("--lpf", type=float, default=8.0, help="Hz low-pass (0 = off)")
    ap.add_argument("--open-deg", type=float, default=60.0, help="flap counts as open above this")
    ap.add_argument("--tail", type=float, default=0.5, help="s kept after both flaps open")
    ap.add_argument("--start", type=float, default=None)
    ap.add_argument("--start-near-box", type=float, nargs=2, default=None, metavar=("H", "DZ"),
                    help="start the episode when the tool is within H m (horizontal) of the box centre "
                         "and less than DZ m above it - skips bringing the tool in (e.g. 0.30 0.35)")
    ap.add_argument("--start-preroll", type=float, default=0.5, help="s kept before --start-near-box")
    ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--on-long-gap", choices=["fail", "longest", "interpolate"], default="fail")
    ap.add_argument("--gripper", type=float, default=0.0, help="constant gripper value (placeholder)")
    ap.add_argument("--no-gripper", action="store_true",
                    help="tool has no moving jaw: state/action = eef_9d only (use the matching config)")
    ap.add_argument("--gripper-csv", type=Path, default=None, help="CSV with columns time,gripper (take time)")
    ap.add_argument("--skip-ik", action="store_true")
    ap.add_argument("--joints-pos-only", action="store_true",
                    help="also write q1..q6 from POSITION-ONLY IK (tool orientation ignored) - for visualization")
    ap.add_argument("--no-csv", action="store_true", help="don't write <out>/cleaned_csv/<take>_clean_<fps>hz.csv")
    ap.add_argument("--csv-only", action="store_true",
                    help="only write the cleaned CSV (+QC), no GR00T dataset - no video needed")
    args = ap.parse_args()

    meta, t, fs, bodies = load_motive(args.csv)
    take = meta.get("Take Name", args.csv.stem)
    tool_name, base_name, flap_names = resolve_bodies(bodies, args.tool_body, args.box_body)
    tool, base = bodies[tool_name], bodies[base_name]
    stem = args.csv.stem.lower()
    need_flaps = args.open_flaps
    if need_flaps == "auto":
        need_flaps = "left" if "left" in stem else "right" if "right" in stem else "all"
    required = list(flap_names) if need_flaps == "all" else [need_flaps]
    if args.task is None:
        args.task = {"left": "open the left flap of the box", "right": "open the right flap of the box"}.get(
            need_flaps, "open both flaps of the box")
    report = {"take": take, "file": args.csv.name, "task": args.task, "capture_fps": fs,
              "frames": len(t), "duration_s": float(t[-1]),
              "bodies": {"tool": tool_name, "box": base_name, "flaps": flap_names},
              "tool_markers": {"labeled_raw": tool["n_markers"], "rigid_body": tool["n_rb_markers"]}}

    # validity & gaps
    v_tool = validity(tool, args.min_markers)
    report["tool_tracked_pct"] = round(100 * v_tool.mean(), 1)
    report["tool_lost_segments_s"] = [(round(t[a], 2), round(t[min(b, len(t) - 1)], 2)) for a, b in runs(~v_tool)]
    pos, quat, filled, long_gaps = fill_gaps(t, tool["pos"], continuous_quat(tool["quat"]), v_tool,
                                            int(round(args.max_gap * fs)))
    usable = v_tool | filled

    # flaps
    flap_ang, open_t = {}, {}
    for f, bname in flap_names.items():
        vf = validity(bodies[bname], args.min_markers)
        q = bodies[bname]["quat"].copy()
        q[np.isnan(q).any(axis=1)] = [0, 0, 0, 1]
        i0 = np.argmax(vf)
        ang = np.degrees((R.from_quat(q[i0]).inv() * R.from_quat(q)).magnitude())
        ang[~vf] = np.nan
        flap_ang[f] = ang
        hit = np.where(ang > args.open_deg)[0]
        open_t[f] = float(t[hit[0]]) if len(hit) else None
    report["flap_open_time_s"] = open_t

    # window
    first = int(np.argmax(usable))
    start = args.start if args.start is not None else float(t[first])
    if args.start_near_box is not None:
        h_max, dz_max = args.start_near_box
        vb = validity(base, args.min_markers)
        b_c = np.median(base["pos"][vb] if vb.any() else base["pos"], axis=0)
        rel = pos - b_c
        up_v, _ = detect_up(base, vb, pos[usable], args.up)
        dz = rel @ up_v
        horiz = np.linalg.norm(rel - np.outer(dz, up_v), axis=1)
        near = usable & (horiz < h_max) & (dz < dz_max) & (t >= start)
        if near.any():
            start = max(start, float(t[np.argmax(near)]) - args.start_preroll)
            report["start_near_box_s"] = round(start, 2)
        else:
            report["warning_start"] = "tool never came near the box; --start-near-box ignored"
    if args.end is not None:
        end = args.end
    elif required and all(open_t.get(f) is not None for f in required):
        end = max(open_t[f] for f in required) + args.tail
    else:
        last = len(usable) - 1 - int(np.argmax(usable[::-1]))
        end = float(t[last])
        report["warning_end"] = f"required flap(s) {required} not seen open; window ends at last tracked tool frame"
    wrong = [f for f in flap_names if f not in required and open_t.get(f) is not None and open_t[f] <= end]
    if wrong:
        report["warning_task"] = f"flap(s) {wrong} also opened - label may be wrong"
    i0, i1 = int(np.searchsorted(t, start)), int(np.searchsorted(t, end, side="right"))
    inner = [(a, b) for a, b in runs(~usable[i0:i1])]
    if inner:
        segs = [(round(t[i0 + a], 2), round(t[min(i0 + b, len(t) - 1)], 2)) for a, b in inner]
        report["long_gaps_in_window_s"] = segs
        if args.on_long_gap == "fail":
            report["status"] = "REJECTED: tool lost for > --max-gap inside the task window"
            print(json.dumps(report, indent=2))
            sys.exit(2)
        elif args.on_long_gap == "longest":
            a, b = max(runs(usable[i0:i1]), key=lambda r: r[1] - r[0])
            i0, i1 = i0 + a, i0 + b
            report["warning_window"] = "cut to longest tracked segment - task may be incomplete"
        else:  # interpolate everything
            v2 = usable.copy()
            pos, quat, f2, _ = fill_gaps(t, pos, quat, v2, 10**9)
            filled |= f2
            report["warning_window"] = "long gaps interpolated - motion inside them is invented"
    win = (float(t[i0]), float(t[i1 - 1]))
    report["window_s"] = [round(win[0], 3), round(win[1], 3)]

    # filter + resample
    tw, pw, qw = t[i0:i1], pos[i0:i1], continuous_quat(quat[i0:i1])
    pw = lowpass(pw, fs, args.lpf)
    qw = lowpass(qw, fs, args.lpf)
    qw /= np.linalg.norm(qw, axis=1, keepdims=True)
    tk = np.arange(tw[0], tw[-1] + 1e-9, 1.0 / args.fps)
    pk = np.stack([np.interp(tk, tw, pw[:, k]) for k in range(3)], 1)
    rk = Slerp(tw, R.from_quat(qw))(tk)

    # frames: world -> robot base, tool body -> TCP
    v_base = validity(base, args.min_markers)
    up, up_note = detect_up(base, v_base, pos[usable], args.up)
    report["world_up"] = up_note
    if args.calib is not None:
        cal = json.load(open(args.calib))
        T_w_r_cal = np.asarray(cal["T_world_base"])
        report["frame"] = f"from calibration {args.calib.name} (mean err {cal['error_mm']['mean']} mm)"
        origin = T_w_r_cal[:3, 3]
    elif args.robot_origin is None:
        sel = v_base if v_base.any() else np.ones(len(v_base), bool)
        origin = np.median(base["pos"][sel], axis=0)
        report["warning_frame"] = "no --robot-origin: state expressed in a PROVISIONAL frame at the box; IK skipped"
        args.skip_ik = True
    else:
        origin = args.robot_origin
    report["box_pose_world"] = {"pos": np.round(np.nanmedian(base["pos"][v_base] if v_base.any() else base["pos"], 0), 4).tolist(),
                                "tracked_pct": round(100 * v_base.mean(), 1)}
    T_w_r = robot_frame_in_world(up, origin, args.robot_yaw) if args.calib is None else T_w_r_cal
    T_r_w = np.linalg.inv(T_w_r)
    if args.calib is not None:
        T_tool_tcp = np.linalg.inv(np.asarray(cal["T_flange_markers"]))
        args.flange_to_tcp = [0.0, 0.0, 0.0]
    else:
        T_tool_tcp = T_from(args.tcp_offset, R.from_euler("xyz", args.tcp_rpy or [0, 0, 0], degrees=True))
    full_ik = args.calib is not None or args.tcp_rpy is not None
    T_w_tool = np.stack([T_from(p, r) for p, r in zip(pk, rk)])
    T_r_tcp = T_r_w @ T_w_tool @ T_tool_tcp
    xyz = T_r_tcp[:, :3, 3]
    eef = np.c_[xyz, rot6d_rows(T_r_tcp[:, :3, :3])]
    speed = np.r_[0, np.linalg.norm(np.diff(xyz, axis=0), axis=1) * args.fps]
    report["tcp_speed_m_s"] = {"p50": round(float(np.median(speed)), 3), "p99": round(float(np.percentile(speed, 99)), 3)}
    report["tcp_range_robot_frame_m"] = {"min": xyz.min(0).round(3).tolist(), "max": xyz.max(0).round(3).tolist()}

    # gripper
    if args.no_gripper:
        grip = None
    elif args.gripper_csv:
        g = np.loadtxt(args.gripper_csv, delimiter=",", skiprows=1)
        grip = np.interp(tk, g[:, 0], g[:, 1])
    else:
        grip = np.full(len(tk), args.gripper)
        report["warning_gripper"] = f"no gripper signal - constant {args.gripper} written (placeholder)"

    # IK
    ik = None
    if not args.skip_ik:
        T_tcp_flange = np.linalg.inv(T_from(args.flange_to_tcp, R.identity()))
        ik_pos = ik_check(T_r_tcp @ T_tcp_flange, args.fps, w_rot=0.0)   # position only
        ik = ik_check(T_r_tcp @ T_tcp_flange, args.fps) if full_ik else None  # full 6-DoF
        report["rebel_ik"] = {
            "position_only_reachable_pct": round(100 * ik_pos["reachable"].mean(), 1),
            "worst_pos_err_mm": round(1000 * float(ik_pos["pos_err"].max()), 1),
            "max_joint_step_deg": round(ik_pos["max_step_deg"], 1),
        }
        if ik is not None:
            report["rebel_ik"].update({
                "full_pose_reachable_pct": round(100 * ik["reachable"].mean(), 1),
                "joint_speed_violation_pct": round(100 * ik["too_fast"].mean(), 1),
                "max_joint_speed_deg_s": ik["max_joint_speed_deg_s"], "limit_deg_s": 45})
        else:
            report["rebel_ik"]["full_pose"] = "skipped: pass --tcp-rpy once the TCP axes are measured"
            ik = ik_pos

    # -- cleaned per-take CSV (human-readable copy of everything computed above)
    if not args.no_csv:
        import pandas as pd

        near = np.clip(np.searchsorted(t, tk), 0, len(t) - 1)          # raw frame nearest to each tk
        col = {"time_take_s": tk, "time_episode_s": tk - tk[0],
               "tool_tracked": v_tool[near].astype(int)}               # 0 = gap-filled
        qw_k = rk.as_quat()
        for i, a in enumerate("xyz"):
            col[f"tool_world_{a}_m"] = pk[:, i]
        for i, a in enumerate(["qx", "qy", "qz", "qw"]):
            col[f"tool_world_{a}"] = qw_k[:, i]
        q_r = R.from_matrix(T_r_tcp[:, :3, :3]).as_quat()
        for i, a in enumerate("xyz"):
            col[f"tcp_robot_{a}_m"] = xyz[:, i]
        for i, a in enumerate(["qx", "qy", "qz", "qw"]):
            col[f"tcp_robot_{a}"] = q_r[:, i]
        for i, n in enumerate(STATE_NAMES[3:9]):
            col[f"tcp_robot_rot6d_{n}"] = eef[:, 3 + i]
        col["tcp_speed_m_s"] = speed
        bsel = v_base if v_base.any() else np.ones(len(v_base), bool)
        b_p = np.median(base["pos"][bsel], axis=0)
        b_r = R.from_quat(continuous_quat(base["quat"][bsel])).mean()
        T_w_box = T_from(b_p, b_r)
        T_box_tool = np.linalg.inv(T_w_box) @ T_w_tool                 # motion relative to the box (step 7)
        q_b = R.from_matrix(T_box_tool[:, :3, :3]).as_quat()
        for i, a in enumerate("xyz"):
            col[f"box_world_{a}_m"] = np.full(len(tk), b_p[i])
        for i, a in enumerate("xyz"):
            col[f"tool_in_box_{a}_m"] = T_box_tool[:, i, 3]
        for i, a in enumerate(["qx", "qy", "qz", "qw"]):
            col[f"tool_in_box_{a}"] = q_b[:, i]
        for f, a in flap_ang.items():
            ok = ~np.isnan(a)
            col[f"flap_{f}_deg"] = np.interp(tk, t[ok], a[ok]) if ok.any() else np.nan
        if grip is not None:
            col["gripper"] = grip
        if ik is not None:
            col["ik_reachable"] = ik["reachable"].astype(int)
            col["ik_pos_err_mm"] = ik["pos_err"] * 1000
            if full_ik or args.joints_pos_only:                         # pos-only: wrist orientation is free (viz only)
                for j in range(6):
                    col[f"q{j + 1}_deg"] = np.degrees(ik["q"][:, j])
        csv_dir = args.out / "cleaned_csv"
        csv_dir.mkdir(parents=True, exist_ok=True)
        pd.DataFrame(col).to_csv(csv_dir / f"{args.csv.stem}_clean_{args.fps}hz.csv", index=False, float_format="%.6f")
        if args.csv_only:
            qc_dir = args.out / "qc"
            qc_dir.mkdir(parents=True, exist_ok=True)
            qc_plot(qc_dir / f"{args.csv.stem}.png", t, v_tool, filled, win, flap_ang, tk, xyz, speed, ik)
            report.update(status="OK (csv only)", frames_written=len(tk))
            json.dump(report, open(qc_dir / f"{args.csv.stem}.json", "w"), indent=2)
            print(json.dumps(report, indent=2))
            return

    # -- dataset
    state = (eef if grip is None else np.c_[eef, grip]).astype(np.float32)
    action = np.r_[state[1:], state[-1:]].astype(np.float32)
    info, eps, tasks = load_or_init_meta(args.out, args.fps, STATE_NAMES[:9] if args.no_gripper else STATE_NAMES)
    ep = len(eps)
    task_idx = next((x["task_index"] for x in tasks if x["task"] == args.task), None)
    if task_idx is None:
        task_idx = len(tasks)
        tasks.append({"task_index": task_idx, "task": args.task})
    chunk = ep // info["chunks_size"]
    n = len(tk)

    cams = {}
    if not args.no_video:
        if not args.video:
            sys.exit("give --video wrist=<file> (or --no-video for a debug export that cannot train)")
        offs = dict(o.split("=", 1) for o in args.video_offset)
        for spec in args.video:
            cam, path = spec.split("=", 1)
            dst = args.out / f"videos/chunk-{chunk:03d}/observation.images.{cam}/episode_{ep:06d}.mp4"
            (h, w), worst = retime_video(Path(path), dst, tk + float(offs.get(cam, 0.0)), args.video_size, args.fps)
            cams[cam] = (h, w)
            report[f"video_{cam}_max_sync_err_ms"] = round(1000 * worst, 1)
            info["features"][f"observation.images.{cam}"] = {
                "dtype": "video", "shape": [h, w, 3], "names": ["height", "width", "channels"],
                "info": {"video.height": h, "video.width": w, "video.codec": "h264", "video.pix_fmt": "yuv420p",
                         "video.is_depth_map": False, "video.fps": args.fps, "video.channels": 3, "has_audio": False},
            }
    else:
        report["warning_video"] = "no video - GR00T training needs at least one camera"
    cams_all = sorted({k.split(".")[-1] for k in info["features"] if k.startswith("observation.images.")})

    import pandas as pd
    df = pd.DataFrame({
        "observation.state": list(state),
        "action": list(action),
        "timestamp": (np.arange(n) / args.fps).astype(np.float32),
        "frame_index": np.arange(n, dtype=np.int64),
        "episode_index": np.full(n, ep, np.int64),
        "index": np.arange(n, dtype=np.int64) + info["total_frames"],
        "task_index": np.full(n, task_idx, np.int64),
        "annotation.human.task_description": np.full(n, task_idx, np.int64),
        "next.reward": np.r_[np.zeros(n - 1), 1.0].astype(np.float32),
        "next.done": np.r_[np.zeros(n - 1, bool), True],
    })
    write_parquet(df, args.out / f"data/chunk-{chunk:03d}/episode_{ep:06d}.parquet")
    eps.append({"episode_index": ep, "tasks": [args.task], "length": n, "source_take": take,
                "source_window_s": report["window_s"]})
    write_meta(args.out, info, eps, tasks, cams_all)

    qc_dir = args.out / "qc"
    qc_dir.mkdir(exist_ok=True)
    qc_plot(qc_dir / f"episode_{ep:06d}_{args.csv.stem}.png", t, v_tool, filled, win, flap_ang, tk, xyz, speed, ik)
    report.update(status="OK", episode_index=ep, frames_written=n)
    json.dump(report, open(qc_dir / f"episode_{ep:06d}_{args.csv.stem}.json", "w"), indent=2)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
