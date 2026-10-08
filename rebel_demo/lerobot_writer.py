"""Writes episodes in the LeRobot v2 layout that GR00T N1.x fine-tuning loads (no ROS in here).

<root>/
  meta/info.json  episodes.jsonl  tasks.jsonl  modality.json
  data/chunk-000/episode_000000.parquet
  videos/chunk-000/observation.images.<cam>/episode_000000.mp4

Parquet columns per frame: observation.state, action (float32 lists), timestamp, frame_index,
episode_index, index, task_index, annotation.human.action.task_description (= task_index).
modality.json splits the state / action vectors into named parts (e.g. single_arm 0:6, gripper 6:7).
Needs: pyarrow, and the ffmpeg binary (H.264 mp4).
"""
import json
import os
import subprocess

import numpy as np

CHUNK = 1000


class VideoStream:
    """Pipes RGB frames into ffmpeg -> H.264 mp4. Starts on the first frame (size known then)."""

    def __init__(self, path, fps, size=None):
        self.path, self.fps, self.size = path, fps, size     # size = (w, h) output, None = as received
        self.proc = None
        self.out_shape = None

    def write(self, rgb):
        h, w = rgb.shape[:2]
        if self.proc is None:
            ow, oh = self.size or (w, h)
            self.out_shape = (oh, ow, 3)
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            self.proc = subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                 "-s", f"{w}x{h}", "-r", str(self.fps), "-i", "-", "-vf", f"scale={ow}:{oh}",
                 "-c:v", "libx264", "-pix_fmt", "yuv420p", "-g", "2", "-crf", "20", self.path],
                stdin=subprocess.PIPE)
            self.in_shape = (h, w)
        if (h, w) != self.in_shape:
            raise ValueError(f"camera resolution changed mid-episode: {self.in_shape} -> {(h, w)}")
        self.proc.stdin.write(np.ascontiguousarray(rgb, dtype=np.uint8).tobytes())

    def close(self):
        if self.proc is not None:
            self.proc.stdin.close()
            self.proc.wait()
            self.proc = None


class LeRobotWriter:
    def __init__(self, root, fps, cameras, state_parts, action_parts, robot_type="igus_rebel_xeg32",
                 image_size=None):
        """cameras: [name]; state_parts / action_parts: [(name, [element names])] in vector order."""
        import pyarrow  # noqa: F401  (fail early with a clear error)
        self.root, self.fps, self.cameras = root, fps, list(cameras)
        self.state_parts, self.action_parts = state_parts, action_parts
        self.robot_type, self.image_size = robot_type, image_size
        self.state_names = [n for _, names in state_parts for n in names]
        self.action_names = [n for _, names in action_parts for n in names]
        os.makedirs(os.path.join(root, "meta"), exist_ok=True)
        self.episodes = self._read_jsonl("episodes.jsonl")
        self.tasks = {t["task"]: t["task_index"] for t in self._read_jsonl("tasks.jsonl")}
        self.total_frames = sum(e["length"] for e in self.episodes)
        self.video_shapes = {}
        info = os.path.join(root, "meta", "info.json")
        if os.path.exists(info):
            with open(info) as f:
                old = json.load(f)["features"]
            for c in self.cameras:
                if f"observation.images.{c}" in old:
                    self.video_shapes[c] = old[f"observation.images.{c}"]["shape"]
        self.ep = None

    # ------------------------------------------------------------------ episode
    def start(self, task):
        idx = len(self.episodes)
        self.ep = {"index": idx, "task": task, "state": [], "action": [], "streams": {
            c: VideoStream(self._video_path(c, idx), self.fps, self.image_size) for c in self.cameras}}
        return idx

    def add_frame(self, state, action, images):
        """state / action: 1-D arrays; images: {camera: HxWx3 uint8 RGB}."""
        for c in self.cameras:
            self.ep["streams"][c].write(images[c])
        self.ep["state"].append(np.asarray(state, np.float32))
        self.ep["action"].append(np.asarray(action, np.float32))

    def __len__(self):
        return 0 if self.ep is None else len(self.ep["state"])

    def discard(self):
        if self.ep is None:
            return
        for c, s in self.ep["streams"].items():
            s.close()
            if os.path.exists(s.path):
                os.remove(s.path)
        self.ep = None

    def stop(self):
        """Close the episode and write parquet + meta. Returns (episode_index, n_frames) or None if empty."""
        import pyarrow as pa
        import pyarrow.parquet as pq
        ep, n = self.ep, len(self)
        if n == 0:
            self.discard()
            return None
        for c, s in ep["streams"].items():
            s.close()
            self.video_shapes[c] = list(s.out_shape)
        if ep["task"] not in self.tasks:
            self.tasks[ep["task"]] = len(self.tasks)
        ti, idx = self.tasks[ep["task"]], ep["index"]
        f32 = pa.list_(pa.float32())
        table = pa.table({
            "observation.state": pa.array([s.tolist() for s in ep["state"]], f32),
            "action": pa.array([a.tolist() for a in ep["action"]], f32),
            "timestamp": pa.array(np.arange(n, dtype=np.float32) / self.fps, pa.float32()),
            "frame_index": pa.array(np.arange(n), pa.int64()),
            "episode_index": pa.array(np.full(n, idx), pa.int64()),
            "index": pa.array(np.arange(n) + self.total_frames, pa.int64()),
            "task_index": pa.array(np.full(n, ti), pa.int64()),
            "annotation.human.action.task_description": pa.array(np.full(n, ti), pa.int64()),
            "next.done": pa.array([False] * (n - 1) + [True], pa.bool_()),
        })
        path = os.path.join(self.root, f"data/chunk-{idx // CHUNK:03d}/episode_{idx:06d}.parquet")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        pq.write_table(table, path)
        self.episodes.append({"episode_index": idx, "tasks": [ep["task"]], "length": n})
        self.total_frames += n
        self.ep = None
        self._write_meta()
        return idx, n

    # ------------------------------------------------------------------ meta
    def _video_path(self, cam, idx):
        return os.path.join(self.root, f"videos/chunk-{idx // CHUNK:03d}/observation.images.{cam}/"
                                       f"episode_{idx:06d}.mp4")

    def _read_jsonl(self, name):
        p = os.path.join(self.root, "meta", name)
        if not os.path.exists(p):
            return []
        with open(p) as f:
            return [json.loads(line) for line in f if line.strip()]

    def _write_jsonl(self, name, rows):
        with open(os.path.join(self.root, "meta", name), "w") as f:
            for r in rows:
                f.write(json.dumps(r) + "\n")

    def _write_meta(self):
        n_ep = len(self.episodes)
        self._write_jsonl("episodes.jsonl", self.episodes)
        self._write_jsonl("tasks.jsonl", [{"task_index": i, "task": t}
                                          for t, i in sorted(self.tasks.items(), key=lambda kv: kv[1])])
        scalar = lambda dt: {"dtype": dt, "shape": [1], "names": None}  # noqa: E731
        features = {
            f"observation.images.{c}": {
                "dtype": "video", "shape": self.video_shapes[c], "names": ["height", "width", "channel"],
                "video_info": {"video.fps": float(self.fps), "video.codec": "h264",
                               "video.pix_fmt": "yuv420p", "video.is_depth_map": False, "has_audio": False}}
            for c in self.cameras if c in self.video_shapes}
        features.update({
            "observation.state": {"dtype": "float32", "shape": [len(self.state_names)], "names": self.state_names},
            "action": {"dtype": "float32", "shape": [len(self.action_names)], "names": self.action_names},
            "timestamp": scalar("float32"), "frame_index": scalar("int64"), "episode_index": scalar("int64"),
            "index": scalar("int64"), "task_index": scalar("int64"),
            "annotation.human.action.task_description": scalar("int64"), "next.done": scalar("bool"),
        })
        info = {
            "codebase_version": "v2.0", "robot_type": self.robot_type,
            "total_episodes": n_ep, "total_frames": self.total_frames, "total_tasks": len(self.tasks),
            "total_videos": n_ep * len(self.cameras), "total_chunks": (n_ep - 1) // CHUNK + 1,
            "chunks_size": CHUNK, "fps": float(self.fps), "splits": {"train": f"0:{n_ep}"},
            "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
            "features": features,
        }
        with open(os.path.join(self.root, "meta", "info.json"), "w") as f:
            json.dump(info, f, indent=2)

        def parts(ps):
            out, i = {}, 0
            for name, names in ps:
                out[name] = {"start": i, "end": i + len(names)}
                i += len(names)
            return out
        modality = {
            "state": parts(self.state_parts),
            "action": parts(self.action_parts),
            "video": {c: {"original_key": f"observation.images.{c}"} for c in self.cameras},
            "annotation": {"human.action.task_description": {}},
        }
        with open(os.path.join(self.root, "meta", "modality.json"), "w") as f:
            json.dump(modality, f, indent=2)
