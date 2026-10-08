"""recorder node - records teleop demonstrations as a GR00T / LeRobot v2 dataset.

Every 1/fps s while recording, one frame = the latest of each input:
  observation.state  [joint1..joint6 (rad, measured /joint_states), gripper (0 closed .. 1 open, measured)]
  action             [joint1..joint6 (rad, commanded on command_topic), gripper (0/1 commanded state)]
  observation.images.<cam>   from each sensor_msgs/Image topic in `cameras`
A frame is skipped (and counted) when an input is missing or older than max_age.

Services (std_srvs/Trigger):
  ~/start     begin an episode (task text = parameter `task`)
  ~/stop      end it and save          ~/discard   end it and throw it away
  ros2 param set /rebel_recorder task "pick up the red cube and place it in the box"

Parameters (defaults in brackets)
  dataset_dir [~/rebel_datasets/rebel_xeg32]   appended to if it already exists
  fps [30]   cameras ['front:/rgb']  ("name:/topic,name2:/topic2")   image_size [[0, 0]] (w, h; 0 = as received)
  joint_states_topic [/joint_states]   command_topic [/isaac_joint_commands]
  gripper_state_topic [/rebel_gripper/state]   max_age [0.5]
  gripper_joint [xeg32_left_carriage_joint]  closed_pos / open_pos   (measured gripper -> 0..1)
  joint_prefix ['']   arm joints = prefix + joint1..joint6 (arm_left_ / arm_right_ for the dual-arm rig);
                      the dataset always names them joint1..joint6
  joint_names ['']    6 comma-separated names (base to wrist), overrides joint_prefix
  task ['']
Needs pyarrow (pip install pyarrow) and ffmpeg (sudo apt install ffmpeg).
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from .gripper_node import CLOSED_POS, OPEN_POS
from .kinematics import JOINT_NAMES
from .lerobot_writer import LeRobotWriter

CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


def image_to_rgb(msg):
    """sensor_msgs/Image -> HxWx3 uint8 RGB (8-bit encodings only)."""
    ch = CHANNELS.get(msg.encoding)
    if ch is None:
        raise ValueError(f"unsupported image encoding '{msg.encoding}' (use rgb8/bgr8/rgba8/bgra8/mono8)")
    a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    a = a.reshape(msg.height, msg.width, ch)
    if ch == 1:
        return np.repeat(a, 3, axis=2)
    if msg.encoding.startswith("bgr"):
        a = a[:, :, 2::-1]
    return a[:, :, :3]


class Recorder(Node):
    def __init__(self):
        super().__init__("rebel_recorder")
        d = self.declare_parameter
        root = os.path.expanduser(d("dataset_dir", "~/rebel_datasets/rebel_xeg32").value)
        self.fps = float(d("fps", 30.0).value)
        cams = [c.split(":", 1) for c in d("cameras", "front:/rgb").value.split(",") if c.strip()]
        w, h = (int(v) for v in d("image_size", [0, 0]).value)
        self.grip_joint = d("gripper_joint", "xeg32_left_carriage_joint").value
        self.closed = float(d("closed_pos", CLOSED_POS).value)
        self.open = float(d("open_pos", OPEN_POS).value)
        self.max_age = float(d("max_age", 0.5).value)
        prefix = d("joint_prefix", "").value
        custom = [n.strip() for n in d("joint_names", "").value.split(",") if n.strip()]
        if custom and len(custom) != 6:
            raise ValueError(f"joint_names needs 6 comma-separated names, got {len(custom)}: {custom}")
        self.joint_names = custom or [prefix + n for n in JOINT_NAMES]
        d("task", "")

        self.cameras = [n.strip() for n, _ in cams]
        self.writer = LeRobotWriter(
            root, self.fps, self.cameras,
            state_parts=[("single_arm", JOINT_NAMES), ("gripper", ["gripper"])],
            action_parts=[("single_arm", JOINT_NAMES), ("gripper", ["gripper"])],
            image_size=(w, h) if w > 0 and h > 0 else None)

        self.q_meas = self.q_cmd = None
        self.grip_meas = self.grip_cmd = None
        self.images = {}
        self.stamp = {}                                     # input -> receive time [s]
        for name, topic in cams:
            self.create_subscription(Image, topic.strip(), lambda m, n=name.strip(): self.on_image(n, m), 2)
        self.create_subscription(JointState, d("joint_states_topic", "/joint_states").value, self.on_js, 10)
        self.create_subscription(JointState, d("command_topic", "/isaac_joint_commands").value, self.on_cmd, 10)
        self.create_subscription(Int32, d("gripper_state_topic", "/rebel_gripper/state").value, self.on_grip, 10)
        self.create_service(Trigger, "~/start", self.on_start)
        self.create_service(Trigger, "~/stop", self.on_stop)
        self.create_service(Trigger, "~/discard", self.on_discard)

        self.recording = False
        self.skipped = 0
        self.create_timer(1.0 / self.fps, self.step)
        self.get_logger().info(f"recorder: {root}, {self.fps:.0f} fps, cameras {dict(cams)}, "
                               f"{len(self.writer.episodes)} episodes already there. Call ~/start")

    # ------------------------------------------------------------------ inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_image(self, name, msg):
        self.images[name] = msg                             # converted only when a frame is taken
        self.stamp[name] = self.now()

    def on_js(self, msg):
        if all(n in msg.name for n in self.joint_names):
            self.q_meas = [msg.position[msg.name.index(n)] for n in self.joint_names]
            self.stamp["state"] = self.now()
        if self.grip_joint in msg.name:
            pos = msg.position[msg.name.index(self.grip_joint)]
            self.grip_meas = (pos - self.closed) / (self.open - self.closed)

    def on_cmd(self, msg):                                  # arm and gripper commands arrive as separate messages
        if all(n in msg.name for n in self.joint_names):
            self.q_cmd = [msg.position[msg.name.index(n)] for n in self.joint_names]
            self.stamp["action"] = self.now()

    def on_grip(self, msg):
        self.grip_cmd = float(msg.data)

    # ------------------------------------------------------------------ services
    def on_start(self, req, res):
        if self.recording:
            res.success, res.message = False, "already recording"
            return res
        task = self.get_parameter("task").value.strip()
        if not task:
            res.success, res.message = False, "set the task first: ros2 param set /rebel_recorder task '...'"
            return res
        idx = self.writer.start(task)
        self.recording, self.skipped = True, 0
        res.success, res.message = True, f"recording episode {idx}: '{task}'"
        self.get_logger().info(res.message)
        return res

    def on_stop(self, req, res):
        if not self.recording:
            res.success, res.message = False, "not recording"
            return res
        self.recording = False
        out = self.writer.stop()
        res.success = out is not None
        res.message = (f"saved episode {out[0]}: {out[1]} frames ({out[1] / self.fps:.1f} s), "
                       f"{self.skipped} skipped" if out else "episode empty, nothing saved")
        self.get_logger().info(res.message)
        return res

    def on_discard(self, req, res):
        self.recording = False
        self.writer.discard()
        res.success, res.message = True, "episode discarded"
        self.get_logger().info(res.message)
        return res

    # ------------------------------------------------------------------ main loop
    def step(self):
        if not self.recording:
            return
        now = self.now()
        missing = [k for k in ["state", "action", *self.cameras] if now - self.stamp.get(k, -1e9) > self.max_age]
        if self.grip_meas is None:
            missing.append("gripper joint in /joint_states")
        if self.grip_cmd is None:
            missing.append("gripper state")
        if missing:
            self.skipped += 1
            self.get_logger().warn(f"skipping frames, no recent: {', '.join(missing)}", throttle_duration_sec=2.0)
            return
        state = self.q_meas + [self.grip_meas]
        action = self.q_cmd + [self.grip_cmd]
        try:
            self.writer.add_frame(state, action, {c: image_to_rgb(self.images[c]) for c in self.cameras})
        except ValueError as e:
            self.get_logger().error(f"{e} - episode discarded")
            self.recording = False
            self.writer.discard()


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.recording:                                      # Ctrl+C mid-episode: keep what was recorded
        node.writer.stop()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
