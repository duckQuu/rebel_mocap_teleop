"""teleop node - your hand (OptiTrack rigid body, via mocap4ros2_optitrack TF) drives the igus.

Pipeline, every cycle (default 60 Hz):
  1. TF lookup  base_frame <- hand_frame          (hand pose in the robot base frame)
  2. tracking check: missing / stale / frozen pose -> HOLD (robot keeps its last command)
  3. LOW-PASS filter on the hand pose            (position: 1st-order; orientation: slerp)
  4. clutch + mapping: relative (default) or absolute, scale, workspace clamp
  5. real-time IK  target flange pose -> 6 joint angles, seeded with the last command
  6. joint speed limit                            (max_joint_speed_deg per joint)
  7. publish sensor_msgs/JointState on command_topic (/isaac_joint_commands for Isaac Sim)

Clutch (relative mode): call the service  ~/engage  (std_srvs/SetBool)
  ros2 service call /rebel_teleop/engage std_srvs/srv/SetBool "{data: true}"    # start following
  ros2 service call /rebel_teleop/engage std_srvs/srv/SetBool "{data: false}"   # stop, reposition hand
While engaged:  robot_target = robot_pose_at_engage + scale * (hand_now - hand_at_engage)

Parameters (defaults in brackets)
  base_frame [base_link]  hand_frame [HAND]          TF frames (hand = rigid body name in mocap config)
  rate_hz [60]            mode [relative]            relative | absolute
  scale [1.0]             orientation [false]        false = position only (wrist free), true = full pose
  cutoff_hz [4.0]         low-pass cutoff for the hand pose
  max_joint_speed_deg [45]  stale_timeout [0.2]      s without a new pose -> hold
  workspace_min [-0.7,-0.7,-0.3]  workspace_max [0.7,0.7,0.8]   target clamp in base_frame [m]
  command_topic [/isaac_joint_commands]   topic Isaac Sim's ROS2 Subscribe Joint State node listens to
  publish_joint_states [false]   true = also publish /joint_states (RViz-only testing, no Isaac)
  feedback_from_joint_states [false]  true = seed IK from Isaac's joint states instead of last command
  joint_prefix ['']       joint names = prefix + joint1..joint6 (arm_left_ / arm_right_ for the dual-arm rig)
  joint_names ['']        6 comma-separated names (base to wrist), overrides joint_prefix, e.g. "a_1,a_2,...,a_6"
  joint_states_topic [/joint_states]   where Isaac publishes the measured joints (feedback only)
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from .kinematics import HOME, JOINT_NAMES, LOWER, UPPER, fk, ik, quat_to_matrix


def matrix_to_quat(R):
    """3x3 -> [x, y, z, w]"""
    t = np.trace(R)
    if t > 0:
        s = 2 * math.sqrt(t + 1)
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
    q = np.zeros(4)
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def slerp(q0, q1, a):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + a * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - a) * th) * q0 + math.sin(a * th) * q1) / math.sin(th)


class PoseLowPass:
    """First-order low-pass on position + slerp on orientation; alpha from cutoff and dt."""

    def __init__(self, cutoff_hz):
        self.fc = cutoff_hz
        self.p = None
        self.q = None

    def reset(self, p, q):
        self.p, self.q = np.array(p, float), np.array(q, float)

    def __call__(self, p, q, dt):
        if self.p is None or self.fc <= 0:
            self.reset(p, q)
            return self.p, self.q
        a = 1.0 - math.exp(-2 * math.pi * self.fc * dt)
        self.p = self.p + a * (np.asarray(p) - self.p)
        self.q = slerp(self.q, q, a)
        return self.p, self.q


class Teleop(Node):
    def __init__(self):
        super().__init__("rebel_teleop")
        d = self.declare_parameter
        self.base_frame = d("base_frame", "base_link").value
        self.hand_frame = d("hand_frame", "HAND").value
        self.rate = float(d("rate_hz", 60.0).value)
        self.mode = d("mode", "relative").value
        self.scale = float(d("scale", 1.0).value)
        self.use_orientation = bool(d("orientation", False).value)
        self.filter = PoseLowPass(float(d("cutoff_hz", 4.0).value))
        self.vmax = math.radians(float(d("max_joint_speed_deg", 45.0).value))
        self.stale = float(d("stale_timeout", 0.2).value)
        self.ws_min = np.array(d("workspace_min", [-0.7, -0.7, -0.3]).value, float)
        self.ws_max = np.array(d("workspace_max", [0.7, 0.7, 0.8]).value, float)
        self.engaged = bool(d("start_engaged", self.mode == "absolute").value)
        topic = d("command_topic", "/isaac_joint_commands").value
        self.pub_js = bool(d("publish_joint_states", False).value)
        self.use_feedback = bool(d("feedback_from_joint_states", False).value)
        prefix = d("joint_prefix", "").value
        custom = [n.strip() for n in d("joint_names", "").value.split(",") if n.strip()]
        if custom and len(custom) != 6:
            raise ValueError(f"joint_names needs 6 comma-separated names, got {len(custom)}: {custom}")
        self.joint_names = custom or [prefix + n for n in JOINT_NAMES]
        js_topic = d("joint_states_topic", "/joint_states").value

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pub_cmd = self.create_publisher(JointState, topic, 10)
        self.pub_state = self.create_publisher(JointState, "/joint_states", 10) if self.pub_js else None
        self.pub_target = self.create_publisher(PoseStamped, "~/target", 10)
        if self.use_feedback and not self.pub_js:
            self.create_subscription(JointState, js_topic, self.on_js, 10)
        self.create_service(SetBool, "~/engage", self.on_engage)

        self.q_cmd = HOME.copy()           # last commanded joints (the robot starts at HOME)
        self.q_meas = None
        self.last_stamp = None
        self.last_raw = None
        self.frozen_since = None
        self.hand_ref = None               # (p, R) of the hand at engage
        self.robot_ref = None              # flange pose at engage
        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f"teleop: {self.hand_frame} -> {self.base_frame} ({self.joint_names[0]}..), mode={self.mode}, "
            f"{'full pose' if self.use_orientation else 'position only'}, {self.rate:.0f} Hz -> {topic}"
            + ("" if self.engaged else "  (NOT engaged: call ~/engage true)"))

    # ------------------------------------------------------------------ inputs
    def on_js(self, msg):
        if all(n in msg.name for n in self.joint_names):
            self.q_meas = np.array([msg.position[msg.name.index(n)] for n in self.joint_names])

    def on_engage(self, req, res):
        self.engaged = bool(req.data)
        self.hand_ref = None                # re-anchor on the next cycle
        res.success, res.message = True, "engaged" if self.engaged else "released (robot holds)"
        self.get_logger().info(res.message)
        return res

    def read_hand(self):
        """Hand pose in base_frame, or None if missing / stale / frozen."""
        try:
            t = self.tf.lookup_transform(self.base_frame, self.hand_frame, Time())
        except TransformException:
            return None
        stamp = Time.from_msg(t.header.stamp).nanoseconds * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - stamp > self.stale:
            return None
        tr, rq = t.transform.translation, t.transform.rotation
        raw = (tr.x, tr.y, tr.z, rq.x, rq.y, rq.z, rq.w)
        if raw == self.last_raw:                           # bit-identical = frozen (marker loss)
            self.frozen_since = self.frozen_since or now
            if now - self.frozen_since > self.stale:
                return None
        else:
            self.frozen_since = None
        self.last_raw = raw
        return np.array(raw[:3]), np.array(raw[3:])

    # ------------------------------------------------------------------ main loop
    def step(self):
        hand = self.read_hand()
        if hand is None:
            self.hand_ref = None                           # re-anchor when tracking returns
            self.filter.p = None                           # restart the filter (no glide from a stale pose)
            self.publish(self.q_cmd)                       # HOLD
            return
        p_f, q_f = self.filter(hand[0], hand[1], self.dt)  # 3. low-pass
        R_f = quat_to_matrix(q_f)
        seed = self.q_meas if (self.use_feedback and self.q_meas is not None) else self.q_cmd

        if not self.engaged:
            self.publish(self.q_cmd)
            return
        # 4. mapping
        if self.mode == "relative":
            if self.hand_ref is None:                      # anchor: hand now <-> robot now
                self.hand_ref = (p_f.copy(), R_f.copy())
                self.robot_ref = fk(self.q_cmd)
            T = self.robot_ref.copy()
            T[:3, 3] = self.robot_ref[:3, 3] + self.scale * (p_f - self.hand_ref[0])
            if self.use_orientation:
                T[:3, :3] = (R_f @ self.hand_ref[1].T) @ self.robot_ref[:3, :3]
        else:                                               # absolute: flange = hand pose
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = R_f, p_f
        T[:3, 3] = np.clip(T[:3, 3], self.ws_min, self.ws_max)
        self.publish_target(T)

        # 5. real-time IK, warm-started from the last command
        w_rot = 0.3 if self.use_orientation else 0.0
        q_ik, pe, re = ik(T, seed, w_rot=w_rot, iters=25, q_rest=None if self.use_orientation else HOME)
        if pe > 0.02:
            self.get_logger().warn(f"target not reachable ({pe * 1000:.0f} mm off) - moving as close as possible",
                                   throttle_duration_sec=1.0)
        # 6. joint speed limit
        dq = np.clip(q_ik - self.q_cmd, -self.vmax * self.dt, self.vmax * self.dt)
        self.q_cmd = np.clip(self.q_cmd + dq, LOWER, UPPER)
        self.publish(self.q_cmd)                           # 7.

    def publish(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(v) for v in q]
        self.pub_cmd.publish(msg)
        if self.pub_state is not None:
            self.pub_state.publish(msg)

    def publish_target(self, T):
        m = PoseStamped()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in T[:3, 3])
        qx, qy, qz, qw = matrix_to_quat(T[:3, :3])
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (
            float(qx), float(qy), float(qz), float(qw))
        self.pub_target.publish(m)


def main():
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
