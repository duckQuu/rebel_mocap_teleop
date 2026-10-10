"""teleop node - your hand (OptiTrack rigid body, via mocap4ros2_optitrack TF) drives the igus.

Pipeline, every cycle (default 60 Hz):
  1. TF lookup  base_frame <- hand_frame          (hand pose in the robot base frame)
  2. tracking check: missing / stale / frozen pose -> HOLD (robot keeps its last command)
  3. LOW-PASS filter on the hand pose            (position: 1st-order; orientation: slerp)
  4. clutch + mapping: relative (default) or absolute, scale, workspace clamp
  5. real-time IK  target flange pose -> 6 joint angles, seeded with the last command
  6. joint speed limit                            (max_joint_speed_deg per joint)
  7. publish sensor_msgs/JointState on command_topic (/dual_arm/isaac_joint_commands for the Isaac rig)

Clutch (relative mode): call the service  ~/engage  (std_srvs/SetBool)
  ros2 service call /rebel_teleop/engage std_srvs/srv/SetBool "{data: true}"    # start following
  ros2 service call /rebel_teleop/engage std_srvs/srv/SetBool "{data: false}"   # stop, reposition hand
  ros2 service call /rebel_teleop/toggle std_srvs/srv/Trigger                     # pause <-> resume
  ros2 run rebel_demo teleop_keyboard                                              # SPACE = toggle (own terminal)
Pause = the robot holds its last command; resume = the robot's last pose and your hand's pose at that moment
become the new reference, so you continue from where it stopped (the gripper node is not affected).
Topic ~/engaged (std_msgs/Bool, latched) shows the current state.
While engaged:  robot_target = robot_pose_at_engage + scale * (hand_now - hand_at_engage)

Parameters (defaults in brackets)
  base_frame [arm_left_base_link]  hand_frame [HAND]          TF frames (hand = rigid body name in mocap config)
  rate_hz [60]            mode [relative]            relative | absolute
  scale [1.0]             orientation [false]        false = position only (wrist free), true = full pose
  cutoff_hz [4.0]         low-pass cutoff for the hand pose
  max_joint_speed_deg [45]  stale_timeout [0.2]      s without a new pose -> hold
  workspace_min [-0.7,-0.7,-0.45]  workspace_max [0.7,0.7,0.95]   target clamp in base_frame [m]
  command_topic [/dual_arm/isaac_joint_commands]   topic Isaac's ROS2 Subscribe Joint State node listens to
                          (the launch file gives each teleop node its own topic and joint_merger forwards them)
  publish_joint_states [false]   true = also publish /joint_states (stand-alone testing; the launch uses joint_merger)
  feedback_from_joint_states [false]  true = seed IK from Isaac's joint states instead of last command
  limit_warn_mm [5]       warn "joint limit reached" when the IK is further than this from the target and a joint
                          sits at a limit (the elbow-up bound counts); also warns when the workspace box clamps the target
  reanchor_after [2.0]    s of lost tracking before the clutch re-anchors (shorter gaps: hold, keep the anchor)
  max_hand_speed [2.0]    m/s; a sample jumping further than this (+1 cm) is a marker glitch and ignored for up to 0.1 s
  max_tool_speed [0.5]    m/s; the IK target never moves faster (smooth catch-up after a tracking gap)
  home_wait [1.0]         s to wait for joint states at start; the robot then moves from there to HOME

Startup / HOME: the node starts from the measured joints (or HOME if none arrive) and drives to HOME in joint space
at max_joint_speed_deg; engage is refused until it is there. Service ~/home (std_srvs/Trigger) does the same later:
  ros2 service call /rebel_teleop/home std_srvs/srv/Trigger
  joint_prefix [arm_left_]  joint names = prefix + joint1..joint6 (arm_left_ / arm_right_ on the dual-arm rig)
  joint_names ['']        6 comma-separated names (base to wrist), overrides joint_prefix, e.g. "a_1,a_2,...,a_6"
  joint_states_topic [/dual_arm/isaac_joint_states]   where Isaac publishes the measured joints (feedback only)
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .kinematics import HOME, JOINT_NAMES, LOWER, UPPER, fk, ik, limits_hit, quat_to_matrix


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
        self.base_frame = d("base_frame", "arm_left_base_link").value
        self.hand_frame = d("hand_frame", "HAND").value
        self.rate = float(d("rate_hz", 60.0).value)
        self.mode = d("mode", "relative").value
        self.scale = float(d("scale", 1.0).value)
        self.use_orientation = bool(d("orientation", False).value)
        self.filter = PoseLowPass(float(d("cutoff_hz", 4.0).value))
        self.vmax = math.radians(float(d("max_joint_speed_deg", 45.0).value))
        self.stale = float(d("stale_timeout", 0.2).value)
        self.ws_min = np.array(d("workspace_min", [-0.7, -0.7, -0.45]).value, float)
        self.ws_max = np.array(d("workspace_max", [0.7, 0.7, 0.95]).value, float)
        self.limit_warn = float(d("limit_warn_mm", 5.0).value) * 1e-3
        self.limit_active = False
        self.reanchor_after = float(d("reanchor_after", 2.0).value)
        self.max_hand_speed = float(d("max_hand_speed", 2.0).value)
        self.max_tool_speed = float(d("max_tool_speed", 0.5).value)
        self.good_p, self.good_t, self.glitch_since = None, None, None   # outlier gate state
        self.target = None                                                # last IK target position (rate limit)
        self.home_wait = float(d("home_wait", 1.0).value)
        self.engaged = bool(d("start_engaged", self.mode == "absolute").value)
        topic = d("command_topic", "/dual_arm/isaac_joint_commands").value
        self.pub_js = bool(d("publish_joint_states", False).value)
        self.use_feedback = bool(d("feedback_from_joint_states", False).value)
        prefix = d("joint_prefix", "arm_left_").value
        custom = [n.strip() for n in d("joint_names", "").value.split(",") if n.strip()]
        if custom and len(custom) != 6:
            raise ValueError(f"joint_names needs 6 comma-separated names, got {len(custom)}: {custom}")
        self.joint_names = custom or [prefix + n for n in JOINT_NAMES]
        js_topic = d("joint_states_topic", "/dual_arm/isaac_joint_states").value

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self, spin_thread=True)   # TF never waits for the IK
        self.pub_cmd = self.create_publisher(JointState, topic, 10)
        self.pub_state = self.create_publisher(JointState, "/joint_states", 10) if self.pub_js else None
        self.pub_target = self.create_publisher(PoseStamped, "~/target", 10)
        if not self.pub_js:                                # measured joints: start pose, optional IK seed
            self.create_subscription(JointState, js_topic, self.on_js, 10)
        self.create_service(SetBool, "~/engage", self.on_engage)
        self.create_service(Trigger, "~/home", self.on_home)
        self.create_service(Trigger, "~/toggle", self.on_toggle)
        self.pub_engaged = self.create_publisher(
            Bool, "~/engaged", QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                          durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.q_cmd = None                  # last commanded joints (set from the measured joints at start)
        self.q_meas = None
        self.t_start = None
        self.homing = True                 # drive to HOME before teleop can engage
        self.lost_since = None
        self.last_stamp = None
        self.last_raw = None
        self.frozen_since = None
        self.why = ""
        self.hand_ref = None               # (p, R) of the hand at engage
        self.robot_ref = None              # flange pose at engage
        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f"teleop: {self.hand_frame} -> {self.base_frame} ({self.joint_names[0]}..), mode={self.mode}, "
            f"{'full pose' if self.use_orientation else 'position only'}, {self.rate:.0f} Hz -> {topic}"
            + ("" if self.engaged else "  (NOT engaged: call ~/engage true, or press SPACE in teleop_keyboard)"))
        self.pub_engaged.publish(Bool(data=self.engaged))

    # ------------------------------------------------------------------ inputs
    def on_js(self, msg):
        if all(n in msg.name for n in self.joint_names):
            self.q_meas = np.array([msg.position[msg.name.index(n)] for n in self.joint_names])

    def set_engaged(self, on):
        """Clutch in / out. Either way the reference is retaken on the next cycle: robot_ref = fk(q_cmd) (the last
        commanded pose) and hand_ref = the hand's pose at that moment, so a resume continues from where it stopped.
        Returns (success, message); engaging is refused while the arm moves to HOME."""
        if on and self.homing:
            msg = "moving to HOME - engage again when 'at HOME' is logged"
            self.get_logger().warn(msg)
            return False, msg
        self.engaged = bool(on)
        self.hand_ref = None                # re-anchor on the next cycle
        self.target = None
        self.pub_engaged.publish(Bool(data=self.engaged))
        msg = "engaged (RUNNING)" if self.engaged else "released (PAUSED, robot holds)"
        self.get_logger().info(msg)
        return True, msg

    def on_engage(self, req, res):
        res.success, res.message = self.set_engaged(req.data)
        return res

    def on_toggle(self, req, res):
        res.success, res.message = self.set_engaged(not self.engaged)
        return res

    def on_home(self, req, res):
        self.engaged, self.homing, self.hand_ref, self.target = False, True, None, None
        self.pub_engaged.publish(Bool(data=False))
        res.success, res.message = True, "moving to HOME (disengaged)"
        self.get_logger().info(res.message)
        return res

    def drive_home(self):
        """One cycle of the joint-space move to HOME: all joints arrive together, none faster than vmax."""
        if self.q_cmd is None:                                     # wait for the measured start pose
            now = self.get_clock().now().nanoseconds * 1e-9
            self.t_start = self.t_start or now
            if self.q_meas is None and now - self.t_start < self.home_wait:
                return
            # start from the measured pose as it is (may be outside the IK bounds, e.g. Isaac's straight-up start with
            # joint3 = 0 < the 15 deg elbow bound); clipping here would jump joint3 in one cycle
            self.q_cmd = (self.q_meas if self.q_meas is not None else HOME).copy()
            self.get_logger().info("start pose %s deg -> moving to HOME %s deg" % (
                np.degrees(self.q_cmd).round(1).tolist(), np.degrees(HOME).round(1).tolist()))
        d = HOME - self.q_cmd
        n = np.max(np.abs(d)) / (self.vmax * self.dt)              # cycles the slowest joint needs
        self.q_cmd = HOME.copy() if n <= 1 else self.q_cmd + d / n
        if n <= 1:
            self.homing = False
            self.get_logger().info("at HOME - call ~/engage true to start")
        self.publish(self.q_cmd)

    def read_hand(self):
        """Hand pose in base_frame, or None if missing / stale / frozen."""
        try:
            t = self.tf.lookup_transform(self.base_frame, self.hand_frame, Time())
        except TransformException as e:
            self.why = f"no TF {self.base_frame} <- {self.hand_frame} ({type(e).__name__})"
            return None
        stamp = Time.from_msg(t.header.stamp).nanoseconds * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - stamp > self.stale:
            self.why = f"hand pose stale ({now - stamp:.2f} s old)"
            return None
        tr, rq = t.transform.translation, t.transform.rotation
        raw = (tr.x, tr.y, tr.z, rq.x, rq.y, rq.z, rq.w)
        if raw == self.last_raw:                           # bit-identical = frozen (marker loss)
            self.frozen_since = self.frozen_since or now
            if now - self.frozen_since > self.stale:
                self.why = "hand pose frozen (bit-identical, marker loss?)"
                return None
        else:
            self.frozen_since = None
        self.last_raw = raw
        p = np.array(raw[:3])
        # outlier gate: a jump faster than a hand can move is a marker glitch (swap / ghost) - skip it; if it
        # persists for 0.1 s it is real (e.g. hand moved during a drop-out) and accepted
        if self.good_p is not None:
            allowed = self.max_hand_speed * max(now - self.good_t, self.dt) + 0.01
            if np.linalg.norm(p - self.good_p) > allowed:
                self.glitch_since = self.glitch_since or now
                if now - self.glitch_since < 0.1:
                    self.why = "marker glitch (pose jump) ignored"
                    return "glitch"
        self.good_p, self.good_t, self.glitch_since = p, now, None
        return p, np.array(raw[3:])

    # ------------------------------------------------------------------ main loop
    def step(self):
        if self.homing:
            self.drive_home()
            return
        hand = self.read_hand()
        now = self.get_clock().now().nanoseconds * 1e-9
        if isinstance(hand, str):                          # glitch sample: hold this cycle, keep everything
            self.publish(self.q_cmd)
            return
        if hand is None:
            self.lost_since = self.lost_since or now
            if now - self.lost_since > self.reanchor_after:  # really lost: re-anchor when tracking returns
                if self.hand_ref is not None and self.engaged:
                    self.get_logger().warn(f"tracking lost: {self.why} - will re-anchor")
                self.hand_ref = None
                self.filter.p = None                       # restart the filter (no glide from a stale pose)
                self.good_p = None
            else:                                          # short gap: hold, keep the anchor
                self.get_logger().warn(f"holding: {self.why}", throttle_duration_sec=2.0)
            self.publish(self.q_cmd)                       # HOLD
            return
        self.lost_since = None
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
                self.get_logger().info("anchored: tool at (%.3f, %.3f, %.3f) m <-> hand at (%.3f, %.3f, %.3f) m"
                                       % (*self.robot_ref[:3, 3], *p_f))
            T = self.robot_ref.copy()
            T[:3, 3] = self.robot_ref[:3, 3] + self.scale * (p_f - self.hand_ref[0])
            if self.use_orientation:
                T[:3, :3] = (R_f @ self.hand_ref[1].T) @ self.robot_ref[:3, :3]
        else:                                               # absolute: flange = hand pose
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = R_f, p_f
        # workspace box, widened to contain the anchor pose: engaging never moves the robot
        lo, hi = self.ws_min, self.ws_max
        if self.mode == "relative":
            lo, hi = np.minimum(lo, self.robot_ref[:3, 3]), np.maximum(hi, self.robot_ref[:3, 3])
        wanted = T[:3, 3].copy()
        T[:3, 3] = np.clip(wanted, lo, hi)
        beyond = wanted - T[:3, 3]                         # > 0: past the max, < 0: past the min
        for ax in np.nonzero(np.abs(beyond) > 1e-3)[0]:
            side, bound = ("max", hi[ax]) if beyond[ax] > 0 else ("min", lo[ax])
            self.get_logger().warn(
                f"target clamped by workspace_{side} {'xyz'[ax]} = {bound:+.2f} m "
                f"(hand is {abs(beyond[ax]) * 100:.1f} cm beyond) - raise the limit if the arm can go there",
                throttle_duration_sec=2.0)
        # target speed limit: after a gap the hand may be elsewhere; the tool follows smoothly, never jumps
        if self.target is not None:
            step = T[:3, 3] - self.target
            n = np.linalg.norm(step)
            if n > self.max_tool_speed * self.dt:
                T[:3, 3] = self.target + step * (self.max_tool_speed * self.dt / n)
        self.target = T[:3, 3].copy()
        self.publish_target(T)

        # 5. real-time IK, warm-started from the last command
        w_rot = 0.3 if self.use_orientation else 0.0
        q_ik, pe, re = ik(T, seed, w_rot=w_rot, iters=25, q_rest=None if self.use_orientation else HOME)
        hits = limits_hit(q_ik) if pe > self.limit_warn else []
        if hits:                                           # a joint limit is what keeps the IK off the target
            self.limit_active = True
            what = ", ".join(
                f"{self.joint_names[i]} at {side} {'elbow-up bound' if (i == 2 and side == 'lower') else 'limit'} "
                f"{np.degrees(LOWER[i] if side == 'lower' else UPPER[i]):.0f} deg" for i, side in hits)
            self.get_logger().warn(f"joint limit reached: {what} - target {pe * 1000:.0f} mm out of reach, "
                                   f"moving to the closest reachable pose", throttle_duration_sec=1.0)
        else:
            if self.limit_active and pe <= self.limit_warn:
                self.limit_active = False
                self.get_logger().info("joint limits: target back in range")
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
