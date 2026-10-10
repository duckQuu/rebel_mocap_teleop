"""gripper node - a second OptiTrack rigid body opens / closes the HIWIN XEG-32 gripper (binary).

State is set by two TF frames:
  gripper_frame  (the control rigid body, e.g. on a fingertip)
  ref_frame      (another rigid body, by default the palm body that drives the arm)
mode "distance" (default):
  distance < threshold - hysteresis/2  -> CLOSED (0)
  distance > threshold + hysteresis/2  -> OPEN   (1)
  in between                           -> keep the current state (no chatter from marker jitter)
mode "orientation": position is ignored, only the rotation of gripper_frame seen from ref_frame
  counts. That relative rotation does not change when the whole hand is turned (both bodies turn together), so it
  works facing up / down / left / right. angle = total rotation angle of it, 0..180 deg, measured from the pose saved
  by the ~/calibrate service (identity until then):
  angle < open_angle_deg   -> OPEN   (1)        angle > close_angle_deg -> CLOSED (0)
  in between               -> keep the current state
Either way a new state must hold for `debounce` s before the gripper switches.

Pipeline, every cycle (default 60 Hz):
  1. TF lookup  ref_frame <- gripper_frame  -> distance or rotation angle
  2. tracking check: missing / stale / frozen pose -> HOLD (gripper keeps its last state)
  3. distance / angle -> state 0/1 with hysteresis + debounce; carriages drive to closed_pos or open_pos at max_speed
  4. publish sensor_msgs/JointState (<prefix>left_carriage_joint, <prefix>right_carriage_joint, metres)
     on command_topic, plus std_msgs/Float64 ~/opening_mm (jaw gap beyond closed, for a real XEG driver)

Parameters (defaults in brackets)
  gripper_frame [rigid_body_2]   ref_frame [rigid_body_1]
  mode [distance]          distance | orientation
  threshold [0.08]  hysteresis [0.02]   switching distance and dead band [m]   (mode distance)
  open_angle_deg [30]  close_angle_deg [100]   open below / closed above this angle [deg]   (mode orientation)
  invert [false]           true = far closes, near opens
  joint_prefix [arm_left_xeg32_]    arm_left_xeg32_ / arm_right_xeg32_ on the dual-arm rig
  closed_pos [-0.0116631]  open_pos [0.00483688]   carriage joint values [m] (xeg32_gripper.urdf.xacro limits)
  rate_hz [60]  max_speed [0.05] jaw speed [m/s]  stale_timeout [0.2]
  start_open [true]
  debounce [0.1]   s the new state must hold before the gripper switches (one-frame marker glitches)
  command_topic [/dual_arm/isaac_joint_commands]   publish_joint_states [false]
  (publish_joint_states also sends <prefix>camera_tilt_joint = 0 so RViz can place the camera mount)
Topic ~/state (std_msgs/Int32): 0 = closed, 1 = open.   ~/angle_deg (std_msgs/Float64): the angle (orientation mode).
Service ~/calibrate (std_srvs/Trigger): hold the hand relaxed / open, call it, that pose becomes 0 deg.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

CARRIAGE_JOINTS = ["left_carriage_joint", "right_carriage_joint"]   # right mimics left in the model
CLOSED_POS, OPEN_POS = -0.0116631, 0.00483688                        # joint limits = closed / open


def next_state(d, state, threshold, hysteresis, invert=False):
    """distance [m] + current state (0 closed / 1 open) -> new state, with a dead band."""
    if d > threshold + hysteresis / 2:
        far = True
    elif d < threshold - hysteresis / 2:
        far = False
    else:
        return state
    return int(far != invert)


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_mul(a, b):
    """Hamilton product of two [x, y, z, w] quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotation_angle_deg(q, q_ref=None):
    """Total rotation angle [0, 180] deg of quaternion q [x, y, z, w], measured from q_ref (None = identity).
    q and -q are the same rotation, so |w| is used; atan2 stays accurate near 0 and 180 deg."""
    if q_ref is not None:
        q = quat_mul(quat_conj(q_ref), q)
    v = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2)
    return math.degrees(2.0 * math.atan2(v, abs(q[3])))


def next_state_angle(angle, state, open_below, close_above, invert=False):
    """angle [deg] + current state (0 closed / 1 open) -> new state; between the two angles keep the state."""
    if angle < open_below:
        is_open = True
    elif angle > close_above:
        is_open = False
    else:
        return state
    return int(is_open != invert)


class Gripper(Node):
    def __init__(self):
        super().__init__("rebel_gripper")
        d = self.declare_parameter
        self.gripper_frame = d("gripper_frame", "rigid_body_2").value
        self.ref_frame = d("ref_frame", "rigid_body_1").value
        self.mode = d("mode", "distance").value
        if self.mode not in ("distance", "orientation"):
            raise ValueError(f"mode must be 'distance' or 'orientation', got {self.mode!r}")
        self.open_angle = float(d("open_angle_deg", 30.0).value)
        self.close_angle = float(d("close_angle_deg", 100.0).value)
        if self.open_angle >= self.close_angle:
            raise ValueError(f"open_angle_deg ({self.open_angle}) must be below close_angle_deg ({self.close_angle})")
        self.threshold = float(d("threshold", 0.08).value)
        self.hysteresis = float(d("hysteresis", 0.02).value)
        self.invert = bool(d("invert", False).value)
        prefix = d("joint_prefix", "arm_left_xeg32_").value
        self.joints = [prefix + j for j in CARRIAGE_JOINTS]
        self.camera_joint = prefix + "camera_tilt_joint"
        self.closed = float(d("closed_pos", CLOSED_POS).value)
        self.open = float(d("open_pos", OPEN_POS).value)
        self.rate = float(d("rate_hz", 60.0).value)
        self.vmax = float(d("max_speed", 0.05).value)
        self.stale = float(d("stale_timeout", 0.2).value)
        topic = d("command_topic", "/dual_arm/isaac_joint_commands").value
        pub_js = bool(d("publish_joint_states", False).value)

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pub_cmd = self.create_publisher(JointState, topic, 10)
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10) if pub_js else None
        self.pub_mm = self.create_publisher(Float64, "~/opening_mm", 10)
        self.pub_state = self.create_publisher(Int32, "~/state", 10)
        self.pub_angle = self.create_publisher(Float64, "~/angle_deg", 10)
        self.create_service(Trigger, "~/calibrate", self.on_calibrate)

        self.state = 1 if bool(d("start_open", True).value) else 0
        self.debounce = float(d("debounce", 0.1).value)
        self.pending_since = None
        self.jaw = self.open if self.state else self.closed   # commanded carriage position [m]
        self.last_raw = None
        self.frozen_since = None
        self.last_q = None                                  # latest relative rotation [x, y, z, w]
        self.q_open = None                                  # relative rotation at the calibrated open pose
        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.step)
        if self.mode == "orientation":
            self.get_logger().info(
                f"gripper: rotation of {self.gripper_frame} in {self.ref_frame}: < {self.open_angle:.0f} deg = "
                f"{'CLOSED' if self.invert else 'OPEN'}, > {self.close_angle:.0f} deg = "
                f"{'OPEN' if self.invert else 'CLOSED'} -> {topic}  (call ~/calibrate with the hand open)")
        else:
            self.get_logger().info(
                f"gripper: |{self.gripper_frame} - {self.ref_frame}| "
                f"{'>' if self.invert else '<'} {self.threshold * 100:.1f} cm = CLOSED "
                f"(dead band {self.hysteresis * 100:.1f} cm) -> {topic}")

    def on_calibrate(self, req, res):
        if self.last_q is None:
            res.success, res.message = False, "no fresh pose of both rigid bodies - is mocap tracking them?"
        else:
            self.q_open = self.last_q
            res.success, res.message = True, "open pose saved (angle = 0 deg here)"
        self.get_logger().info(res.message)
        return res

    def read_measure(self):
        """Distance [m] (mode distance) or rotation angle [deg] (mode orientation); None if missing / stale / frozen."""
        try:
            t = self.tf.lookup_transform(self.ref_frame, self.gripper_frame, Time())
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
        self.last_q = raw[3:]
        if self.mode == "orientation":                     # rotation of gripper_frame seen from ref_frame; no position
            return rotation_angle_deg(raw[3:], self.q_open)
        return math.sqrt(tr.x ** 2 + tr.y ** 2 + tr.z ** 2)

    def step(self):
        value = self.read_measure()
        if value is not None:                              # None = tracking lost -> keep the state
            if self.mode == "orientation":
                new = next_state_angle(value, self.state, self.open_angle, self.close_angle, self.invert)
                shown = f"angle {value:.0f} deg"
                self.pub_angle.publish(Float64(data=float(value)))
            else:
                new = next_state(value, self.state, self.threshold, self.hysteresis, self.invert)
                shown = f"distance {value * 100:.1f} cm"
            now = self.get_clock().now().nanoseconds * 1e-9
            if new == self.state:
                self.pending_since = None
            elif self.pending_since is None:
                self.pending_since = now                   # new state must hold for `debounce` s
            elif now - self.pending_since >= self.debounce:
                self.get_logger().info(f"{'OPEN' if new else 'CLOSE'}  ({shown})")
                self.state, self.pending_since = new, None
        step = self.vmax * self.dt
        target = self.open if self.state else self.closed
        self.jaw += min(step, max(-step, target - self.jaw))
        self.publish()

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joints)
        msg.position = [float(self.jaw), float(self.jaw)]
        self.pub_cmd.publish(msg)
        if self.pub_js is not None:                        # RViz only: robot_state_publisher needs every joint
            msg.name.append(self.camera_joint)
            msg.position.append(0.0)
            self.pub_js.publish(msg)
        self.pub_mm.publish(Float64(data=2000.0 * (self.jaw - self.closed)))
        self.pub_state.publish(Int32(data=self.state))


def main():
    rclpy.init()
    node = Gripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
