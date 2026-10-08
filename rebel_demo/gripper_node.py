"""gripper node - a second OptiTrack rigid body opens / closes the HIWIN XEG-32 gripper (binary).

State is set by the DISTANCE between two TF frames:
  gripper_frame  (the control rigid body, e.g. on your thumb or a handheld pinch tool)
  ref_frame      (another rigid body, by default the hand body that drives the arm)
  distance < threshold - hysteresis/2  -> CLOSED (0)
  distance > threshold + hysteresis/2  -> OPEN   (1)
  in between                           -> keep the current state (no chatter from marker jitter)

Pipeline, every cycle (default 60 Hz):
  1. TF lookup  ref_frame <- gripper_frame  -> distance
  2. tracking check: missing / stale / frozen pose -> HOLD (gripper keeps its last state)
  3. distance -> state 0/1 with hysteresis; carriages drive to closed_pos or open_pos at max_speed
  4. publish sensor_msgs/JointState (<prefix>left_carriage_joint, <prefix>right_carriage_joint, metres)
     on command_topic, plus std_msgs/Float64 ~/opening_mm (jaw gap beyond closed, for a real XEG driver)

Parameters (defaults in brackets)
  gripper_frame [rigid_body_2]   ref_frame [rigid_body_1]
  threshold [0.08]  hysteresis [0.02]   switching distance and dead band [m]
  invert [false]           true = far closes, near opens
  joint_prefix [xeg32_]    arm_left_xeg32_ / arm_right_xeg32_ for the dual-arm rig
  closed_pos [-0.0116631]  open_pos [0.00483688]   carriage joint values [m] (xeg32_gripper.urdf.xacro limits)
  rate_hz [60]  max_speed [0.05] jaw speed [m/s]  stale_timeout [0.2]
  start_open [true]
  command_topic [/isaac_joint_commands]   publish_joint_states [false]
  (publish_joint_states also sends <prefix>camera_tilt_joint = 0 so RViz can place the camera mount)
Topic ~/state (std_msgs/Int32): 0 = closed, 1 = open.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int32
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


class Gripper(Node):
    def __init__(self):
        super().__init__("rebel_gripper")
        d = self.declare_parameter
        self.gripper_frame = d("gripper_frame", "rigid_body_2").value
        self.ref_frame = d("ref_frame", "rigid_body_1").value
        self.threshold = float(d("threshold", 0.08).value)
        self.hysteresis = float(d("hysteresis", 0.02).value)
        self.invert = bool(d("invert", False).value)
        prefix = d("joint_prefix", "xeg32_").value
        self.joints = [prefix + j for j in CARRIAGE_JOINTS]
        self.camera_joint = prefix + "camera_tilt_joint"
        self.closed = float(d("closed_pos", CLOSED_POS).value)
        self.open = float(d("open_pos", OPEN_POS).value)
        self.rate = float(d("rate_hz", 60.0).value)
        self.vmax = float(d("max_speed", 0.05).value)
        self.stale = float(d("stale_timeout", 0.2).value)
        topic = d("command_topic", "/isaac_joint_commands").value
        pub_js = bool(d("publish_joint_states", False).value)

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pub_cmd = self.create_publisher(JointState, topic, 10)
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10) if pub_js else None
        self.pub_mm = self.create_publisher(Float64, "~/opening_mm", 10)
        self.pub_state = self.create_publisher(Int32, "~/state", 10)

        self.state = 1 if bool(d("start_open", True).value) else 0
        self.jaw = self.open if self.state else self.closed   # commanded carriage position [m]
        self.last_raw = None
        self.frozen_since = None
        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f"gripper: |{self.gripper_frame} - {self.ref_frame}| "
            f"{'>' if self.invert else '<'} {self.threshold * 100:.1f} cm = CLOSED "
            f"(dead band {self.hysteresis * 100:.1f} cm) -> {topic}")

    def read_distance(self):
        try:
            t = self.tf.lookup_transform(self.ref_frame, self.gripper_frame, Time())
        except TransformException:
            return None
        stamp = Time.from_msg(t.header.stamp).nanoseconds * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - stamp > self.stale:
            return None
        tr = t.transform.translation
        raw = (tr.x, tr.y, tr.z)
        if raw == self.last_raw:                           # bit-identical = frozen (marker loss)
            self.frozen_since = self.frozen_since or now
            if now - self.frozen_since > self.stale:
                return None
        else:
            self.frozen_since = None
        self.last_raw = raw
        return math.sqrt(tr.x ** 2 + tr.y ** 2 + tr.z ** 2)

    def step(self):
        dist = self.read_distance()
        if dist is not None:                               # None = tracking lost -> keep the state
            new = next_state(dist, self.state, self.threshold, self.hysteresis, self.invert)
            if new != self.state:
                self.get_logger().info(f"{'OPEN' if new else 'CLOSE'}  (distance {dist * 100:.1f} cm)")
            self.state = new
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
