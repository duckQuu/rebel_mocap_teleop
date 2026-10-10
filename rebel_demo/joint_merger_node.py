"""joint_merger node - combines several JointState command streams into ONE message per cycle.

Isaac's ROS2 Subscribe Joint State node passes on only the latest message each frame. When the arm(s) and
gripper(s) publish separately on the same topic, each joint gets its command only every few frames. This node
keeps the latest position of every joint it has heard and publishes all of them together at rate_hz.

Parameters (defaults in brackets)
  inputs ['']                 comma-separated input topics, e.g. "/rebel_teleop_left/joint_commands,..."
  output [/dual_arm/isaac_joint_commands]   topic Isaac subscribes to (RViz-only: the joint-state topic)
  rate_hz [60]
  defaults ['']               comma-separated joints published at 0.0 until a real value arrives (RViz-only testing:
                              the joints Isaac normally supplies, so robot_state_publisher gets a complete state)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointMerger(Node):
    def __init__(self):
        super().__init__("joint_merger")
        d = self.declare_parameter
        inputs = [t.strip() for t in d("inputs", "").value.split(",") if t.strip()]
        output = d("output", "/dual_arm/isaac_joint_commands").value
        rate = float(d("rate_hz", 60.0).value)
        if not inputs:
            raise ValueError("joint_merger: set 'inputs' (comma-separated JointState topics)")
        defaults = [n.strip() for n in d("defaults", "").value.split(",") if n.strip()]
        self.latest = {n: 0.0 for n in defaults}            # joint name -> position (insertion order kept)
        for t in inputs:
            self.create_subscription(JointState, t, self.on_cmd, 10)
        self.pub = self.create_publisher(JointState, output, 10)
        self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(f"joint_merger: {inputs} -> {output} at {rate:.0f} Hz")

    def on_cmd(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.latest[name] = pos

    def step(self):
        if not self.latest:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.latest)
        msg.position = [float(v) for v in self.latest.values()]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
