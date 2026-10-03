import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MocapTF(Node):
    def __init__(self):
        super().__init__("mocap_tf")
        try:
            from mocap4r2_msgs.msg import RigidBodies
        except ImportError as e:
            raise RuntimeError("mocap4r2_msgs not found - run: source ~/mocap_ws/install/setup.bash") from e
        d = self.declare_parameter
        topic = d("topic", "/rigid_bodies").value
        self.prefix = d("prefix", "rigid_body_").value
        self.parent = d("parent_frame", "").value
        self.receive_time = bool(d("use_receive_time", True).value)
        self.tf = TransformBroadcaster(self)
        self.seen = set()
        self.create_subscription(RigidBodies, topic, self.on_bodies, 10)
        self.get_logger().info(f"{topic} -> TF  (child frames '{self.prefix}<name>')")

    def on_bodies(self, msg):
        stamp = self.get_clock().now().to_msg() if self.receive_time else msg.header.stamp
        parent = self.parent or msg.header.frame_id or "map"
        out = []
        for rb in msg.rigidbodies:
            child = f"{self.prefix}{rb.rigid_body_name}"
            if child not in self.seen:
                self.seen.add(child)
                self.get_logger().info(f"new rigid body: {parent} -> {child}")
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            p, q = rb.pose.position, rb.pose.orientation
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
            t.transform.rotation = q
            out.append(t)
        if out:
            self.tf.sendTransform(out)


def main():
    rclpy.init()
    node = MocapTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
