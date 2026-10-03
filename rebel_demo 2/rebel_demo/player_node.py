"""player node - VISUALIZATION ONLY: plays the demo trajectories on the RViz model.

Publishes /joint_states (-> robot_state_publisher -> RViz) and /demo_markers (flange path,
take name, joint axes with angles). It does not talk to any robot controller.

Parameters
  trajectory_dir  folder with *.csv (default: <share>/rebel_demo/trajectories)
  only            play only files containing this text ("all" = every file)
  rate            playback speed factor (1.0 = real demo speed)
  loop            repeat forever
  pause           s to hold at the end of each take
  joint_markers   draw joint axes + angles
  frame           frame of the flange path columns (base_link)
"""
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .kinematics import HOME, JOINT_COLORS, JOINT_LABELS, JOINT_NAMES, fk, joint_frames
from .trajectory_io import Trajectory, list_trajectories, quintic, transit_duration

DT = 1.0 / 30.0


def default_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("rebel_demo"), "trajectories")
    except Exception:
        return os.path.join(os.path.dirname(__file__), "..", "trajectories")


class Player(Node):
    def __init__(self):
        super().__init__("rebel_demo_player")
        p = self.declare_parameter
        self.dir = p("trajectory_dir", default_dir()).value
        only = p("only", "all").value
        self.rate = float(p("rate", 1.0).value)
        self.loop = bool(p("loop", False).value)
        pause = float(p("pause", 1.0).value)
        self.show_joints = bool(p("joint_markers", True).value)
        self.frame = p("frame", "base_link").value

        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_mk = self.create_publisher(MarkerArray, "/demo_markers", 10)

        files = list_trajectories(self.dir, only)
        if not files:
            raise RuntimeError(f"no trajectory CSVs in {self.dir} (only={only})")

        # one long timeline of (q, segment index); -1 = moving between takes
        self.q, self.seg, self.takes = [], [], []
        self._add(np.repeat(HOME[None], int(1.0 / DT), 0), -1)
        q_prev = HOME
        for f in files:
            try:
                tr = Trajectory(f)
            except ValueError as e:
                self.get_logger().error(str(e))
                continue
            k = len(self.takes)
            self.takes.append(tr)
            self._add(quintic(q_prev, tr.q[0], transit_duration(q_prev, tr.q[0]), DT)[1], -1)
            start = 0
            for j in tr.jumps():
                self.get_logger().warn(f"{tr.name}: posture switch at t={tr.t[j]:.1f}s - shown as a slow move")
                self._add(tr.q[start:j + 1], k)
                self._add(quintic(tr.q[j], tr.q[j + 1], transit_duration(tr.q[j], tr.q[j + 1]), DT)[1], k)
                start = j + 1
            self._add(tr.q[start:], k)
            self._add(np.repeat(tr.q[-1][None], int(pause / DT), 0), k)
            q_prev = tr.q[-1]
            self.get_logger().info(f"loaded {tr.name}: {len(tr.q)} frames ({tr.t[-1]:.1f} s)")
        self._add(quintic(q_prev, HOME, transit_duration(q_prev, HOME), DT)[1], -1)
        self.q = np.array(self.q)
        self.get_logger().info(f"{len(self.takes)} takes, {len(self.q) * DT / 60:.1f} min at x{self.rate}")

        self.i, self.cur, self.done = 0, None, False
        self.timer = self.create_timer(DT / self.rate, self.tick)

    def _add(self, qs, seg):
        for q in qs:
            self.q.append(np.asarray(q, float))
            self.seg.append(seg)

    def tick(self):
        if self.i >= len(self.q):
            if self.loop:
                self.i = 0
            else:
                if not self.done:
                    self.done = True
                    self.get_logger().info("done - holding the home pose (Ctrl+C to quit)")
                self.i = len(self.q) - 1
        q = self.q[self.i]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = q.tolist()
        self.pub_js.publish(msg)
        if self.show_joints:
            self.pub_mk.publish(MarkerArray(markers=joint_markers(q, self.frame, msg.header.stamp)))
        s = self.seg[self.i]
        if s != -1 and s != self.cur:
            self.cur = s
            self.pub_mk.publish(MarkerArray(markers=path_markers(self.takes[s], self.frame, msg.header.stamp)))
            self.get_logger().info(f"playing {self.takes[s].name}")
        self.i += 1


def joint_markers(q, frame, stamp):
    """Coloured arrow along each joint axis + label with the current angle."""
    ms = []
    for j, ((p, ax), lab, col) in enumerate(zip(joint_frames(q), JOINT_LABELS, JOINT_COLORS)):
        a0, a1 = p - 0.06 * ax, p + 0.06 * ax
        arrow = Marker()
        arrow.header.frame_id, arrow.header.stamp = frame, stamp
        arrow.ns, arrow.id, arrow.type, arrow.action = "joint_axes", j, Marker.ARROW, Marker.ADD
        arrow.points = [Point(x=float(a0[0]), y=float(a0[1]), z=float(a0[2])),
                        Point(x=float(a1[0]), y=float(a1[1]), z=float(a1[2]))]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.012, 0.025, 0.025
        arrow.pose.orientation.w = 1.0
        arrow.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=1.0)
        text = Marker()
        text.header.frame_id, text.header.stamp = frame, stamp
        text.ns, text.id, text.type, text.action = "joint_labels", j, Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.x, text.pose.position.y, text.pose.position.z = float(a1[0]), float(a1[1]), float(a1[2]) + 0.03
        text.pose.orientation.w = 1.0
        text.scale.z = 0.03
        text.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=1.0)
        text.text = f"{lab}  {math.degrees(q[j]):+.0f}°"
        ms += [arrow, text]
    dot = Marker()                                   # flange (tool0): must sit on the blue path line
    dot.header.frame_id, dot.header.stamp = frame, stamp
    dot.ns, dot.id, dot.type, dot.action = "flange_dot", 0, Marker.SPHERE, Marker.ADD
    fp = fk(q)[:3, 3]
    dot.pose.position.x, dot.pose.position.y, dot.pose.position.z = float(fp[0]), float(fp[1]), float(fp[2])
    dot.pose.orientation.w = 1.0
    dot.scale.x = dot.scale.y = dot.scale.z = 0.02
    dot.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=1.0)
    ms.append(dot)
    return ms


BOX_SIZE = (0.27, 0.22, 0.11)      # approx. cardboard box (from the box marker plate + flap height)


def scene_markers(tr, frame, stamp):
    """Approximate table, box and pedestal for the current take (from the mocap box pose)."""
    if tr.box_pose is None:
        return []
    T = tr.box_pose
    p, R = T[:3, 3], T[:3, :3]
    table_z = p[2] - 0.005                                    # box markers sit ~on the table
    yaw = math.atan2(R[1, 0], R[0, 0])                        # keep only the box's yaw (table is level)
    ms = []

    def mk(i, typ, pos, scale, rgba, yaw_=0.0, ns="scene"):
        m = Marker()
        m.header.frame_id, m.header.stamp = frame, stamp
        m.ns, m.id, m.type, m.action = ns, i, typ, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw_ / 2), math.cos(yaw_ / 2)
        m.scale.x, m.scale.y, m.scale.z = map(float, scale)
        m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
        return m

    ms.append(mk(0, Marker.CUBE, (p[0], p[1], table_z - 0.01), (1.2, 1.2, 0.02), (0.55, 0.45, 0.35, 0.35)))      # table
    ms.append(mk(1, Marker.CUBE, (p[0], p[1], table_z + BOX_SIZE[2] / 2), BOX_SIZE, (0.85, 0.65, 0.40, 0.55), yaw))  # box
    if table_z < -0.02:                                                                                          # pedestal
        ms.append(mk(2, Marker.CYLINDER, (0, 0, table_z / 2), (0.16, 0.16, -table_z), (0.35, 0.35, 0.38, 0.9)))
    return ms


def path_markers(tr, frame, stamp):
    """Flange path of the take (blue, red where IK failed) + take name."""
    path = Marker()
    path.header.frame_id, path.header.stamp = frame, stamp
    path.ns, path.id, path.type, path.action = "flange_path", 0, Marker.LINE_STRIP, Marker.ADD
    path.scale.x = 0.004
    path.pose.orientation.w = 1.0
    for p, ok in zip(tr.path, tr.reachable):
        path.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        path.colors.append(ColorRGBA(r=0.1, g=0.6, b=1.0, a=1.0) if ok else ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0))
    name = Marker()
    name.header.frame_id, name.header.stamp = frame, stamp
    name.ns, name.id, name.type, name.action = "take_name", 1, Marker.TEXT_VIEW_FACING, Marker.ADD
    name.pose.position.z = 1.0
    name.pose.orientation.w = 1.0
    name.scale.z = 0.06
    name.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    name.text = tr.name
    return [path, name] + scene_markers(tr, frame, stamp)


def main():
    rclpy.init()
    node = Player()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
