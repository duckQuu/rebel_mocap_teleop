"""sender node - sends ONE demo trajectory to a joint_trajectory_controller
(irc_ros mock hardware in RViz, or the real igus ReBeL).

What it does
  1. waits for the robot's current joint angles on /joint_states
  2. builds the program: slow smooth move current pose -> demo start, then the demo,
     slowed so no joint exceeds speed_fraction x 45 deg/s; posture switches become slow moves
  3. checks joint limits, prints a summary
  4. sends it as a FollowJointTrajectory goal ONLY if execute:=true (default: dry run)

Parameters
  trajectory   CSV file name or path (name is looked up in trajectory_dir), e.g. take_v2_left_flaps_001
  trajectory_dir
  action       /joint_trajectory_controller/follow_joint_trajectory
  speed_fraction  0.0-1.0 of the 45 deg/s joint limit (default 0.5)
  execute      false = dry run (print only), true = send to the controller

  ros2 run rebel_demo sender --ros-args -p trajectory:=take_v2_left_flaps_001 -p execute:=true
"""
import math
import os
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .kinematics import JOINT_NAMES
from .player_node import default_dir
from .trajectory_io import Trajectory, robot_program


def resolve(name, directory):
    if os.path.isfile(name):
        return name
    for cand in (name, name + ".csv", name + "_clean_30hz.csv"):
        p = os.path.join(directory, cand)
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(f"trajectory '{name}' not found in {directory}")


class Sender(Node):
    def __init__(self):
        super().__init__("rebel_demo_sender")
        p = self.declare_parameter
        self.traj_name = p("trajectory", "").value
        self.dir = p("trajectory_dir", default_dir()).value
        self.action_name = p("action", "/joint_trajectory_controller/follow_joint_trajectory").value
        self.speed_fraction = float(p("speed_fraction", 0.5).value)
        self.execute = bool(p("execute", False).value)
        self.current = None
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.client = ActionClient(self, FollowJointTrajectory, self.action_name)

    def on_js(self, msg):
        if all(n in msg.name for n in JOINT_NAMES):
            self.current = [msg.position[msg.name.index(n)] for n in JOINT_NAMES]

    def run(self):
        if not self.traj_name:
            self.get_logger().error("set the parameter 'trajectory' (e.g. take_v2_left_flaps_001)")
            return 1
        path = resolve(self.traj_name, self.dir)
        tr = Trajectory(path)

        self.get_logger().info("waiting for /joint_states ...")
        t_end = self.get_clock().now().nanoseconds + 5e9
        while self.current is None and self.get_clock().now().nanoseconds < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current is None:
            self.get_logger().error("no /joint_states received - is the robot / mock hardware running?")
            return 1

        import numpy as np
        t, q, info = robot_program(tr, np.asarray(self.current), self.speed_fraction)
        self.get_logger().info(f"{tr.name}: {info}")
        if not info["within_limits"]:
            self.get_logger().error("trajectory comes within 2 deg of a joint limit - not sending")
            return 1
        if not self.execute:
            self.get_logger().info("dry run (execute:=false) - nothing sent")
            return 0

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"action server {self.action_name} not available")
            return 1
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        for ti, qi in zip(t, q):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in qi]
            sec = int(math.floor(ti))
            pt.time_from_start = Duration(sec=sec, nanosec=int(round((ti - sec) * 1e9)) % 1000000000)
            goal.trajectory.points.append(pt)
        self.get_logger().info(f"sending {len(t)} points, {t[-1]:.1f} s ...")
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error("goal rejected by the controller")
            return 1
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        code = res.result().result.error_code
        self.get_logger().info("done" if code == 0 else f"controller error code {code}")
        return 0 if code == 0 else 1


def main():
    rclpy.init()
    node = Sender()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 1
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
