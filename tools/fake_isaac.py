#!/usr/bin/env python3
"""Fake Isaac Sim for testing the rig without the simulator: stands in for dual_arm_rig_ros2_bridge.py.

  /clock                           simulation time (starts at 0, runs at real speed)
  /dual_arm/isaac_joint_commands   subscribed: position targets by joint name (like the Articulation Controller;
                                   unknown names are reported, as Isaac would)
  /dual_arm/isaac_joint_states     published at 60 Hz, stamped with simulation time: every rig joint, starting at 0
                                   (Isaac's start pose), each following its target with a short drive lag

Run instead of Isaac, then launch teleop with target:=isaac:
    python3 tools/fake_isaac.py
    ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both mocap_bridge:=false
"""
import argparse
import math
import time

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

ARM = [f"joint{i}" for i in range(1, 7)] + ["xeg32_left_carriage_joint", "xeg32_right_carriage_joint",
                                              "xeg32_camera_tilt_joint"]
RIG_JOINTS = ["lift_joint", "slider_left_joint", "slider_right_joint"] + \
             [f"arm_{side}_{j}" for side in ("left", "right") for j in ARM]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=60.0)
    ap.add_argument("--tau", type=float, default=0.05, help="drive lag [s] (first order); 0 = joints jump to targets")
    args = ap.parse_args()

    rclpy.init()
    n = Node("fake_isaac")
    clock = n.create_publisher(Clock, "/clock", 10)
    pub = n.create_publisher(JointState, "/dual_arm/isaac_joint_states", 10)
    pos = {j: 0.0 for j in RIG_JOINTS}
    target = dict(pos)
    unknown = set()

    def on_cmd(msg):
        for name, p in zip(msg.name, msg.position):
            if name in target:
                target[name] = p
            elif name not in unknown:
                unknown.add(name)
                n.get_logger().error(f"Articulation Controller: unknown joint '{name}' (ignored)")

    n.create_subscription(JointState, "/dual_arm/isaac_joint_commands", on_cmd, 10)
    n.get_logger().info(f"fake Isaac: {len(RIG_JOINTS)} rig joints at 0, sim clock running")
    t0, dt = time.time(), 1.0 / args.rate
    a = 1.0 if args.tau <= 0 else 1.0 - math.exp(-dt / args.tau)
    while rclpy.ok():
        start = time.time()
        sim = start - t0
        stamp = TimeMsg(sec=int(sim), nanosec=int((sim % 1) * 1e9))
        clock.publish(Clock(clock=stamp))
        for j in pos:
            pos[j] += a * (target[j] - pos[j])
        m = JointState()
        m.header.stamp = stamp
        m.name = list(pos)
        m.position = [float(v) for v in pos.values()]
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(max(0.0, dt - (time.time() - start)))
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
