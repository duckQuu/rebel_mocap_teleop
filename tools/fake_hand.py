#!/usr/bin/env python3
"""Fake mocap hand(s) for testing without OptiTrack: a fake teleop data stream.

Publishes TF  map -> palm / fingertip rigid bodies (like mocap_tf does for real mocap) at 120 Hz, engages the clutch
(retrying until teleop has reached HOME and accepts), then moves each palm in its ARM's own axes (read from TF, so it
works wherever the arm is mounted): hold, down, forward, back, up. The fingertip stays 10 cm from the palm (gripper
open), then curls to 5 cm during the 'forward' part (gripper closes) and opens again at the end.

  --arm left | right | both     both: left hand = rigid_body_1/2, right hand = rigid_body_3/4 (the rig:=both defaults),
                                the right hand runs half a cycle behind so the arms visibly move independently
  --pattern reach | cross       reach (default): hold, down, forward (gripper closes), back, up (gripper opens)
                                cross: hold, up, back, down, back, left, back, right, back (--amp, gripper stays open)
  --output tf | rigid_bodies    tf (default): TF map -> rigid_body_<id>, as mocap_tf would publish it
                                rigid_bodies: mocap4r2_msgs/RigidBodies on /rigid_bodies, like the OptiTrack driver,
                                so mocap_tf converts it (launch with mocap_bridge:=true; needs mocap4r2_msgs sourced)
  --loop                        repeat forever (a continuous stream; Ctrl-C to stop)
  --descend STEP                go down STEP metres (and --descend-forward x STEP forward), hold, again ... (prints
                                "depth NN cm, forward NN cm") up to --max-down, then back: shows where a joint limit
                                or the workspace box stops the arm
  --unstable                    bad mocap: marker jitter, drop-outs (rigid body missing), frozen frames (Motive repeats
                                the last pose), single-frame outliers (marker swap); or set them one by one
  --rotate-finger               close the gripper by turning the fingertip body 150 deg (gripper_mode:=orientation)
  --sim-time                    stamp with Isaac's /clock - needed with target:=isaac (teleop runs on sim time there)

Run after:  ros2 launch rebel_demo teleop.launch.py target:=rviz mocap_bridge:=false            (rig:=left default)
    python3 tools/fake_hand.py
    python3 tools/fake_hand.py --arm both --loop        # with  ... rig:=both
    python3 tools/fake_hand.py --arm both --pattern cross --loop   # up / down / left / right
    python3 tools/fake_hand.py --arm both --sim-time    # with  ... target:=isaac rig:=both  (Isaac or fake_isaac.py)
"""
import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def smooth(t):                                   # 0..1 with zero speed at both ends
    t = min(1.0, max(0.0, t))
    return t * t * (3 - 2 * t)


class Hand:
    """One fake hand: palm + fingertip rigid bodies driving one arm."""

    def __init__(self, node, arm, palm, finger, service, start, delay):
        self.arm, self.palm, self.finger, self.start, self.delay = arm, palm, finger, np.array(start), delay
        self.engage = node.create_client(SetBool, service)
        self.service = service
        self.pending, self.last_try, self.engaged = None, 0.0, False
        self.fwd = self.up = None
        self.gap_until = self.freeze_until = 0.0
        self.frozen_msgs = None
        self.last_name = None

    def try_engage(self):
        if self.engaged:
            return
        if self.pending is not None and self.pending.done():
            if self.pending.result().success:
                self.engaged = True
                print(f"[{self.arm}] engaged")
            self.pending = None
        if self.pending is None and self.engage.service_is_ready() and time.time() - self.last_try > 1.0:
            self.pending, self.last_try = self.engage.call_async(SetBool.Request(data=True)), time.time()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left", choices=["left", "right", "both"])
    ap.add_argument("--service", default=None, help="engage service (default /rebel_teleop/engage, "
                                                      "with --arm both /rebel_teleop_left|right/engage)")
    ap.add_argument("--down", type=float, default=0.10, help="metres down in the arm's base frame")
    ap.add_argument("--forward", type=float, default=0.15, help="metres forward (arm base +x)")
    ap.add_argument("--pattern", default="reach", choices=["reach", "cross"],
                    help="reach: down / forward / back / up; cross: up / down / left / right")
    ap.add_argument("--amp", type=float, default=0.10, help="--pattern cross: metres up / down / left / right")
    ap.add_argument("--output", default="tf", choices=["tf", "rigid_bodies"],
                    help="tf: TF frames (no mocap_tf needed); rigid_bodies: /rigid_bodies like the OptiTrack driver")
    ap.add_argument("--descend", type=float, default=0.0, metavar="STEP",
                    help="progressively lower: go down STEP metres, hold 2 s, again STEP deeper ... up to --max-down "
                         "(replaces the down/forward/back/up sequence; use to find the joint limit)")
    ap.add_argument("--max-down", type=float, default=0.70, help="--descend: deepest point below the start [m]")
    ap.add_argument("--descend-forward", type=float, default=0.3, metavar="RATIO",
                    help="--descend: metres forward per metre of depth (0 = straight down); a hand reaching lower "
                         "also reaches out. Measured: 0 -> limit at 60 cm down, 0.3 -> ~47 cm down / 14 cm forward")
    ap.add_argument("--seg", type=float, default=3.0, help="seconds per movement")
    ap.add_argument("--loop", action="store_true", help="repeat the movement forever")
    ap.add_argument("--noise", type=float, default=0.0003, help="marker noise [m] (0 = perfectly still = frozen)")
    ap.add_argument("--unstable", action="store_true", help="jitter 2 mm, drop-outs, frozen frames, outliers")
    ap.add_argument("--jitter", type=float, default=None, help="extra noise std [m]")
    ap.add_argument("--dropouts", type=float, default=0.0, help="drop-outs per second (0.1-0.8 s long)")
    ap.add_argument("--freezes", type=float, default=0.0, help="frozen-pose periods per second (0.3-1.0 s long)")
    ap.add_argument("--outliers", type=float, default=0.0, help="single-frame 5-10 cm jumps per second")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--rotate-finger", action="store_true",
                    help="close the gripper by rotating the fingertip body 150 deg (gripper_mode:=orientation) "
                         "instead of moving it closer")
    ap.add_argument("--sim-time", action="store_true", help="use Isaac's /clock (target:=isaac)")
    ap.add_argument("--palm", default="rigid_body_1", help="one arm: palm rigid body")
    ap.add_argument("--finger", default="rigid_body_2", help="one arm: fingertip rigid body")
    args = ap.parse_args()
    if args.unstable:
        args.jitter = 0.002 if args.jitter is None else args.jitter
        args.dropouts, args.freezes, args.outliers = args.dropouts or 0.3, args.freezes or 0.15, args.outliers or 0.5
    jitter = args.jitter or 0.0

    rclpy.init()
    n = Node("fake_hand", parameter_overrides=[Parameter("use_sim_time", value=args.sim_time)])
    buf = Buffer()
    TransformListener(buf, n)
    br = TransformBroadcaster(n)
    if args.arm == "both":
        hands = [Hand(n, "left", "rigid_body_1", "rigid_body_2", "/rebel_teleop_left/engage", (0.5, 0.3, 1.3), 0.0),
                 Hand(n, "right", "rigid_body_3", "rigid_body_4", "/rebel_teleop_right/engage", (0.5, -0.3, 1.3),
                      args.seg / 2)]
    else:
        hands = [Hand(n, args.arm, args.palm, args.finger, args.service or "/rebel_teleop/engage", (0.5, 0.3, 1.3), 0.0)]

    # axes of each arm's base frame, expressed in the mocap frame
    for h in hands:
        while h.fwd is None:
            rclpy.spin_once(n, timeout_sec=0.1)
            try:
                tr = buf.lookup_transform("map", f"arm_{h.arm}_base_link", Time()).transform.rotation
                R = quat_to_matrix([tr.x, tr.y, tr.z, tr.w])
                h.fwd, h.left, h.up = R[:, 0], R[:, 1], R[:, 2]
            except Exception:
                pass

    rb_pub, frame = None, 0
    if args.output == "rigid_bodies":
        try:
            from mocap4r2_msgs.msg import RigidBodies, RigidBody
        except ImportError:
            raise SystemExit("--output rigid_bodies needs mocap4r2_msgs: source ~/mocap_ws/install/setup.bash")
        rb_pub = n.create_publisher(RigidBodies, "/rigid_bodies", 10)
    rng = np.random.default_rng(args.seed)
    events = {"drop-outs": 0, "freezes": 0, "outliers": 0}
    def sequence(h):
        """[(name, duration, displacement)] of one hand: the normal movement or the progressive descent."""
        if args.descend > 0:
            k = max(1, int(round(args.max_down / args.descend)))
            step = -h.up * args.descend + h.fwd * args.descend * args.descend_forward   # down and a bit forward
            return [("hold", 3.0, np.zeros(3))] + [x for i in range(1, k + 1) for x in (
                (f"depth {i * args.descend * 100:.0f} cm, forward {i * args.descend * args.descend_forward * 100:.0f} cm",
                 args.seg, step), ("hold", 2.0, np.zeros(3)))] + [
                ("return up", 3 * args.seg, -k * step), ("hold", 3.0, np.zeros(3))]
        if args.pattern == "cross":                   # each move goes out and comes back to the centre
            a = args.amp
            return [("hold", 3.0, np.zeros(3))] + [x for nm, v in (("up", h.up), ("down", -h.up), ("left", h.left),
                                                                    ("right", -h.left)) for x in (
                (nm, args.seg, v * a), ("hold", 1.0, np.zeros(3)), ("back", args.seg, -v * a), ("hold", 1.0, np.zeros(3)))]
        return [("hold", 3.0, np.zeros(3)), ("down", args.seg, -h.up * args.down),
                ("forward", args.seg, h.fwd * args.forward), ("hold", 1.5, np.zeros(3)),
                ("back", args.seg, -h.fwd * args.forward), ("up", args.seg, h.up * args.down),
                ("hold", 2.0, np.zeros(3))]

    cycle = sum(d for _, d, _ in sequence(hands[0]))   # length of one sequence (for --loop)
    t0 = None                                    # movement clock starts once every hand is engaged
    while rclpy.ok():
        for h in hands:
            h.try_engage()
        if t0 is None and all(h.engaged for h in hands):
            t0 = time.time()
        now = n.get_clock().now().to_msg()
        wall, dt = time.time(), 1 / 120
        moving = t0 is not None
        finished = True
        msgs_all = []
        for h in hands:
            t = 0.0 if t0 is None else max(0.0, time.time() - t0 - h.delay)
            if args.loop:
                t %= cycle
            steps = sequence(h)
            acc, seg_start, name = np.zeros(3), 0.0, "end"
            for nm, dur, delta in steps:
                if t >= seg_start + dur:
                    acc = acc + delta
                    seg_start += dur
                    continue
                name = nm
                acc = acc + delta * smooth((t - seg_start) / dur)
                break
            finished = finished and name == "end"
            pos = h.start + acc
            finger = 0.10                            # curl while moving forward, open again in the last hold
            if args.pattern == "cross" or args.descend > 0:
                pass                                 # gripper stays open
            elif name == "forward":
                finger = 0.10 - 0.05 * smooth((t - 3.0 - args.seg) / args.seg)
            elif name in ("hold", "back", "up") and 3.0 + 2 * args.seg < t < cycle - 2.0:
                finger = 0.05
            # real mocap always has ~0.3 mm of noise; a perfectly constant pose would be treated as frozen tracking
            noise = lambda: rng.normal(0, args.noise + jitter, 3)   # noqa: E731
            if moving and wall >= h.gap_until and wall >= h.freeze_until:   # start a new disturbance?
                if rng.random() < args.dropouts * dt:
                    h.gap_until = wall + rng.uniform(0.1, 0.8)
                    events["drop-outs"] += 1
                elif rng.random() < args.freezes * dt:
                    h.freeze_until, h.frozen_msgs = wall + rng.uniform(0.3, 1.0), None
                    events["freezes"] += 1
            spikes = [np.zeros(3), np.zeros(3)]            # marker glitch hits one rigid body
            if moving and rng.random() < args.outliers * dt:
                v = rng.normal(size=3)
                spikes[int(rng.integers(2))] = v / np.linalg.norm(v) * rng.uniform(0.05, 0.10)
                events["outliers"] += 1
            if wall < h.gap_until:
                pass                                 # rigid bodies missing: publish nothing
            elif wall < h.freeze_until and h.frozen_msgs is not None:
                for m in h.frozen_msgs:              # Motive repeats the last pose, only the stamp changes
                    m.header.stamp = now
                msgs_all += h.frozen_msgs
            else:
                # rotate-finger mode: fingertip stays 10 cm away and turns about x by 0 (open) .. 150 deg (closed)
                f_pos = pos + np.array([0.10 if args.rotate_finger else finger, 0, 0])
                ang = math.radians(150) * (0.10 - finger) / 0.05 if args.rotate_finger else 0.0
                f_rot = (math.sin(ang / 2), 0.0, 0.0, math.cos(ang / 2))
                msgs = []
                for child, p, r in ((h.palm, pos + spikes[0] + noise(), (0.0, 0.0, 0.0, 1.0)),
                                    (h.finger, f_pos + spikes[1] + noise(), f_rot)):
                    m = TransformStamped()
                    m.header.stamp, m.header.frame_id, m.child_frame_id = now, "map", child
                    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, p)
                    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = r
                    msgs.append(m)
                msgs_all += msgs
                if wall < h.freeze_until:
                    h.frozen_msgs = msgs
            if name != h.last_name and moving:
                print(f"[{h.arm}] {t:5.1f}s  {name}")
                h.last_name = name
        if msgs_all:
            if rb_pub is None:
                br.sendTransform(msgs_all)
            else:                                    # same poses as the OptiTrack driver's message
                rb = RigidBodies()
                rb.header.stamp, rb.header.frame_id = now, "map"
                rb.frame_number = frame
                for m in msgs_all:
                    b = RigidBody()
                    b.rigid_body_name = m.child_frame_id.replace("rigid_body_", "")   # mocap_tf adds the prefix back
                    tr = m.transform.translation
                    b.pose.position.x, b.pose.position.y, b.pose.position.z = tr.x, tr.y, tr.z
                    b.pose.orientation = m.transform.rotation
                    rb.rigidbodies.append(b)
                rb_pub.publish(rb)
            frame += 1
        if moving and finished and not args.loop:
            break
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(max(0.0, 1 / 120 - (time.time() - wall)))  # 120 Hz like Motive
    print("done", events)
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
