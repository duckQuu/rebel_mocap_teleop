"""teleop_keyboard node - SPACE pauses / resumes the arm teleop (calls ~/toggle of the teleop node(s)).

Run it in its OWN terminal (ros2 launch does not give nodes a keyboard) and keep that terminal focused:
  ros2 run rebel_demo teleop_keyboard
  SPACE  pause <-> resume      q / Ctrl-C  quit
Only the arm teleop is paused; Isaac Sim keeps running and the gripper node is not affected. On resume the robot's
last pose and your hand's pose at that moment are the new reference (see teleop_node.py).

Parameters
  teleop [/rebel_teleop]   teleop node(s), comma-separated; rig:=both: "/rebel_teleop_left,/rebel_teleop_right"
                           SPACE: if any of them runs -> pause all, otherwise resume all (never swaps arms)
  debounce [0.3]                   s; key repeat from a held SPACE inside this time is ignored
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class KeyDebounce:
    """Accepts a key press only if the previous accepted one is at least `gap` seconds old."""

    def __init__(self, gap):
        self.gap = gap
        self.last = None

    def __call__(self, now):
        if self.last is not None and now - self.last < self.gap:
            return False
        self.last = now
        return True


class Keyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        nodes = [n.strip().rstrip("/") for n in self.declare_parameter("teleop", "/rebel_teleop").value.split(",")]
        self.debounce = KeyDebounce(float(self.declare_parameter("debounce", 0.3).value))
        self.engage_clients = {n: self.create_client(SetBool, f"{n}/engage") for n in nodes}
        self.running = {n: False for n in nodes}          # from each node's latched ~/engaged topic
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for n in nodes:
            self.create_subscription(Bool, f"{n}/engaged", lambda msg, n=n: self.on_state(n, msg), latched)

    def on_state(self, node, msg):
        self.running[node] = msg.data
        print(f"\r[{node}] {'RUNNING' if msg.data else 'PAUSED '}   (SPACE = pause / resume, q = quit)", flush=True)

    def toggle(self):
        engage = not any(self.running.values())            # any arm running -> pause all, else resume all
        for node, client in self.engage_clients.items():
            if not client.service_is_ready():
                print(f"\r[{node}] engage service not available - is the teleop node running?", flush=True)
                continue
            client.call_async(SetBool.Request(data=engage))  # the new state is printed via <node>/engaged


def main():
    if not sys.stdin.isatty():
        sys.exit("teleop_keyboard needs a terminal: run  ros2 run rebel_demo teleop_keyboard  in its own terminal")
    rclpy.init()
    node = Keyboard()
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())                  # single keypresses, no Enter
        print("SPACE = pause / resume the arm teleop, q = quit")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == " " and node.debounce(node.get_clock().now().nanoseconds * 1e-9):
                    node.toggle()
                elif ch in ("q", "\x03"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
