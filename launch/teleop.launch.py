# Hand teleoperation on the Isaac dual-arm rig: OptiTrack rigid bodies (mocap4ros2_optitrack) -> IK -> joint commands.
#
# Start mocap4ros2_optitrack first (and activate it), then:
#   ros2 launch rebel_demo teleop.launch.py target:=rviz               # RViz only, no Isaac (safe first test)
#   ros2 launch rebel_demo teleop.launch.py target:=isaac rviz:=false  # Isaac playing, rig bridge scripts run
#   ros2 service call /rebel_teleop/engage std_srvs/srv/SetBool "{data: true}"
# rig:=left (default) | right = one arm of the rig; rig:=both = two hands, engage /rebel_teleop_left|right/engage.
#
# robot_xyz / robot_ypr = where the rig's "world" sits in the mocap "map" frame (Z-up, metres, radians).
# This publishes the static TF  map -> world.
#
# Robot model: igus_rebel_description/urdf/<rig_xacro> (dual_arm_rig_v2 by default), the same model Isaac uses.
# mesh_lod:=low|full picks the mesh detail.
import os
import xml.etree.ElementTree as ET

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def movable_joints(urdf):
    """Names of all non-mimic movable joints in a URDF string (robot_state_publisher needs a value for each)."""
    root = ET.fromstring(urdf)
    return [j.get("name") for j in root.iter("joint")
            if j.get("type") in ("revolute", "prismatic", "continuous") and j.find("mimic") is None]


def setup(context):
    share = get_package_share_directory("rebel_demo")
    a = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    rig = a("rig")
    if rig not in ("left", "right", "both"):
        raise RuntimeError(f"rig:={rig} - use rig:=left, right or both")
    if rig == "both" and a("joint_names"):
        raise RuntimeError("joint_names is for one arm only; rig:=both uses arm_left_/arm_right_ joint1..6")
    isaac = a("target") == "isaac"
    urdf = xacro.process_file(
        os.path.join(get_package_share_directory("igus_rebel_description"), "urdf", a("rig_xacro")),
        mappings={"mesh_lod": a("mesh_lod"), "include_ros2_control": "false",
                  "lock_gripper_joints": a("lock_gripper")}).toxml()
    # Isaac (dual_arm_rig_ros2_bridge.py) listens on the command topic and publishes the whole rig's joint states.
    command_topic = a("command_topic") or "/dual_arm/isaac_joint_commands"
    js_topic = "/dual_arm/isaac_joint_states"
    sides = ["left", "right"] if rig == "both" else [rig]
    arms = []
    for side in sides:
        hand = a(f"{side}_hand_frame") if rig == "both" else a("hand_frame")
        arms.append({
            "suffix": f"_{side}" if rig == "both" else "",     # node names: rebel_teleop_left / rebel_teleop
            "prefix": f"arm_{side}_", "base_frame": f"arm_{side}_base_link", "grip_prefix": f"arm_{side}_xeg32_",
            "hand": hand,
            "finger": a(f"{side}_gripper_frame") if rig == "both" else a("gripper_frame"),
            "ref": hand if rig == "both" else (a("gripper_ref_frame") or hand),
        })
    # Isaac's bridge stamps joint states with SIM time and publishes /clock; all nodes must use that clock, otherwise
    # the arm TF (sim time) and the hand TF (PC time) never share a time and teleop holds at HOME. RViz-only: PC time.
    sim_time = (a("use_sim_time") or ("true" if isaac else "false")).lower() == "true"
    x, y, z = a("robot_xyz").split()
    yaw, pitch, roll = a("robot_ypr").split()
    nodes = [
        SetParameter(name="use_sim_time", value=sim_time),
        Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_world",
             arguments=["--x", x, "--y", y, "--z", z, "--yaw", yaw, "--pitch", pitch, "--roll", roll,
                        "--frame-id", a("mocap_frame"), "--child-frame-id", "world"]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}], output="screen",
             remappings=[("joint_states", js_topic)]),
    ]
    # Every arm / gripper publishes on its own topic and joint_merger sends ONE combined message per cycle (Isaac's
    # Subscribe Joint State keeps only the latest message, so separate messages would take turns).
    merge_inputs = []
    for arm in arms:
        teleop_name, grip_name = f"rebel_teleop{arm['suffix']}", f"rebel_gripper{arm['suffix']}"
        teleop_out, grip_out = f"/{teleop_name}/joint_commands", f"/{grip_name}/joint_commands"
        nodes.append(Node(package="rebel_demo", executable="teleop", name=teleop_name, output="screen",
                          parameters=[{
                              "base_frame": arm["base_frame"],
                              "joint_prefix": arm["prefix"],
                              "joint_names": a("joint_names"),
                              "joint_states_topic": js_topic,
                              "hand_frame": arm["hand"],
                              "mode": a("mode"),
                              "scale": float(a("scale")),
                              "orientation": a("orientation").lower() == "true",
                              "cutoff_hz": float(a("cutoff_hz")),
                              "max_joint_speed_deg": float(a("max_joint_speed_deg")),
                              "limit_warn_mm": float(a("limit_warn_mm")),
                              "workspace_min": [float(v) for v in a("workspace_min").split()],
                              "workspace_max": [float(v) for v in a("workspace_max").split()],
                              "command_topic": teleop_out,
                              "publish_joint_states": False,
                          }]))
        merge_inputs.append(teleop_out)
        if a("gripper").lower() == "true":      # fingertip rigid body -> HIWIN XEG-32 open / close
            nodes.append(Node(package="rebel_demo", executable="gripper", name=grip_name, output="screen",
                              parameters=[{
                                  "gripper_frame": arm["finger"],
                                  "ref_frame": arm["ref"],
                                  "mode": a("gripper_mode"),
                                  "open_angle_deg": float(a("gripper_open_angle")),
                                  "close_angle_deg": float(a("gripper_close_angle")),
                                  "threshold": float(a("gripper_threshold")),
                                  "hysteresis": float(a("gripper_hysteresis")),
                                  "invert": a("gripper_invert").lower() == "true",
                                  "joint_prefix": arm["grip_prefix"],
                                  "command_topic": grip_out,
                                  "publish_joint_states": False,
                              }]))
            merge_inputs.append(grip_out)
    # Isaac: merged commands go to Isaac. RViz-only: the merger plays Isaac and publishes the joint states, with the
    # joints nobody commands (lift, sliders, other arm) at 0.
    nodes.append(Node(package="rebel_demo", executable="joint_merger", output="screen",
                      parameters=[{"inputs": ",".join(merge_inputs),
                                   "output": command_topic if isaac else js_topic,
                                   "defaults": "" if isaac else ",".join(movable_joints(urdf))}]))
    if a("mocap_bridge").lower() == "true":     # mocap4r2 driver publishes /rigid_bodies only, no TF
        nodes.append(Node(package="rebel_demo", executable="mocap_tf", output="screen"))
    if a("rviz").lower() == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", output="screen",
                          arguments=["-d", os.path.join(share, "rviz", "rebel_demo.rviz")]))
    return nodes


def generate_launch_description():
    D = DeclareLaunchArgument
    return LaunchDescription([
        D("target", default_value="rviz", description="rviz (no Isaac: joint states are faked) | isaac"),
        D("hand_frame", default_value="rigid_body_1",
          description="TF name of your hand: rigid_body_<Motive streaming ID> when mocap_bridge is on"),
        D("mocap_bridge", default_value="true", description="convert /rigid_bodies to TF (mocap4r2 driver)"),
        D("mocap_frame", default_value="map", description="Z-up mocap frame from mocap4ros2_optitrack"),
        D("robot_xyz", default_value="0 0 0", description="robot base position in mocap_frame [m]"),
        D("robot_ypr", default_value="0 0 0", description="robot base yaw pitch roll in mocap_frame [rad]"),
        D("mode", default_value="relative", description="relative (clutch) | absolute"),
        D("scale", default_value="1.0"),
        D("orientation", default_value="false", description="true = follow hand rotation too"),
        D("cutoff_hz", default_value="4.0", description="low-pass cutoff on the hand pose"),
        D("max_joint_speed_deg", default_value="45.0"),
        D("limit_warn_mm", default_value="5", description="warn 'joint limit reached' when the IK is this far [mm] off the target"),
        D("workspace_min", default_value="-0.7 -0.7 -0.45", description="IK target clamp min x y z in the arm base frame [m]"),
        D("workspace_max", default_value="0.7 0.7 0.95",
          description="IK target clamp max x y z [m]; the box is widened to include the pose at engage"),
        D("rviz", default_value="true"),
        D("mesh_lod", default_value="low", description="low (decimated meshes, light RViz) | full (CAD)"),
        D("lock_gripper", default_value="false", description="true = XEG joints fixed in the model"),
        D("gripper", default_value="true", description="control the XEG-32 with a second rigid body"),
        D("gripper_frame", default_value="rigid_body_2", description="TF name of the gripper-control rigid body"),
        D("gripper_ref_frame", default_value="",
          description="opening = distance gripper_frame <-> this frame ('' = hand_frame)"),
        D("gripper_mode", default_value="distance",
          description="distance = fingertip <-> palm distance | orientation = their relative rotation angle"),
        D("gripper_open_angle", default_value="30", description="orientation mode: open below this angle [deg]"),
        D("gripper_close_angle", default_value="100", description="orientation mode: closed above this angle [deg]"),
        D("gripper_threshold", default_value="0.08", description="palm <-> fingertip switching distance [m]"),
        D("gripper_hysteresis", default_value="0.02", description="dead band [m] around the threshold (anti-chatter)"),
        D("gripper_invert", default_value="false", description="true = far closes, near opens"),
        D("rig", default_value="left",
          description="left | right = that arm of the Isaac dual-arm rig | both = both arms, two hands"),
        D("left_hand_frame", default_value="rigid_body_1", description="rig:=both: left palm rigid body"),
        D("left_gripper_frame", default_value="rigid_body_2", description="rig:=both: left fingertip rigid body"),
        D("right_hand_frame", default_value="rigid_body_3", description="rig:=both: right palm rigid body"),
        D("right_gripper_frame", default_value="rigid_body_4", description="rig:=both: right fingertip rigid body"),
        D("rig_xacro", default_value="dual_arm_rig_v2.urdf.xacro",
          description="rig model in igus_rebel_description/urdf (dual_arm_rig_v2 or dual_arm_rig)"),
        D("joint_names", default_value="",
          description="6 comma-separated arm joint names (base to wrist) sent to Isaac; '' = arm_<side>_joint1..6"),
        D("use_sim_time", default_value="",
          description="'' = auto (true with target:=isaac, Isaac publishes /clock) | true | false"),
        D("command_topic", default_value="",
          description="'' = /dual_arm/isaac_joint_commands (must match Isaac's Subscribe Joint State node)"),
        OpaqueFunction(function=setup),
    ])
