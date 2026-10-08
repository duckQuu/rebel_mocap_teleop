# Hand teleoperation: OptiTrack rigid body (mocap4ros2_optitrack TF) -> IK -> /isaac_joint_commands.
#
# Start mocap4ros2_optitrack first (and activate it), then:
#   ros2 launch rebel_demo teleop.launch.py target:=rviz      # RViz only, no Isaac (safe first test)
#   ros2 launch rebel_demo teleop.launch.py target:=isaac     # Isaac Sim publishes /joint_states
#   ros2 service call /rebel_teleop/engage std_srvs/srv/SetBool "{data: true}"
#
# robot_xyz / robot_ypr = where the robot base sits in the mocap "map" frame (Z-up, metres, radians).
# This publishes the static TF  map -> world  (the URDF already has world -> base_link).
#
# Robot model: urdf/rebel_xeg32.urdf.xacro = igus rebel2 arm + XEG-32 from igus_rebel_description
# (the same macros as the Isaac dual-arm rig). mesh_lod:=low|full picks the mesh detail.
import os

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory("rebel_demo")
    a = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    rig = a("rig")
    if rig:                                     # one arm of the Isaac dual-arm rig (dual_arm_rig_ros2_bridge.py)
        if rig not in ("left", "right"):
            raise RuntimeError(f"rig:={rig} - use rig:=left or rig:=right ('' = single arm)")
        if a("target") != "isaac":
            raise RuntimeError("rig:=left|right needs target:=isaac (Isaac publishes the rig's joint states)")
        urdf = xacro.process_file(
            os.path.join(get_package_share_directory("igus_rebel_description"), "urdf", a("rig_xacro")),
            mappings={"mesh_lod": a("mesh_lod"), "include_ros2_control": "false",
                      "lock_gripper_joints": a("lock_gripper")}).toxml()
        arm = f"arm_{rig}_"
        base_frame, grip_prefix = f"{arm}base_link", f"{arm}xeg32_"
        command_topic = a("command_topic") or "/dual_arm/isaac_joint_commands"
        js_topic = "/dual_arm/isaac_joint_states"
    else:                                       # single rebel2 + XEG-32
        urdf = xacro.process_file(os.path.join(share, "urdf", "rebel_xeg32.urdf.xacro"),
                                  mappings={"mesh_lod": a("mesh_lod"), "lock_gripper": a("lock_gripper")}).toxml()
        arm, base_frame, grip_prefix = "", "base_link", "xeg32_"
        command_topic = a("command_topic") or "/isaac_joint_commands"
        js_topic = "/joint_states"
    rviz_only = a("target") == "rviz"
    x, y, z = a("robot_xyz").split()
    yaw, pitch, roll = a("robot_ypr").split()
    nodes = [
        Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_world",
             arguments=["--x", x, "--y", y, "--z", z, "--yaw", yaw, "--pitch", pitch, "--roll", roll,
                        "--frame-id", a("mocap_frame"), "--child-frame-id", "world"]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}], output="screen",
             remappings=[("joint_states", js_topic)]),
        Node(package="rebel_demo", executable="teleop", output="screen",
             parameters=[{
                 "base_frame": base_frame,
                 "joint_prefix": arm,
                 "joint_states_topic": js_topic,
                 "hand_frame": a("hand_frame"),
                 "mode": a("mode"),
                 "scale": float(a("scale")),
                 "orientation": a("orientation").lower() == "true",
                 "cutoff_hz": float(a("cutoff_hz")),
                 "max_joint_speed_deg": float(a("max_joint_speed_deg")),
                 "command_topic": command_topic,
                 "publish_joint_states": rviz_only,           # Isaac publishes the joint states itself
             }]),
    ]
    if a("gripper").lower() == "true":          # second rigid body -> HIWIN XEG-32 jaw opening
        nodes.append(Node(package="rebel_demo", executable="gripper", output="screen",
                          parameters=[{
                              "gripper_frame": a("gripper_frame"),
                              "ref_frame": a("gripper_ref_frame") or a("hand_frame"),
                              "threshold": float(a("gripper_threshold")),
                              "hysteresis": float(a("gripper_hysteresis")),
                              "invert": a("gripper_invert").lower() == "true",
                              "joint_prefix": grip_prefix,
                              "command_topic": command_topic,
                              "publish_joint_states": rviz_only,
                          }]))
    if a("mocap_bridge").lower() == "true":     # mocap4r2 driver publishes /rigid_bodies only, no TF
        nodes.append(Node(package="rebel_demo", executable="mocap_tf", output="screen"))
    if a("rviz").lower() == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", output="screen",
                          arguments=["-d", os.path.join(share, "rviz", "rebel_demo.rviz")]))
    return nodes


def generate_launch_description():
    D = DeclareLaunchArgument
    return LaunchDescription([
        D("target", default_value="rviz", description="rviz (teleop also publishes /joint_states) | isaac"),
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
        D("rviz", default_value="true"),
        D("mesh_lod", default_value="low", description="low (decimated meshes, light RViz) | full (CAD)"),
        D("lock_gripper", default_value="false", description="true = XEG joints fixed in the model"),
        D("gripper", default_value="true", description="control the XEG-32 with a second rigid body"),
        D("gripper_frame", default_value="rigid_body_2", description="TF name of the gripper-control rigid body"),
        D("gripper_ref_frame", default_value="",
          description="opening = distance gripper_frame <-> this frame ('' = hand_frame)"),
        D("gripper_threshold", default_value="0.08", description="palm <-> fingertip switching distance [m]"),
        D("gripper_hysteresis", default_value="0.02", description="dead band [m] around the threshold (anti-chatter)"),
        D("gripper_invert", default_value="false", description="true = far closes, near opens"),
        D("rig", default_value="",
          description="'' = single arm | left | right = that arm of the Isaac dual-arm rig (needs target:=isaac)"),
        D("rig_xacro", default_value="dual_arm_rig_v2.urdf.xacro",
          description="rig model in igus_rebel_description/urdf (dual_arm_rig_v2 or dual_arm_rig)"),
        D("command_topic", default_value="",
          description="'' = /isaac_joint_commands (single) or /dual_arm/isaac_joint_commands (rig)"),
        OpaqueFunction(function=setup),
    ])
