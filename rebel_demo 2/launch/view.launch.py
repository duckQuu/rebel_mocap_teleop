# RViz visualization of the demo trajectories (no robot, no controller needed).
#   ros2 launch rebel_demo view.launch.py
#   ros2 launch rebel_demo view.launch.py only:=left rate:=2.0 loop:=true joints:=false
#   ros2 launch rebel_demo view.launch.py play:=false        # joint sliders instead of the player
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("rebel_demo")
    with open(os.path.join(share, "urdf", "igus_rebel_6dof.urdf")) as f:
        urdf = f.read()
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("play", default_value="true", description="run the player (false = sliders)"),
        DeclareLaunchArgument("trajectory_dir", default_value=os.path.join(share, "trajectories")),
        DeclareLaunchArgument("only", default_value="all", description="only files containing this text"),
        DeclareLaunchArgument("rate", default_value="1.0", description="playback speed factor"),
        DeclareLaunchArgument("loop", default_value="false"),
        DeclareLaunchArgument("joints", default_value="true", description="show joint axes + angles"),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}], output="screen"),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", os.path.join(share, "rviz", "rebel_demo.rviz")]),
        Node(package="rebel_demo", executable="player", output="screen",
             condition=IfCondition(arg("play")),
             parameters=[{
                 "trajectory_dir": arg("trajectory_dir"),
                 "only": arg("only"),
                 "rate": ParameterValue(arg("rate"), value_type=float),
                 "loop": ParameterValue(arg("loop"), value_type=bool),
                 "joint_markers": ParameterValue(arg("joints"), value_type=bool),
             }]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             condition=UnlessCondition(arg("play"))),
    ])
