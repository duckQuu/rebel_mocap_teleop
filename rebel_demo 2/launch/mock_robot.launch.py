# igus ReBeL with the REAL controller stack on MOCK hardware (needs the igus irc_ros packages built
# in your workspace: github.com/CommonplaceRobotics/iRC_ROS). Then, in a second terminal:
#   ros2 run rebel_demo sender --ros-args -p trajectory:=take_v2_left_flaps_001 -p execute:=true
# For the real robot use hardware_protocol:=cri (or cprcanv2) - same sender command.
#   ros2 launch rebel_demo mock_robot.launch.py
#   ros2 launch rebel_demo mock_robot.launch.py hardware_protocol:=cri     # REAL robot
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory("irc_ros_bringup"), "launch", "rebel.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("hardware_protocol", default_value="mock_hardware",
                              description="mock_hardware | cri | cprcanv2"),
        DeclareLaunchArgument("rebel_version", default_value="01", description="pre | 00 | 01"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={
                "hardware_protocol": LaunchConfiguration("hardware_protocol"),
                "rebel_version": LaunchConfiguration("rebel_version"),
            }.items()),
    ])
