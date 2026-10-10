import os
from glob import glob

from setuptools import setup

package_name = "rebel_demo"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rebel_demo maintainer",
    maintainer_email="user@example.com",
    description="igus ReBeL hand teleoperation from OptiTrack mocap (RViz / Isaac Sim)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "teleop = rebel_demo.teleop_node:main",
            "mocap_tf = rebel_demo.mocap_tf_node:main",
            "gripper = rebel_demo.gripper_node:main",
            "recorder = rebel_demo.recorder_node:main",
            "joint_merger = rebel_demo.joint_merger_node:main",
            "teleop_keyboard = rebel_demo.keyboard_node:main",
        ],
    },
)
