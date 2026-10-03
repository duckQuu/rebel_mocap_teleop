import os
from glob import glob

from setuptools import setup

package_name = "rebel_demo"


def tree(src):
    """(install_dir, [files]) entries for every sub-folder of src (keeps the folder layout)."""
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join("share", package_name, root), [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "trajectories"), glob("trajectories/*.csv")),
    ] + tree("meshes"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rebel_demo maintainer",
    maintainer_email="user@example.com",
    description="igus ReBeL mocap demo player and trajectory sender",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "player = rebel_demo.player_node:main",
            "sender = rebel_demo.sender_node:main",
            "teleop = rebel_demo.teleop_node:main",
            "mocap_tf = rebel_demo.mocap_tf_node:main",
        ],
    },
)
