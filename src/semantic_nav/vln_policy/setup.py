import os
from glob import glob

from setuptools import find_packages, setup

package_name = "vln_policy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml") + glob("config/*.rviz")),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="justinshih",
    maintainer_email="justinshih@gapp.nthu.edu.tw",
    description="Swappable VLN backends (StreamVLN first) with cmd_vel and "
                "nav2 waypoint executors for the Stretch robot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vln_agent_node = vln_policy.vln_agent_node:main",
            "vln_status_monitor = vln_policy.vln_status_monitor:main",
            "vln_viz_node = vln_policy.vln_viz_node:main",
        ],
    },
)
