import os
from glob import glob

from setuptools import find_packages, setup

package_name = "semantic_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="justinshih",
    maintainer_email="justinshih@gapp.nthu.edu.tw",
    description="Open-set VLM (Grounding DINO) perception for action-aware "
                "navigation (arXiv:2310.08873).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grounding_dino_node = semantic_perception.grounding_dino_node:main",
            "static_region_publisher = "
            "semantic_perception.static_region_publisher:main",
            "detection_viz_node = semantic_perception.detection_viz_node:main",
        ],
    },
)
