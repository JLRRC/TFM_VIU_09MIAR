from setuptools import setup, find_packages
import os
from glob import glob

package_name = "ur5_visual_autopick"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="laboratorio",
    maintainer_email="laboratorio@localhost",
    description="Visual autopick panel for UR5 RG2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "visual_autopick_node = ur5_visual_autopick.visual_autopick_node:main",
            "ros_trace_logger     = ur5_visual_autopick.ros_trace_logger:main",
        ],
    },
)
