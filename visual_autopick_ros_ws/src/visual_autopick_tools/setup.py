from setuptools import find_packages, setup

package_name = "visual_autopick_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="laboratorio",
    maintainer_email="laboratorio@example.com",
    description="Tools and pick executors for visual_autopick_ros_ws.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "world_tf_publisher = visual_autopick_tools.world_tf_publisher:main",
            "object_pose_provider = visual_autopick_tools.object_pose_provider:main",
            "step_pick_server = visual_autopick_tools.step_pick_server:main",
            "simple_direct_pick = visual_autopick_tools.simple_direct_pick:main",
            "simple_moveit_pick = visual_autopick_tools.simple_moveit_pick:main",
            "tf_diagnostics = visual_autopick_tools.tf_diagnostics:main",
        ],
    },
)
