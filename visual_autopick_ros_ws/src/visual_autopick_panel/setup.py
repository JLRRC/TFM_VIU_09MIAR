from setuptools import find_packages, setup

package_name = "visual_autopick_panel"

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
    description="Qt panels for visual_autopick_ros_ws.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "simple_panel         = visual_autopick_panel.simple_panel:main",
            "step_panel           = visual_autopick_panel.step_panel:main",
            "autopick_panel       = visual_autopick_panel.autopick_panel:main",
            "step_by_step_panel   = visual_autopick_panel.step_by_step_panel:main",
        ],
    },
)
