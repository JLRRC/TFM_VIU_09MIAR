#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/setup.py
"""Setuptools entry point for the tfm_orchestrator package."""

from setuptools import setup

package_name = "tfm_orchestrator"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="laboratorio",
    maintainer_email="jesus.lozano.rodriguez@gmail.com",
    description="Pick & place orchestrator node — hosts PickPlace.action.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Canonical orchestrator desde F9: LifecycleNode con
            # configure/activate/deactivate/cleanup/shutdown.
            "pick_orchestrator_lifecycle = tfm_orchestrator.pick_orchestrator_lifecycle_node:main",
        ],
    },
)
