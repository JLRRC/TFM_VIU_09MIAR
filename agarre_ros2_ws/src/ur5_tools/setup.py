# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/setup.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/setup.py
# Summary: Setuptools setup for ur5_tools package.
"""Setuptools entry point for the ur5_tools package."""

from setuptools import setup

package_name = "ur5_tools"

setup(
    name=package_name,
    version="0.0.0",
    # F11 (auditoría 2026-05-10): subpaquetes namespace por dominio.
    # Los archivos físicos siguen en ``ur5_tools/`` top-level por
    # compat — los subpaquetes solo re-exportan. F11 iter 2 (futuro)
    # moverá los archivos físicamente.
    packages=[
        package_name,
        f"{package_name}.geometry",
        f"{package_name}.planning",
        f"{package_name}.gripper",
        f"{package_name}.state",
        f"{package_name}.diagnostics",
        f"{package_name}.bridges",
        f"{package_name}.utils",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="laboratorio",
    maintainer_email="jesus.lozano.rodriguez@gmail.com",
    description="Tools for UR5 simulation (planning, gripper attach, geometry, state, evidence).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller_bootstrap = ur5_tools.controller_bootstrap:main",
            "evidence_logger = ur5_tools.evidence_logger:main",
            "gz_pose_bridge = ur5_tools.gz_pose_bridge:main",
            "gz_ros_control_guard = ur5_tools.gz_ros_control_guard:main",
            "gripper_attach_backend = ur5_tools.gripper_attach_backend:main",
            "planning_scene_sync = ur5_tools.planning_scene_sync:main",
            "plan_to_pose_server = ur5_tools.plan_to_pose_server:main",
            "release_objects_service = ur5_tools.release_objects_service:main",
            "system_state_manager = ur5_tools.system_state_manager:main",
            "world_tf_publisher = ur5_tools.world_tf_publisher:main",
            "tf_probe = ur5_tools.tf_probe:main",
            "clock_probe = ur5_tools.clock_probe:main",
            "jt_smoke_test = ur5_tools.jt_smoke_test:main",
            "tf_geometry_service = ur5_tools.tf_geometry_service:main",
            "object_pose_resolver_service = ur5_tools.object_pose_resolver_service:main",
            "cycle_timing = ur5_tools.cli_cycle_timing:main",
            "cycle_timing_aggregator = ur5_tools.cycle_timing_aggregator:main",
        ],
    },
)
