# Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/setup.py
# Contenido: Codigo de percepcion y agarre del paquete tfm_grasping.
# Uso breve: Se importa desde el stack ROS 2 para calculo y publicacion de informacion de agarre.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/tfm_grasping/setup.py
# Summary: Setuptools setup for tfm_grasping package.
"""Setuptools entry point for the tfm_grasping package."""
from setuptools import setup

package_name = "tfm_grasping"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="laboratorio",
    maintainer_email="jesus.lozano.rodriguez@gmail.com",
    description="TFM grasping module (perception, grasp geometry, ROS publishing).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grasp_inference=tfm_grasping.grasp_inference:main",
        ],
    },
)
