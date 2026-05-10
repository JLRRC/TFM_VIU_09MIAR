#!/usr/bin/env python3
# Ruta/archivo: src/ur5_gazebo/launch/spawn_objects.launch.py
# Contenido: F-iter2 audit (2026-05-10) — placeholder launch para spawn de objetos.
"""F-iter2 audit (2026-05-10): launch placeholder spawn_objects.

Este launch es un placeholder estructurado para futuras integraciones
de spawn batch de objetos en Gazebo Harmonic. Actualmente, el spawn
real se hace vía:

  * El world SDF (``ur5_mesa_objetos.sdf``) que ya incluye los modelos.
  * El servicio ``/world/<name>/create/blocking`` (ros_gz) si se
    quiere spawn dinámico.
  * ``simulation_reset_service`` (F9.3) para reset entre ciclos.

Este archivo cumple el contrato "el directorio launch/ del paquete
ur5_gazebo no debe estar vacío" (audit iteración 2). Cualquier launch
sub-comando relacionado con spawn de objetos debería vivir aquí.

Uso (futuro)::

    ros2 launch ur5_gazebo spawn_objects.launch.py world_name:=ur5_mesa_objetos
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_name",
                default_value="ur5_mesa_objetos",
                description="Nombre del world Gazebo donde spawnar objetos.",
            ),
            DeclareLaunchArgument(
                "objects_yaml",
                default_value="",
                description="Path opcional a un YAML con la lista de objetos.",
            ),
            LogInfo(
                msg=[
                    "[ur5_gazebo/spawn_objects] placeholder launch — el spawn ",
                    "real lo hace el world SDF + simulation_reset_service. ",
                    "Para spawn dinámico custom, extender este launch con ",
                    "ros_gz_sim CreateService calls.",
                    " world_name=", LaunchConfiguration("world_name"),
                ]
            ),
        ]
    )
