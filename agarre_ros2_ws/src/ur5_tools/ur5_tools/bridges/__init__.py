"""F11 (auditoría 2026-05-10): namespace de bridges Gazebo ↔ ROS.

Subpaquete que re-exporta los módulos de "bridges Gazebo Sim moderno
hacia ROS 2":
  * ``gz_pose_bridge`` — Node bridge para pose/info de Gazebo.
  * ``gz_ros_control_guard`` — Node guardian del controller_manager.
  * ``moveit_bridge_utils`` — helpers compartidos del bridge MoveIt.

Los archivos físicos siguen en top-level; F11 iter 2 los moverá.
"""
from __future__ import annotations

# Re-exports lazy: estos módulos suelen importar rclpy en top-level y
# no queremos que el simple `import ur5_tools.bridges` rompa en
# entornos sin rclpy. Los consumers deben importar directamente:
#   from ur5_tools.bridges import gz_pose_bridge

__all__: list[str] = []
