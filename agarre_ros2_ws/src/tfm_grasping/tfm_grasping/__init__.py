# Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/tfm_grasping/__init__.py
# Contenido: Codigo de percepcion y agarre del paquete tfm_grasping.
# Uso breve: Se importa desde el stack ROS 2 para calculo y publicacion de informacion de agarre.
"""TFM grasping package with decoupled perception, model, and ROS interfaces."""

from .grasp_module import TFMGraspModule

__all__ = ["TFMGraspModule"]
