# Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/tfm_grasping/geometry.py
# Contenido: Codigo de percepcion y agarre del paquete tfm_grasping.
# Uso breve: Se importa desde el stack ROS 2 para calculo y publicacion de informacion de agarre.
"""Grasp representation primitives for the TFM pipeline."""

from dataclasses import dataclass
from typing import Dict, Optional


@dataclass(frozen=True)
class Grasp2D:
    """2D/2.5D grasp representation in image space."""

    center_x: float
    center_y: float
    angle_rad: float
    width_px: float
    quality: float
    height_px: Optional[float] = None
    depth_m: Optional[float] = None
    frame_id: str = ""

    def to_dict(self) -> Dict[str, object]:
        """Return a JSON-serializable representation."""
        return {
            "center_x": self.center_x,
            "center_y": self.center_y,
            "angle_rad": self.angle_rad,
            "width_px": self.width_px,
            "height_px": self.height_px,
            "quality": self.quality,
            "depth_m": self.depth_m,
            "frame_id": self.frame_id,
        }
