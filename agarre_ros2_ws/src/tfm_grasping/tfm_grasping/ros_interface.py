# Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/tfm_grasping/ros_interface.py
# Contenido: Codigo de percepcion y agarre del paquete tfm_grasping.
# Uso breve: Se importa desde el stack ROS 2 para calculo y publicacion de informacion de agarre.
"""ROS 2 publishing adapter for grasp outputs."""

import json
from typing import Optional, Callable

try:
    import rclpy  # type: ignore
    from std_msgs.msg import String  # type: ignore
except Exception:  # pragma: no cover - optional runtime dependency
    rclpy = None
    String = None

from .config import DEFAULT_ROS_TOPIC
from .geometry import Grasp2D


class RosGraspPublisher:
    """ROS 2 publisher wrapper for the grasp representation."""

    def __init__(
        self,
        *,
        topic: str = DEFAULT_ROS_TOPIC,
        node: Optional[object] = None,
        logger: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._topic = topic
        self._node = node
        self._pub = None
        self._logger = logger

    def _ensure_node(self) -> bool:
        if self._node is not None:
            return True
        if rclpy is None:
            return False
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node("tfm_grasp_module")
        return True

    def _ensure_pub(self) -> bool:
        if self._pub is not None:
            return True
        if String is None:
            return False
        if not self._ensure_node():
            return False
        self._pub = self._node.create_publisher(String, self._topic, 10)
        return True

    def publish(self, grasp: Grasp2D) -> bool:
        if grasp is None:
            return False
        if not self._ensure_pub():
            return False
        msg = String()
        msg.data = json.dumps(grasp.to_dict())
        self._pub.publish(msg)
        if self._logger:
            self._logger(f"[TFM] grasp publicado en {self._topic}")
        return True
