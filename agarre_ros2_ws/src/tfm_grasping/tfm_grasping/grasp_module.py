# Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/tfm_grasping/grasp_module.py
# Contenido: Codigo de percepcion y agarre del paquete tfm_grasping.
# Uso breve: Se importa desde el stack ROS 2 para calculo y publicacion de informacion de agarre.
"""High-level TFM module orchestrating perception, inference, and ROS publication."""

from typing import Optional, Callable

from .config import DEFAULT_FRAME_ID
from .geometry import Grasp2D
from .model import GraspModel
from .perception import PerceptionPipeline, InputFrame
from .ros_interface import RosGraspPublisher


class TFMGraspModule:
    """
    Modulo independiente del TFM:
    deteccion de poses de agarre 2D/2.5D a partir de imagenes RGB-D
    y publicacion de resultados en ROS 2.
    """

    def __init__(self, *, logger: Optional[Callable[[str], None]] = None, model_path: str = "") -> None:
        self._logger = logger
        self._perception = PerceptionPipeline()
        self._model = GraspModel(model_path=model_path)
        self._ros = RosGraspPublisher(logger=logger)
        self._last_frame: Optional[InputFrame] = None
        self._last_grasp: Optional[Grasp2D] = None

    def load_model(self, model_path: Optional[str] = None) -> bool:
        """Load the grasp model weights."""
        ok = self._model.load(model_path)
        if not ok and self._logger:
            err = self._model.last_error() or "model_path vacío"
            self._logger(f"[TFM] Modelo no cargado ({err}).")
        return ok

    def is_model_loaded(self) -> bool:
        return bool(self._model.info.loaded)

    def last_error(self) -> str:
        return self._model.last_error()

    def model_info(self) -> dict:
        """Return basic model metadata for audit/logging."""
        info = self._model.info
        return {
            "model_path": info.model_path,
            "loaded": bool(info.loaded),
            "model_name": info.model_name,
            "in_channels": info.in_channels,
            "img_size": self._model.img_size(),
        }

    def set_input_image(
        self,
        image: object,
        width: int = 0,
        height: int = 0,
        roi: Optional[tuple[int, int, int]] = None,
        preprocessed: bool = False,
    ) -> None:
        """Set the current RGB-D input frame for inference."""
        self._last_frame = self._perception.prepare(
            image,
            width=width,
            height=height,
            roi=roi,
            preprocessed=preprocessed,
        )

    def infer_grasp(self) -> Optional[Grasp2D]:
        """Run grasp inference on the last input frame."""
        if self._last_frame is None:
            if self._logger:
                self._logger("[TFM] infer_grasp sin imagen de entrada.")
            return None
        grasp = self._model.infer(self._last_frame)
        if grasp is None:
            if self._logger:
                err = self._model.last_error() or "modelo no cargado"
                self._logger(f"[TFM] infer_grasp sin salida ({err}).")
            return None
        if not grasp.frame_id:
            grasp = Grasp2D(
                center_x=grasp.center_x,
                center_y=grasp.center_y,
                angle_rad=grasp.angle_rad,
                width_px=grasp.width_px,
                height_px=grasp.height_px,
                quality=grasp.quality,
                depth_m=grasp.depth_m,
                frame_id=DEFAULT_FRAME_ID,
            )
        self._last_grasp = grasp
        return grasp

    def infer_grasp_params(self) -> Optional[dict]:
        """Return grasp parameters in Cornell format (cx, cy, w, h, angle_deg)."""
        grasp = self.infer_grasp()
        if grasp is None:
            return None
        return {
            "cx": float(grasp.center_x),
            "cy": float(grasp.center_y),
            "w": float(grasp.width_px),
            "h": float(grasp.height_px if grasp.height_px is not None else 0.0),
            "angle_deg": float(grasp.angle_rad * 180.0 / 3.141592653589793),
        }

    def get_grasp_representation(self) -> Optional[dict]:
        """Return the latest grasp representation as a serializable dict."""
        if self._last_grasp is None:
            if self._logger:
                self._logger("[TFM] No hay grasp para visualizar.")
            return None
        return self._last_grasp.to_dict()

    def publish_grasp(self) -> bool:
        """Publish the latest grasp result over ROS 2."""
        if self._last_grasp is None:
            if self._logger:
                self._logger("[TFM] No hay grasp para publicar.")
            return False
        return self._ros.publish(self._last_grasp)

    def set_last_grasp(self, grasp: Grasp2D) -> None:
        """Set the latest grasp from an external inference source."""
        self._last_grasp = grasp

    def reset(self) -> None:
        """Clear cached data and unload the model."""
        self._last_frame = None
        self._last_grasp = None
        self._model.reset()
