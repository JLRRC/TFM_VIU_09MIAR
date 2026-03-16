# Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/tfm_grasping/perception.py
# Contenido: Codigo de percepcion y agarre del paquete tfm_grasping.
# Uso breve: Se importa desde el stack ROS 2 para calculo y publicacion de informacion de agarre.
"""Perception utilities for preparing RGB-D inputs for the grasp model."""

from dataclasses import dataclass
from typing import Optional
import time

try:
    import numpy as np  # type: ignore
except Exception:  # pragma: no cover - optional for inference
    np = None


@dataclass
class InputFrame:
    """Normalized input container for the grasping pipeline."""

    image: object
    width: int
    height: int
    timestamp: float
    roi: Optional[tuple[int, int, int]] = None
    preprocessed: bool = False


class PerceptionPipeline:
    """Prepare input data for the grasp model without GUI dependencies."""

    @staticmethod
    def normalize_depth(
        depth: "np.ndarray",
        img_size: int = 224,
    ) -> Optional["np.ndarray"]:
        """
        Convert depth image to float32 HxW normalized to [0,1] at img_size x img_size.
        Mirrors the training dataset behavior used for RGB-D experiments.
        """
        if np is None:
            return None
        try:
            import cv2  # type: ignore
        except Exception:
            return None
        if not isinstance(depth, np.ndarray):
            return None
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        if depth.ndim != 2:
            return None
        arr = depth.astype(np.float32, copy=False)
        arr = np.ascontiguousarray(arr)
        arr[~np.isfinite(arr)] = 0.0
        max_val = float(np.max(arr)) if arr.size else 0.0
        if max_val > 0.0:
            arr = arr / (max_val + 1e-6)
        else:
            arr = np.zeros_like(arr, dtype=np.float32)
        if arr.shape[0] != img_size or arr.shape[1] != img_size:
            arr = cv2.resize(arr, (img_size, img_size), interpolation=cv2.INTER_LINEAR)
        if arr.dtype != np.float32:
            arr = arr.astype(np.float32, copy=False)
        if not arr.flags["C_CONTIGUOUS"]:
            arr = np.ascontiguousarray(arr)
        return arr

    @staticmethod
    def to_preprocessed(
        image: "np.ndarray",
        img_size: int = 224,
    ) -> Optional["np.ndarray"]:
        """
        Convert HWC uint8 RGB -> CHW float32 [0,1] at img_size x img_size.
        Returns None if numpy/cv2 not available or input invalid.
        """
        if np is None:
            return None
        try:
            import cv2  # type: ignore
        except Exception:
            return None
        if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] < 3:
            return None
        if image.shape[0] != img_size or image.shape[1] != img_size:
            image = cv2.resize(image, (img_size, img_size), interpolation=cv2.INTER_LINEAR)
        rgb = image[:, :, :3]
        if rgb.dtype != np.uint8:
            rgb = rgb.astype(np.uint8, copy=False)
        x = rgb.astype(np.float32) / 255.0
        x = np.transpose(x, (2, 0, 1))
        if not x.flags["C_CONTIGUOUS"]:
            x = np.ascontiguousarray(x)
        return x

    @staticmethod
    def to_preprocessed_rgbd(
        image: "np.ndarray",
        depth: "np.ndarray",
        img_size: int = 224,
    ) -> Optional["np.ndarray"]:
        """Convert RGB + depth into CHW float32 [0,1] with 4 channels."""
        if np is None:
            return None
        rgb = PerceptionPipeline.to_preprocessed(image, img_size=img_size)
        depth_norm = PerceptionPipeline.normalize_depth(depth, img_size=img_size)
        if rgb is None or depth_norm is None:
            return None
        x = np.concatenate([rgb, depth_norm[None, :, :]], axis=0)
        if x.dtype != np.float32:
            x = x.astype(np.float32, copy=False)
        if not x.flags["C_CONTIGUOUS"]:
            x = np.ascontiguousarray(x)
        return x

    @staticmethod
    def qimage_to_rgb(
        image: object,
        width: int = 0,
        height: int = 0,
    ) -> Optional["np.ndarray"]:
        if np is None:
            return None
        try:
            from PyQt5.QtGui import QImage  # type: ignore
        except Exception:
            return None
        if not isinstance(image, QImage):
            return None
        qimg = image.convertToFormat(QImage.Format_RGB888)
        w = int(width or qimg.width())
        h = int(height or qimg.height())
        if w <= 0 or h <= 0:
            return None
        ptr = qimg.bits()
        ptr.setsize(qimg.byteCount())
        arr = np.frombuffer(ptr, dtype=np.uint8)
        bytes_per_line = int(qimg.bytesPerLine())
        if bytes_per_line <= 0:
            return None
        arr = arr.reshape((h, bytes_per_line))
        rgb = arr[:, : w * 3].reshape((h, w, 3))
        return np.ascontiguousarray(rgb)

    @staticmethod
    def qimage_to_preprocessed(
        image: object,
        width: int = 0,
        height: int = 0,
        img_size: int = 224,
    ) -> Optional["np.ndarray"]:
        rgb = PerceptionPipeline.qimage_to_rgb(image, width=width, height=height)
        if rgb is None:
            return None
        return PerceptionPipeline.to_preprocessed(rgb, img_size=img_size)

    @staticmethod
    def qimage_to_preprocessed_rgbd(
        image: object,
        depth: "np.ndarray",
        width: int = 0,
        height: int = 0,
        img_size: int = 224,
    ) -> Optional["np.ndarray"]:
        rgb = PerceptionPipeline.qimage_to_rgb(image, width=width, height=height)
        if rgb is None:
            return None
        return PerceptionPipeline.to_preprocessed_rgbd(rgb, depth, img_size=img_size)

    def _qimage_to_rgb(self, image: object, width: int, height: int) -> Optional["np.ndarray"]:
        if np is None:
            return None
        try:
            from PyQt5.QtGui import QImage  # type: ignore
        except Exception:
            return None
        if not isinstance(image, QImage):
            return None
        qimg = image.convertToFormat(QImage.Format_RGB888)
        w = int(width or qimg.width())
        h = int(height or qimg.height())
        if w <= 0 or h <= 0:
            return None
        ptr = qimg.bits()
        ptr.setsize(qimg.byteCount())
        arr = np.frombuffer(ptr, dtype=np.uint8)
        bytes_per_line = int(qimg.bytesPerLine())
        if bytes_per_line <= 0:
            return None
        arr = arr.reshape((h, bytes_per_line))
        rgb = arr[:, : w * 3].reshape((h, w, 3))
        # QImage buffer puede invalidarse fuera de este scope, fuerza copia segura
        return np.ascontiguousarray(rgb)

    def _to_rgb(self, image: object, width: int, height: int) -> Optional["np.ndarray"]:
        if np is None:
            return None
        if isinstance(image, np.ndarray):
            arr = image
            if arr.ndim == 3 and arr.shape[2] >= 3:
                view = arr[:, :, :3]
                if view.dtype != np.uint8 or not view.flags["C_CONTIGUOUS"]:
                    return np.ascontiguousarray(view)
                # Si ya es uint8 y contiguous, evitamos copia
                return view
            return None
        return self._qimage_to_rgb(image, width, height)

    def prepare(
        self,
        image: object,
        width: int = 0,
        height: int = 0,
        roi: Optional[tuple[int, int, int]] = None,
        preprocessed: bool = False,
    ) -> Optional[InputFrame]:
        """Wrap the incoming image in a consistent structure."""
        if image is None:
            return None
        if preprocessed:
            if np is None:
                return None
            if not isinstance(image, np.ndarray):
                return None
            if image.ndim != 3:
                return None
            # Expect CHW float32 in [0,1]
            if image.shape[0] not in (3, 4):
                return None
            h = int(height or image.shape[1])
            w = int(width or image.shape[2])
            return InputFrame(
                image=image,
                width=w,
                height=h,
                timestamp=time.time(),
                roi=roi,
                preprocessed=True,
            )
        rgb = self._to_rgb(image, width, height)
        if rgb is None:
            return None
        h = int(height or rgb.shape[0])
        w = int(width or rgb.shape[1])
        return InputFrame(image=rgb, width=w, height=h, timestamp=time.time(), roi=roi)
