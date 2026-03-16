# Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/tfm_grasping/model.py
# Contenido: Codigo de percepcion y agarre del paquete tfm_grasping.
# Uso breve: Se importa desde el stack ROS 2 para calculo y publicacion de informacion de agarre.
"""Model loading and inference for the TFM grasping pipeline."""

from dataclasses import dataclass
import os
import sys
from typing import Optional, Tuple, List

try:
    import numpy as np  # type: ignore
except Exception:  # pragma: no cover - optional for inference
    np = None

try:
    import torch  # type: ignore
except Exception:  # pragma: no cover - optional for inference
    torch = None

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - optional for inference
    cv2 = None

from .geometry import Grasp2D
from .perception import InputFrame


@dataclass
class ModelInfo:
    """Simple model metadata container."""

    model_path: str
    loaded: bool = False
    model_name: str = ""
    in_channels: int = 0


class GraspModel:
    """Grasp model wrapper with lazy load semantics."""

    _AUTO_NORMALIZED_THRESHOLD = 2.0
    _AUTO_NORMALIZED_ANGLE_THRESHOLD = 1.5

    def __init__(self, model_path: str = "", img_size: int = 224) -> None:
        self.info = ModelInfo(model_path=model_path, loaded=False)
        self._img_size = int(max(1, img_size))
        self._model = None
        self._device = self._resolve_device()
        self._last_error: str = ""

    def _resolve_device(self):
        if torch is None:
            return None
        dev = os.environ.get("TFM_INFER_DEVICE", "auto").strip().lower()
        if dev == "cpu":
            return torch.device("cpu")
        if dev == "cuda":
            if not torch.cuda.is_available():
                return torch.device("cpu")
            return torch.device("cuda")
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")

    def img_size(self) -> int:
        return int(self._img_size)

    def _ensure_graspnet_on_path(self) -> None:
        vision_dir = os.environ.get("VISION_DIR", "~/TFM/agarre_inteligente")
        vision_dir = os.path.expanduser(vision_dir)
        src_dir = os.path.join(vision_dir, "src")
        if src_dir not in sys.path:
            sys.path.insert(0, src_dir)

    def _build_model_candidates(self, state_dict) -> List[Tuple[object, int, str]]:
        self._ensure_graspnet_on_path()
        if torch is None:
            self._last_error = "torch no disponible"
            return []
        keys = list(state_dict.keys())
        if any(k.startswith("backbone.") for k in keys) or any(
            k.startswith("model.") for k in keys
        ):
            try:
                from graspnet.models.resnet18_grasp import ResNet18Grasp  # type: ignore
            except Exception as exc:
                # Fallback for current agarre_inteligente layout.
                try:
                    from models.resnet_variants import ResNetGrasp  # type: ignore

                    conv_key = "backbone.conv1.weight"
                    if conv_key not in state_dict:
                        conv_key = "model.conv1.weight"
                    in_ch = 3
                    if conv_key in state_dict and hasattr(state_dict[conv_key], "shape"):
                        try:
                            in_ch = int(state_dict[conv_key].shape[1])
                        except Exception:
                            in_ch = 3
                    model = ResNetGrasp(input_channels=in_ch, pretrained=False)
                    return [(model, in_ch, "resnet18")]
                except Exception:
                    self._last_error = f"no se pudo importar ResNet18Grasp: {exc}"
                    return []

            conv_key = "backbone.conv1.weight"
            in_ch = 3
            if conv_key in state_dict and hasattr(state_dict[conv_key], "shape"):
                try:
                    in_ch = int(state_dict[conv_key].shape[1])
                except Exception:
                    in_ch = 3
            model = ResNet18Grasp(in_channels=in_ch, pretrained=False)
            return [(model, in_ch, "resnet18")]
        if any(k.startswith("features.") for k in keys):
            candidates: List[Tuple[object, int, str]] = []
            conv_key = "features.0.weight"
            in_ch = 3
            if conv_key in state_dict and hasattr(state_dict[conv_key], "shape"):
                try:
                    in_ch = int(state_dict[conv_key].shape[1])
                except Exception:
                    in_ch = 3

            try:
                from graspnet.models.simple_cnn import SimpleGraspCNN as SimpleCnn  # type: ignore
                candidates.append((SimpleCnn(in_channels=in_ch, img_size=self._img_size), in_ch, "simple_cnn"))
            except Exception as exc:
                self._last_error = f"no se pudo importar simple_cnn.SimpleGraspCNN: {exc}"

            # Fallback for current agarre_inteligente layout.
            try:
                from models.simple_cnn import SimpleCNN  # type: ignore

                candidates.append((SimpleCNN(input_channels=in_ch), in_ch, "simple_cnn"))
            except Exception:
                pass

            try:
                from graspnet.models.simple_grasp_cnn import SimpleGraspCNN as SimpleLegacy  # type: ignore
                candidates.append((SimpleLegacy(in_channels=in_ch), in_ch, "simple_grasp_cnn"))
            except Exception:
                pass

            if candidates:
                return candidates
        self._last_error = "checkpoint incompatible (keys no reconocidas)"
        return []

    def _decode_model_axis(self, value: float) -> float:
        """Accept normalized outputs [0,1] and legacy pixel outputs in model space."""
        if abs(value) <= self._AUTO_NORMALIZED_THRESHOLD:
            return value * float(self._img_size)
        return value

    def _decode_model_angle(self, angle_deg: float, geom_values: Tuple[float, float, float, float]) -> float:
        if abs(angle_deg) > self._AUTO_NORMALIZED_ANGLE_THRESHOLD:
            return angle_deg
        if any(abs(value) <= self._AUTO_NORMALIZED_THRESHOLD for value in geom_values):
            return angle_deg * 90.0
        return angle_deg

    def _decode_prediction(
        self,
        pred: "np.ndarray",
        frame: InputFrame,
        roi_info: Optional[tuple[int, int, int, int]],
    ) -> Optional[Grasp2D]:
        if np is None:
            return None
        if pred is None:
            self._last_error = "prediccion vacia"
            return None
        flat = np.asarray(pred, dtype=float).reshape(-1)
        if flat.size != 5:
            self._last_error = f"prediccion invalida: se esperaban 5 valores y llegaron {flat.size}"
            return None
        raw_cx, raw_cy, raw_w, raw_h, raw_angle_deg = [float(v) for v in flat.tolist()]
        cx = self._decode_model_axis(raw_cx)
        cy = self._decode_model_axis(raw_cy)
        w = abs(self._decode_model_axis(raw_w))
        h = abs(self._decode_model_axis(raw_h))
        angle_deg = self._decode_model_angle(raw_angle_deg, (raw_cx, raw_cy, raw_w, raw_h))

        if roi_info:
            roi_x, roi_y, roi_w, roi_h = roi_info
            scale_x = roi_w / float(self._img_size)
            scale_y = roi_h / float(self._img_size)
        else:
            roi_x = 0
            roi_y = 0
            scale_x = frame.width / float(self._img_size)
            scale_y = frame.height / float(self._img_size)

        return Grasp2D(
            center_x=cx * scale_x + roi_x,
            center_y=cy * scale_y + roi_y,
            angle_rad=float(np.deg2rad(angle_deg)),
            width_px=w * scale_x,
            height_px=h * scale_y,
            quality=0.0,
            depth_m=None,
            frame_id="",
        )

    def _roi_to_box(self, frame: InputFrame) -> Optional[tuple[int, int, int, int]]:
        if not frame.roi:
            return None
        roi_cx, roi_cy, roi_size = frame.roi
        roi_size = int(max(1, min(int(roi_size), frame.width, frame.height)))
        if roi_size <= 0:
            return None
        cx = int(round(max(0, min(frame.width - 1, roi_cx))))
        cy = int(round(max(0, min(frame.height - 1, roi_cy))))
        x0 = int(round(cx - roi_size / 2.0))
        y0 = int(round(cy - roi_size / 2.0))
        x0 = max(0, min(frame.width - roi_size, x0))
        y0 = max(0, min(frame.height - roi_size, y0))
        return x0, y0, roi_size, roi_size

    def _preprocess(self, rgb: "np.ndarray", roi: Optional[tuple[int, int, int]]):
        if np is None:
            self._last_error = "numpy no disponible"
            return None, None
        if cv2 is None:
            self._last_error = "cv2 no disponible"
            return None, None
        h, w, _ = rgb.shape
        roi_info = None
        if roi:
            roi_cx, roi_cy, roi_size = roi
            roi_size = int(max(1, roi_size))
            roi_size = int(min(roi_size, w, h))
            cx = int(round(max(0, min(w - 1, roi_cx))))
            cy = int(round(max(0, min(h - 1, roi_cy))))
            x0 = int(round(cx - roi_size / 2.0))
            y0 = int(round(cy - roi_size / 2.0))
            x0 = max(0, min(w - roi_size, x0))
            y0 = max(0, min(h - roi_size, y0))
            x1 = x0 + roi_size
            y1 = y0 + roi_size
            rgb = rgb[y0:y1, x0:x1]
            roi_info = (x0, y0, roi_size, roi_size)
        if rgb.shape[0] == self._img_size and rgb.shape[1] == self._img_size:
            resized = rgb if rgb.flags["C_CONTIGUOUS"] else np.ascontiguousarray(rgb)
        else:
            resized = cv2.resize(rgb, (self._img_size, self._img_size), interpolation=cv2.INTER_LINEAR)
        x = resized.astype("float32") / 255.0
        x = x.transpose(2, 0, 1)
        return x, roi_info

    def load(self, model_path: Optional[str] = None) -> bool:
        """Load the model weights."""
        if model_path is not None:
            self.info.model_path = model_path
        if not self.info.model_path:
            self.info.loaded = False
            return False
        if torch is None:
            self._last_error = "torch no disponible"
            self.info.loaded = False
            return False
        if not os.path.isfile(self.info.model_path):
            self._last_error = f"checkpoint no existe: {self.info.model_path}"
            self.info.loaded = False
            return False
        ckpt = torch.load(self.info.model_path, map_location="cpu")
        state_dict = ckpt.get("model_state_dict", ckpt)
        candidates = self._build_model_candidates(state_dict)
        if not candidates:
            self.info.loaded = False
            return False
        last_err = ""
        for model, in_ch, model_name in candidates:
            try:
                model.load_state_dict(state_dict, strict=True)
            except Exception as exc:
                last_err = f"fallo cargando {model_name}: {exc}"
                continue
            model.eval()
            self._model = model.to(self._device) if self._device else model
            self.info.loaded = True
            self.info.model_name = model_name
            self.info.in_channels = in_ch
            self._last_error = ""
            return True
        self._last_error = last_err or "no se pudo cargar el checkpoint"
        self.info.loaded = False
        return False

    def infer(self, frame: InputFrame) -> Optional[Grasp2D]:
        """Infer a grasp from an input frame."""
        if not self.info.loaded or not frame or self._model is None:
            return None
        if torch is None or np is None:
            return None
        if frame.width <= 0 or frame.height <= 0:
            return None
        if frame.preprocessed:
            x = frame.image
            if not isinstance(x, np.ndarray) or x.ndim != 3:
                self._last_error = "input preprocesado inválido"
                return None
            if x.shape[0] not in (3, 4):
                self._last_error = f"input preprocesado requiere 3 o 4 canales, got {x.shape[0]}"
                return None
            if x.dtype != np.float32:
                x = x.astype(np.float32, copy=False)
            if not x.flags["C_CONTIGUOUS"]:
                x = np.ascontiguousarray(x)
            if x.shape[1] != self._img_size or x.shape[2] != self._img_size:
                self._last_error = "input preprocesado con tamaño distinto a img_size"
                return None
            roi_info = self._roi_to_box(frame)
        else:
            rgb = frame.image
            if not isinstance(rgb, np.ndarray) or rgb.ndim != 3:
                return None
            x, roi_info = self._preprocess(rgb, frame.roi)
        if x is None:
            return None
        got_channels = int(x.shape[0]) if hasattr(x, "shape") and len(x.shape) >= 1 else 0
        if self.info.in_channels != got_channels:
            self._last_error = (
                f"modelo requiere {self.info.in_channels} canales y llegaron {got_channels}"
            )
            return None
        tensor = torch.from_numpy(x).unsqueeze(0)
        if self._device is not None:
            tensor = tensor.to(self._device)
        with torch.no_grad():
            pred = self._model(tensor).squeeze(0).cpu().numpy()
        return self._decode_prediction(pred, frame, roi_info)

    def last_error(self) -> str:
        return self._last_error

    def reset(self) -> None:
        """Unload the model and clear metadata."""
        self._model = None
        self.info.loaded = False
        self.info.model_name = ""
        self.info.in_channels = 0
        self._last_error = ""
