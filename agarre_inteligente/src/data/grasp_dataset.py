"""Dataset para Cornell-style grasp annotations y modo sintetico reproducible."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np
import pandas as pd
import torch
import torch.nn.functional as F
from PIL import Image
from PIL import ImageFile
from torch.utils.data import Dataset

# Cornell real puede contener algunos TIFF parcialmente truncados.
ImageFile.LOAD_TRUNCATED_IMAGES = True

@dataclass
class Sample:
    image: torch.Tensor
    target: torch.Tensor
    image_path: str


class GraspDataset(Dataset):
    """
    Espera CSV con columnas:
    image_path, depth_path(optional), cx, cy, w, h, angle_deg
    """

    def __init__(
        self,
        csv_path: str | Path,
        data_root: str | Path,
        modality: str = "rgb",
        transform: Callable | None = None,
        allow_synthetic: bool = False,
        synthetic_size: int = 256,
        seed: int = 0,
    ):
        self.csv_path = Path(csv_path)
        self.data_root = Path(data_root)
        self.modality = modality
        self.transform = transform
        self.allow_synthetic = allow_synthetic
        self.synthetic_size = synthetic_size
        self.rng = np.random.default_rng(seed)

        if self.csv_path.exists():
            self.df = pd.read_csv(self.csv_path)
            required = ["cx", "cy", "w", "h", "angle_deg"]
            missing = [c for c in required if c not in self.df.columns]
            if missing:
                raise ValueError(f"CSV sin columnas requeridas: {missing}")
            finite_mask = np.isfinite(self.df[required].to_numpy(dtype=np.float64)).all(axis=1)
            if not bool(np.all(finite_mask)):
                self.df = self.df.loc[finite_mask].reset_index(drop=True)
            if len(self.df) == 0 and allow_synthetic:
                self.df = pd.DataFrame({"id": np.arange(synthetic_size)})
                self.synthetic = True
            else:
                self.synthetic = False
        elif allow_synthetic:
            self.df = pd.DataFrame({"id": np.arange(synthetic_size)})
            self.synthetic = True
        else:
            raise FileNotFoundError(f"No existe CSV de split: {self.csv_path}")

    def __len__(self) -> int:
        return len(self.df)

    def _synthetic_item(self, idx: int) -> Sample:
        channels = 4 if self.modality == "rgbd" else 3
        image = torch.from_numpy(self.rng.random((channels, 224, 224), dtype=np.float32))
        target = torch.tensor(
            [
                float(self.rng.uniform(0.1, 0.9)),
                float(self.rng.uniform(0.1, 0.9)),
                float(self.rng.uniform(0.1, 0.6)),
                float(self.rng.uniform(0.1, 0.6)),
                float(self.rng.uniform(-1.0, 1.0)),
            ],
            dtype=torch.float32,
        )
        return Sample(image=image, target=target, image_path=f"synthetic_{idx}.png")

    def _load_rgb(self, rel_path: str) -> Image.Image:
        p = self.data_root / rel_path
        try:
            return Image.open(p).convert("RGB")
        except Exception:
            # Fallback seguro para no abortar entrenamiento por un archivo corrupto.
            return Image.new("RGB", (224, 224), color=(0, 0, 0))

    def __getitem__(self, idx: int):
        if self.synthetic:
            s = self._synthetic_item(idx)
            return s.image, s.target, s.image_path

        row = self.df.iloc[idx]
        rgb = self._load_rgb(str(row["image_path"]))
        
        # Guardar tamaño original para reescalar coordenadas
        orig_w, orig_h = rgb.size  # PIL: (width, height)

        if self.transform is not None:
            rgb_tensor = self.transform(rgb)
        else:
            rgb_tensor = torch.from_numpy(np.asarray(rgb).transpose(2, 0, 1)).float() / 255.0

        # Obtener tamaño final después del transform
        _, final_h, final_w = rgb_tensor.shape
        
        # Factores de escala
        scale_x = final_w / orig_w
        scale_y = final_h / orig_h

        if self.modality == "rgbd":
            depth_rel = row.get("depth_path", None)
            if depth_rel is None or pd.isna(depth_rel):
                depth = torch.zeros((1, rgb_tensor.shape[1], rgb_tensor.shape[2]), dtype=torch.float32)
            else:
                try:
                    d_img = Image.open(self.data_root / str(depth_rel))
                    d_np = np.asarray(d_img).astype(np.float32)
                    if d_np.ndim == 3:
                        d_np = d_np[..., 0]
                    d_np = d_np / (np.max(d_np) + 1e-6)
                    depth = torch.from_numpy(d_np).unsqueeze(0)
                except Exception:
                    depth = torch.zeros((1, rgb_tensor.shape[1], rgb_tensor.shape[2]), dtype=torch.float32)

            if depth.shape[1:] != rgb_tensor.shape[1:]:
                depth = F.interpolate(
                    depth.unsqueeze(0),
                    size=rgb_tensor.shape[1:],
                    mode="bilinear",
                    align_corners=False,
                ).squeeze(0)
            image = torch.cat([rgb_tensor, depth], dim=0)
        else:
            image = rgb_tensor

        # Reescalar al tamaño final y normalizar para estabilizar la regresion.
        target = torch.tensor(
            [
                (row["cx"] * scale_x) / max(float(final_w), 1.0),
                (row["cy"] * scale_y) / max(float(final_h), 1.0),
                (row["w"] * scale_x) / max(float(final_w), 1.0),
                (row["h"] * scale_y) / max(float(final_h), 1.0),
                float(row["angle_deg"]) / 90.0,
            ],
            dtype=torch.float32
        )
        return image, target, str(row["image_path"])
