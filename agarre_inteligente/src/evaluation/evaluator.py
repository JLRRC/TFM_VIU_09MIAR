"""Evaluacion Cornell-style y export de resultados por split."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
import torch


@dataclass
class EvalResult:
    val_success: float
    val_iou: float
    val_angle_deg: float
    val_loss: float


class Evaluator:
    def __init__(self, model: torch.nn.Module, dataloader, device: str = "cpu", csv_path: str | Path | None = None):
        self.model = model
        self.dataloader = dataloader
        self.device = device
        self.csv_path = Path(csv_path) if csv_path else None
        
        # Precargar todos los GT agrupados por imagen si tenemos el CSV
        self.gt_by_image = {}
        if self.csv_path and self.csv_path.exists():
            df = pd.read_csv(self.csv_path)
            for img_path, group in df.groupby('image_path'):
                # Guardar todos los grasps GT de esta imagen
                self.gt_by_image[img_path] = group[['cx', 'cy', 'w', 'h', 'angle_deg']].values

    @torch.no_grad()
    def evaluate(self, criterion) -> EvalResult:
        # Import here to avoid circular dependency
        from src.training.metrics import cornell_success, iou_axis_aligned_boxes, angle_error_deg
        
        self.model.eval()
        
        all_ious = []
        all_angles = []
        all_successes = []
        losses = []
        
        for batch in self.dataloader:
            x, y, names = batch
            x = x.to(self.device)
            y = y.to(self.device)
            pred = self.model(x)
            loss = criterion(pred, y)
            if torch.isfinite(loss):
                losses.append(float(loss.item()))

            # Desnormalizar predicciones
            pred_np = pred.detach().cpu().numpy().copy()
            pred_np = np.nan_to_num(pred_np, nan=0.0, posinf=1.0, neginf=-1.0)
            
            h = float(x.shape[2])
            w = float(x.shape[3])
            pred_np[:, 0] = pred_np[:, 0] * w
            pred_np[:, 1] = pred_np[:, 1] * h
            pred_np[:, 2] = np.abs(pred_np[:, 2] * w)
            pred_np[:, 3] = np.abs(pred_np[:, 3] * h)
            pred_np[:, 4] = pred_np[:, 4] * 90.0

            # Para cada predicción, comparar contra TODOS los GT de esa imagen
            for i, img_path in enumerate(names):
                pred_single = pred_np[i:i+1]  # Shape (1, 5)
                
                if self.gt_by_image and img_path in self.gt_by_image:
                    # Cargar todos los GT de esta imagen
                    gt_all = self.gt_by_image[img_path].copy()  # Shape (N, 5) donde N es número de grasps
                    
                    # Reescalar los GT al tamaño de la imagen (originalmente en coordenadas de 640x480)
                    # Necesitamos aplicar la misma transformación que en el dataset
                    orig_w, orig_h = 640.0, 480.0  # Tamaño original de Cornell
                    scale_x = w / orig_w
                    scale_y = h / orig_h
                    
                    gt_all[:, 0] = gt_all[:, 0] * scale_x  # cx
                    gt_all[:, 1] = gt_all[:, 1] * scale_y  # cy
                    gt_all[:, 2] = gt_all[:, 2] * scale_x  # w
                    gt_all[:, 3] = gt_all[:, 3] * scale_y  # h
                    # angle_deg ya está en grados, no necesita escala
                    
                    # Calcular métricas contra todos los GT
                    ious = iou_axis_aligned_boxes(pred_single, gt_all)
                    angles = angle_error_deg(pred_single[:, 4], gt_all[:, 4])
                    successes = cornell_success(pred_single, gt_all)
                    
                    # Tomar el MEJOR match (máximo IoU, mínimo ángulo, success si al menos uno)
                    all_ious.append(float(np.max(ious)))
                    all_angles.append(float(np.min(angles)))
                    all_successes.append(float(np.any(successes)))
                else:
                    # Fallback: usar el GT del batch (modo antiguo)
                    gt_np = y[i:i+1].detach().cpu().numpy().copy()
                    gt_np = np.nan_to_num(gt_np, nan=0.0, posinf=1.0, neginf=-1.0)
                    
                    gt_np[:, 0] = gt_np[:, 0] * w
                    gt_np[:, 1] = gt_np[:, 1] * h
                    gt_np[:, 2] = np.abs(gt_np[:, 2] * w)
                    gt_np[:, 3] = np.abs(gt_np[:, 3] * h)
                    gt_np[:, 4] = gt_np[:, 4] * 90.0
                    
                    iou = iou_axis_aligned_boxes(pred_single, gt_np)[0]
                    ang = angle_error_deg(pred_single[:, 4], gt_np[:, 4])[0]
                    succ = cornell_success(pred_single, gt_np)[0]
                    
                    all_ious.append(float(iou))
                    all_angles.append(float(ang))
                    all_successes.append(float(succ))

        val_success = float(np.mean(all_successes)) if all_successes else float("nan")
        val_iou = float(np.mean(all_ious)) if all_ious else float("nan")
        val_angle_deg = float(np.mean(all_angles)) if all_angles else float("nan")
        val_loss = float(np.mean(losses)) if losses else float("nan")
        return EvalResult(val_success=val_success, val_iou=val_iou, val_angle_deg=val_angle_deg, val_loss=val_loss)

    @torch.no_grad()
    def save_predictions(self, output_csv: str | Path) -> None:
        out = Path(output_csv)
        out.parent.mkdir(parents=True, exist_ok=True)
        rows = []
        self.model.eval()
        for batch in self.dataloader:
            x, y, names = batch
            pred = self.model(x.to(self.device)).cpu().numpy()
            gt = y.numpy()

            h = float(x.shape[2])
            w = float(x.shape[3])
            pred = np.nan_to_num(pred, nan=0.0, posinf=1.0, neginf=-1.0)
            gt = np.nan_to_num(gt, nan=0.0, posinf=1.0, neginf=-1.0)

            pred[:, 0] = pred[:, 0] * w
            pred[:, 1] = pred[:, 1] * h
            pred[:, 2] = np.abs(pred[:, 2] * w)
            pred[:, 3] = np.abs(pred[:, 3] * h)
            pred[:, 4] = pred[:, 4] * 90.0

            gt[:, 0] = gt[:, 0] * w
            gt[:, 1] = gt[:, 1] * h
            gt[:, 2] = np.abs(gt[:, 2] * w)
            gt[:, 3] = np.abs(gt[:, 3] * h)
            gt[:, 4] = gt[:, 4] * 90.0

            for name, p, g in zip(names, pred, gt):
                rows.append(
                    {
                        "image_path": name,
                        "pred_cx": p[0],
                        "pred_cy": p[1],
                        "pred_w": p[2],
                        "pred_h": p[3],
                        "pred_angle_deg": p[4],
                        "gt_cx": g[0],
                        "gt_cy": g[1],
                        "gt_w": g[2],
                        "gt_h": g[3],
                        "gt_angle_deg": g[4],
                    }
                )
        pd.DataFrame(rows).to_csv(out, index=False)
