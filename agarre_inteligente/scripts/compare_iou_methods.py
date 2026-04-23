#!/usr/bin/env python3
"""Compara las métricas axis-aligned (oficial TFM) vs orientadas (alineación metodológica)
sobre un checkpoint existente de EXP1..EXP4.

Uso:
    python3 scripts/compare_iou_methods.py \
        --checkpoint experiments/EXP3_RESNET18_RGB_AUGMENT/seed_0/checkpoints/best.pth \
        --config config/exp3_resnet18_rgb_augment.yaml

No modifica nada. Es solo lectura + evaluación.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# El script puede ejecutarse desde cualquier directorio. Cambiamos a PROJECT_ROOT
# para que las rutas relativas del YAML (data_root, val_csv) resuelvan correctamente.
import os
_orig_cwd = Path(os.getcwd()).resolve()
os.chdir(PROJECT_ROOT)

import torch
from torch import nn
from torch.utils.data import DataLoader

from src.data.grasp_dataset import GraspDataset
from src.data.transforms import get_val_transforms
from src.evaluation.evaluator import Evaluator, EvaluatorOriented
from src.models.factory import build_model
from src.utils.config_loader import load_config


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True, help="Ruta al best.pth")
    ap.add_argument("--config", required=True, help="YAML del experimento")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--batch-size", type=int, default=32)
    args = ap.parse_args()

    # Resolver rutas absolutas ANTES del chdir (ya aplicado al importar)
    checkpoint_path = ((_orig_cwd / args.checkpoint) if not Path(args.checkpoint).is_absolute()
                       else Path(args.checkpoint)).resolve()
    config_path = ((_orig_cwd / args.config) if not Path(args.config).is_absolute()
                   else Path(args.config)).resolve()

    cfg = load_config(str(config_path))
    device = args.device
    image_size = int(cfg["data"].get("image_size", 224))
    val_csv = cfg["data"]["val_csv"]
    data_root = cfg["data"]["data_root"]
    modality = cfg["data"]["modality"]

    # data_root en los YAMLs oficiales es "." relativo a la raíz del workspace TFM
    # (donde se ejecutó el entrenamiento, /home/laboratorio/TFM/).
    # El script hace chdir a agarre_inteligente/, así que usamos el directorio padre.
    effective_data_root = str(PROJECT_ROOT.parent) if data_root in (".", "") else data_root
    val_ds = GraspDataset(
        val_csv, effective_data_root, modality=modality,
        transform=get_val_transforms(image_size=image_size),
    )
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False)

    model = build_model(cfg["model"])
    ckpt = torch.load(str(checkpoint_path), map_location="cpu")
    state_dict = ckpt.get("model_state_dict", ckpt)
    model.load_state_dict(state_dict, strict=True)
    model.to(device)
    model.eval()
    print(f"[OK] Checkpoint cargado: {checkpoint_path}")

    criterion = nn.SmoothL1Loss()

    print("\n  Evaluando con Evaluator OFICIAL (IoU axis-aligned)...")
    ev_aa = Evaluator(model, val_loader, device, csv_path=val_csv)
    res_aa = ev_aa.evaluate(criterion)

    print("  Evaluando con EvaluatorOriented (IoU rectángulos orientados)...")
    ev_or = EvaluatorOriented(model, val_loader, device, csv_path=val_csv)
    res_or = ev_or.evaluate(criterion)

    print()
    print("=" * 60)
    print(f"  Checkpoint: {Path(args.checkpoint).parent.parent.name}")
    print("=" * 60)
    print(f"{'Métrica':<20} {'Axis-aligned (oficial)':>22} {'Orientada (metodol.)':>22} {'Delta':>10}")
    print("-" * 76)

    for name, aa, oo in [
        ("val_success", res_aa.val_success, res_or.val_success),
        ("val_iou", res_aa.val_iou, res_or.val_iou),
        ("val_angle_deg", res_aa.val_angle_deg, res_or.val_angle_deg),
        ("val_loss", res_aa.val_loss, res_or.val_loss),
    ]:
        delta = oo - aa
        sign = "+" if delta >= 0 else ""
        print(f"  {name:<18} {aa:>22.4f} {oo:>22.4f} {sign}{delta:>9.4f}")

    print()
    print("  Nota: val_loss es idéntico porque ambos usan SmoothL1Loss.")
    print("  El delta negativo en val_success es la discrepancia metodológica real.")
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
