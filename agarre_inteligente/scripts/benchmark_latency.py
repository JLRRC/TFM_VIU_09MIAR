#!/usr/bin/env python3
"""Benchmark de latencia/FPS para checkpoints de experimentos."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import pandas as pd
import torch

from src.models.resnet_variants import ResNetGrasp
from src.models.simple_cnn import SimpleCNN
from src.utils.config_loader import load_config


def build_model(model_cfg: dict):
    if model_cfg["name"] == "SimpleGraspCNN":
        return SimpleCNN(input_channels=int(model_cfg["input_channels"]))
    return ResNetGrasp(input_channels=int(model_cfg["input_channels"]), pretrained=False)


def measure(model, x, warmup: int, repeats: int, device: str):
    model.eval()
    with torch.no_grad():
        for _ in range(warmup):
            _ = model(x)
            if device == "cuda":
                torch.cuda.synchronize()

        times = []
        for _ in range(repeats):
            t0 = time.perf_counter()
            _ = model(x)
            if device == "cuda":
                torch.cuda.synchronize()
            t1 = time.perf_counter()
            times.append((t1 - t0) * 1000.0)
    arr = np.array(times)
    return float(arr.mean()), float(arr.std()), float(np.percentile(arr, 95)), float(1000.0 / arr.mean())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--output", default="reports/bench/latency_results.csv")
    ap.add_argument("--warmup", type=int, default=20)
    ap.add_argument("--repeats", type=int, default=100)
    args = ap.parse_args()

    cfg = load_config(args.config)
    device = cfg["benchmark"].get("device", "cpu")
    if device == "cuda" and not torch.cuda.is_available():
        device = "cpu"

    model = build_model(cfg["model"]).to(device)
    state = torch.load(args.checkpoint, map_location=device)
    model.load_state_dict(state["model_state_dict"])

    channels = int(cfg["model"]["input_channels"])
    image_size = int(cfg["data"].get("image_size", 224))
    x = torch.randn(1, channels, image_size, image_size, device=device)

    mean_ms, std_ms, p95_ms, fps = measure(model, x, args.warmup, args.repeats, device)
    n_params = sum(p.numel() for p in model.parameters())
    model_size_mb = sum(p.numel() * p.element_size() for p in model.parameters()) / (1024 * 1024)

    row = {
        "experiment": cfg["experiment"]["name"],
        "device": device,
        "batch_size": 1,
        "latency_ms_mean": mean_ms,
        "latency_ms_std": std_ms,
        "latency_ms_p95": p95_ms,
        "fps": fps,
        "n_params": int(n_params),
        "model_size_mb": model_size_mb,
    }

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        df = pd.read_csv(out)
        df = pd.concat([df, pd.DataFrame([row])], ignore_index=True)
        df.to_csv(out, index=False)
    else:
        pd.DataFrame([row]).to_csv(out, index=False)

    print(f"[OK] benchmark guardado en {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
