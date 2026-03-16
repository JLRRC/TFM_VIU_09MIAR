#!/usr/bin/env python3
"""Ejecuta todas las seeds definidas en un YAML de experimento."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.utils.config_loader import load_config


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--allow-synthetic", action="store_true")
    args = ap.parse_args()

    cfg = load_config(args.config)
    seeds = list(cfg["experiment"]["seeds"])

    for seed in seeds:
        cmd = [
            sys.executable,
            "scripts/train.py",
            "--config",
            args.config,
            "--seed",
            str(seed),
        ]
        if args.allow_synthetic:
            cmd.append("--allow-synthetic")
        print("[RUN]", " ".join(cmd))
        subprocess.run(cmd, check=True)

    print("[OK] Experimento completado para todas las seeds")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
