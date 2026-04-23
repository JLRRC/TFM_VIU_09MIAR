#!/usr/bin/env python3
"""Valida que los artefactos oficiales no incluyan experimentos auxiliares."""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd


OFFICIAL_EXPERIMENTS = {
    "EXP1_SIMPLE_RGB",
    "EXP2_SIMPLE_RGBD",
    "EXP3_RESNET18_RGB_AUGMENT",
    "EXP4_RESNET18_RGBD",
}


def _check_csv_scope(csv_path: Path) -> None:
    if not csv_path.exists():
        raise FileNotFoundError(f"No existe el artefacto esperado: {csv_path}")
    df = pd.read_csv(csv_path)
    if df.empty or "experiment" not in df.columns:
        return
    leaked = sorted(set(df["experiment"].dropna().astype(str).tolist()) - OFFICIAL_EXPERIMENTS)
    if leaked:
        raise ValueError(f"Artefacto oficial contaminado por experimentos auxiliares en {csv_path}: {leaked}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--summary", default="reports/tables/summary_results.csv")
    ap.add_argument("--results-by-seed", default="reports/tables/results_by_seed.csv")
    args = ap.parse_args()

    _check_csv_scope(Path(args.summary))
    _check_csv_scope(Path(args.results_by_seed))
    print("[OK] Scope oficial validado: solo EXP1..EXP4 en artefactos oficiales")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
