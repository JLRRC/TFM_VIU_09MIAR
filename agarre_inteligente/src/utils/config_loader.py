"""Carga y valida configuraciones YAML para experimentos del TFM."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

REQUIRED_TOP_LEVEL = [
    "experiment",
    "data",
    "model",
    "training",
    "metrics",
    "checkpointing",
    "outputs",
]


def load_config(config_path: str | Path) -> dict[str, Any]:
    path = Path(config_path)
    if not path.exists():
        raise FileNotFoundError(f"Config no encontrado: {path}")
    with path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    validate_config(cfg, str(path))
    return cfg


def validate_config(config: dict[str, Any], source: str = "<in-memory>") -> None:
    if not isinstance(config, dict):
        raise ValueError(f"Config invalido en {source}: se esperaba dict")
    missing = [k for k in REQUIRED_TOP_LEVEL if k not in config]
    if missing:
        raise ValueError(f"Config invalido en {source}: faltan secciones {missing}")

    exp = config["experiment"]
    if "name" not in exp:
        raise ValueError(f"Config invalido en {source}: experiment.name es obligatorio")
    if "seeds" not in exp or not exp["seeds"]:
        raise ValueError(f"Config invalido en {source}: experiment.seeds no puede estar vacio")


def save_config_snapshot(config: dict[str, Any], output_path: str | Path) -> None:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, sort_keys=False)
