"""Guardado/carga de checkpoints para modelos de agarre."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import torch


def save_checkpoint(state: dict[str, Any], path: str | Path) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    torch.save(state, out)


def load_checkpoint(path: str | Path, map_location: str = "cpu") -> dict[str, Any]:
    return torch.load(path, map_location=map_location)


@dataclass
class CheckpointManager:
    base_dir: Path

    def __init__(self, base_dir: str | Path):
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(parents=True, exist_ok=True)

    def best_path(self) -> Path:
        return self.base_dir / "best.pth"

    def last_path(self) -> Path:
        return self.base_dir / "last.pth"

    def save_best(self, state: dict[str, Any]) -> None:
        save_checkpoint(state, self.best_path())

    def save_last(self, state: dict[str, Any]) -> None:
        save_checkpoint(state, self.last_path())
