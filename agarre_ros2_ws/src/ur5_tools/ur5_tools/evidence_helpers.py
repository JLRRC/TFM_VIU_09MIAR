#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/evidence_helpers.py
# Contenido: F4 — helpers puros del evidence_logger (sin rclpy).
"""Helpers puros del evidence_logger.

Extraído de ``evidence_logger.py`` para que sean importables sin
rclpy. El propio ``evidence_logger`` re-exporta las funciones para no
romper consumidores.
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path


def now_iso() -> str:
    """Timestamp ISO 8601 UTC con microsegundos, sufijo ``Z``."""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def safe_unique_dir(root: Path) -> Path:
    """Crea un directorio nuevo bajo ``root`` con timestamp + sufijo si colisiona."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    candidate = root / stamp
    suffix = 1
    while candidate.exists():
        candidate = root / f"{stamp}_{suffix}"
        suffix += 1
    candidate.mkdir(parents=True, exist_ok=False)
    return candidate
