"""Utilidades de logging para ejecucion reproducible de experimentos."""

from __future__ import annotations

import logging
from pathlib import Path


def setup_logger(name: str, log_file: str | Path, level: int = logging.INFO) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(level)

    if logger.handlers:
        return logger

    Path(log_file).parent.mkdir(parents=True, exist_ok=True)

    fmt = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")

    file_handler = logging.FileHandler(log_file)
    file_handler.setFormatter(fmt)
    logger.addHandler(file_handler)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(fmt)
    logger.addHandler(stream_handler)

    return logger


def get_logger(name: str) -> logging.Logger:
    return logging.getLogger(name)
