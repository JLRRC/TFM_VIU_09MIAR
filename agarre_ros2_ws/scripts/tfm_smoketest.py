#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/scripts/tfm_smoketest.py
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
"""Minimal smoke tests for the TFM grasping pipeline."""

from __future__ import annotations

import importlib
import os
import sys
from typing import Optional


def _note(level: str, msg: str) -> None:
    print(f"[TFM_SMOKETEST] {level}: {msg}")


def _check(cond: bool, ok_msg: str, fail_msg: str, *, fatal: bool = False) -> int:
    if cond:
        _note("OK", ok_msg)
        return 0
    _note("FAIL" if fatal else "WARN", fail_msg)
    return 1 if fatal else 0


def _try_import(name: str) -> bool:
    try:
        importlib.import_module(name)
        _note("OK", f"import {name}")
        return True
    except Exception as exc:
        _note("WARN", f"no import {name}: {exc}")
        return False


def _add_path(path: str) -> None:
    if path and os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)


def _check_ckpt(path: Optional[str]) -> None:
    if not path:
        _note("WARN", "INFER_CKPT no definido")
        return
    _check(os.path.isfile(path), "INFER_CKPT existe", f"INFER_CKPT no existe: {path}")


def _check_script(path: Optional[str]) -> None:
    if not path:
        _note("WARN", "INFER_SCRIPT no definido")
        return
    _check(os.path.isfile(path), "INFER_SCRIPT existe", f"INFER_SCRIPT no existe: {path}")


def main() -> int:
    failures = 0

    vision_dir = os.path.expanduser(os.environ.get("VISION_DIR", "~/TFM/agarre_inteligente"))
    _note("INFO", f"VISION_DIR={vision_dir}")
    failures += _check(os.path.isdir(vision_dir), "VISION_DIR existe", f"VISION_DIR no existe: {vision_dir}")

    ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    py_ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    ws_site = os.path.join(ws_dir, "install", "ur5_qt_panel", "lib", py_ver, "site-packages")
    _add_path(ws_site)
    tfm_site = os.path.join(ws_dir, "install", "tfm_grasping", "lib", py_ver, "site-packages")
    _add_path(tfm_site)
    _add_path(os.path.join(ws_dir, "build", "tfm_grasping"))

    try:
        from ur5_qt_panel import panel_config as cfg
    except Exception as exc:
        _note("FAIL", f"no import ur5_qt_panel.panel_config: {exc}")
        return 1

    _check_script(getattr(cfg, "INFER_SCRIPT", None))
    _check_ckpt(getattr(cfg, "INFER_CKPT", None))

    try:
        import tfm_grasping
        _note("OK", f"import tfm_grasping ({tfm_grasping.__file__})")
    except Exception as exc:
        _note("FAIL", f"no import tfm_grasping: {exc}")
        return 1

    try:
        from tfm_grasping.geometry import Grasp2D

        g = Grasp2D(center_x=10.0, center_y=20.0, angle_rad=0.1, width_px=30.0, quality=0.9)
        d = g.to_dict()
        failures += _check(isinstance(d, dict), "Grasp2D.to_dict OK", "Grasp2D.to_dict fallo", fatal=True)
    except Exception as exc:
        _note("FAIL", f"Grasp2D error: {exc}")
        return 1

    _try_import("numpy")
    _try_import("cv2")
    _try_import("torch")

    _add_path(vision_dir)
    _add_path(os.path.join(vision_dir, "src"))
    # Backward/forward compatibility with historical and current ML layouts.
    if not _try_import("graspnet.utils.metrics"):
        _try_import("training.metrics")

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
