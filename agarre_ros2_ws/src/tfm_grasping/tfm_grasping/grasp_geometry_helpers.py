#!/usr/bin/env python3
"""F8 audit (2026-05-10): helpers geométricos puros para grasp 2D→RG2.

Extraídos de ``ur5_qt_panel/panel_tfm_geometry.py`` para que el
backend (``tfm_grasping``) tenga la geometría como fuente única.

API (pura, sin ROS, sin Qt):

* :func:`clamp` — ``max(lo, min(hi, v))``.
* :func:`normalize_angle` — wrap a ``(-π, π]``.
* :func:`compute_minor_axis_from_grasp_rect` — eje fino del rectángulo
  rojo de inferencia + dirección de apertura/cierre del gripper en
  espacio de imagen.
* :func:`compute_rg2_preopen_from_minor_width` — apertura RG2 a partir
  del ancho fino del rectángulo rojo.

El panel mantiene su propio módulo ``panel_tfm_geometry.py`` como
shim por compatibilidad (re-exporta desde aquí). Refactor futuro F8b:
el panel deja de tener su copia y consume directamente desde aquí.
"""
from __future__ import annotations

import math
from typing import Tuple


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp escalar a ``[lo, hi]``."""
    return max(lo, min(hi, value))


def normalize_angle(angle: float) -> float:
    """Wrap angle a ``(-π, π]``."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def compute_minor_axis_from_grasp_rect(
    w_px: float, h_px: float, theta_img: float
) -> Tuple[float, float]:
    """Eje fino del rectángulo de inferencia.

    Devuelve ``(minor_px, opening_axis_theta_img)``.
    ``opening_axis_theta_img`` es la dirección de apertura/cierre de
    la pinza en espacio de imagen.
    """
    if w_px <= h_px:
        minor_px = w_px
        opening_axis_theta_img = normalize_angle(theta_img + math.pi / 2.0)
    else:
        minor_px = h_px
        opening_axis_theta_img = normalize_angle(theta_img)
    return minor_px, opening_axis_theta_img


def compute_rg2_preopen_from_minor_width(
    minor_width_m: float,
    safety_margin_m: float = 0.015,
    min_open_m: float = 0.015,
    max_open_m: float = 0.110,
    max_finger_rad: float = 1.18,
) -> Tuple[float, float]:
    """Pre-apertura RG2 desde el ancho fino del rectángulo rojo.

    Devuelve ``(pre_open_width_m, finger_cmd_rad)``. El comando del
    finger es lineal en ``pre_open_width_m / max_open_m`` y se clampa
    a ``[0, max_finger_rad]``.
    """
    pre_open_width_m = clamp(
        minor_width_m + safety_margin_m, min_open_m, max_open_m
    )
    finger_cmd_rad = clamp(
        (pre_open_width_m / max_open_m) * max_finger_rad, 0.0, max_finger_rad
    )
    return pre_open_width_m, finger_cmd_rad
