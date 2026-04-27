#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tfm_geometry.py
# Contenido: Helpers geometricos puros del flujo TFM grasp.
"""Helpers geometricos puros del flujo TFM grasp.

Extraidos de ``panel_tfm.py`` y ``panel_v2.py`` (duplicados antes del
refactor) para tener una unica fuente de verdad.

Funciones publicas:

* ``tfm_clamp(value, lo, hi)`` — clamp scalar a un rango cerrado.
* ``tfm_normalize_angle(angle)`` — wrap a (-pi, pi].
* ``compute_minor_axis_from_grasp_rect(w_px, h_px, theta_img)``
  — eje fino del rectangulo de inferencia + direccion de apertura.
* ``compute_rg2_preopen_from_minor_width(minor_width_m, ...)``
  — apertura previa del RG2 desde el ancho fino del rectangulo.

Sin acceso a panel ni dependencias ROS: testeable en aislamiento.
"""
from __future__ import annotations

import math


def tfm_clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def tfm_normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def compute_minor_axis_from_grasp_rect(
    w_px: float, h_px: float, theta_img: float
) -> tuple[float, float]:
    """Eje fino del rectangulo rojo de inferencia.

    Devuelve ``(minor_px, opening_axis_theta_img)``.
    ``opening_axis_theta_img`` es la direccion de apertura/cierre de la
    pinza en imagen.
    """
    if w_px <= h_px:
        minor_px = w_px
        opening_axis_theta_img = tfm_normalize_angle(theta_img + math.pi / 2.0)
    else:
        minor_px = h_px
        opening_axis_theta_img = tfm_normalize_angle(theta_img)
    return minor_px, opening_axis_theta_img


def compute_rg2_preopen_from_minor_width(
    minor_width_m: float,
    safety_margin_m: float = 0.015,
    min_open_m: float = 0.015,
    max_open_m: float = 0.110,
    max_finger_rad: float = 1.18,
) -> tuple[float, float]:
    """Apertura previa del RG2 desde el ancho fino del rectangulo rojo."""
    pre_open_width_m = tfm_clamp(
        minor_width_m + safety_margin_m, min_open_m, max_open_m
    )
    finger_cmd_rad = tfm_clamp(
        (pre_open_width_m / max_open_m) * max_finger_rad, 0.0, max_finger_rad
    )
    return pre_open_width_m, finger_cmd_rad


# Aliases con prefijo `_` para callers historicos.
_tfm_clamp = tfm_clamp
_tfm_normalize_angle = tfm_normalize_angle
_compute_minor_axis_from_grasp_rect = compute_minor_axis_from_grasp_rect
_compute_rg2_preopen_from_minor_width = compute_rg2_preopen_from_minor_width
