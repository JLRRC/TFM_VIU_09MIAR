#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_object/parsing_helpers.py
# Contenido: F3 — helpers puros de parsing/format extraídos de panel_pick_object.run_pick_object.
"""Helpers puros de parsing y validación para pick_object flow.

Extraídos de las closures internas de ``run_pick_object``:

* ``norm_frame(name)``: normaliza un frame_id (strip + lstrip('/')).
* ``tf_stamp_ns(tf_msg)``: extrae timestamp en ns de un TF message
  (acepta cualquier objeto con ``.header.stamp.sec`` y ``.header.stamp.nanosec``).
* ``sanitize_ros_ns(raw)``: convierte raw a int ns, descartando
  valores ≤0 y wall-time epoch (>1e15) usado en checks de sim-time.
* ``is_step_tf_mismatch(msg, label)``: pattern check sobre mensaje.
* ``is_step_exec_failed(msg, label)``: pattern check con varias variantes.
* ``max_joint_error(current, target)``: error articular máximo absoluto.

Cero dependencia ROS run-time — testeable en pytest puro.
"""

from __future__ import annotations

from typing import List, Optional


def norm_frame(frame_name: object) -> str:
    """Normaliza ``frame_name``: strip + lstrip('/'). Vacío si None."""
    return (str(frame_name) if frame_name is not None else "").strip().lstrip("/")


def tf_stamp_ns(tf_msg: Optional[object]) -> int:
    """Devuelve timestamp en ns del header de un TF/stamped msg.

    Acepta cualquier objeto con ``msg.header.stamp.sec`` y
    ``msg.header.stamp.nanosec``. Si falla la lectura devuelve 0.
    """
    try:
        stamp = tf_msg.header.stamp  # type: ignore[attr-defined]
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except Exception:
        return 0


_WALLTIME_EPOCH_THRESHOLD_NS = 1_000_000_000_000_000


def sanitize_ros_ns(raw: object) -> int:
    """Convierte ``raw`` a int ns válido para sim-time checks.

    Descarta:
      * valores ≤ 0
      * valores > 1e15 (epoch wall-time, no sim-time)

    Devuelve 0 si raw no es convertible o no pasa los checks.
    """
    try:
        value = int(raw or 0)
    except Exception:
        return 0
    if value <= 0:
        return 0
    if value > _WALLTIME_EPOCH_THRESHOLD_NS:
        return 0
    return value


def is_step_tf_mismatch(msg: object, label: object) -> bool:
    """True si ``msg`` indica TF mismatch en el step ``label``."""
    text = str(msg)
    return "exec_succeeded_but_tf_mismatch" in text and f"label={label}" in text


def is_step_exec_failed(msg: object, label: object) -> bool:
    """True si ``msg`` indica execute failure en el step ``label``."""
    txt = str(msg)
    label_match = f"label={label}"
    return (
        f"execute failed ({label})" in txt
        or (label_match in txt and "exec_failed:" in txt)
        or (label_match in txt and "fjt_result_timeout" in txt)
        or (label_match in txt and "deterministic_joint_mode" in txt)
    )


def max_joint_error(
    current: Optional[List[float]],
    target: Optional[List[float]],
) -> float:
    """Error articular máximo absoluto entre ``current`` y ``target``.

    Devuelve ``inf`` si cualquiera de los dos está vacío.
    """
    if not current or not target:
        return float("inf")
    return max(
        abs(float(curr) - float(goal))
        for curr, goal in zip(current, target)
    )
