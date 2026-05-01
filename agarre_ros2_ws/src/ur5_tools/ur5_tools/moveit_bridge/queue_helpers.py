#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/moveit_bridge/queue_helpers.py
# Contenido: F3 — helpers puros del queue management de UR5MoveItBridge.
"""Helpers puros del queue / settle params del MoveIt bridge.

Extraídos de ``_plan_worker`` y ``_pose_callback``:

* ``compute_request_stamp_ns(stamp_obj)``: combina sec + nanosec.
* ``is_invalid_business_frame(frame_clean, base_frame)``: rechaza
  frames "base"/"/base" cuando el bridge está en base_link.
* ``compute_settle_params(env_getter, joint_state_valid_timeout_sec)``:
  los 3 valores de joint settle (timeout, stable, tol) con clamps.
* ``stale_request_age(queued_mono, now_mono)``: edad en segundos
  clamped a >=0.
* ``is_stale_request(queued_mono, now_mono, ttl_sec)``: True si edad
  excede TTL (TTL <=0 desactiva el check).

Cero dependencia ROS — ``env_getter`` se inyecta como callable.
"""

from __future__ import annotations

from typing import Any, Callable, Tuple


def compute_request_stamp_ns(stamp_obj: Any) -> int:
    """Combina ``sec*1e9 + nanosec`` de un stamp ROS, robusto a None."""
    if stamp_obj is None:
        return 0
    sec = int(getattr(stamp_obj, "sec", 0) or 0)
    nanosec = int(getattr(stamp_obj, "nanosec", 0) or 0)
    return sec * 1_000_000_000 + nanosec


def is_invalid_business_frame(frame_clean: str, base_frame: str) -> bool:
    """True si ``frame_clean`` es ``base``/``/base`` y el bridge usa base_link.

    Razón histórica: alias incorrecto del panel cuando el bridge requiere
    explícitamente ``base_link`` para evitar ambigüedad en la cadena TF.
    """
    return frame_clean in ("base", "/base") and base_frame == "base_link"


def compute_settle_params(
    env_getter: Callable[[str, float], float],
    joint_state_valid_timeout_sec: float,
) -> Tuple[float, float, float]:
    """Calcula los 3 settle params con sus clamps.

    Devuelve ``(settle_timeout_sec, settle_stable_sec, settle_tol_rad)``.

    Defaults y clamps:
      * timeout: max(0.4, env "...JOINT_SETTLE_TIMEOUT_SEC" o
        min(1.5, max(0.4, joint_state_valid_timeout_sec)))
      * stable:  max(0.05, env "...JOINT_SETTLE_STABLE_SEC" o 0.25)
      * tol_rad: max(0.005, env "...JOINT_SETTLE_TOL_RAD" o 0.02)
    """
    default_timeout = min(1.5, max(0.4, float(joint_state_valid_timeout_sec)))
    settle_timeout_sec = max(
        0.4,
        env_getter("PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_TIMEOUT_SEC", default_timeout),
    )
    settle_stable_sec = max(
        0.05,
        env_getter("PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_STABLE_SEC", 0.25),
    )
    settle_tol_rad = max(
        0.005,
        env_getter("PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_TOL_RAD", 0.02),
    )
    return settle_timeout_sec, settle_stable_sec, settle_tol_rad


def stale_request_age(queued_mono: float, now_mono: float) -> float:
    """Edad en segundos del request encolado, clamped a >=0."""
    return max(0.0, float(now_mono) - float(queued_mono))


def is_stale_request(
    queued_mono: float,
    now_mono: float,
    ttl_sec: float,
) -> Tuple[bool, float]:
    """Devuelve ``(is_stale, age_sec)``. Si TTL <=0 nunca se considera stale."""
    age = stale_request_age(queued_mono, now_mono)
    if float(ttl_sec) <= 0.0:
        return False, age
    return age > float(ttl_sec), age
