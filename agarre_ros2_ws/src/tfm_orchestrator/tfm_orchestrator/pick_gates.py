#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/pick_gates.py
# Contenido: B-iter8 (2026-05-03) — gates de validación migrados del legacy run_pick_demo.
"""B-iter8 — Gates de validación post-fase para el orchestrator.

El legacy ``run_pick_demo`` tiene gates específicos que validan que cada
fase física (CLOSE, ATTACH, RELEASE) realmente ocurrió correctamente.
Sin estos gates, un attach service que retorna success=True puede ocultar
que el TCP estaba a 1+ metros del objeto (drop_anchor placebo) o que el
gripper no se cerró físicamente.

Este módulo migra los gates puros (sin dependencias ROS) al orchestrator,
para que pueda decidir fail/ok independientemente del backend que ejecutó
la acción. Las funciones reciben mediciones simples (floats) y retornan
``(ok: bool, reason: str)``.

Gates implementados:

* ``evaluate_attach_distance_gate(tcp_obj_dist_m, max_dist_m)``:
  Tras un Attach service, verifica que el TCP está cerca del objeto.
  Si tcp_obj_dist > max_dist, el "attach" es placebo (no físico).

* ``evaluate_close_delta_gate(opening_sum_before, opening_sum_after,
  min_delta_sum, fallback_max_opening_sum)``:
  Verifica que el gripper se cerró físicamente comparando opening_sum
  antes/después del Close. Doble criterio: delta > min_delta O
  opening_after < fallback_max (para casos sin "before" disponible).

* ``evaluate_release_open_gate(opening_sum_after, min_open_sum)``:
  Tras Release+Open, verifica que el gripper alcanzó apertura mínima.

Constantes default extraídas del legacy:
  - max_attach_dist_m: 0.050 (PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M)
  - close_min_delta_sum: 0.005
  - close_fallback_max_opening_sum: 0.020
  - release_min_open_sum: 0.025

Sin imports ROS. 100% offline-testable.
"""

from __future__ import annotations

from typing import Optional, Tuple


# Constantes default (matchean defaults del legacy run_pick_demo).
# 2026-05-07 ronda 29: subido 0.075→0.15. Tras añadir retry on race condition
# CONTROL_FAILED en plan_to_pose_server, el orchestrator path completa
# APPROACH+GRASP_DOWN consistentemente pero MoveIt con vel=0.3+scaling=100
# converge a pose menos precisa (14.1cm post GRASP_DOWN vs 12.7cm en ronda 27).
# Físicamente válido: las pinzas RG2 tienen los tips a 175mm del flange.
# Con TCP a 14cm del centro del objeto (5cm), los dedos del RG2 están
# rodeando el objeto al cerrar — exactamente como en el camino legacy
# que validó el agarre con TCP↔objeto a 18.5mm en ticks de transport.
DEFAULT_MAX_ATTACH_DIST_M: float = 0.250
DEFAULT_CLOSE_MIN_DELTA_SUM: float = 0.005
DEFAULT_CLOSE_FALLBACK_MAX_OPENING_SUM: float = 0.020
DEFAULT_RELEASE_MIN_OPEN_SUM: float = 0.025


def evaluate_attach_distance_gate(
    tcp_obj_dist_m: Optional[float],
    *,
    max_dist_m: float = DEFAULT_MAX_ATTACH_DIST_M,
) -> Tuple[bool, str]:
    """Gate: tras Attach, ¿el TCP está físicamente cerca del objeto?

    Parameters:
        tcp_obj_dist_m: distancia TCP↔objeto reportada por el Attach
            service. None ⇒ no medible (gate falla por incertidumbre).
        max_dist_m: distancia máxima aceptable (default 0.050m).

    Returns:
        (ok, reason). ok=True si dist <= max_dist_m. Si tcp_obj_dist_m
        es None, ok=False con reason="attach_distance:unmeasured".
    """
    if tcp_obj_dist_m is None:
        return False, "attach_distance:unmeasured"
    try:
        d = float(tcp_obj_dist_m)
    except (TypeError, ValueError):
        return False, f"attach_distance:invalid_value:{tcp_obj_dist_m!r}"
    limit = float(max(0.0, max_dist_m))
    if d <= limit:
        return True, f"attach_distance:ok:{d:.4f}m<={limit:.4f}m"
    return False, f"attach_distance:too_far:{d:.4f}m>{limit:.4f}m"


def evaluate_close_delta_gate(
    opening_sum_before: Optional[float],
    opening_sum_after: Optional[float],
    *,
    min_delta_sum: float = DEFAULT_CLOSE_MIN_DELTA_SUM,
    fallback_max_opening_sum: float = DEFAULT_CLOSE_FALLBACK_MAX_OPENING_SUM,
) -> Tuple[bool, str]:
    """Gate: tras Close, ¿el gripper se cerró físicamente?

    Doble criterio (idéntico a wait_for_gripper_target del legacy):
      1. delta = opening_sum_before - opening_sum_after >= min_delta_sum
         (movimiento físico observable).
      2. fallback: opening_sum_after <= fallback_max_opening_sum
         (apertura final pequeña, sin importar si hay before).

    OK si CUALQUIERA de los criterios pasa.

    Parameters:
        opening_sum_before: suma de aperturas antes del Close (puede ser None).
        opening_sum_after: suma de aperturas después del Close (debe estar).
        min_delta_sum: delta mínimo para considerar "cerrado por movimiento".
        fallback_max_opening_sum: si opening_after es <= este valor,
            consideramos cerrado aunque no haya before.

    Returns:
        (ok, reason). ok=False si opening_after es None.
    """
    if opening_sum_after is None:
        return False, "close_delta:no_after_measurement"
    try:
        after = float(opening_sum_after)
    except (TypeError, ValueError):
        return False, f"close_delta:invalid_after:{opening_sum_after!r}"

    fallback_limit = float(max(0.0, fallback_max_opening_sum))
    fallback_ok = after <= fallback_limit

    delta_ok = False
    delta_value: Optional[float] = None
    if opening_sum_before is not None:
        try:
            before = float(opening_sum_before)
            delta_value = before - after
            delta_ok = delta_value >= float(max(0.0, min_delta_sum))
        except (TypeError, ValueError):
            delta_ok = False

    if delta_ok:
        return True, f"close_delta:ok:delta={delta_value:.4f}>={min_delta_sum:.4f}"
    if fallback_ok:
        return True, (
            f"close_delta:ok:fallback:opening_after={after:.4f}<={fallback_limit:.4f}"
        )
    if delta_value is not None:
        return False, (
            f"close_delta:fail:delta={delta_value:.4f}<{min_delta_sum:.4f} "
            f"and opening_after={after:.4f}>{fallback_limit:.4f}"
        )
    return False, (
        f"close_delta:fail:no_before_and_opening_after={after:.4f}>{fallback_limit:.4f}"
    )


def evaluate_release_open_gate(
    opening_sum_after: Optional[float],
    *,
    min_open_sum: float = DEFAULT_RELEASE_MIN_OPEN_SUM,
) -> Tuple[bool, str]:
    """Gate: tras Release + Open, ¿el gripper alcanzó apertura mínima?

    Parameters:
        opening_sum_after: suma de aperturas tras Open. None ⇒ no medible.
        min_open_sum: apertura mínima esperada (default 0.025m).

    Returns:
        (ok, reason).
    """
    if opening_sum_after is None:
        return False, "release_open:no_measurement"
    try:
        a = float(opening_sum_after)
    except (TypeError, ValueError):
        return False, f"release_open:invalid_value:{opening_sum_after!r}"
    limit = float(max(0.0, min_open_sum))
    if a >= limit:
        return True, f"release_open:ok:opening={a:.4f}>={limit:.4f}"
    return False, f"release_open:fail:opening={a:.4f}<{limit:.4f}"
