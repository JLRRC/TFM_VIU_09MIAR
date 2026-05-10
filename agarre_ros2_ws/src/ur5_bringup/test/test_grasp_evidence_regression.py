# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/test_grasp_evidence_regression.py
# Contenido: T26 — regresión del agarre físico (evidencia hard del log live).
"""T26 — regresión del agarre físico via ATTACH_BACKEND log evidence.

Valida que un log de validación live (`ros2_launch.log`) contenga la
evidencia hard del agarre:

    [ATTACH_BACKEND] demo_transport_follow_tick
        object=<name> mode=world_locked
        desired=(x_obj, y_obj, z_obj)
        tcp=(x_tcp, y_tcp, z_tcp)

con `tcp_obj_dist` ≤ 0.05m (pinzas envolviendo el objeto) y `tcp_z` > 0.05m
sobre la mesa (objeto levantado), confirmando agarre físico real.

Este test NO arranca el stack — lee el log existente. Skip si no lo hay.

Para forzar la verificación, ejecutar primero:
    PICK_E2E_LIVE=1 PICK_VALIDATE_CYCLES=1 python3 -m pytest \\
        src/ur5_bringup/test/test_e2e_pick_cycles.py
y luego:
    python3 -m pytest src/ur5_bringup/test/test_grasp_evidence_regression.py
"""
from __future__ import annotations

import math
import re
from pathlib import Path

import pytest


WS_DIR = Path(__file__).resolve().parents[3]
ROS2_LAUNCH_LOG = WS_DIR / "log" / "ros2_launch.log"

# Patrón del log de attach:
# [ATTACH_BACKEND] demo_transport_follow_tick object=X mode=Y
#   desired=(x,y,z) tcp=(x,y,z) tcp_src=Z
ATTACH_FOLLOW_TICK_PATTERN = re.compile(
    r"\[ATTACH_BACKEND\] demo_transport_follow_tick\s+"
    r"object=(?P<obj>\S+)\s+"
    r"mode=(?P<mode>\S+)\s+"
    r"desired=\(\s*(?P<dx>-?\d+\.?\d*)\s*,\s*(?P<dy>-?\d+\.?\d*)\s*,\s*(?P<dz>-?\d+\.?\d*)\s*\)\s+"
    r"tcp=\(\s*(?P<tx>-?\d+\.?\d*)\s*,\s*(?P<ty>-?\d+\.?\d*)\s*,\s*(?P<tz>-?\d+\.?\d*)\s*\)"
)

# Tolerancias para considerar agarre válido.
MAX_TCP_OBJ_DIST_M = 0.050      # pinzas tocando objeto
MIN_TCP_Z_LIFTED_M = 1.000      # TCP levantado sobre mesa (mesa Z=0.875)


def _read_log_text() -> str:
    if not ROS2_LAUNCH_LOG.is_file():
        return ""
    return ROS2_LAUNCH_LOG.read_text(encoding="utf-8", errors="replace")


def test_grasp_evidence_log_exists_or_skip():
    """El log existe (válido para un test posterior real) o se salta."""
    if not ROS2_LAUNCH_LOG.is_file():
        pytest.skip(
            f"Log no encontrado en {ROS2_LAUNCH_LOG}. "
            "Ejecuta primero PICK_E2E_LIVE=1 pytest src/ur5_bringup/test/test_e2e_pick_cycles.py"
        )
    assert ROS2_LAUNCH_LOG.stat().st_size > 0


def test_grasp_evidence_attach_follow_tick_present():
    """Hay al menos un tick de attach con objeto en world_locked."""
    text = _read_log_text()
    if not text:
        pytest.skip("Log vacío o ausente.")
    matches = list(ATTACH_FOLLOW_TICK_PATTERN.finditer(text))
    if not matches:
        pytest.skip(
            "No hay logs de [ATTACH_BACKEND] demo_transport_follow_tick. "
            "El último ciclo pick no llegó a la fase de transport con attach activo."
        )
    # Verificar que al menos uno está en world_locked (attach físico).
    locked = [m for m in matches if m.group("mode") == "world_locked"]
    assert locked, (
        f"{len(matches)} ticks de attach pero ninguno en mode=world_locked. "
        "El attach lógico no se activó correctamente."
    )


def test_grasp_evidence_tcp_close_to_object():
    """En al menos un tick, TCP↔objeto ≤ 5cm (pinzas envolviendo)."""
    text = _read_log_text()
    if not text:
        pytest.skip("Log vacío o ausente.")
    matches = list(ATTACH_FOLLOW_TICK_PATTERN.finditer(text))
    if not matches:
        pytest.skip("No hay ticks de attach.")

    valid_distances = []
    for m in matches:
        if m.group("mode") != "world_locked":
            continue
        dx = float(m.group("dx")) - float(m.group("tx"))
        dy = float(m.group("dy")) - float(m.group("ty"))
        dz = float(m.group("dz")) - float(m.group("tz"))
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        valid_distances.append(dist)

    if not valid_distances:
        pytest.skip("No hay ticks en world_locked con datos parseables.")

    min_dist = min(valid_distances)
    assert min_dist <= MAX_TCP_OBJ_DIST_M, (
        f"TCP↔objeto mínimo: {min_dist*1000:.1f}mm > tolerance {MAX_TCP_OBJ_DIST_M*1000:.1f}mm. "
        f"Las pinzas no llegaron a tocar el objeto en ningún tick "
        f"({len(valid_distances)} ticks medidos, distancia mediana "
        f"{sorted(valid_distances)[len(valid_distances)//2]*1000:.1f}mm)."
    )


def test_grasp_evidence_object_lifted_above_table():
    """En al menos un tick, el TCP está levantado sobre la mesa (Z > 1.0m)."""
    text = _read_log_text()
    if not text:
        pytest.skip("Log vacío o ausente.")
    matches = list(ATTACH_FOLLOW_TICK_PATTERN.finditer(text))
    if not matches:
        pytest.skip("No hay ticks de attach.")

    tcp_zs = []
    for m in matches:
        if m.group("mode") != "world_locked":
            continue
        tcp_zs.append(float(m.group("tz")))

    if not tcp_zs:
        pytest.skip("No hay ticks en world_locked.")

    max_z = max(tcp_zs)
    assert max_z >= MIN_TCP_Z_LIFTED_M, (
        f"TCP Z máximo: {max_z:.3f}m < tolerance {MIN_TCP_Z_LIFTED_M:.3f}m. "
        "El robot no levantó el objeto sobre la mesa."
    )


def test_grasp_evidence_object_pose_summary():
    """Resumen estadístico (no falla, solo informa) del agarre detectado."""
    text = _read_log_text()
    if not text:
        pytest.skip("Log vacío o ausente.")
    matches = list(ATTACH_FOLLOW_TICK_PATTERN.finditer(text))
    locked = [m for m in matches if m.group("mode") == "world_locked"]
    if not locked:
        pytest.skip("No hay ticks en world_locked.")

    distances = []
    for m in locked:
        dx = float(m.group("dx")) - float(m.group("tx"))
        dy = float(m.group("dy")) - float(m.group("ty"))
        dz = float(m.group("dz")) - float(m.group("tz"))
        distances.append(math.sqrt(dx * dx + dy * dy + dz * dz))

    print(
        f"\n[T26 grasp_evidence] {len(locked)} ticks en world_locked\n"
        f"  TCP↔objeto: min={min(distances)*1000:.1f}mm "
        f"max={max(distances)*1000:.1f}mm "
        f"avg={sum(distances)/len(distances)*1000:.1f}mm\n"
        f"  TCP Z: min={min(float(m.group('tz')) for m in locked):.3f}m "
        f"max={max(float(m.group('tz')) for m in locked):.3f}m"
    )
    # F-iter4 audit (2026-05-10): test informativo — sin assert real
    # (los datos se imprimen para análisis manual). Mantener vacío
    # explícitamente con `return` documenta la intención.
    return
