#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/test_interfaces_contracts.py
# Contenido: F4 — verificación estática de contratos action/srv del paquete
#            ur5_panel_interfaces.
"""Tests de contrato para action/srv definidas en ur5_panel_interfaces.

Parsea los ficheros ``.action`` / ``.srv`` y verifica que contienen los
campos esperados (nombres + tipos). NO genera interfaces ni requiere
ROS — sólo lee texto.

El propósito es detectar regresiones cuando alguien renombra un campo
o cambia un tipo: si una de las 8 srv o el ``PickPlace.action`` se
rompe, varios componentes del panel y de los nodos auxiliares dejan
de funcionar. Este test guarda la API pública del paquete.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Tuple

import pytest


WS_SRC = Path(__file__).resolve().parents[2]
INTERFACES_DIR = WS_SRC / "ur5_panel_interfaces"

if not INTERFACES_DIR.is_dir():
    pytest.skip(
        f"ur5_panel_interfaces no encontrado en {INTERFACES_DIR}",
        allow_module_level=True,
    )


def _parse_section_fields(text: str) -> List[Tuple[str, str]]:
    """De un bloque .srv/.action, devuelve [(name, type), ...].

    Salta comentarios (``#``) y líneas vacías. NO valida tipos contra
    paquetes ROS — sólo separa "tipo nombre" (en .srv el formato es
    ``<type> <name>``, devolvemos ``(name, type)`` para que el dict
    construido tenga el nombre como clave).
    """
    fields: List[Tuple[str, str]] = []
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 2:
            fields.append((parts[1], parts[0]))
    return fields


def _split_action(text: str) -> Tuple[str, str, str]:
    """Devuelve (goal, result, feedback) de un .action file."""
    parts = text.split("---")
    if len(parts) != 3:
        raise AssertionError(
            f".action file mal formado, esperaban 3 secciones separadas por ---, "
            f"encontradas {len(parts)}"
        )
    return parts[0], parts[1], parts[2]


def _split_srv(text: str) -> Tuple[str, str]:
    """Devuelve (request, response) de un .srv file."""
    parts = text.split("---")
    if len(parts) != 2:
        raise AssertionError(
            f".srv file mal formado, esperaban 2 secciones separadas por ---, "
            f"encontradas {len(parts)}"
        )
    return parts[0], parts[1]


# ---------------------------------------------------------------------------
# PickPlace.action
# ---------------------------------------------------------------------------


def test_pickplace_action_exists():
    f = INTERFACES_DIR / "action" / "PickPlace.action"
    assert f.is_file(), f"falta PickPlace.action en {f}"


def test_pickplace_action_has_expected_sections():
    text = (INTERFACES_DIR / "action" / "PickPlace.action").read_text()
    goal, result, feedback = _split_action(text)
    g_fields = dict(_parse_section_fields(goal))
    r_fields = dict(_parse_section_fields(result))
    f_fields = dict(_parse_section_fields(feedback))

    # Goal: object_name (str) + drop_xyz_world (geometry_msgs/Point)
    assert g_fields.get("object_name") == "string", f"goal: {g_fields}"
    assert g_fields.get("drop_xyz_world") == "geometry_msgs/Point", f"goal: {g_fields}"

    # Result: success, reason, duration_sec, cycles_completed
    assert r_fields.get("success") == "bool"
    assert r_fields.get("reason") == "string"
    assert r_fields.get("duration_sec") == "float64"
    assert r_fields.get("cycles_completed") == "int32"

    # Feedback: current_phase, progress, phase_index, detail
    assert f_fields.get("current_phase") == "string"
    assert f_fields.get("progress") == "float32"
    assert f_fields.get("phase_index") == "int32"
    assert f_fields.get("detail") == "string"


# ---------------------------------------------------------------------------
# PlanToPose.action (F6.3)
# ---------------------------------------------------------------------------


def test_plantopose_action_exists():
    f = INTERFACES_DIR / "action" / "PlanToPose.action"
    assert f.is_file(), f"falta PlanToPose.action en {f}"


def test_plantopose_action_has_expected_sections():
    text = (INTERFACES_DIR / "action" / "PlanToPose.action").read_text()
    goal, result, feedback = _split_action(text)
    g = dict(_parse_section_fields(goal))
    r = dict(_parse_section_fields(result))
    f = dict(_parse_section_fields(feedback))

    # Goal: target_pose_base + ee_frame + cartesian + timeout
    assert g.get("target_pose_base") == "geometry_msgs/Pose", f"goal: {g}"
    assert g.get("ee_frame") == "string", f"goal: {g}"
    assert g.get("cartesian") == "bool", f"goal: {g}"
    assert g.get("timeout_sec") == "float64", f"goal: {g}"

    # Result: success + reason + final_pose_base + duration_sec + attempts
    assert r.get("success") == "bool"
    assert r.get("reason") == "string"
    assert r.get("final_pose_base") == "geometry_msgs/Pose"
    assert r.get("duration_sec") == "float64"
    assert r.get("attempts") == "int32"

    # Feedback: current_state + progress + attempts + detail
    assert f.get("current_state") == "string"
    assert f.get("progress") == "float32"
    assert f.get("attempts") == "int32"
    assert f.get("detail") == "string"


# ---------------------------------------------------------------------------
# Service contracts
# ---------------------------------------------------------------------------

EXPECTED_SRV: Dict[str, Tuple[Dict[str, str], Dict[str, str]]] = {
    # name: (request_fields, response_fields)
    "Open.srv": (
        {},  # sin argumentos
        {"success": "bool", "message": "string"},
    ),
    "Close.srv": (
        {},
        {"success": "bool", "message": "string"},
    ),
    "SetWidth.srv": (
        {"width_m": "float64"},
        {"success": "bool", "message": "string", "actual_width_m": "float64"},
    ),
    "Attach.srv": (
        {"object_name": "string"},
        {
            "success": "bool",
            "message": "string",
            "method": "string",
            "tcp_obj_dist_m": "float64",
        },
    ),
    "Detach.srv": (
        {"object_name": "string"},
        {"success": "bool", "message": "string", "detached_count": "int32"},
    ),
    "SelectObject.srv": (
        {"name": "string"},
        {"success": "bool", "message": "string"},
    ),
    "WorldToBase.srv": (
        {"world_xyz": "geometry_msgs/Point"},
        {
            "base_xyz": "geometry_msgs/Point",
            "success": "bool",
            "detail": "string",
        },
    ),
    "ComputeApproachPose.srv": (
        {"object_pose_base": "geometry_msgs/Pose", "z_clearance_m": "float64"},
        {
            "approach_pose_base": "geometry_msgs/Pose",
            "success": "bool",
            "detail": "string",
        },
    ),
}


@pytest.mark.parametrize("srv_name", sorted(EXPECTED_SRV))
def test_srv_file_has_expected_fields(srv_name):
    f = INTERFACES_DIR / "srv" / srv_name
    assert f.is_file(), f"falta srv {srv_name}"
    text = f.read_text()
    req_text, resp_text = _split_srv(text)
    req = dict(_parse_section_fields(req_text))
    resp = dict(_parse_section_fields(resp_text))
    expected_req, expected_resp = EXPECTED_SRV[srv_name]
    for fname, ftype in expected_req.items():
        assert req.get(fname) == ftype, (
            f"{srv_name} request: campo '{fname}' debería ser '{ftype}', "
            f"encontrado '{req.get(fname)}' (request={req})"
        )
    for fname, ftype in expected_resp.items():
        assert resp.get(fname) == ftype, (
            f"{srv_name} response: campo '{fname}' debería ser '{ftype}', "
            f"encontrado '{resp.get(fname)}' (response={resp})"
        )


def test_cmake_lists_all_interfaces():
    """CMakeLists.txt debe listar todas las interfaces como rosidl_generate_interfaces."""
    cmake = (INTERFACES_DIR / "CMakeLists.txt").read_text()
    for srv in EXPECTED_SRV:
        assert srv in cmake, f"CMakeLists.txt no genera {srv}"
    assert "PickPlace.action" in cmake, "CMakeLists.txt no genera PickPlace.action"
    assert "PlanToPose.action" in cmake, "CMakeLists.txt no genera PlanToPose.action"


def test_package_xml_dependencies():
    """package.xml debe declarar geometry_msgs (usado por srv/action)."""
    xml = (INTERFACES_DIR / "package.xml").read_text()
    assert "geometry_msgs" in xml, "package.xml debe depender de geometry_msgs"
    assert "rosidl_default_generators" in xml, (
        "package.xml debe usar rosidl_default_generators como buildtool"
    )
