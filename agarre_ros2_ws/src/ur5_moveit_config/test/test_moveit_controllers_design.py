#!/usr/bin/env python3
# Ruta/archivo: src/ur5_moveit_config/test/test_moveit_controllers_design.py
# Contenido: F4.1 audit (2026-05-10) — contract test del diseño moveit_controllers.
"""F4.1 audit (2026-05-10): contract test del moveit_controllers.yaml.

Garantiza la decisión arquitectónica P-04:
  * MoveIt planifica el brazo UR5 (6 joints) bajo
    `joint_trajectory_controller`.
  * El gripper RG2 está deliberadamente FUERA de MoveIt — se controla
    vía servicios /gripper/open|close de gripper_attach_backend.
  * La decisión está documentada en el YAML.

Si alguien quiere meter el gripper en MoveIt en el futuro, debe:
  1. Añadir `rg2_finger_joint1`/2 al joint_trajectory_controller (o
     crear un controller `gripper_controller` aparte).
  2. Eliminar este test o adaptarlo al nuevo diseño.
  3. Añadir un planning_group `gripper` al SRDF.
"""
from __future__ import annotations

from pathlib import Path

import pytest


def _resolve_yaml() -> Path:
    here = Path(__file__).resolve().parent
    candidates = [
        here.parent / "config" / "moveit_controllers.yaml",
        here.parent.parent / "ur5_moveit_config" / "config" / "moveit_controllers.yaml",
    ]
    for cand in candidates:
        if cand.is_file():
            return cand
    pytest.skip(f"moveit_controllers.yaml not found in: {candidates}")


def _load_yaml() -> dict:
    yaml_path = _resolve_yaml()
    try:
        import yaml  # type: ignore
    except Exception:
        pytest.skip("PyYAML not available")
    with open(yaml_path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    assert isinstance(data, dict), f"YAML root not dict: {type(data)}"
    return data


UR5_ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
RG2_GRIPPER_JOINTS = ("rg2_finger_joint1", "rg2_finger_joint2")


def test_yaml_uses_simple_controller_manager() -> None:
    """El controller manager debe ser MoveItSimpleControllerManager."""
    data = _load_yaml()
    assert (
        data.get("moveit_controller_manager")
        == "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    )


def test_joint_trajectory_controller_present_with_arm_joints() -> None:
    """El JTC debe estar registrado y declarar los 6 joints UR5 en orden."""
    data = _load_yaml()
    mgr = data.get("moveit_simple_controller_manager", {})
    assert isinstance(mgr, dict)
    names = mgr.get("controller_names", [])
    assert "joint_trajectory_controller" in names

    jtc = mgr.get("joint_trajectory_controller", {})
    assert jtc.get("type") == "FollowJointTrajectory"
    assert jtc.get("default") is True
    assert jtc.get("action_ns") == "follow_joint_trajectory"
    declared_joints = tuple(jtc.get("joints", []))
    assert declared_joints == UR5_ARM_JOINTS, (
        f"joint_trajectory_controller joints drift: declared={declared_joints}, expected={UR5_ARM_JOINTS}"
    )


def test_gripper_joints_explicitly_excluded() -> None:
    """RG2 finger joints NO deben aparecer en moveit_controllers (P-04 design)."""
    data = _load_yaml()
    mgr = data.get("moveit_simple_controller_manager", {})
    for ctrl_name in mgr.get("controller_names", []):
        ctrl = mgr.get(ctrl_name, {}) if isinstance(mgr, dict) else {}
        if not isinstance(ctrl, dict):
            continue
        joints = ctrl.get("joints", [])
        for gj in RG2_GRIPPER_JOINTS:
            assert gj not in joints, (
                f"RG2 gripper joint {gj!r} aparece en controller {ctrl_name!r} — "
                "esta es una decisión arquitectónica P-04: el gripper se controla "
                "vía /gripper/open|close, no por MoveIt. Si quieres cambiarla, "
                "actualiza moveit_controllers.yaml + SRDF + este test."
            )


def test_design_decision_documented_in_yaml() -> None:
    """El YAML debe documentar la decisión de excluir gripper (P-04)."""
    yaml_path = _resolve_yaml()
    text = yaml_path.read_text(encoding="utf-8")
    assert "P-04" in text or "gripper" in text.lower(), (
        "moveit_controllers.yaml debe documentar la decisión de excluir gripper "
        "(buscar 'P-04' o 'gripper'). Sin documentación, este patrón parece un olvido."
    )


def test_srdf_planning_group_aligned_with_joints() -> None:
    """El SRDF debe declarar planning_group 'manipulator' con los 6 joints UR5."""
    here = Path(__file__).resolve().parent
    candidates = [
        here.parent / "config" / "ur5.srdf",
        here.parent.parent / "ur5_moveit_config" / "config" / "ur5.srdf",
    ]
    srdf_path = next((p for p in candidates if p.is_file()), None)
    if srdf_path is None:
        pytest.skip("ur5.srdf not found")
    text = srdf_path.read_text(encoding="utf-8")
    assert '<group name="manipulator"' in text
    for joint in UR5_ARM_JOINTS:
        assert f'name="{joint}"' in text, (
            f"SRDF: planning_group manipulator no incluye joint {joint!r}"
        )
