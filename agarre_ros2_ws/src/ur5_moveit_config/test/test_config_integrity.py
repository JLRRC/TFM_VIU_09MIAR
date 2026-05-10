"""Integridad estructural de la configuración MoveIt 2 del UR5+RG2.

Audit 2026-05-10 (Action 7) — el paquete tenía 0 tests.

Verifica offline (sin lanzar move_group):
  1. Los 5 YAMLs canónicos cargan como dict.
  2. kinematics.yaml define grupo `manipulator` con solver válido.
  3. moveit_controllers.yaml lista joint_trajectory_controller.
  4. joint_limits.yaml cubre los 6 joints UR5.
  5. ompl_planning.yaml define al menos 1 planner para `manipulator`.
  6. Ambos SRDFs (perfil permisivo + estricto) son XML válido.
  7. Ambos SRDFs declaran group `manipulator` con tip_link `rg2_tcp`.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PKG_DIR = Path(__file__).resolve().parent.parent
CONFIG_DIR = PKG_DIR / "config"

UR5_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def _yaml_load(path: Path) -> dict:
    import yaml

    assert path.is_file(), f"missing config: {path}"
    with path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    assert isinstance(data, dict), f"{path} is not a YAML mapping"
    return data


# ------------------------------------------------------------------ kinematics


def test_kinematics_yaml_has_manipulator() -> None:
    data = _yaml_load(CONFIG_DIR / "kinematics.yaml")
    assert "manipulator" in data, "kinematics.yaml missing 'manipulator' group"
    assert "kinematics_solver" in data["manipulator"], (
        "manipulator group missing kinematics_solver"
    )


# --------------------------------------------------------- moveit_controllers


def test_moveit_controllers_lists_jt_controller() -> None:
    data = _yaml_load(CONFIG_DIR / "moveit_controllers.yaml")
    assert "moveit_controller_manager" in data
    sub = data["moveit_simple_controller_manager"]
    assert "controller_names" in sub
    assert "joint_trajectory_controller" in sub["controller_names"]


# ------------------------------------------------------------- joint_limits


@pytest.mark.parametrize("joint_name", UR5_JOINTS)
def test_joint_limits_covers_ur5_joint(joint_name: str) -> None:
    data = _yaml_load(CONFIG_DIR / "joint_limits.yaml")
    # joint_limits.yaml por convención MoveIt anida los límites bajo
    # `joint_limits:` o los expone como top-level joint names.
    if "joint_limits" in data:
        joint_section = data["joint_limits"]
    else:
        joint_section = data
    assert joint_name in joint_section, f"joint_limits.yaml missing {joint_name}"


# --------------------------------------------------------------- ompl_planning


def test_ompl_planning_has_manipulator_planner() -> None:
    data = _yaml_load(CONFIG_DIR / "ompl_planning.yaml")
    # ompl_planning.yaml puede tener `manipulator` como key directo
    # o anidado bajo `move_group.planning_pipelines.ompl`.
    found = False
    for key in ("manipulator",):
        if key in data:
            found = True
            break
    if not found:
        # Búsqueda profunda: cualquier sub-dict que contenga `manipulator`.
        for value in data.values():
            if isinstance(value, dict) and "manipulator" in value:
                found = True
                break
    assert found, "ompl_planning.yaml does not define any planner for 'manipulator'"


# ------------------------------------------------------------------------ srdf


@pytest.mark.parametrize("srdf_name", ["ur5.srdf", "ur5_strict.srdf"])
def test_srdf_is_valid_xml(srdf_name: str) -> None:
    srdf_path = CONFIG_DIR / srdf_name
    assert srdf_path.is_file(), f"missing SRDF: {srdf_path}"
    # Lanza ParseError si XML inválido.
    ET.parse(srdf_path)


@pytest.mark.parametrize("srdf_name", ["ur5.srdf", "ur5_strict.srdf"])
def test_srdf_declares_manipulator_with_rg2_tcp(srdf_name: str) -> None:
    srdf_path = CONFIG_DIR / srdf_name
    tree = ET.parse(srdf_path)
    root = tree.getroot()
    groups = root.findall("group")
    manipulator = next((g for g in groups if g.get("name") == "manipulator"), None)
    assert manipulator is not None, f"{srdf_name}: group 'manipulator' missing"
    chain = manipulator.find("chain")
    assert chain is not None, f"{srdf_name}: manipulator group missing <chain>"
    assert chain.get("base_link") == "base_link"
    assert chain.get("tip_link") == "rg2_tcp", (
        f"{srdf_name}: tip_link debe ser rg2_tcp (canónico tras B2)"
    )


# ------------------------------------------------------ planning_scene_monitor


def test_planning_scene_monitor_yaml_loads() -> None:
    _yaml_load(CONFIG_DIR / "planning_scene_monitor_parameters.yaml")
