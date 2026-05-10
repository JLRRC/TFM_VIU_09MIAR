#!/usr/bin/env python3
"""F4 audit (2026-05-10): parity URDF YAML ↔ SDF runtime YAML.

El SDF embebe gz_ros2_control con su propio YAML de controllers
(`models/ur5_rg2/ur5_controllers.yaml`). El URDF apunta al YAML del
paquete (`src/ur5_description/config/ur5_controllers.yaml`). En runtime
ambos se sincronizan vía `patch_runtime_model_sdf`. Cualquier divergencia
permanente entre ambos es un bug latente: el runtime patch sólo cubre
campos parcheados, no añadidos o renombrados.

Este test garantiza que ambos YAML producen la misma vista canónica:
* Mismos nombres de joints en `joint_trajectory_controller`.
* Mismo `update_rate` en `controller_manager`.
* Mismos types para los controllers declarados.
"""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

WS = Path(__file__).resolve().parents[3]
URDF_YAML = WS / "src" / "ur5_description" / "config" / "ur5_controllers.yaml"
SDF_RUNTIME_YAML = WS / "src" / "ur5_gazebo" / "models" / "ur5_rg2" / "ur5_controllers.yaml"


def _load(p: Path) -> dict:
    assert p.is_file(), f"controllers yaml missing: {p}"
    return yaml.safe_load(p.read_text(encoding="utf-8"))


def test_both_yamls_exist() -> None:
    assert URDF_YAML.is_file()
    assert SDF_RUNTIME_YAML.is_file()


def test_update_rate_matches() -> None:
    urdf = _load(URDF_YAML)["controller_manager"]["ros__parameters"]
    sdf = _load(SDF_RUNTIME_YAML)["controller_manager"]["ros__parameters"]
    assert urdf["update_rate"] == sdf["update_rate"], (
        f"controller_manager.update_rate diverge: "
        f"URDF={urdf['update_rate']} vs SDF={sdf['update_rate']}"
    )


def test_jtc_joints_match() -> None:
    urdf = _load(URDF_YAML)["joint_trajectory_controller"]["ros__parameters"]
    sdf = _load(SDF_RUNTIME_YAML)["joint_trajectory_controller"]["ros__parameters"]
    assert urdf["joints"] == sdf["joints"], (
        f"joint_trajectory_controller.joints diverge:\n"
        f"  URDF: {urdf['joints']}\n"
        f"  SDF:  {sdf['joints']}"
    )


def test_controller_types_match() -> None:
    """Los tipos de los controladores declarados deben coincidir."""
    urdf_cm = _load(URDF_YAML)["controller_manager"]["ros__parameters"]
    sdf_cm = _load(SDF_RUNTIME_YAML)["controller_manager"]["ros__parameters"]
    common = set(urdf_cm.keys()) & set(sdf_cm.keys())
    common -= {"update_rate"}
    for name in sorted(common):
        if not isinstance(urdf_cm[name], dict):
            continue
        if "type" not in urdf_cm[name] or "type" not in sdf_cm[name]:
            continue
        assert urdf_cm[name]["type"] == sdf_cm[name]["type"], (
            f"controller {name!r} type diverge: "
            f"URDF={urdf_cm[name]['type']} vs SDF={sdf_cm[name]['type']}"
        )


def test_jtc_command_interfaces_match() -> None:
    urdf = _load(URDF_YAML)["joint_trajectory_controller"]["ros__parameters"]
    sdf = _load(SDF_RUNTIME_YAML)["joint_trajectory_controller"]["ros__parameters"]
    assert urdf.get("command_interfaces") == sdf.get("command_interfaces"), (
        "JTC command_interfaces diverge URDF↔SDF"
    )


@pytest.mark.parametrize("ctrl_name", [
    "joint_state_broadcaster",
    "joint_trajectory_controller",
])
def test_canonical_controller_present_in_both(ctrl_name: str) -> None:
    urdf_txt = URDF_YAML.read_text(encoding="utf-8")
    sdf_txt = SDF_RUNTIME_YAML.read_text(encoding="utf-8")
    assert ctrl_name in urdf_txt, f"{ctrl_name} ausente en URDF YAML"
    assert ctrl_name in sdf_txt, f"{ctrl_name} ausente en SDF YAML"
