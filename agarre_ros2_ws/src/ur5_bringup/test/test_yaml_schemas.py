# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/test_yaml_schemas.py
# Contenido: T4 — schemas YAML del workspace (parseables + claves canónicas).
"""T4 — validación de schemas YAML.

Asegura que los archivos YAML de configuración del workspace:

1. Son YAML válidos (parseables sin error).
2. No están vacíos (estructura mínima coherente).
3. Tienen las claves canónicas esperadas (cada YAML tiene su contrato).

Esto detecta corrupción accidental, BOM marks, indentación rota, claves
mal escritas tras refactor.

NO valida tipos profundos (eso requeriría pydantic). Es validación
estructural ligera, suficiente para CI.
"""
from __future__ import annotations

from pathlib import Path

import yaml

WS_DIR = Path(__file__).resolve().parents[3]


def _load_yaml(rel_path: str) -> dict:
    path = WS_DIR / rel_path
    assert path.is_file(), f"YAML no encontrado: {path}"
    with path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    assert data is not None, f"YAML vacío: {path}"
    assert isinstance(data, dict), f"YAML no es un mapping: {path}"
    return data


# ---------------------------------------------------------------------------
# ur5_bringup
# ---------------------------------------------------------------------------


def test_runtime_defaults_yaml_parseable():
    data = _load_yaml("src/ur5_bringup/config/runtime_defaults.yaml")
    # Esperamos al menos algunas claves típicas del runtime panel.
    keys = list(data.keys())
    assert keys, "runtime_defaults.yaml está vacío"


def test_runtime_defaults_has_approach_coarse_keys():
    """Las claves de tolerance de approach coarse deben existir."""
    data = _load_yaml("src/ur5_bringup/config/runtime_defaults.yaml")
    expected = [
        "PANEL_PICK_DEMO_APPROACH_COARSE_GATE_XY_TOL_M",
        "PANEL_PICK_DEMO_APPROACH_COARSE_GATE_Z_TOL_M",
    ]
    for key in expected:
        assert key in data, f"Falta clave canónica: {key}"


def test_system_state_manager_yaml_parseable():
    data = _load_yaml("src/ur5_bringup/config/system_state_manager.yaml")
    assert data, "system_state_manager.yaml vacío"


# ---------------------------------------------------------------------------
# ur5_qt_panel
# ---------------------------------------------------------------------------


def test_panel_settings_yaml_parseable():
    data = _load_yaml("src/ur5_qt_panel/config/panel_settings.yaml")
    assert data, "panel_settings.yaml vacío"


def test_pick_demo_runtime_has_tolerance_keys():
    """pick_demo_runtime.yaml debe tener las tolerances canónicas."""
    data = _load_yaml("src/ur5_qt_panel/config/pick_demo_runtime.yaml")
    expected = [
        "approach_coarse_tcp_tol_m",
        "grasp_down_tcp_tol_m",
        "basket_transport_tcp_tol_m",
    ]
    for key in expected:
        assert key in data, f"Falta tolerance canónica en pick_demo_runtime.yaml: {key}"


def test_pick_object_runtime_yaml_parseable():
    data = _load_yaml("src/ur5_qt_panel/config/pick_object_runtime.yaml")
    assert data, "pick_object_runtime.yaml vacío"


def test_panel_ros_runtime_yaml_parseable():
    data = _load_yaml("src/ur5_qt_panel/config/panel_ros_runtime.yaml")
    assert data, "panel_ros_runtime.yaml vacío"


def test_panel_ui_runtime_yaml_parseable():
    data = _load_yaml("src/ur5_qt_panel/config/panel_ui_runtime.yaml")
    assert data, "panel_ui_runtime.yaml vacío"


def test_panel_tfm_runtime_yaml_parseable():
    data = _load_yaml("src/ur5_qt_panel/config/panel_tfm_runtime.yaml")
    assert data, "panel_tfm_runtime.yaml vacío"


def test_panel_test_tuning_yaml_parseable():
    data = _load_yaml("src/ur5_qt_panel/config/panel_test_tuning.yaml")
    assert data, "panel_test_tuning.yaml vacío"


# ---------------------------------------------------------------------------
# ur5_moveit_config
# ---------------------------------------------------------------------------


def test_kinematics_yaml_parseable():
    data = _load_yaml("src/ur5_moveit_config/config/kinematics.yaml")
    assert data, "kinematics.yaml vacío"


def test_joint_limits_yaml_parseable():
    data = _load_yaml("src/ur5_moveit_config/config/joint_limits.yaml")
    assert "joint_limits" in data, (
        "joint_limits.yaml debe tener clave raíz 'joint_limits'"
    )


def test_ompl_planning_yaml_parseable():
    data = _load_yaml("src/ur5_moveit_config/config/ompl_planning.yaml")
    assert data, "ompl_planning.yaml vacío"


def test_moveit_controllers_yaml_parseable():
    data = _load_yaml("src/ur5_moveit_config/config/moveit_controllers.yaml")
    assert "moveit_controller_manager" in data or "moveit_simple_controller_manager" in data, (
        "moveit_controllers.yaml debe declarar el controller manager"
    )


# ---------------------------------------------------------------------------
# ur5_description (controllers)
# ---------------------------------------------------------------------------


def test_ur5_description_controllers_yaml_parseable():
    data = _load_yaml("src/ur5_description/config/ur5_controllers.yaml")
    assert "controller_manager" in data
    assert "joint_trajectory_controller" in data


def test_ur5_description_controllers_has_constraints_with_goal_time():
    """2026-05-07 fix: goal_time debe ser >0 (default 0.0 fallaba en sim)."""
    data = _load_yaml("src/ur5_description/config/ur5_controllers.yaml")
    constraints = data.get("joint_trajectory_controller", {}).get("ros__parameters", {}).get("constraints", {})
    goal_time = constraints.get("goal_time", 0.0)
    assert goal_time > 0.0, (
        f"goal_time={goal_time} debe ser >0 para tolerar drift sim_time vs wall_time. "
        "Default 0.0 hacía fallar el controller en Gazebo Sim (rondas 11-15 live)."
    )


# ---------------------------------------------------------------------------
# Modelo SDF runtime
# ---------------------------------------------------------------------------


def test_models_ur5_rg2_controllers_yaml_parseable():
    """Copia del controllers.yaml en models/ (Gazebo lo carga vía SDF plugin).

    F5 audit (2026-05-10): models movidos a src/ur5_gazebo/models.
    """
    data = _load_yaml("src/ur5_gazebo/models/ur5_rg2/ur5_controllers.yaml")
    assert "controller_manager" in data
    assert "joint_trajectory_controller" in data
