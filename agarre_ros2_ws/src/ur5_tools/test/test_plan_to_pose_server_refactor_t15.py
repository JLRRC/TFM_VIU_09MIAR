#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_plan_to_pose_server_refactor_t15.py
"""Tests offline post refactor T15 (2026-05-09): _execute_fjt_direct + _execute_moveit_direct.

Verifica que tras el split de `_execute_fjt_direct` (261→61 LOC) y
`_execute_moveit_direct` (364→118 LOC) en sub-helpers:

1. El nodo se instancia con todos los helpers callable.
2. Los atributos críticos del init se preservan (defaults H14, H14b).
3. El AST de `_execute_fjt_direct` ya NO contiene la referencia ``seed_positions``
   antes de la línea de definición (regresión del bug latente).
4. El UR5_JOINTS class constant existe y tiene los 6 joints en orden.

Sin ROS al ejecutar (rclpy.init opcional via fixture).
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

WS_ROOT = Path(__file__).resolve().parents[3]
PLAN_TO_POSE_SRC = (
    WS_ROOT / "src" / "ur5_tools" / "ur5_tools" / "plan_to_pose_server.py"
)


# ---------------------------------------------------------------------------
# AST static checks (no requieren rclpy)
# ---------------------------------------------------------------------------


def _load_class() -> ast.ClassDef:
    tree = ast.parse(PLAN_TO_POSE_SRC.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "PlanToPoseServer":
            return node
    pytest.fail("PlanToPoseServer no encontrada")


def _find_method(cls: ast.ClassDef, name: str) -> ast.FunctionDef:
    for node in cls.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    pytest.fail(f"método {name} no encontrado en PlanToPoseServer")


def test_t15_fjt_direct_helpers_exist() -> None:
    """Los 5 sub-helpers de _execute_fjt_direct están definidos como métodos."""
    cls = _load_class()
    expected = {
        "_execute_fjt_direct",
        "_fjt_extract_seed_positions",
        "_fjt_compute_traj_params",
        "_fjt_call_compute_ik",
        "_fjt_build_trajectory",
        "_fjt_send_and_wait_result",
    }
    found = {n.name for n in cls.body if isinstance(n, ast.FunctionDef)}
    missing = expected - found
    assert not missing, f"Helpers FJT directo missing: {missing}"


def test_t15_moveit_direct_helpers_exist() -> None:
    """Los 5 sub-helpers de _execute_moveit_direct están definidos como métodos."""
    cls = _load_class()
    expected = {
        "_execute_moveit_direct",
        "_moveit_try_fjt_bypass",
        "_moveit_send_first_attempt",
        "_moveit_post_timeout_tf_check",
        "_moveit_retry_after_failure",
        "_moveit_final_tf_recovery",
    }
    found = {n.name for n in cls.body if isinstance(n, ast.FunctionDef)}
    missing = expected - found
    assert not missing, f"Helpers MoveIt directo missing: {missing}"


def test_t15_fjt_direct_under_200_loc() -> None:
    """_execute_fjt_direct < 200 LOC tras refactor (era 261)."""
    cls = _load_class()
    fn = _find_method(cls, "_execute_fjt_direct")
    loc = fn.end_lineno - fn.lineno + 1
    assert loc < 200, f"_execute_fjt_direct sigue oversize: {loc} LOC"


def test_t15_moveit_direct_under_200_loc() -> None:
    """_execute_moveit_direct < 200 LOC tras refactor (era 364)."""
    cls = _load_class()
    fn = _find_method(cls, "_execute_moveit_direct")
    loc = fn.end_lineno - fn.lineno + 1
    assert loc < 200, f"_execute_moveit_direct sigue oversize: {loc} LOC"


def test_t15_init_under_50_loc() -> None:
    """__init__ < 50 LOC tras refactor (era 228)."""
    cls = _load_class()
    fn = _find_method(cls, "__init__")
    loc = fn.end_lineno - fn.lineno + 1
    assert loc < 50, f"__init__ sigue oversize: {loc} LOC"


def test_t15_fjt_direct_no_seed_positions_before_extract() -> None:
    """Regresión bug latente: ``seed_positions`` no se referencia antes
    de llamar a ``_fjt_extract_seed_positions`` en _execute_fjt_direct.

    El bug original (commit 9cf4cb2) tenía:

        try:
            cur_x, cur_y, cur_z = (float(seed_positions[0]) * 0, 0.0, 0.0)
            tf_pos = self._lookup_ee_position_in_base(...)
        except Exception:
            dist_to_target = 0.5  # fallback siempre triggered

    causando NameError en cada llamada → fallback always-on.
    """
    cls = _load_class()
    fn = _find_method(cls, "_execute_fjt_direct")
    src = ast.unparse(fn)
    # Buscar primer uso de seed_positions y primer call a _fjt_extract_seed_positions.
    first_seed_use = src.find("seed_positions")
    first_extract_call = src.find("_fjt_extract_seed_positions")
    if first_seed_use == -1:
        return  # no usa seed_positions — OK trivially
    assert first_extract_call != -1, "_fjt_extract_seed_positions no llamado"
    assert first_extract_call < first_seed_use, (
        "BUG REGRESIÓN: seed_positions referenciado antes de "
        "_fjt_extract_seed_positions(). Ver commit b68b165 para contexto."
    )


def test_t15_ur5_joints_class_constant() -> None:
    """_UR5_JOINTS class constant existe con 6 joints en orden controller."""
    cls = _load_class()
    found = False
    for node in cls.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "_UR5_JOINTS":
                    found = True
                    # Verifica que es un tuple de 6 strings.
                    assert isinstance(node.value, ast.Tuple), (
                        "_UR5_JOINTS debe ser tuple inmutable"
                    )
                    assert len(node.value.elts) == 6, (
                        f"_UR5_JOINTS debe tener 6 joints, tiene {len(node.value.elts)}"
                    )
                    names = [
                        e.value for e in node.value.elts
                        if isinstance(e, ast.Constant)
                    ]
                    assert names == [
                        "shoulder_pan_joint",
                        "shoulder_lift_joint",
                        "elbow_joint",
                        "wrist_1_joint",
                        "wrist_2_joint",
                        "wrist_3_joint",
                    ], f"Orden de _UR5_JOINTS roto: {names}"
    assert found, "_UR5_JOINTS class constant no encontrada"


def _find_declare_default(cls: ast.ClassDef, param_name: str) -> object:
    """Busca ``self.declare_parameter("<param_name>", <default>)`` y devuelve
    el default (constante AST) en cualquier método del class. Usa walk para
    encontrar la llamada independientemente de qué helper la contenga."""
    for node in ast.walk(cls):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "declare_parameter"
            and len(node.args) >= 2
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == param_name
            and isinstance(node.args[1], ast.Constant)
        ):
            return node.args[1].value
    return None


def test_t15_h14b_default_ik_timeout_5s() -> None:
    """fjt_direct_ik_timeout_sec default = 5.0s (H14b post T35×5 stress)."""
    cls = _load_class()
    default = _find_declare_default(cls, "fjt_direct_ik_timeout_sec")
    assert default == 5.0, (
        f"H14b regresión: default fjt_direct_ik_timeout_sec={default!r}, "
        "esperado 5.0. Ver commit c9a3aea (TRAC-IK seed-dependent post-restart)."
    )


def test_t15_h14_default_path_tolerance_0_3() -> None:
    """fjt_direct_path_tolerance_rad default = 0.3 rad (H14 anti-flakiness)."""
    cls = _load_class()
    default = _find_declare_default(cls, "fjt_direct_path_tolerance_rad")
    assert default == 0.3, (
        f"H14 regresión: default fjt_direct_path_tolerance_rad={default!r}, "
        "esperado 0.3 rad."
    )


# ---------------------------------------------------------------------------
# Runtime smoke test (requiere rclpy)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy = pytest.importorskip("rclpy")
    if not rclpy.ok():
        rclpy.init()
    yield rclpy
    if rclpy.ok():
        rclpy.shutdown()


def test_t15_smoke_node_instantiates(rclpy_init) -> None:
    """PlanToPoseServer se instancia sin error y atributos críticos son correctos."""
    pytest.importorskip("rclpy")
    pytest.importorskip("ur5_panel_interfaces")
    pytest.importorskip("moveit_msgs")
    from ur5_tools.plan_to_pose_server import PlanToPoseServer

    node = PlanToPoseServer()
    try:
        assert node._UR5_JOINTS == (
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        )
        # H14 + H14b defaults.
        assert node._fjt_direct_path_tolerance_rad == pytest.approx(0.3)
        assert node._fjt_direct_ik_timeout == pytest.approx(5.0)
        # Helpers callable.
        for h in (
            "_execute_fjt_direct",
            "_fjt_extract_seed_positions",
            "_fjt_compute_traj_params",
            "_fjt_call_compute_ik",
            "_fjt_build_trajectory",
            "_fjt_send_and_wait_result",
            "_execute_moveit_direct",
            "_moveit_try_fjt_bypass",
            "_moveit_send_first_attempt",
            "_moveit_post_timeout_tf_check",
            "_moveit_retry_after_failure",
            "_moveit_final_tf_recovery",
        ):
            assert callable(getattr(node, h)), f"{h} no callable"
    finally:
        node.destroy_node()


def test_t15_extract_seed_positions_returns_none_when_no_joint_state(rclpy_init) -> None:
    """``_fjt_extract_seed_positions`` devuelve None y loguea cuando no hay
    joint_state cacheado (caller debe fallback)."""
    pytest.importorskip("rclpy")
    pytest.importorskip("ur5_panel_interfaces")
    pytest.importorskip("moveit_msgs")
    from ur5_tools.plan_to_pose_server import PlanToPoseServer

    node = PlanToPoseServer()
    try:
        # Por defecto _latest_joint_state es None tras init.
        result = node._fjt_extract_seed_positions()
        assert result is None
    finally:
        node.destroy_node()
