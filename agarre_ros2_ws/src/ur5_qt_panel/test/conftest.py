#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/conftest.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Pytest config to import local ur5_qt_panel package without install."""

import os
import sys
import types

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(TEST_DIR, ".."))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)


def _inject_ur5_tools_stub() -> None:
    """Inject a minimal ur5_tools stub so panel_config is importable without ROS install.

    F8 (auditoría 2026-05-10): si el paquete real ur5_tools es importable
    (caso típico cuando los tests del qt_panel se ejecutan junto con los
    de ur5_tools en la misma sesión pytest), NO inyectar stubs — el real
    es preferible. Si NO es importable, inyectar el stub completo.
    """
    if "ur5_tools" in sys.modules:
        return
    # Probar a añadir ur5_tools/ del workspace al sys.path y luego
    # importar el real. Si funciona, dejarlo y no hacer nada.
    test_dir = os.path.dirname(os.path.abspath(__file__))
    ws_src = os.path.abspath(os.path.join(test_dir, "..", ".."))
    ur5_tools_root = os.path.join(ws_src, "ur5_tools")
    if os.path.isdir(ur5_tools_root) and ur5_tools_root not in sys.path:
        sys.path.insert(0, ur5_tools_root)
    try:
        import ur5_tools  # noqa: F401
        # Real disponible — no inyectar stub.
        return
    except ImportError:
        pass

    stub = types.ModuleType("ur5_tools")
    sys.modules["ur5_tools"] = stub

    # F2 (auditoría 2026-05-10): stub de geometry_constants para tests
    # mixtos qt_panel + ur5_tools en la misma sesión pytest.
    gc_mod = types.ModuleType("ur5_tools.geometry_constants")
    gc_mod.BASE_LINK_IN_WORLD = (-0.85, 0.0, 0.850)
    gc_mod.BASKET_DROP_WORLD = (-1.30, 0.00, 0.82)
    gc_mod.world_to_base = lambda p: (
        p[0] - gc_mod.BASE_LINK_IN_WORLD[0],
        p[1] - gc_mod.BASE_LINK_IN_WORLD[1],
        p[2] - gc_mod.BASE_LINK_IN_WORLD[2],
    )
    gc_mod.base_to_world = lambda p: (
        p[0] + gc_mod.BASE_LINK_IN_WORLD[0],
        p[1] + gc_mod.BASE_LINK_IN_WORLD[1],
        p[2] + gc_mod.BASE_LINK_IN_WORLD[2],
    )
    stub.geometry_constants = gc_mod
    sys.modules["ur5_tools.geometry_constants"] = gc_mod

    # F8 (auditoría 2026-05-10): stubs vacíos de los módulos puros
    # release_objects_* / plan_to_pose_helpers. Tests del qt_panel no
    # los necesitan funcionalmente pero la importabilidad debe estar
    # garantizada cuando se mezcla con tests del propio ur5_tools.
    rol_mod = types.ModuleType("ur5_tools.release_objects_logic")
    stub.release_objects_logic = rol_mod
    sys.modules["ur5_tools.release_objects_logic"] = rol_mod
    rog_mod = types.ModuleType("ur5_tools.release_objects_geometry")
    stub.release_objects_geometry = rog_mod
    sys.modules["ur5_tools.release_objects_geometry"] = rog_mod
    # F10 (auditoría 2026-05-10): stub de plan_to_pose_helpers.
    pph_mod = types.ModuleType("ur5_tools.plan_to_pose_helpers")
    stub.plan_to_pose_helpers = pph_mod
    sys.modules["ur5_tools.plan_to_pose_helpers"] = pph_mod

    gg = types.ModuleType("ur5_tools.gripper_geometry")
    gg.RG2_PINCH_CENTER_FRAME = "rg2_pinch_center"
    gg.RG2_TCP_FRAME = "rg2_tcp"
    gg.TOOL0_FRAME = "tool0"
    gg.contact_z_correction_for_frame = lambda frame: 0.0
    gg.tool0_offset_for_frame = lambda frame: (0.0, 0.0, 0.0)

    class _GripperGeometry:
        def xyz_for_frame(self, frame):
            return (0.0, 0.0, 0.005)

    gg.load_gripper_geometry = lambda: _GripperGeometry()
    stub.gripper_geometry = gg
    sys.modules["ur5_tools.gripper_geometry"] = gg

    ki = types.ModuleType("ur5_tools.ur5_kinematics")
    ki.fk_ur5 = lambda joints: ([float(joints[i]) for i in range(3)], None)
    ki.ik_ur5 = lambda pos, seed=None: (seed or [0.0] * 6, 0.0, True)
    stub.ur5_kinematics = ki
    sys.modules["ur5_tools.ur5_kinematics"] = ki

    # Stub mínimo para ur5_tools.moveit_bridge.params (F2 bucket D).
    # Los tests no necesitan los valores reales — basta con que el
    # módulo sea importable y exponga get_moveit_bridge_params().
    from dataclasses import dataclass

    mb_pkg = types.ModuleType("ur5_tools.moveit_bridge")
    mb_params = types.ModuleType("ur5_tools.moveit_bridge.params")

    @dataclass(frozen=True)
    class _StubMoveItBridgeParams:
        allow_feedback_early_success: bool = False
        allow_joint_early_success: bool = False
        allow_ee_early_success: bool = False
        approach_internal_replan: bool = True
        approach_skip_constraints: bool = False
        approach_relaxed_constraint_retry: bool = True
        request_timeout_sec: float = 60.0
        tf_gate_timeout_sec: float = 1.2
        startup_timeout_sec: float = 40.0
        wait_joint_target_max_age_sec: float = 0.35
        wait_joint_target_max_vel_rad_s: float = 0.05
        wait_joint_target_stable_samples: int = 3
        disable_joint_wrap_align: bool = False

    _stub_mb_params_instance = _StubMoveItBridgeParams()
    mb_params.MoveItBridgeParams = _StubMoveItBridgeParams
    mb_params.get_moveit_bridge_params = lambda: _stub_mb_params_instance
    mb_params.reset_moveit_bridge_params_cache = lambda: None
    mb_pkg.params = mb_params
    stub.moveit_bridge = mb_pkg
    sys.modules["ur5_tools.moveit_bridge"] = mb_pkg
    sys.modules["ur5_tools.moveit_bridge.params"] = mb_params


_inject_ur5_tools_stub()


import pytest as _pytest


@_pytest.fixture(autouse=True)
def _reset_param_caches():
    """Invalida los singletons de los módulos *_params antes y después de cada test.

    Previene contaminación entre tests que mutan env vars con
    monkeypatch — sin esto, un test que setea PANEL_X="..." después de
    que otro test ya invocó get_*_params() vería el valor cacheado del
    primero, no el suyo.
    """
    _reset_all()
    yield
    _reset_all()


def _reset_all() -> None:
    for mod_name in (
        "ur5_qt_panel.panel_pick_demo_params",
        # 2026-05-09: panel_pick_object_params borrado.
        "ur5_qt_panel.panel_ros_params",
        "ur5_qt_panel.panel_ui_params",
        "ur5_qt_panel.panel_tfm_params",
    ):
        mod = sys.modules.get(mod_name)
        if mod is None:
            continue
        reset = getattr(mod, "reset_" + mod_name.rsplit(".", 1)[-1].replace("panel_", "panel_") + "_cache", None)
        # Above is brittle; use the well-known names directly:
    # Direct invocation (avoids fragile name juggling).
    for func_name in (
        ("ur5_qt_panel.panel_pick_demo_params", "reset_pick_demo_params_cache"),
        # 2026-05-09: panel_pick_object_params borrado.
        ("ur5_qt_panel.panel_ros_params", "reset_panel_ros_params_cache"),
        ("ur5_qt_panel.panel_ui_params", "reset_panel_ui_params_cache"),
        ("ur5_qt_panel.panel_tfm_params", "reset_panel_tfm_params_cache"),
    ):
        mod = sys.modules.get(func_name[0])
        if mod is not None:
            fn = getattr(mod, func_name[1], None)
            if callable(fn):
                fn()


def pytest_sessionfinish(session, exitstatus):
    """Forzar salida limpia tras pytest.

    Algunos tests cargan modulos PyQt5/Qt que dejan estado estatico. Al
    terminar la sesion el destructor de QApplication a veces lanza
    'terminate called without an active exception' (SIGABRT) durante el
    shutdown del interprete, devolviendo exit code -6 a colcon test
    aunque todos los tests hayan pasado. Forzar os._exit con el exit
    status real elude ese path destructivo y mantiene el resultado.
    """

    if exitstatus == 0:
        os._exit(0)
