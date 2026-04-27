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
    """Inject a minimal ur5_tools stub so panel_config is importable without ROS install."""
    if "ur5_tools" in sys.modules:
        return

    stub = types.ModuleType("ur5_tools")
    sys.modules["ur5_tools"] = stub

    gg = types.ModuleType("ur5_tools.gripper_geometry")
    gg.RG2_PINCH_CENTER_FRAME = "rg2_pinch_center"
    gg.RG2_TCP_FRAME = "rg2_tcp"
    gg.TOOL0_FRAME = "tool0"
    gg.contact_z_correction_for_frame = lambda frame: 0.0

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


_inject_ur5_tools_stub()


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
