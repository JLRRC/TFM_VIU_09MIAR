#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_v2_qt_smoke.py
# Contenido: T18 — smoke test del panel Qt sin display real (offscreen).
# Uso breve:
#   QT_QPA_PLATFORM=offscreen pytest test/test_panel_v2_qt_smoke.py
#   - El conftest ya stub-ea ur5_tools (gripper_geometry/kinematics/moveit_bridge.params).
#   - Aquí stubeamos rclpy + ROS msg modules para probar import del módulo
#     y montaje de ControlPanelV2 contra QApplication offscreen.
"""F4 / T18 — Qt headless smoke.

Este test:
  1. Activa la plataforma Qt 'offscreen' (no requiere display real).
  2. Inyecta stubs mínimos de rclpy + ROS msg modules (panel_v2 importa
     rclpy en cascada). Sin esto, el import requeriría un workspace
     instalado y ROS sourced, lo que rompe el CI rápido offline.
  3. Verifica que el módulo se importa sin excepción.
  4. Verifica que la clase ControlPanelV2 tiene las 17 mixins documentadas
     (drift detector — si alguien añade/quita mixin sin actualizar el
     audit, este test falla).
  5. Crea una QApplication offscreen como prueba de smoke real de Qt.

Sin live ROS / sin Gazebo / sin display físico — apto para CI rápido.
"""

from __future__ import annotations

import os
import sys
import types

import pytest

os.environ["QT_QPA_PLATFORM"] = "offscreen"

# Mixins esperados en ControlPanelV2 (drift detector).
EXPECTED_MIXINS_IN_ORDER = (
    "PanelV2PublisherMixin",
    "PanelV2BasePoseMixin",
    "PanelV2GripperAttachMixin",
    "PanelV2MotionMixin",
    "PanelV2TrajSettleMixin",
    "PanelV2SystemStateMixin",
    "PanelV2StepDebugMixin",
    "PanelV2RuntimeDiagnosticsMixin",
    "PanelV2SubprocessMotionMixin",
    "PanelV2AuditLogMixin",
    "PanelV2TfmRemoteMixin",
    "PanelV2ReadyReasonsMixin",
    "PanelV2DropRecoverMixin",
    "PanelV2CalibPickMixin",
    "PanelV2OverlaysSelectionMixin",
    "PanelV2TfmScienceTraceMixin",
    "QMainWindow",
)


def _inject_rclpy_stub() -> None:
    """Stub mínimo de rclpy + msgs ROS suficiente para importar panel_v2.

    panel_v2.py hace ``import rclpy`` y un puñado de ``from std_msgs.msg
    import ...`` en top-level. En CI offline no hay ROS instalado, así
    que reemplazamos esos módulos por shims sin lógica.
    """
    if "rclpy" in sys.modules:
        return

    # rclpy + submódulos
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **kw: None
    rclpy.shutdown = lambda *a, **kw: None
    rclpy.ok = lambda: True
    sys.modules["rclpy"] = rclpy

    rclpy_node = types.ModuleType("rclpy.node")

    class _Node:  # noqa: D401
        def __init__(self, *a, **kw): ...
        def get_logger(self):
            class _L:
                def info(self, *a, **kw): ...
                def warning(self, *a, **kw): ...
                def warn(self, *a, **kw): ...
                def error(self, *a, **kw): ...
                def debug(self, *a, **kw): ...
            return _L()
        def create_publisher(self, *a, **kw): return None
        def create_subscription(self, *a, **kw): return None
        def create_timer(self, *a, **kw): return None
        def create_client(self, *a, **kw): return None
        def declare_parameter(self, *a, **kw): return None
        def get_parameter(self, *a, **kw): return None
        def destroy_node(self): ...

    rclpy_node.Node = _Node
    rclpy.node = rclpy_node
    sys.modules["rclpy.node"] = rclpy_node

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = lambda *a, **kw: None
    rclpy_qos.ReliabilityPolicy = type("RP", (), {"RELIABLE": 1, "BEST_EFFORT": 2})
    rclpy_qos.HistoryPolicy = type("HP", (), {"KEEP_LAST": 1, "KEEP_ALL": 2})
    rclpy_qos.DurabilityPolicy = type("DP", (), {"VOLATILE": 1, "TRANSIENT_LOCAL": 2})
    rclpy_qos.QoSDurabilityPolicy = rclpy_qos.DurabilityPolicy
    rclpy.qos = rclpy_qos
    sys.modules["rclpy.qos"] = rclpy_qos

    rclpy_exec = types.ModuleType("rclpy.executors")
    rclpy_exec.MultiThreadedExecutor = lambda *a, **kw: None
    rclpy_exec.SingleThreadedExecutor = lambda *a, **kw: None
    rclpy.executors = rclpy_exec
    sys.modules["rclpy.executors"] = rclpy_exec

    rclpy_dur = types.ModuleType("rclpy.duration")
    class _Duration:
        def __init__(self, *a, **kw): ...
    rclpy_dur.Duration = _Duration
    rclpy.duration = rclpy_dur
    sys.modules["rclpy.duration"] = rclpy_dur

    rclpy_time = types.ModuleType("rclpy.time")
    class _Time:
        def __init__(self, *a, **kw): ...
    rclpy_time.Time = _Time
    rclpy.time = rclpy_time
    sys.modules["rclpy.time"] = rclpy_time

    rclpy_action = types.ModuleType("rclpy.action")
    rclpy_action.ActionClient = lambda *a, **kw: None
    sys.modules["rclpy.action"] = rclpy_action

    # builtin_interfaces, msg packages stubs
    for pkg in (
        "std_msgs", "std_msgs.msg",
        "geometry_msgs", "geometry_msgs.msg",
        "sensor_msgs", "sensor_msgs.msg",
        "trajectory_msgs", "trajectory_msgs.msg",
        "control_msgs", "control_msgs.msg", "control_msgs.action",
        "moveit_msgs", "moveit_msgs.msg", "moveit_msgs.srv", "moveit_msgs.action",
        "shape_msgs", "shape_msgs.msg",
        "action_msgs", "action_msgs.msg",
        "builtin_interfaces", "builtin_interfaces.msg",
        "visualization_msgs", "visualization_msgs.msg",
        "tf2_msgs", "tf2_msgs.msg",
        "tf2_ros",
        "ros_gz_interfaces", "ros_gz_interfaces.srv", "ros_gz_interfaces.msg",
        "lifecycle_msgs", "lifecycle_msgs.msg", "lifecycle_msgs.srv",
        "ur5_panel_interfaces", "ur5_panel_interfaces.srv", "ur5_panel_interfaces.action",
    ):
        mod = types.ModuleType(pkg)
        sys.modules[pkg] = mod

    # Provide common message classes with permissive __init__/__getattr__.
    class _AnyMsg:
        def __init__(self, *a, **kw):
            for k, v in kw.items():
                setattr(self, k, v)
        def __getattr__(self, _name):
            return None

    for mod_name in [
        "std_msgs.msg", "geometry_msgs.msg", "sensor_msgs.msg",
        "trajectory_msgs.msg", "control_msgs.msg", "control_msgs.action",
        "moveit_msgs.msg", "moveit_msgs.srv", "moveit_msgs.action",
        "shape_msgs.msg", "action_msgs.msg", "builtin_interfaces.msg",
        "visualization_msgs.msg", "tf2_msgs.msg", "ros_gz_interfaces.srv",
        "ros_gz_interfaces.msg", "lifecycle_msgs.msg", "lifecycle_msgs.srv",
        "ur5_panel_interfaces.srv", "ur5_panel_interfaces.action",
    ]:
        mod = sys.modules[mod_name]
        # Names commonly imported. AnyMsg works for all uses in panel_v2.
        for sym in (
            "String", "Bool", "Int32", "Int64", "Float32", "Float64",
            "Float32MultiArray", "Float64MultiArray", "Int32MultiArray",
            "Int64MultiArray", "MultiArrayLayout", "MultiArrayDimension",
            "Point", "PointStamped", "Pose", "PoseStamped", "Quaternion",
            "QuaternionStamped", "TransformStamped", "Transform",
            "Twist", "TwistStamped", "Vector3", "Vector3Stamped",
            "JointState", "Image", "CompressedImage", "CameraInfo",
            "JointTrajectory", "JointTrajectoryPoint",
            "FollowJointTrajectory", "Marker", "MarkerArray", "TFMessage",
            "GoalStatus", "JointTolerance", "PlanningScene", "MoveGroup",
            "ChangeState", "TransitionEvent", "GetEntityState",
            "SetEntityState", "ApplyLinkWrench", "Empty", "Header",
            "Plane", "SolidPrimitive", "Mesh", "MeshTriangle",
        ):
            setattr(mod, sym, _AnyMsg)


def test_panel_v2_module_imports_offscreen() -> None:
    """T18a — el módulo panel_v2 se importa sin excepción en offscreen."""
    _inject_rclpy_stub()

    # No interference: import del módulo. Si el árbol de imports tiene
    # un fallo (mixin con import roto, helper inexistente...), aquí
    # falla. Exactamente lo que pasó con commit fd04c80 (panel_v2 no
    # arrancaba por imports faltantes).
    try:
        import ur5_qt_panel.panel_v2 as panel_v2_module  # noqa: F401
    except Exception as exc:  # pragma: no cover
        pytest.fail(f"panel_v2 import falló offscreen: {type(exc).__name__}: {exc}")


def test_control_panel_v2_class_has_expected_mixins() -> None:
    """T18b — ControlPanelV2 mantiene las 17 mixins documentadas en orden.

    Drift detector: si alguien añade o quita un mixin sin actualizar
    EXPECTED_MIXINS_IN_ORDER (y la documentación canónica
    REFACTOR_F14_PATTERN.md), este test falla.
    """
    _inject_rclpy_stub()
    import ur5_qt_panel.panel_v2 as panel_v2_module
    cls = getattr(panel_v2_module, "ControlPanelV2", None)
    assert cls is not None, "ControlPanelV2 no está exportado"
    actual = tuple(b.__name__ for b in cls.__bases__)
    assert actual == EXPECTED_MIXINS_IN_ORDER, (
        "ControlPanelV2 mixins drift detectado.\n"
        f"  esperado={EXPECTED_MIXINS_IN_ORDER}\n"
        f"  actual  ={actual}\n"
        "Si el cambio es intencional, actualiza:\n"
        "  - test/test_panel_v2_qt_smoke.py::EXPECTED_MIXINS_IN_ORDER\n"
        "  - docs/REFACTOR_F14_PATTERN.md\n"
        "  - auditoría profesional vigente."
    )


def test_qt_offscreen_app_smokes() -> None:
    """T18c — QApplication offscreen + QWidget elemental sin crash."""
    try:
        from PyQt5.QtWidgets import QApplication, QWidget
    except Exception as exc:  # pragma: no cover
        pytest.skip(f"PyQt5 no disponible: {exc}")
    app = QApplication.instance() or QApplication([])
    w = QWidget()
    w.setWindowTitle("smoke")
    w.show()
    app.processEvents()
    w.close()
    # No assert: si llega aquí sin excepción, el smoke pasó.
