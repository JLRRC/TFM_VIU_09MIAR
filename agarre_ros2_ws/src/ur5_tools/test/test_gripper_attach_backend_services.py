"""F5-step6b: smoke tests de los 4 services añadidos a gripper_attach_backend.

Tests mínimos sin levantar ROS — usan SimpleNamespace + MagicMock para
emular el LifecycleNode + estado interno. Cubren:

  * Detach con object_name vacío libera todos los _attached.
  * Detach con object_name específico libera sólo ese.
  * Open/Close construyen el Float64MultiArray correctamente.
  * Attach con object_name vacío devuelve success=False.
  * Errores en el publisher se reportan como success=False con detail.

NOTA: estos tests NO instancian el LifecycleNode real (requeriría rclpy
init + lifecycle transitions). Sólo verifican la lógica de los handlers
con mocks. Tests E2E con ROS vivo van por launch_testing (F7-step3).
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest


# Skip si ros2_panel_interfaces no está disponible (entorno offline).
_INTERFACES_AVAILABLE = True
try:
    from ur5_panel_interfaces.srv import Attach as AttachSrv
    from ur5_panel_interfaces.srv import Close as CloseSrv
    from ur5_panel_interfaces.srv import Detach as DetachSrv
    from ur5_panel_interfaces.srv import Open as OpenSrv
except Exception:
    _INTERFACES_AVAILABLE = False

pytestmark = pytest.mark.skipif(
    not _INTERFACES_AVAILABLE,
    reason="ur5_panel_interfaces no disponible",
)


def _make_backend_mock():
    """Mock del LifecycleNode con los attrs/métodos que los handlers usan.

    `_command_gripper_position` se bindea como método REAL para que los
    tests Open/Close validen el flujo completo (mock devolvería tupla
    incorrecta y rompería el unpack).
    """
    from ur5_tools.gripper_attach_backend import GripperAttachBackend

    backend = MagicMock()
    backend._gripper_cmd_topic = "/gripper_controller/commands"
    backend._gripper_open_rad = 0.0425
    backend._gripper_closed_rad = 0.0
    backend._gripper_joint2_sign = 1.0
    backend._gripper_cmd_pub = None
    backend._attached = {}
    backend._demo_transport_objects = set()
    backend._prefer_tool_anchor_objects = set()
    backend._attach_mode = "follow_tcp"
    backend.create_publisher = MagicMock(return_value=MagicMock())
    backend.get_logger = MagicMock(return_value=MagicMock())
    backend._lookup_pose = MagicMock(return_value=None)
    backend._lookup_tcp_pose = MagicMock(return_value=None)
    backend._on_gripper_attach = MagicMock()
    backend._on_gripper_detach = MagicMock()
    # Bind el método real (no mock) para que devuelva tupla(ok, detail).
    backend._command_gripper_position = (
        lambda target_rad, *, label:
        GripperAttachBackend._command_gripper_position(
            backend, target_rad, label=label,
        )
    )
    return backend


# ---------------------------------------------------------------------------
# Open / Close
# ---------------------------------------------------------------------------


def test_open_service_publishes_open_rad_to_gripper_cmd():
    from ur5_tools.gripper_attach_backend import GripperAttachBackend

    backend = _make_backend_mock()
    response = OpenSrv.Response()
    result = GripperAttachBackend._on_open_service(backend, OpenSrv.Request(), response)

    assert result.success is True
    assert "0.0425" in result.message  # gripper_open_rad
    # create_publisher invocado lazy una vez.
    backend.create_publisher.assert_called_once()
    # Publish llamado con Float64MultiArray con los 2 valores correctos.
    publisher = backend.create_publisher.return_value
    publisher.publish.assert_called_once()
    msg = publisher.publish.call_args[0][0]
    assert list(msg.data) == [0.0425, 0.0425]


def test_close_service_publishes_zero_rad():
    from ur5_tools.gripper_attach_backend import GripperAttachBackend

    backend = _make_backend_mock()
    response = CloseSrv.Response()
    result = GripperAttachBackend._on_close_service(backend, CloseSrv.Request(), response)

    assert result.success is True
    publisher = backend.create_publisher.return_value
    msg = publisher.publish.call_args[0][0]
    assert list(msg.data) == [0.0, 0.0]


def test_open_service_with_joint2_sign_negative():
    from ur5_tools.gripper_attach_backend import GripperAttachBackend

    backend = _make_backend_mock()
    backend._gripper_joint2_sign = -1.0
    response = OpenSrv.Response()
    GripperAttachBackend._on_open_service(backend, OpenSrv.Request(), response)
    publisher = backend.create_publisher.return_value
    msg = publisher.publish.call_args[0][0]
    assert list(msg.data) == [0.0425, -0.0425]


def test_open_service_publish_failure_reported():
    from ur5_tools.gripper_attach_backend import GripperAttachBackend

    backend = _make_backend_mock()
    pub = MagicMock()
    pub.publish.side_effect = RuntimeError("dds_error")
    backend.create_publisher = MagicMock(return_value=pub)
    response = OpenSrv.Response()
    result = GripperAttachBackend._on_open_service(backend, OpenSrv.Request(), response)
    assert result.success is False
    assert "publish_failed" in result.message
    assert "dds_error" in result.message


def test_open_service_create_publisher_failure_reported():
    from ur5_tools.gripper_attach_backend import GripperAttachBackend

    backend = _make_backend_mock()
    backend.create_publisher = MagicMock(side_effect=RuntimeError("init_failed"))
    response = OpenSrv.Response()
    result = GripperAttachBackend._on_open_service(backend, OpenSrv.Request(), response)
    assert result.success is False
    assert "create_publisher_failed" in result.message


# ---------------------------------------------------------------------------
# Attach
# ---------------------------------------------------------------------------


def test_attach_service_empty_name_returns_failure():
    from ur5_tools.gripper_attach_backend import GripperAttachBackend

    backend = _make_backend_mock()
    request = AttachSrv.Request()
    request.object_name = ""
    response = AttachSrv.Response()
    result = GripperAttachBackend._on_attach_service(backend, request, response)
    assert result.success is False
    assert result.message == "object_name_empty"
    backend._on_gripper_attach.assert_not_called()


def test_attach_service_delegates_to_legacy_handler():
    from ur5_tools.gripper_attach_backend import GripperAttachBackend

    backend = _make_backend_mock()
    request = AttachSrv.Request()
    request.object_name = "box_red"
    response = AttachSrv.Response()
    result = GripperAttachBackend._on_attach_service(backend, request, response)
    assert result.success is True
    assert result.message == "attach_dispatched"
    assert result.method == "follow_tcp"  # default attach_mode
    backend._on_gripper_attach.assert_called_once()
    # Verificar args del delegate.
    call_kwargs = backend._on_gripper_attach.call_args.kwargs
    assert call_kwargs["name"] == "box_red"
    assert call_kwargs["src_topic"] == "/orchestrator/attach"


def test_attach_service_demo_transport_object_uses_demo_method():
    from ur5_tools.gripper_attach_backend import GripperAttachBackend

    backend = _make_backend_mock()
    backend._demo_transport_objects = {"pick_demo"}
    request = AttachSrv.Request()
    request.object_name = "pick_demo"
    response = AttachSrv.Response()
    result = GripperAttachBackend._on_attach_service(backend, request, response)
    assert result.success is True
    assert result.method == "demo_transport"


def test_attach_service_handler_exception_reports_failure():
    from ur5_tools.gripper_attach_backend import GripperAttachBackend

    backend = _make_backend_mock()
    backend._on_gripper_attach = MagicMock(side_effect=RuntimeError("boom"))
    request = AttachSrv.Request()
    request.object_name = "box_red"
    response = AttachSrv.Response()
    result = GripperAttachBackend._on_attach_service(backend, request, response)
    assert result.success is False
    assert "attach_exception" in result.message
    assert "boom" in result.message


# ---------------------------------------------------------------------------
# Detach
# ---------------------------------------------------------------------------


def test_detach_service_empty_name_releases_all_attached():
    from ur5_tools.gripper_attach_backend import GripperAttachBackend

    backend = _make_backend_mock()
    backend._attached = {"box_red": "x", "cyl_orange": "y", "box_blue": "z"}
    request = DetachSrv.Request()
    request.object_name = ""
    response = DetachSrv.Response()
    result = GripperAttachBackend._on_detach_service(backend, request, response)
    assert result.success is True
    assert result.detached_count == 3
    assert "detached_all" in result.message
    assert backend._on_gripper_detach.call_count == 3


def test_detach_service_specific_name_releases_one():
    from ur5_tools.gripper_attach_backend import GripperAttachBackend

    backend = _make_backend_mock()
    backend._attached = {"box_red": "x", "cyl_orange": "y"}
    request = DetachSrv.Request()
    request.object_name = "box_red"
    response = DetachSrv.Response()
    result = GripperAttachBackend._on_detach_service(backend, request, response)
    assert result.success is True
    assert result.detached_count == 1
    assert "detached=box_red" in result.message
    backend._on_gripper_detach.assert_called_once()
    assert backend._on_gripper_detach.call_args.kwargs["name"] == "box_red"


def test_detach_service_handler_exceptions_collected():
    from ur5_tools.gripper_attach_backend import GripperAttachBackend

    def _maybe_raise(_msg, *, name, src_topic):
        if name == "bad":
            raise RuntimeError("explode")

    backend = _make_backend_mock()
    backend._attached = {"good": "x", "bad": "y"}
    backend._on_gripper_detach = MagicMock(side_effect=_maybe_raise)
    request = DetachSrv.Request()
    request.object_name = ""
    response = DetachSrv.Response()
    result = GripperAttachBackend._on_detach_service(backend, request, response)
    assert result.success is False
    assert result.detached_count == 1  # 'good' fue OK
    assert "detach_errors" in result.message
    assert "bad" in result.message
    assert "explode" in result.message
