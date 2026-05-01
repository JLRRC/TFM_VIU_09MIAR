#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_place_client.py
# Contenido: F6.4 — ActionClient wrapper de PickPlace para el panel Qt.
"""Cliente thin de la action ``PickPlace`` para el panel Qt.

Encapsula el ``ActionClient`` de rclpy y expone una API minimal
amigable al panel:

* ``PickPlaceClient(node, action_name=...)``: constructor.
* ``send_goal(object_name, drop_xyz_world, on_feedback, on_done)``:
  envía el goal asíncronamente. Los callbacks se invocan desde el
  hilo del executor ROS — el panel debe re-postear a Qt vía
  ``signal_run_ui.emit`` o similar.
* ``cancel_active_goal()``: cancela el goal actual si existe.

La lógica de validación / conversión vive en
``pick_place_client_logic.py`` (testeable sin ROS).

Si rclpy no está disponible, el constructor levanta ``ImportError``.
El panel debería verificar ``rclpy_available()`` antes de instanciar.
"""

from __future__ import annotations

from typing import Any, Callable, Optional, Tuple


def rclpy_available() -> bool:
    """Devuelve True si rclpy + la action PickPlace son importables."""
    try:
        import rclpy  # noqa: F401
        from rclpy.action import ActionClient  # noqa: F401
        from ur5_panel_interfaces.action import PickPlace  # noqa: F401
    except ImportError:
        return False
    return True


class PickPlaceClient:
    """Action client de ``/pick_place`` con interfaz amigable al panel."""

    def __init__(
        self,
        node: Any,
        action_name: str = "/pick_place",
    ) -> None:
        from rclpy.action import ActionClient
        from ur5_panel_interfaces.action import PickPlace

        self._node = node
        self._action_name = str(action_name or "/pick_place").strip()
        self._action_type = PickPlace
        self._client = ActionClient(node, PickPlace, self._action_name)
        self._active_goal_handle: Any = None

    def wait_for_server(self, timeout_sec: float = 2.0) -> bool:
        """True si el server aparece en ``timeout_sec``."""
        return bool(self._client.wait_for_server(timeout_sec=timeout_sec))

    def send_goal(
        self,
        object_name: str,
        drop_xyz_world: Tuple[float, float, float],
        *,
        on_feedback: Optional[Callable[[dict], None]] = None,
        on_done: Optional[Callable[[dict], None]] = None,
        on_rejected: Optional[Callable[[], None]] = None,
    ) -> bool:
        """Envía el goal asíncronamente.

        Validación previa via ``build_goal_request``. Si inválido,
        no envía nada y devuelve False.

        Devuelve True si el goal se envió (no garantiza accept).
        """
        from geometry_msgs.msg import Point
        from .pick_place_client_logic import (
            build_goal_request,
            feedback_to_panel_event,
            result_to_panel_event,
        )

        request, reason = build_goal_request(object_name, drop_xyz_world)
        if request is None:
            self._node.get_logger().warning(
                f"[PICK_PLACE_CLIENT] goal inválido: {reason}"
            )
            if on_done is not None:
                try:
                    on_done({
                        "success": False,
                        "reason": reason,
                        "duration_sec": 0.0,
                        "cycles_completed": 0,
                    })
                except Exception:
                    pass
            return False

        goal = self._action_type.Goal()
        goal.object_name = request.object_name
        goal.drop_xyz_world = Point(
            x=float(request.drop_xyz_world[0]),
            y=float(request.drop_xyz_world[1]),
            z=float(request.drop_xyz_world[2]),
        )

        def _feedback_cb(fb_msg):
            if on_feedback is None:
                return
            try:
                fb = getattr(fb_msg, "feedback", fb_msg)
                on_feedback(feedback_to_panel_event(fb))
            except Exception:
                pass

        send_future = self._client.send_goal_async(
            goal, feedback_callback=_feedback_cb
        )

        def _on_send_done(future):
            try:
                handle = future.result()
            except Exception:
                handle = None
            if handle is None or not getattr(handle, "accepted", False):
                if on_rejected is not None:
                    try:
                        on_rejected()
                    except Exception:
                        pass
                return
            self._active_goal_handle = handle
            result_future = handle.get_result_async()

            def _on_result_done(rf):
                try:
                    wrapper = rf.result()
                    payload = getattr(wrapper, "result", None)
                except Exception:
                    payload = None
                event = result_to_panel_event(payload)
                if on_done is not None:
                    try:
                        on_done(event)
                    except Exception:
                        pass

            result_future.add_done_callback(_on_result_done)

        send_future.add_done_callback(_on_send_done)
        return True

    def cancel_active_goal(self) -> None:
        """Cancela el goal activo si existe. Best-effort."""
        if self._active_goal_handle is None:
            return
        try:
            self._active_goal_handle.cancel_goal_async()
        except Exception:
            pass
