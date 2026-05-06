# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_action_client.py
# Contenido: Cliente del action PickPlace para el panel Qt (Bloque 2 cierre).
# Uso breve: Reemplazo del legacy run_pick_demo. Activable con
#            PANEL_PICK_VIA_ACTION=1.
"""Cliente del action ``/pick_place`` (PickPlace.action).

Permite al panel Qt invocar el orchestrator ``pick_orchestrator_lifecycle``
en lugar del legacy ``run_pick_demo`` (panel_pick_demo.py:586). El
orchestrator ya cubre 9/9 fases reales (B-iter1..14, tag
``B-iter6-9-of-9-real-orchestrator-independent-20260503``) y es la fuente
única de la lógica de pick post-cierre del Bloque 2.

Uso:
    client = PickActionClient(panel_node)
    client.request_pick(
        object_name="pick_demo",
        drop_xyz_world=(-1.300, 0.000, 0.820),
        on_phase=lambda phase, progress, detail: panel._emit_log(
            f"[PICK][ACTION][PHASE] {phase} progress={progress:.2f} {detail}"
        ),
        on_result=lambda success, reason, dur, cycles: panel._emit_log(
            f"[PICK][ACTION][RESULT] success={success} reason={reason} dur={dur:.1f}"
        ),
    )
"""
from __future__ import annotations

from typing import Callable, Optional, Tuple

from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Pose, Quaternion
from ur5_panel_interfaces.action import PickPlace


PhaseCallback = Callable[[str, float, str], None]
ResultCallback = Callable[[bool, str, float, int], None]


class PickActionClient:
    """Wrapper de ``rclpy.action.ActionClient`` para ``PickPlace``.

    Lifecycle:

    * ``__init__`` crea el ``ActionClient`` (no bloquea).
    * ``request_pick`` envía un goal asíncrono. Devuelve ``False`` si el
      action server no está disponible tras ``server_wait_timeout_sec``.
    * Los callbacks ``on_phase`` y ``on_result`` se invocan en el thread
      del executor del ``panel_node``. **No** son thread-safe respecto a
      la UI Qt — el caller debe routearlos a slots si toca widgets.
    * ``cancel`` solicita cancelación del goal en curso (no bloquea).
    """

    def __init__(
        self,
        node,
        action_topic: str = "/pick_place",
        server_wait_timeout_sec: float = 2.0,
    ) -> None:
        self._node = node
        self._client = ActionClient(node, PickPlace, action_topic)
        self._server_wait_timeout_sec = float(server_wait_timeout_sec)
        self._goal_handle = None
        self._on_phase: Optional[PhaseCallback] = None
        self._on_result: Optional[ResultCallback] = None
        self._inflight: bool = False

    @property
    def inflight(self) -> bool:
        """True si hay un goal aceptado y aún sin result."""
        return bool(self._inflight)

    def request_pick(
        self,
        object_name: str,
        drop_xyz_world: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        object_pose_world_hint: Optional[Pose] = None,
        on_phase: Optional[PhaseCallback] = None,
        on_result: Optional[ResultCallback] = None,
    ) -> Tuple[bool, str]:
        """Envía un goal PickPlace al orchestrator.

        Devuelve ``(ok, reason)``:
        * ``(True, "sent")`` si el goal se envió (aceptación es asíncrona).
        * ``(False, "server_not_available")`` si el action server no responde.
        * ``(False, "already_inflight")`` si ya hay un goal en curso.
        """
        if self._inflight:
            return False, "already_inflight"
        if not self._client.wait_for_server(timeout_sec=self._server_wait_timeout_sec):
            return False, "server_not_available"

        goal = PickPlace.Goal()
        goal.object_name = str(object_name)
        goal.drop_xyz_world = Point(
            x=float(drop_xyz_world[0]),
            y=float(drop_xyz_world[1]),
            z=float(drop_xyz_world[2]),
        )
        goal.object_pose_world_hint = (
            object_pose_world_hint
            if object_pose_world_hint is not None
            else Pose(orientation=Quaternion(w=1.0))
        )

        self._on_phase = on_phase
        self._on_result = on_result
        self._inflight = True

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(self._on_goal_response)
        return True, "sent"

    def cancel(self) -> bool:
        """Solicita cancelación del goal en curso. No bloquea."""
        if not self._inflight or self._goal_handle is None:
            return False
        try:
            self._goal_handle.cancel_goal_async()
            return True
        except Exception:
            return False

    # ------------------------------------------------------------------
    # Callbacks internos del action client.
    # ------------------------------------------------------------------

    def _on_feedback(self, feedback_msg) -> None:
        if self._on_phase is None:
            return
        try:
            fb = feedback_msg.feedback
            self._on_phase(
                str(fb.current_phase),
                float(fb.progress),
                str(fb.detail),
            )
        except Exception:
            pass  # nunca dejar que el callback rompa el client

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self._inflight = False
            if self._on_result is not None:
                self._on_result(False, f"send_failed:{exc}", 0.0, 0)
            return

        if not handle.accepted:
            self._inflight = False
            if self._on_result is not None:
                self._on_result(False, "rejected_by_orchestrator", 0.0, 0)
            return

        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_result_done)

    def _on_result_done(self, future) -> None:
        self._inflight = False
        self._goal_handle = None
        if self._on_result is None:
            return
        try:
            result = future.result().result
            self._on_result(
                bool(result.success),
                str(result.reason),
                float(result.duration_sec),
                int(result.cycles_completed),
            )
        except Exception as exc:
            self._on_result(False, f"result_failed:{exc}", 0.0, 0)
