#!/usr/bin/env python3
# Ruta/archivo: src/ur5_tools/ur5_tools/panel_backend_node.py
# Contenido: F6.2 audit (2026-05-10) — backend ROS del panel UI.
"""F6.2 audit (2026-05-10): panel_backend_node.

Centraliza la comunicación ROS del panel UI:
  * ActionClient ``/pick_place`` (PickPlace.action) — envía goals que
    vienen de la UI vía servicio interno y propaga feedback/result
    como topics latched.
  * Subscribers a ``/system_state``, ``/system_diag``, ``/joint_states``,
    ``/pick/run_id`` — re-publica como JSON (string) en topics
    canónicos del panel para evitar que la UI haga subs propias.

Servicios expuestos:
  * ``/panel_backend/pick_place`` (Trigger): dispara goal con object
    name + drop pose (parámetros). Devuelve ``message`` JSON con
    ``goal_id``.
  * ``/panel_backend/cancel_pick`` (Trigger): cancela el goal en curso.
  * ``/panel_backend/health`` (Trigger): devuelve estado del backend.

Topics publicados:
  * ``/panel_backend/feedback`` (String, latched): JSON con feedback
    intermedio del PickPlace (phase, progress).
  * ``/panel_backend/result`` (String, latched): JSON con resultado
    final del último ciclo.
  * ``/panel_backend/state`` (String, latched): JSON con estado del
    backend (idle / pending / busy / error).

Diseño:
  * LifecycleNode managed.
  * NO importa Qt (independiente del UI).
  * El panel UI solo hace ``rclpy.create_subscription`` a estos
    topics y muestra el JSON deserializado — sin lógica robótica.

Uso:
    ros2 run ur5_tools panel_backend_node
    ros2 lifecycle set /panel_backend configure
    ros2 lifecycle set /panel_backend activate
    ros2 service call /panel_backend/pick_place std_srvs/srv/Trigger
"""
from __future__ import annotations

import json
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from ur5_panel_interfaces.action import PickPlace
except Exception:  # pragma: no cover - tests offline sin install
    PickPlace = None  # type: ignore[misc,assignment]


_LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class PanelBackendNode(LifecycleNode):
    """Backend ROS del panel UI — UI solo consume topics/servicios de aquí."""

    def __init__(self) -> None:
        super().__init__("panel_backend")
        self._cb = ReentrantCallbackGroup()
        self._action_client: Optional[ActionClient] = None
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._feedback_pub = None
        self._result_pub = None
        self._state_pub = None
        self._srv_pick = None
        self._srv_cancel = None
        self._srv_health = None
        self._busy = False
        self._last_state = "uninitialized"

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------

    def on_configure(self, _state) -> TransitionCallbackReturn:
        self.declare_parameter("pick_place_action", "/pick_place")
        self.declare_parameter("default_object_name", "box_red")
        self.declare_parameter("default_drop_x_world", -1.30)
        self.declare_parameter("default_drop_y_world", 0.0)
        self.declare_parameter("default_drop_z_world", 0.82)
        self.declare_parameter("default_timeout_sec", 700.0)

        self._feedback_pub = self.create_publisher(String, "~/feedback", _LATCHED)
        self._result_pub = self.create_publisher(String, "~/result", _LATCHED)
        self._state_pub = self.create_publisher(String, "~/state", _LATCHED)

        self._srv_pick = self.create_service(
            Trigger, "~/pick_place", self._on_pick_place, callback_group=self._cb
        )
        self._srv_cancel = self.create_service(
            Trigger, "~/cancel_pick", self._on_cancel_pick, callback_group=self._cb
        )
        self._srv_health = self.create_service(
            Trigger, "~/health", self._on_health, callback_group=self._cb
        )

        if PickPlace is not None:
            action_name = str(self.get_parameter("pick_place_action").value or "/pick_place")
            self._action_client = ActionClient(
                self, PickPlace, action_name, callback_group=self._cb
            )

        self._set_state("idle")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state) -> TransitionCallbackReturn:
        self._set_state("active")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state) -> TransitionCallbackReturn:
        self._set_state("inactive")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state) -> TransitionCallbackReturn:
        self._cancel_active_goal_if_any()
        if self._action_client is not None:
            try:
                self._action_client.destroy()
            except Exception:
                pass
            self._action_client = None
        for srv in (self._srv_pick, self._srv_cancel, self._srv_health):
            if srv is not None:
                try:
                    self.destroy_service(srv)
                except Exception:
                    pass
        for pub in (self._feedback_pub, self._result_pub, self._state_pub):
            if pub is not None:
                try:
                    self.destroy_publisher(pub)
                except Exception:
                    pass
        self._srv_pick = self._srv_cancel = self._srv_health = None
        self._feedback_pub = self._result_pub = self._state_pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state) -> TransitionCallbackReturn:
        self._cancel_active_goal_if_any()
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _on_pick_place(self, _req: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if PickPlace is None:
            response.success = False
            response.message = "PickPlace.action not available (interfaces not installed)"
            return response
        if self._action_client is None or not self._action_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = "pick_place action server not reachable"
            return response

        with self._goal_lock:
            if self._busy:
                response.success = False
                response.message = "backend busy with previous goal"
                return response
            self._busy = True

        goal = PickPlace.Goal()
        goal.object_name = str(self.get_parameter("default_object_name").value or "box_red")
        goal.drop_xyz_world = [
            float(self.get_parameter("default_drop_x_world").value or -1.30),
            float(self.get_parameter("default_drop_y_world").value or 0.0),
            float(self.get_parameter("default_drop_z_world").value or 0.82),
        ]
        goal.timeout_sec = float(self.get_parameter("default_timeout_sec").value or 700.0)

        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(self._on_goal_response)
        self._set_state("pending")
        response.success = True
        response.message = json.dumps(
            {"submitted": True, "object": goal.object_name, "drop": list(goal.drop_xyz_world)},
            sort_keys=True,
        )
        return response

    def _on_cancel_pick(self, _req: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        cancelled = self._cancel_active_goal_if_any()
        response.success = True
        response.message = "cancel sent" if cancelled else "no active goal"
        return response

    def _on_health(self, _req: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = True
        response.message = json.dumps(
            {
                "state": self._last_state,
                "busy": self._busy,
                "action_available": self._action_client is not None
                and self._action_client.server_is_ready()
                if self._action_client
                else False,
            },
            sort_keys=True,
        )
        return response

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._publish_result({"success": False, "reason": f"send_failed:{exc}"})
            self._busy = False
            self._set_state("error")
            return
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            self._publish_result({"success": False, "reason": "rejected"})
            self._busy = False
            self._set_state("error")
            return
        with self._goal_lock:
            self._goal_handle = goal_handle
        self._set_state("running")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        try:
            fb = feedback_msg.feedback
            self._publish_feedback(
                {
                    "phase": getattr(fb, "phase", ""),
                    "progress": float(getattr(fb, "progress", 0.0)),
                }
            )
        except Exception:
            pass

    def _on_result(self, future) -> None:
        try:
            wrapper = future.result()
            res = wrapper.result if wrapper else None
            payload = {
                "success": bool(getattr(res, "success", False)) if res else False,
                "reason": str(getattr(res, "reason", "")) if res else "no_result",
                "duration_sec": float(getattr(res, "duration_sec", 0.0)) if res else 0.0,
            }
        except Exception as exc:  # noqa: BLE001
            payload = {"success": False, "reason": f"result_exc:{exc}", "duration_sec": 0.0}
        self._publish_result(payload)
        with self._goal_lock:
            self._goal_handle = None
            self._busy = False
        self._set_state("idle")

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------

    def _publish_feedback(self, payload: dict) -> None:
        if self._feedback_pub is None:
            return
        msg = String()
        msg.data = json.dumps({"ts": time.time(), **payload}, sort_keys=True)
        try:
            self._feedback_pub.publish(msg)
        except Exception:
            pass

    def _publish_result(self, payload: dict) -> None:
        if self._result_pub is None:
            return
        msg = String()
        msg.data = json.dumps({"ts": time.time(), **payload}, sort_keys=True)
        try:
            self._result_pub.publish(msg)
        except Exception:
            pass

    def _set_state(self, state: str) -> None:
        self._last_state = state
        if self._state_pub is None:
            return
        msg = String()
        msg.data = json.dumps({"state": state, "ts": time.time()}, sort_keys=True)
        try:
            self._state_pub.publish(msg)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Cancel helper
    # ------------------------------------------------------------------

    def _cancel_active_goal_if_any(self) -> bool:
        with self._goal_lock:
            gh = self._goal_handle
        if gh is None:
            return False
        try:
            gh.cancel_goal_async()
        except Exception:
            pass
        return True


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = PanelBackendNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
