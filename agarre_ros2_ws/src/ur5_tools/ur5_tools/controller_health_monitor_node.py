#!/usr/bin/env python3
# Ruta/archivo: src/ur5_tools/ur5_tools/controller_health_monitor_node.py
# Contenido: F9.2 audit (2026-05-10) — watchdog ros2_control controllers.
"""F9.2 audit (2026-05-10): controller_health_monitor_node.

LifecycleNode que monitorea el estado de los controllers de
ros2_control vía ``/controller_manager/list_controllers``. Publica un
estado agregado en ``/controller_health`` (latched String JSON) y
expone un servicio ``~/health`` para query síncrona.

Propósito:
  * Detectar early que ``joint_trajectory_controller`` o
    ``joint_state_broadcaster`` no están en estado ``active``.
  * Alimentar un dashboard / panel UI con estado fresco.
  * Aportar trazabilidad ante fallos del bootstrap.

Topics publicados:
  * ``/controller_health`` (String, latched): JSON
    ``{ts, all_active, controllers: [{name, type, state, claimed}], details}``.

Servicios expuestos:
  * ``~/health`` (Trigger): devuelve el último snapshot como mensaje JSON.

Parámetros:
  * ``poll_period_sec`` (default 1.0)
  * ``required_controllers`` (lista, default
    ['joint_state_broadcaster', 'joint_trajectory_controller'])
  * ``controller_manager_name`` (default 'controller_manager')

Uso:
    ros2 run ur5_tools controller_health_monitor_node
    ros2 lifecycle set /controller_health_monitor configure
    ros2 lifecycle set /controller_health_monitor activate
"""
from __future__ import annotations

import json
import time
from typing import List, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from controller_manager_msgs.srv import ListControllers
except Exception:  # pragma: no cover
    ListControllers = None  # type: ignore[misc,assignment]


_LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_DEFAULT_REQUIRED = (
    "joint_state_broadcaster",
    "joint_trajectory_controller",
)


class ControllerHealthMonitorNode(LifecycleNode):
    """Watchdog ros2_control: poll periódico + servicio query."""

    def __init__(self) -> None:
        super().__init__("controller_health_monitor")
        self._pub = None
        self._timer = None
        self._srv = None
        self._client = None
        self._last_snapshot: dict = {"ts": 0.0, "all_active": False, "controllers": []}

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------

    def on_configure(self, _state) -> TransitionCallbackReturn:
        self.declare_parameter("poll_period_sec", 1.0)
        self.declare_parameter("required_controllers", list(_DEFAULT_REQUIRED))
        self.declare_parameter("controller_manager_name", "controller_manager")

        self._pub = self.create_publisher(String, "/controller_health", _LATCHED)
        self._srv = self.create_service(Trigger, "~/health", self._on_health)

        if ListControllers is not None:
            cm_name = str(
                self.get_parameter("controller_manager_name").value or "controller_manager"
            )
            self._client = self.create_client(
                ListControllers, f"/{cm_name}/list_controllers"
            )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state) -> TransitionCallbackReturn:
        period = float(self.get_parameter("poll_period_sec").value or 1.0)
        self._timer = self.create_timer(period, self._poll_once)
        # Tick inmediato.
        self._poll_once()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state) -> TransitionCallbackReturn:
        if self._timer is not None:
            try:
                self.destroy_timer(self._timer)
            except Exception:
                pass
            self._timer = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state) -> TransitionCallbackReturn:
        if self._timer is not None:
            try:
                self.destroy_timer(self._timer)
            except Exception:
                pass
            self._timer = None
        if self._client is not None:
            try:
                self.destroy_client(self._client)
            except Exception:
                pass
            self._client = None
        if self._srv is not None:
            try:
                self.destroy_service(self._srv)
            except Exception:
                pass
            self._srv = None
        if self._pub is not None:
            try:
                self.destroy_publisher(self._pub)
            except Exception:
                pass
            self._pub = None
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Polling logic
    # ------------------------------------------------------------------

    def _poll_once(self) -> None:
        if self._client is None or not self._client.service_is_ready():
            self._last_snapshot = {
                "ts": time.time(),
                "all_active": False,
                "controllers": [],
                "error": "controller_manager service not available",
            }
            self._publish_snapshot()
            return
        req = ListControllers.Request()
        future = self._client.call_async(req)
        future.add_done_callback(self._on_list_response)

    def _on_list_response(self, future) -> None:
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self._last_snapshot = {
                "ts": time.time(),
                "all_active": False,
                "controllers": [],
                "error": f"list_controllers failed: {exc}",
            }
            self._publish_snapshot()
            return

        ctrls = []
        names_active = set()
        for c in resp.controller:
            ctrls.append(
                {
                    "name": c.name,
                    "type": c.type,
                    "state": c.state,
                    "claimed": list(c.claimed_interfaces),
                }
            )
            if c.state == "active":
                names_active.add(c.name)

        required = list(self.get_parameter("required_controllers").value or _DEFAULT_REQUIRED)
        missing = [n for n in required if n not in names_active]
        all_active = not missing

        self._last_snapshot = {
            "ts": time.time(),
            "all_active": all_active,
            "controllers": ctrls,
            "required": required,
            "missing": missing,
        }
        self._publish_snapshot()

    def _publish_snapshot(self) -> None:
        if self._pub is None:
            return
        msg = String()
        msg.data = json.dumps(self._last_snapshot, sort_keys=True)
        try:
            self._pub.publish(msg)
        except Exception:
            pass

    def _on_health(self, _req: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = bool(self._last_snapshot.get("all_active", False))
        response.message = json.dumps(self._last_snapshot, sort_keys=True)
        return response


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ControllerHealthMonitorNode()
    executor = MultiThreadedExecutor(num_threads=2)  # iter4-bis: limitar threads (antes default = cpu_count = 8)
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
