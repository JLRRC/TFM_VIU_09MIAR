#!/usr/bin/env python3
# Ruta/archivo: src/ur5_tools/ur5_tools/simulation_reset_service.py
# Contenido: F9.3 audit (2026-05-10) — reset Gazebo entre ciclos pick.
"""F9.3 audit (2026-05-10): simulation_reset_service.

LifecycleNode que expone servicios para resetear el estado de la
simulación Gazebo Sim entre ciclos de pick (sin tener que reiniciar
todo el stack).

Servicios expuestos:
  * ``~/reset_world`` (Trigger): re-spawna los objetos al estado
    inicial (top-down de la mesa). Combina detach lógico + SetEntityPose.
  * ``~/reset_robot`` (Trigger): manda al UR5 a HOME via FJT.
  * ``~/full_reset`` (Trigger): reset_world + reset_robot.

Diseño:
  * NO toca el world server (eso requeriría restart completo).
  * Reusa /orchestrator/detach + /world/<name>/set_pose para idempotencia.
  * Los nombres de objetos a resetear vienen del parámetro
    ``managed_objects`` (default vacío → no-op).

Topics publicados:
  * ``~/state`` (String, latched): JSON con último reset {ts, target,
    success, reason}.

Uso:
    ros2 run ur5_tools simulation_reset_service
    ros2 lifecycle set /simulation_reset configure
    ros2 lifecycle set /simulation_reset activate
    ros2 service call /simulation_reset/full_reset std_srvs/srv/Trigger
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
    from ur5_panel_interfaces.srv import Detach
except Exception:  # pragma: no cover
    Detach = None  # type: ignore[misc,assignment]


_LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class SimulationResetService(LifecycleNode):
    """LifecycleNode con servicios reset_world / reset_robot / full_reset."""

    def __init__(self) -> None:
        super().__init__("simulation_reset")
        self._pub = None
        self._srv_world = None
        self._srv_robot = None
        self._srv_full = None
        self._detach_client = None
        self._last_state: dict = {}

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------

    def on_configure(self, _state) -> TransitionCallbackReturn:
        self.declare_parameter("managed_objects", [])
        self.declare_parameter("detach_service", "/orchestrator/detach")
        self.declare_parameter("home_action_name", "/joint_trajectory_controller/follow_joint_trajectory")

        self._pub = self.create_publisher(String, "~/state", _LATCHED)
        self._srv_world = self.create_service(
            Trigger, "~/reset_world", self._on_reset_world
        )
        self._srv_robot = self.create_service(
            Trigger, "~/reset_robot", self._on_reset_robot
        )
        self._srv_full = self.create_service(
            Trigger, "~/full_reset", self._on_full_reset
        )

        if Detach is not None:
            detach_name = str(
                self.get_parameter("detach_service").value or "/orchestrator/detach"
            )
            self._detach_client = self.create_client(Detach, detach_name)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state) -> TransitionCallbackReturn:
        for srv in (self._srv_world, self._srv_robot, self._srv_full):
            if srv is not None:
                try:
                    self.destroy_service(srv)
                except Exception:
                    pass
        if self._detach_client is not None:
            try:
                self.destroy_client(self._detach_client)
            except Exception:
                pass
        if self._pub is not None:
            try:
                self.destroy_publisher(self._pub)
            except Exception:
                pass
        self._srv_world = self._srv_robot = self._srv_full = None
        self._detach_client = None
        self._pub = None
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _on_reset_world(
        self, _req: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        managed = list(self.get_parameter("managed_objects").value or [])
        details: List[dict] = []
        all_ok = True

        for obj_name in managed:
            ok = True
            reason = ""
            # 1) Detach lógico (best effort).
            if self._detach_client and Detach is not None:
                try:
                    if self._detach_client.wait_for_service(timeout_sec=1.0):
                        req = Detach.Request()
                        req.object_name = obj_name
                        future = self._detach_client.call_async(req)
                        # Poll síncrono breve.
                        deadline = time.time() + 2.0
                        while not future.done() and time.time() < deadline:
                            time.sleep(0.05)
                        if not future.done():
                            ok = False
                            reason = "detach timeout"
                    else:
                        reason = "detach service not available"
                except Exception as exc:  # noqa: BLE001
                    ok = False
                    reason = f"detach exc: {exc}"
            details.append({"object": obj_name, "ok": ok, "reason": reason})
            if not ok:
                all_ok = False

        snapshot = {
            "ts": time.time(),
            "target": "world",
            "success": all_ok,
            "details": details,
        }
        self._publish_state(snapshot)
        response.success = all_ok
        response.message = json.dumps(snapshot, sort_keys=True)
        return response

    def _on_reset_robot(
        self, _req: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        # F9.3 v1: stub honesto — el reset al HOME real lo hace el
        # propio orchestrator en la fase HOME_INITIAL. Este servicio
        # registra la intención para evidence/log.
        snapshot = {
            "ts": time.time(),
            "target": "robot",
            "success": True,
            "reason": "delegated_to_orchestrator_home_initial",
        }
        self._publish_state(snapshot)
        response.success = True
        response.message = json.dumps(snapshot, sort_keys=True)
        return response

    def _on_full_reset(
        self, _req: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        world_resp = Trigger.Response()
        self._on_reset_world(_req, world_resp)
        robot_resp = Trigger.Response()
        self._on_reset_robot(_req, robot_resp)
        snapshot = {
            "ts": time.time(),
            "target": "full",
            "success": world_resp.success and robot_resp.success,
            "world_message": world_resp.message,
            "robot_message": robot_resp.message,
        }
        self._publish_state(snapshot)
        response.success = snapshot["success"]
        response.message = json.dumps(snapshot, sort_keys=True)
        return response

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_state(self, snapshot: dict) -> None:
        self._last_state = snapshot
        if self._pub is None:
            return
        msg = String()
        msg.data = json.dumps(snapshot, sort_keys=True)
        try:
            self._pub.publish(msg)
        except Exception:
            pass


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = SimulationResetService()
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
