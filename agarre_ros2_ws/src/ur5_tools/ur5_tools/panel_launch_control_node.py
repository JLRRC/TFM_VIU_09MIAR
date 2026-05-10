#!/usr/bin/env python3
# Ruta/archivo: src/ur5_tools/ur5_tools/panel_launch_control_node.py
# Contenido: F6.1 audit (2026-05-10) — nodo de control de procesos del stack.
"""F6.1 audit (2026-05-10): panel_launch_control_node.

Centraliza el lanzamiento/parada de procesos del stack UR5+RG2 que
históricamente vivía en ``panel_launchers.py`` del paquete Qt. El UI
ahora solo consume servicios de este nodo y deja de invocar
``subprocess.*`` directamente.

Servicios expuestos:
  * ``/panel_launch_control/start_stack`` (Trigger): arranca el stack
    completo ``ros2 launch ur5_bringup ur5_stack.launch.py``.
  * ``/panel_launch_control/stop_stack`` (Trigger): para todos los
    procesos lanzados por este nodo.
  * ``/panel_launch_control/restart_stack`` (Trigger): stop + start.
  * ``/panel_launch_control/status`` (Trigger): devuelve PID + uptime
    en ``message`` JSON.

Diseño:
  * LifecycleNode managed: ``on_configure`` declara params + servicios;
    ``on_activate`` permite responder a clientes.
  * NO maneja state Qt — responde 1:1 a servicios y publica
    ``/panel_launch_control/state`` (latched String JSON).
  * Subprocess gestionado vía ``subprocess.Popen`` con grupo de
    procesos (``preexec_fn=os.setsid``) para SIGTERM atómico.

Uso:
    ros2 run ur5_tools panel_launch_control_node
    ros2 lifecycle set /panel_launch_control configure
    ros2 lifecycle set /panel_launch_control activate
    ros2 service call /panel_launch_control/start_stack std_srvs/srv/Trigger
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger


@dataclass
class _LaunchedProcess:
    """Estado mínimo de un proceso gestionado."""

    name: str
    cmd: List[str]
    process: subprocess.Popen
    started_at: float = field(default_factory=time.time)

    @property
    def pid(self) -> Optional[int]:
        try:
            return self.process.pid
        except Exception:
            return None

    @property
    def is_running(self) -> bool:
        return self.process.poll() is None

    @property
    def uptime_sec(self) -> float:
        return time.time() - self.started_at


class PanelLaunchControlNode(LifecycleNode):
    """LifecycleNode con servicios start/stop/restart del stack UR5+RG2."""

    def __init__(self) -> None:
        super().__init__("panel_launch_control")
        self._processes: dict[str, _LaunchedProcess] = {}
        self._state_pub = None
        self._srv_start = None
        self._srv_stop = None
        self._srv_restart = None
        self._srv_status = None

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------

    def on_configure(self, _state) -> TransitionCallbackReturn:
        self.declare_parameter("stack_launch_pkg", "ur5_bringup")
        self.declare_parameter("stack_launch_file", "ur5_stack.launch.py")
        self.declare_parameter(
            "stack_extra_args",
            "",
            descriptor=None,  # str con args separados por espacios
        )
        self.declare_parameter("stop_grace_sec", 5.0)
        self.declare_parameter("kill_grace_sec", 3.0)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(
            String, "~/state", latched
        )
        self._srv_start = self.create_service(
            Trigger, "~/start_stack", self._on_start_stack
        )
        self._srv_stop = self.create_service(
            Trigger, "~/stop_stack", self._on_stop_stack
        )
        self._srv_restart = self.create_service(
            Trigger, "~/restart_stack", self._on_restart_stack
        )
        self._srv_status = self.create_service(
            Trigger, "~/status", self._on_status
        )
        self._publish_state(event="configured")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state) -> TransitionCallbackReturn:
        self._publish_state(event="activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state) -> TransitionCallbackReturn:
        self._publish_state(event="deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state) -> TransitionCallbackReturn:
        self._stop_all_processes(reason="cleanup")
        for srv in (self._srv_start, self._srv_stop, self._srv_restart, self._srv_status):
            if srv is not None:
                try:
                    self.destroy_service(srv)
                except Exception:
                    pass
        if self._state_pub is not None:
            try:
                self.destroy_publisher(self._state_pub)
            except Exception:
                pass
        self._state_pub = None
        self._srv_start = None
        self._srv_stop = None
        self._srv_restart = None
        self._srv_status = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state) -> TransitionCallbackReturn:
        self._stop_all_processes(reason="shutdown")
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _on_start_stack(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if "stack" in self._processes and self._processes["stack"].is_running:
            response.success = True
            response.message = (
                f"stack already running pid={self._processes['stack'].pid} "
                f"uptime={self._processes['stack'].uptime_sec:.1f}s"
            )
            return response
        try:
            pkg = str(self.get_parameter("stack_launch_pkg").value or "ur5_bringup")
            launch_file = str(
                self.get_parameter("stack_launch_file").value or "ur5_stack.launch.py"
            )
            extra = str(self.get_parameter("stack_extra_args").value or "").split()
            cmd = ["ros2", "launch", pkg, launch_file] + extra
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            self._processes["stack"] = _LaunchedProcess(name="stack", cmd=cmd, process=proc)
            self._publish_state(event="started")
            response.success = True
            response.message = f"stack started pid={proc.pid}"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"start failed: {exc}"
        return response

    def _on_stop_stack(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        n = self._stop_all_processes(reason="stop_request")
        response.success = True
        response.message = f"stopped {n} process(es)"
        self._publish_state(event="stopped")
        return response

    def _on_restart_stack(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._stop_all_processes(reason="restart")
        return self._on_start_stack(_request, response)

    def _on_status(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        snapshot = self._status_snapshot()
        response.success = True
        response.message = json.dumps(snapshot, sort_keys=True)
        return response

    # ------------------------------------------------------------------
    # Helpers (puros y testables sin ROS)
    # ------------------------------------------------------------------

    def _status_snapshot(self) -> dict:
        return {
            "processes": {
                name: {
                    "pid": p.pid,
                    "running": p.is_running,
                    "uptime_sec": round(p.uptime_sec, 2),
                    "cmd": p.cmd,
                }
                for name, p in self._processes.items()
            },
        }

    def _publish_state(self, event: str) -> None:
        if self._state_pub is None:
            return
        msg = String()
        msg.data = json.dumps({"event": event, **self._status_snapshot()}, sort_keys=True)
        try:
            self._state_pub.publish(msg)
        except Exception:
            pass

    def _stop_all_processes(self, reason: str) -> int:
        """Detiene todos los procesos gestionados; devuelve cuántos se pararon."""
        stopped = 0
        grace = float(self.get_parameter("stop_grace_sec").value or 5.0)
        kill_grace = float(self.get_parameter("kill_grace_sec").value or 3.0)
        for name, info in list(self._processes.items()):
            if not info.is_running:
                self._processes.pop(name, None)
                continue
            try:
                pgid = os.getpgid(info.pid) if info.pid else None
                if pgid is not None:
                    os.killpg(pgid, signal.SIGTERM)
                else:
                    info.process.terminate()
                deadline = time.time() + grace
                while info.is_running and time.time() < deadline:
                    time.sleep(0.05)
                if info.is_running:
                    if pgid is not None:
                        os.killpg(pgid, signal.SIGKILL)
                    else:
                        info.process.kill()
                    deadline = time.time() + kill_grace
                    while info.is_running and time.time() < deadline:
                        time.sleep(0.05)
                stopped += 1
            except Exception:
                pass
            self._processes.pop(name, None)
        return stopped


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = PanelLaunchControlNode()
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
