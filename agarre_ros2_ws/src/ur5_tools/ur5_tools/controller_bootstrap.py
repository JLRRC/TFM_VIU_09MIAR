#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/controller_bootstrap.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/controller_bootstrap.py
# Summary: Load/configure/activate ros2_control controllers once using controller_manager services.
"""Bootstrap ros2_control controllers without respawning active ones."""

from __future__ import annotations

import threading
import time
from typing import Dict

from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

from .param_utils import read_float_param, read_str_list_param, read_str_param


class ControllerBootstrap(Node):
    """One-shot controller loader/activator."""

    def __init__(self) -> None:
        super().__init__("controller_bootstrap")
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter(
            "required_controllers",
            [
                "joint_state_broadcaster",
                "joint_trajectory_controller",
                "gripper_controller",
            ],
        )
        self.declare_parameter("wait_for_clock", True)
        self.declare_parameter("clock_timeout_sec", 10.0)
        self.declare_parameter("service_timeout_sec", 5.0)
        self.declare_parameter("autostart", True)
        self.declare_parameter("stay_alive", False)
        self.declare_parameter("retry_period_sec", 1.0)

        self._cm = read_str_param(self, "controller_manager", "/controller_manager")
        self._required = read_str_list_param(self, "required_controllers")
        self._wait_clock = bool(self.get_parameter("wait_for_clock").value)
        self._clock_timeout = read_float_param(
            self, "clock_timeout_sec", 10.0, min_value=0.5
        )
        self._service_timeout = read_float_param(
            self, "service_timeout_sec", 5.0, min_value=0.5
        )
        self._retry_period = read_float_param(
            self, "retry_period_sec", 1.0, min_value=0.2
        )
        self._last_clock_wall = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._retry_timer = None
        self._retry_attempts = 0
        self._cb_group = ReentrantCallbackGroup()
        self._service = self.create_service(
            Trigger,
            "controller_bootstrap/run",
            self._on_run,
            callback_group=self._cb_group,
        )

        self.create_subscription(
            Clock,
            "/clock",
            self._on_clock,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )

        self._list_client = self.create_client(
            ListControllers,
            f"{self._cm}/list_controllers",
            callback_group=self._cb_group,
        )
        self._load_client = self.create_client(
            LoadController,
            f"{self._cm}/load_controller",
            callback_group=self._cb_group,
        )
        self._configure_client = self.create_client(
            ConfigureController,
            f"{self._cm}/configure_controller",
            callback_group=self._cb_group,
        )
        self._switch_client = self.create_client(
            SwitchController,
            f"{self._cm}/switch_controller",
            callback_group=self._cb_group,
        )

    def _on_run(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok = self.run_once()
        res.success = ok
        res.message = "ok" if ok else "failed"
        return res

    def _on_clock(self, _msg: Clock) -> None:
        self._last_clock_wall = time.monotonic()

    def _clock_ok(self) -> bool:
        if not self._wait_clock:
            return True
        if self._last_clock_wall <= 0.0:
            return False
        return (time.monotonic() - self._last_clock_wall) <= 1.5

    def _wait_for_services(self) -> bool:
        deadline = time.monotonic() + self._service_timeout
        while time.monotonic() < deadline:
            if (
                self._list_client.service_is_ready()
                and self._load_client.service_is_ready()
                and self._configure_client.service_is_ready()
                and self._switch_client.service_is_ready()
            ):
                return True
            time.sleep(0.2)
        return False

    def _call(self, client, req, timeout: float) -> bool:
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return False
        try:
            result = future.result()
            if hasattr(result, "ok"):
                return bool(result.ok)
            if hasattr(result, "success"):
                return bool(result.success)
            return bool(result)
        except Exception:
            return False

    def _list_controllers(self) -> Dict[str, str]:
        req = ListControllers.Request()
        future = self._list_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            return {}
        try:
            resp = future.result()
        except Exception:
            return {}
        states = {}
        for ctrl in resp.controller:
            states[str(ctrl.name)] = str(ctrl.state)
        return states

    def _activate(self, name: str) -> bool:
        req = SwitchController.Request()
        req.activate_controllers = [name]
        req.deactivate_controllers = []
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout.sec = int(max(2.0, self._service_timeout))
        return self._call(self._switch_client, req, max(3.0, self._service_timeout))

    def _configure(self, name: str) -> bool:
        req = ConfigureController.Request()
        req.name = name
        return self._call(self._configure_client, req, max(2.0, self._service_timeout))

    def _load(self, name: str) -> bool:
        req = LoadController.Request()
        req.name = name
        return self._call(self._load_client, req, max(2.0, self._service_timeout))

    def _controllers_active(self) -> bool:
        state_map = self._list_controllers()
        if not state_map:
            return False
        return all(state_map.get(name) == "active" for name in self._required)

    def _start_retry_timer(self) -> None:
        if self._retry_timer is not None:
            return
        self.get_logger().warn(
            f"[CTRL] autostart diferido; reintentando cada {self._retry_period:.1f}s"
        )
        self._retry_timer = self.create_timer(self._retry_period, self._retry_tick)

    def _stop_retry_timer(self) -> None:
        if self._retry_timer is None:
            return
        try:
            self._retry_timer.cancel()
        except Exception:
            pass
        self._retry_timer = None

    def _retry_tick(self) -> None:
        if self._running:
            return
        if self._controllers_active():
            self.get_logger().info("[CTRL] controladores activos; fin de reintentos")
            self._stop_retry_timer()
            return
        self._retry_attempts += 1
        ok = self.run_once()
        if ok:
            self.get_logger().info(
                f"[CTRL] bootstrap recuperado en retry#{self._retry_attempts}"
            )
            self._stop_retry_timer()

    def run_once(self) -> bool:
        with self._lock:
            if self._running:
                self.get_logger().warn("bootstrap ya en curso; ignorando solicitud.")
                return False
            self._running = True
        start = time.monotonic()
        while self._wait_clock and not self._clock_ok():
            # Ensure /clock callbacks are processed while waiting.
            rclpy.spin_once(self, timeout_sec=0.1)
            if (time.monotonic() - start) > self._clock_timeout:
                self.get_logger().error("/clock no disponible; abortando bootstrap")
                with self._lock:
                    self._running = False
                return False
            time.sleep(0.2)

        if not self._wait_for_services():
            self.get_logger().error("controller_manager services no disponibles")
            with self._lock:
                self._running = False
            return False

        state_map = self._list_controllers()
        if not self._required:
            self.get_logger().info("Sin controladores requeridos; bootstrap omitido.")
            with self._lock:
                self._running = False
            return True

        ok_all = True
        for name in self._required:
            state = state_map.get(name)
            if state == "active":
                self.get_logger().info(f"[CTRL] {name} ya activo; skip")
                continue
            if state is None:
                self.get_logger().info(f"[CTRL] {name} no cargado; cargando")
                if not self._load(name):
                    state_after_load = self._list_controllers().get(name)
                    if state_after_load is None:
                        self.get_logger().error(f"[CTRL] load failed: {name}")
                        ok_all = False
                        continue
                    self.get_logger().warn(
                        f"[CTRL] load reported failed; {name} visible ({state_after_load})"
                    )
                if not self._configure(name):
                    state_after_cfg = self._list_controllers().get(name)
                    if state_after_cfg not in ("inactive", "configured", "active"):
                        self.get_logger().error(f"[CTRL] configure failed: {name}")
                        ok_all = False
                        continue
                    self.get_logger().warn(
                        f"[CTRL] configure reported failed; {name} state={state_after_cfg}"
                    )
            elif state in ("unconfigured", "inactive", "configured"):
                if state == "unconfigured":
                    if not self._configure(name):
                        state_after_cfg = self._list_controllers().get(name)
                        if state_after_cfg not in ("inactive", "configured", "active"):
                            self.get_logger().error(f"[CTRL] configure failed: {name}")
                            ok_all = False
                            continue
                        self.get_logger().warn(
                            f"[CTRL] configure reported failed; {name} state={state_after_cfg}"
                        )
            else:
                self.get_logger().warn(f"[CTRL] estado inesperado {name}: {state}")
            if not self._activate(name):
                state_after_act = self._list_controllers().get(name)
                if state_after_act != "active":
                    self.get_logger().error(f"[CTRL] activate failed: {name}")
                    ok_all = False
                    continue
                self.get_logger().warn(
                    f"[CTRL] activate reported failed; {name} state=active"
                )
            self.get_logger().info(f"[CTRL] {name} activo")
        with self._lock:
            self._running = False
        return ok_all


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerBootstrap()
    try:
        autostart = bool(node.get_parameter("autostart").value)
        stay_alive = bool(node.get_parameter("stay_alive").value)
        if autostart:
            ok = node.run_once()
            if stay_alive and not ok:
                node._start_retry_timer()
        if stay_alive:
            executor = MultiThreadedExecutor(num_threads=2)
            executor.add_node(node)
            executor.spin()
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.try_shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
