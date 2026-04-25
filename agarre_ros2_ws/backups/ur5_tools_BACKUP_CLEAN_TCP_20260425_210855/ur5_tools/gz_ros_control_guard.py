#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/gz_ros_control_guard.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/gz_ros_control_guard.py
# Summary: Ensures gz_ros_control parameters are set once /gz_ros_control is available.
"""Set gz_ros_control parameters after the node appears."""

from __future__ import annotations

import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from .param_utils import read_float_param, read_str_param


class GzRosControlGuard(Node):
    """Apply parameter overrides to /gz_ros_control once available."""

    def __init__(self) -> None:
        super().__init__("gz_ros_control_guard")
        self._declare_if_missing("target_node", "/gz_ros_control")
        self._declare_if_missing("use_sim_time", True)
        self._declare_if_missing("hold_joints", False)
        self._declare_if_missing("position_proportional_gain", -1.0)
        self._declare_if_missing("wait_timeout_sec", 10.0)
        self._declare_if_missing("service_wait_sec", 0.5)
        self._declare_if_missing("verify", False)

        self._target = read_str_param(self, "target_node", "/gz_ros_control")
        self._use_sim_time = bool(self.get_parameter("use_sim_time").value)
        self._hold_joints = bool(self.get_parameter("hold_joints").value)
        self._pos_gain = read_float_param(self, "position_proportional_gain", -1.0)
        self._wait_timeout = read_float_param(
            self, "wait_timeout_sec", 10.0, min_value=0.5
        )
        self._service_wait = read_float_param(
            self, "service_wait_sec", 0.5, min_value=0.1
        )
        self._verify = bool(self.get_parameter("verify").value)

        self._client = AsyncParameterClient(self, self._target)

    def _declare_if_missing(self, name: str, value) -> None:
        if self.has_parameter(name):
            return
        self.declare_parameter(name, value)

    def _wait_for_service(self) -> bool:
        deadline = time.monotonic() + self._wait_timeout
        while time.monotonic() < deadline:
            if self._client.wait_for_services(timeout_sec=self._service_wait):
                return True
        return False

    def _build_params(self) -> list[Parameter]:
        params = [
            Parameter("use_sim_time", Parameter.Type.BOOL, self._use_sim_time),
            Parameter("hold_joints", Parameter.Type.BOOL, self._hold_joints),
        ]
        if self._pos_gain >= 0.0:
            params.append(
                Parameter(
                    "position_proportional_gain",
                    Parameter.Type.DOUBLE,
                    self._pos_gain,
                )
            )
        return params

    def _set_params(self, params: list[Parameter]) -> bool:
        future = self._client.set_parameters(params)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done():
            return False
        try:
            result = future.result()
        except Exception:
            return False
        if hasattr(result, "results"):
            results = result.results
        elif isinstance(result, (list, tuple)):
            results = result
        else:
            return False
        return all(r.successful for r in results)

    def _verify_params(self, names: list[str]) -> None:
        future = self._client.get_parameters(names)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            self.get_logger().warn("[VERIFY] timeout leyendo parámetros")
            return
        try:
            response = future.result()
        except Exception:
            self.get_logger().warn("[VERIFY] error leyendo parámetros")
            return
        values = getattr(response, "values", None)
        if values is None:
            self.get_logger().warn("[VERIFY] respuesta inesperada")
            return
        for name, param in zip(names, values):
            self.get_logger().info(f"[VERIFY] {name}={param.value}")

    def run_once(self) -> None:
        if not self._wait_for_service():
            self.get_logger().error(
                "gz_ros_control no disponible; no se aplican parámetros"
            )
            return
        params = self._build_params()
        if not params:
            self.get_logger().info("Sin parámetros para aplicar")
            return
        if not self._set_params(params):
            self.get_logger().warn(
                "No se pudieron aplicar los parámetros de gz_ros_control; se continúa sin override"
            )
            return
        self.get_logger().info("Parámetros de gz_ros_control aplicados")
        if self._verify:
            self._verify_params([p.name for p in params])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzRosControlGuard()
    try:
        node.run_once()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    node.destroy_node()
    try:
        rclpy.try_shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
