#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/tf_probe.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/tf_probe.py
# Summary: TF health probe for availability, age, and time continuity.
"""TF probe to validate availability, age, and time continuity of transforms."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

from .param_utils import read_float_param, read_str_list_param


@dataclass
class ProbeState:
    last_stamp_ns: int | None = None
    last_log_ns: int = 0
    last_ok: bool = False
    last_age_ns: int | None = None


class TfProbe(Node):
    """Periodically probe TF availability and timing for selected frame pairs."""

    def __init__(self) -> None:
        super().__init__("tf_probe")
        self.declare_parameter("pairs", ["world,base_link", "base_link,tool0"])
        self.declare_parameter("period_sec", 0.5)
        self.declare_parameter("timeout_sec", 0.2)
        self.declare_parameter("max_age_sec", 0.5)
        self.declare_parameter("log_every_sec", 2.0)
        self.declare_parameter("jump_warn_sec", 2.0)
        self.declare_parameter("startup_grace_sec", 0.0)

        raw_pairs = read_str_list_param(self, "pairs")
        parsed_pairs = [
            parsed
            for item in raw_pairs
            if isinstance(item, str)
            for parsed in [self._parse_pair(item)]
            if parsed is not None
        ]
        if parsed_pairs:
            self._pairs = tuple(parsed_pairs)
        else:
            self._pairs = ("world", "base_link"), ("base_link", "tool0")

        self._period = read_float_param(self, "period_sec", 0.5, min_value=0.1)
        self._timeout = read_float_param(self, "timeout_sec", 0.2, min_value=0.01)
        self._max_age = read_float_param(self, "max_age_sec", 0.5, min_value=0.0)
        self._log_every = read_float_param(self, "log_every_sec", 2.0, min_value=0.2)
        self._jump_warn = read_float_param(self, "jump_warn_sec", 2.0, min_value=0.0)
        self._startup_grace = read_float_param(
            self, "startup_grace_sec", 0.0, min_value=0.0
        )

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._state: Dict[Tuple[str, str], ProbeState] = {
            pair: ProbeState() for pair in self._pairs
        }
        self._last_time_ns: int | None = None
        self._start_ns = int(self.get_clock().now().nanoseconds)
        self._timer = self.create_timer(self._period, self._on_timer)

        pairs_text = ", ".join([f"{a}->{b}" for a, b in self._pairs])
        self.get_logger().info(
            f"TF probe active (pairs={pairs_text}, period={self._period:.2f}s, "
            f"timeout={self._timeout:.2f}s)"
        )

    def _parse_pair(self, item: str) -> Optional[Tuple[str, str]]:
        parts = [p.strip() for p in item.split(",")]
        if len(parts) != 2 or not parts[0] or not parts[1]:
            return None
        return parts[0], parts[1]

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _log(self, level: str, message: str) -> None:
        if level == "warn":
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def _check_time_continuity(self, now_ns: int) -> None:
        if self._last_time_ns is None:
            self._last_time_ns = now_ns
            return
        if now_ns < self._last_time_ns:
            delta = (self._last_time_ns - now_ns) * 1e-9
            self.get_logger().warn(f"/clock moved backwards by {delta:.6f}s")
        self._last_time_ns = now_ns

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        now_ns = int(now.nanoseconds)
        self._check_time_continuity(now_ns)

        for parent, child in self._pairs:
            if not parent or not child:
                continue
            state = self._state[(parent, child)]
            ok, message, age_ns, stamp_ns = self._probe_pair(parent, child, now)
            suppress = (now_ns - self._start_ns) < int(self._startup_grace * 1e9)

            should_log = False
            if ok != state.last_ok:
                should_log = True
            elif now_ns - state.last_log_ns >= int(self._log_every * 1e9):
                should_log = True

            if should_log and not suppress:
                self._log("info" if ok else "warn", message)
                state.last_log_ns = now_ns
                state.last_ok = ok
                state.last_age_ns = age_ns

            if (
                not suppress
                and stamp_ns is not None
                and state.last_stamp_ns is not None
            ):
                if stamp_ns < state.last_stamp_ns:
                    delta = (state.last_stamp_ns - stamp_ns) * 1e-9
                    self.get_logger().warn(
                        f"TF stamp for {parent}->{child} moved backwards by {delta:.6f}s"
                    )
                jump = (stamp_ns - state.last_stamp_ns) * 1e-9
                if jump > self._jump_warn:
                    self.get_logger().warn(
                        f"TF stamp jump for {parent}->{child}: +{jump:.3f}s"
                    )
            if stamp_ns is not None:
                state.last_stamp_ns = stamp_ns

    def _probe_pair(
        self,
        parent: str,
        child: str,
        now: Time,
    ) -> tuple[bool, str, int | None, int | None]:
        timeout = Duration(seconds=self._timeout)
        try:
            tf = self._buffer.lookup_transform(parent, child, Time(), timeout=timeout)
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            return (
                False,
                f"TF timeout {parent}->{child}: {type(exc).__name__}",
                None,
                None,
            )

        stamp = tf.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns == 0:
            return False, f"TF zero stamp {parent}->{child}", None, stamp_ns
        age_ns = now.nanoseconds - stamp_ns
        age_sec = age_ns * 1e-9
        if age_sec > self._max_age:
            return (
                False,
                f"TF stale {parent}->{child}: age={age_sec:.3f}s",
                age_ns,
                stamp_ns,
            )
        return True, f"TF ok {parent}->{child}: age={age_sec:.3f}s", age_ns, stamp_ns


def main() -> None:
    rclpy.init()
    node = TfProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
