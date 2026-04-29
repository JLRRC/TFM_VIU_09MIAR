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

import math

from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

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
        self.declare_parameter(
            "expected_arm_joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        self.declare_parameter("enforce_joint_identity", True)
        self.declare_parameter("joint_identity_timeout_sec", 3.0)

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
        self._expected_arm_joints = set(read_str_list_param(self, "expected_arm_joints"))
        self._enforce_joint_identity = bool(
            self.get_parameter("enforce_joint_identity").value
        )
        self._joint_identity_timeout = read_float_param(
            self, "joint_identity_timeout_sec", 3.0, min_value=0.2
        )
        self._last_clock_wall = 0.0
        self._last_joint_names = set()
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
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
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

    def _on_joint_states(self, msg: JointState) -> None:
        names = {str(name).strip() for name in (msg.name or []) if str(name).strip()}
        self._last_joint_names = names

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

    def _command_initial_pose(self) -> None:
        """Lleva el brazo a MESA via action FollowJointTrajectory, tolerando wrap.

        Estrategia anti-spawn-roll:
        1) Lee /joint_states real (con timeout) para conocer la pose actual
           tras spawn (los wrists pueden estar enrollados a ±2π).
        2) Calcula MESA aliasada al wrap actual (camino angular corto).
        3) Envía goal via action /joint_trajectory_controller/follow_joint_trajectory
           con tolerancias generosas y duración suficiente.
        4) Espera resultado y verifica convergencia leyendo /joint_states.
        Reintenta hasta 3 veces si no converge.
        """
        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        # STOWED_UP pose: brazo apuntando hacia arriba, fuera del workspace
        # de la mesa, garantiza que el TCP no choque con el tablero al
        # arrancar la simulación. El panel llevará al MESA target después.
        mesa = [
            0.0,                    # shoulder_pan
            -1.5707963267948966,    # shoulder_lift = -90° (apunta arriba)
            0.0,                    # elbow recto
            -1.5707963267948966,    # wrist_1 = -90° (alinea TCP vertical)
            0.0,                    # wrist_2
            0.0,                    # wrist_3
        ]
        # Lee /joint_states actual.
        latest: Dict[str, float] = {}

        def _on_js(msg: JointState) -> None:
            for n, p in zip(msg.name, msg.position):
                latest[n] = float(p)

        sub = self.create_subscription(
            JointState, "/joint_states", _on_js, qos_profile_sensor_data
        )
        try:
            t0 = time.time()
            while time.time() - t0 < 5.0:
                rclpy.spin_once(self, timeout_sec=0.05)
                if all(n in latest for n in joint_names):
                    break
            if not all(n in latest for n in joint_names):
                self.get_logger().warn(
                    f"[CTRL] initial_pose: /joint_states incompleto tras 5s; presentes={list(latest.keys())}"
                )
                return
            curr = [latest[n] for n in joint_names]
            self.get_logger().info(
                f"[CTRL] initial_pose curr={['%.3f' % v for v in curr]}"
            )
            # Comandar la pose canónica HOME_UP directamente (sin alias por
            # múltiplos de 2π). El controller interpolará el camino corto
            # dentro de los límites del joint sin atascarse en wraps.
            two_pi = 2.0 * math.pi
            def shortest(a: float, b: float) -> float:
                d = (b - a + math.pi) % two_pi - math.pi
                return d
            aliased = list(mesa)
            self.get_logger().info(
                f"[CTRL] initial_pose target_aliased={['%.3f' % v for v in aliased]}"
            )
            # Action client.
            ac = ActionClient(
                self,
                FollowJointTrajectory,
                "/joint_trajectory_controller/follow_joint_trajectory",
            )
            if not ac.wait_for_server(timeout_sec=10.0):
                self.get_logger().warn(
                    "[CTRL] initial_pose: FollowJointTrajectory action server no disponible"
                )
                return
            # Duración proporcional al delta máximo (>=4s).
            max_delta = max(abs(aliased[i] - curr[i]) for i in range(6))
            duration_sec = max(4.0, min(15.0, 1.5 * max_delta + 3.0))
            for attempt in range(1, 4):
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = list(joint_names)
                pt0 = JointTrajectoryPoint()
                pt0.positions = list(curr)
                pt0.velocities = [0.0] * 6
                pt0.time_from_start = Duration(sec=0, nanosec=0)
                ptf = JointTrajectoryPoint()
                ptf.positions = list(aliased)
                ptf.velocities = [0.0] * 6
                d_int = int(duration_sec)
                d_nano = int((duration_sec - d_int) * 1e9)
                ptf.time_from_start = Duration(sec=d_int, nanosec=d_nano)
                goal.trajectory.points = [pt0, ptf]
                goal.goal_time_tolerance = Duration(sec=10, nanosec=0)
                self.get_logger().info(
                    f"[CTRL] initial_pose attempt={attempt} duration={duration_sec:.1f}s max_delta={max_delta:.3f}rad"
                )
                send_future = ac.send_goal_async(goal)
                # Spin local hasta que el future complete o timeout.
                t_send = time.time()
                while not send_future.done() and time.time() - t_send < 5.0:
                    rclpy.spin_once(self, timeout_sec=0.05)
                if not send_future.done():
                    self.get_logger().warn("[CTRL] initial_pose: send_goal timeout")
                    continue
                gh = send_future.result()
                if gh is None or not gh.accepted:
                    self.get_logger().warn("[CTRL] initial_pose: goal rejected")
                    continue
                result_future = gh.get_result_async()
                t_res = time.time()
                while not result_future.done() and time.time() - t_res < duration_sec + 8.0:
                    rclpy.spin_once(self, timeout_sec=0.1)
                # Independientemente del resultado, verificamos convergencia.
                # Spin medio segundo extra para refrescar /joint_states.
                t_x = time.time()
                while time.time() - t_x < 0.5:
                    rclpy.spin_once(self, timeout_sec=0.05)
                final = [latest.get(n, float("nan")) for n in joint_names]
                # Convergencia: shortest-angle entre final y mesa < 0.08 rad.
                conv = all(abs(shortest(final[i], mesa[i])) < 0.08 for i in range(6))
                self.get_logger().info(
                    f"[CTRL] initial_pose attempt={attempt} final={['%.3f' % v for v in final]} converged={conv}"
                )
                if conv:
                    return
                # Recalcular aliased desde final para próximo intento.
                curr = list(final)
                aliased = [curr[i] + shortest(curr[i], mesa[i]) for i in range(6)]
                duration_sec = max(3.0, duration_sec * 0.8)
            self.get_logger().error(
                "[CTRL] initial_pose: no convergió a MESA tras 3 intentos"
            )
        except Exception as exc:
            self.get_logger().warn(f"[CTRL] initial_pose error: {exc}")
        finally:
            try:
                self.destroy_subscription(sub)
            except Exception:
                pass

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

    def _wait_state(self, name: str, targets, timeout: float = 3.0) -> str | None:
        """Poll list_controllers until controller `name` is in one of `targets`.

        Returns the observed state on success, or the last observed state on
        timeout. Spins the node so client futures complete during waiting.
        """
        deadline = time.monotonic() + max(0.5, timeout)
        last = None
        while time.monotonic() < deadline:
            state_map = self._list_controllers()
            last = state_map.get(name)
            if last in targets:
                return last
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        return last

    def _wait_joint_identity(self) -> bool:
        if not self._enforce_joint_identity or not self._expected_arm_joints:
            return True
        deadline = time.monotonic() + self._joint_identity_timeout
        while time.monotonic() < deadline:
            current = set(self._last_joint_names)
            if self._expected_arm_joints.issubset(current):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = sorted(self._expected_arm_joints.difference(self._last_joint_names))
        sample = sorted(list(self._last_joint_names))[:8]
        self.get_logger().error(
            "[CTRL] joint identity mismatch missing=%s sample=%s",
            ",".join(missing),
            ",".join(sample),
        )
        return False

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
        jtc_needs_initial_pose = False
        for name in self._required:
            state = state_map.get(name)
            if state == "active":
                self.get_logger().info(f"[CTRL] {name} ya activo; skip")
                if name == "joint_trajectory_controller":
                    jtc_needs_initial_pose = True
                continue
            if state is None:
                self.get_logger().info(f"[CTRL] {name} no cargado; cargando")
                self._load(name)
                state_after_load = self._wait_state(
                    name,
                    ("unconfigured", "inactive", "configured", "active"),
                    timeout=max(3.0, self._service_timeout),
                )
                if state_after_load is None:
                    self.get_logger().error(f"[CTRL] load failed: {name}")
                    ok_all = False
                    continue
                if state_after_load not in ("inactive", "configured", "active"):
                    self._configure(name)
                    state_after_cfg = self._wait_state(
                        name,
                        ("inactive", "configured", "active"),
                        timeout=max(3.0, self._service_timeout),
                    )
                    if state_after_cfg not in ("inactive", "configured", "active"):
                        self.get_logger().error(
                            f"[CTRL] configure failed: {name} state={state_after_cfg}"
                        )
                        ok_all = False
                        continue
            elif state in ("unconfigured", "inactive", "configured"):
                if state == "unconfigured":
                    self._configure(name)
                    state_after_cfg = self._wait_state(
                        name,
                        ("inactive", "configured", "active"),
                        timeout=max(3.0, self._service_timeout),
                    )
                    if state_after_cfg not in ("inactive", "configured", "active"):
                        self.get_logger().error(
                            f"[CTRL] configure failed: {name} state={state_after_cfg}"
                        )
                        ok_all = False
                        continue
            else:
                self.get_logger().warn(f"[CTRL] estado inesperado {name}: {state}")
            # Activate with retry: the controller_manager may take an update
            # cycle to commit state transitions; do not trust the synchronous
            # ok response alone, poll until state==active.
            activate_attempts = 0
            state_after_act = self._list_controllers().get(name)
            while state_after_act != "active" and activate_attempts < 5:
                activate_attempts += 1
                self._activate(name)
                state_after_act = self._wait_state(
                    name, ("active",), timeout=max(2.0, self._service_timeout)
                )
            if state_after_act != "active":
                self.get_logger().error(
                    f"[CTRL] activate failed: {name} state={state_after_act}"
                )
                ok_all = False
                continue
            self.get_logger().info(f"[CTRL] {name} activo")
            # Tras activar joint_trajectory_controller, comandar pose inicial
            # MESA para evitar que el robot quede atrapado por gravedad/spawn
            # en posturas no recuperables (wrists enrollados ~±2π).
            if name == "joint_trajectory_controller":
                self._command_initial_pose()
                jtc_needs_initial_pose = False
        # Si el JTC ya estaba activo (panel re-lanzado contra mismo gz),
        # también enviamos el initial_pose para garantizar HOME_UP de partida.
        if jtc_needs_initial_pose:
            self._command_initial_pose()
        # Final verification: re-list and confirm every required controller is
        # actually active. This guards against ListControllers returning stale
        # data during bootstrap.
        if ok_all:
            final_states = self._list_controllers()
            not_active = [
                f"{n}:{final_states.get(n)}"
                for n in self._required
                if final_states.get(n) != "active"
            ]
            if not_active:
                self.get_logger().error(
                    "[CTRL] verificacion final fallida; no activos: "
                    + ", ".join(not_active)
                )
                ok_all = False
        if ok_all and not self._wait_joint_identity():
            ok_all = False
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
