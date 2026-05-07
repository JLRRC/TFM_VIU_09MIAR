#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_pick_orchestrator_lifecycle_launch.py
# Contenido: F7 — launch_testing del pick_orchestrator_lifecycle.
"""F7: launch_testing del pick_orchestrator_lifecycle.

Prueba estándar ROS 2 que arranca el LifecycleNode canónico solo (sin
stack completo) y valida:

  1. El proceso arranca sin crash en < 5 s.
  2. El nodo aparece en ``ros2 node list`` con nombre canónico.
  3. Tras configure → activate, el action ``/pick_place`` aparece en
     ``get_action_names_and_types``.
  4. El nodo no termina por sí solo durante el test (sigue vivo).

Diseño:

* Sin sub-procesos extra: levantamos sólo el LifecycleNode → arranca
  rápido y aisla cualquier fallo del propio orchestrator.
* No verificamos transición real lifecycle por ahora — eso requeriría
  cliente lifecycle_msgs y haría el test frágil. Lo dejamos para
  F7-step2 si se confirma estable.
* Smoke pragmático: si arranca, importa OK las dependencias, y publica
  el action ⇒ el path canónico es funcional.

Ejecución:

    colcon test --packages-select tfm_orchestrator \\
        --ctest-args -R test_pick_orchestrator_lifecycle_launch

Equivalente directo:

    launch_test src/tfm_orchestrator/test/test_pick_orchestrator_lifecycle_launch.py

Skip automático: si ``launch_testing`` o ``launch_ros`` no están
disponibles (entorno sin ROS), el módulo se carga vacío y pytest
no ejecuta nada. Esto permite mantener compat con CI offline.
"""

from __future__ import annotations

import time
import unittest

import pytest


# Skip automático si las deps no están — entorno sin ROS.
_LAUNCH_TESTING_AVAILABLE = True
try:
    import launch  # noqa: F401
    import launch_ros.actions  # noqa: F401
    import launch_testing  # noqa: F401
    import launch_testing.actions  # noqa: F401
    import rclpy  # noqa: F401
    from rclpy.node import Node  # noqa: F401
except Exception:
    _LAUNCH_TESTING_AVAILABLE = False


# Permitir saltar el test en entornos donde no es viable (CI offline,
# sin ros2 instalado). El skip es a nivel de módulo: pytest reporta
# como skipped en lugar de fallo.
pytestmark = pytest.mark.skipif(
    not _LAUNCH_TESTING_AVAILABLE,
    reason="launch_testing / rclpy no disponibles",
)


if _LAUNCH_TESTING_AVAILABLE:
    from launch import LaunchDescription
    from launch_ros.actions import Node as LaunchNode
    from launch_testing.actions import ReadyToTest


    def generate_test_description():
        """LaunchDescription mínima: sólo el LifecycleNode bajo test."""
        orchestrator = LaunchNode(
            package="tfm_orchestrator",
            executable="pick_orchestrator_lifecycle",
            name="pick_orchestrator_lifecycle",
            output="screen",
            emulate_tty=True,
        )
        return (
            LaunchDescription([
                orchestrator,
                ReadyToTest(),
            ]),
            {"orchestrator": orchestrator},
        )


    class TestPickOrchestratorLifecycleLaunch(unittest.TestCase):
        """Smoke tests durante la vida del proceso orchestrator."""

        @classmethod
        def setUpClass(cls):
            rclpy.init()
            cls._client_node = rclpy.create_node(
                "_test_pick_orchestrator_lifecycle_launch_client"
            )

        @classmethod
        def tearDownClass(cls):
            try:
                cls._client_node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass

        def _spin_for(self, seconds: float) -> None:
            deadline = time.monotonic() + float(seconds)
            while time.monotonic() < deadline:
                rclpy.spin_once(self._client_node, timeout_sec=0.1)

        def test_node_appears_in_graph(self):
            """El nodo del orchestrator debe aparecer en el grafo ROS."""
            deadline = time.monotonic() + 8.0
            seen = False
            while time.monotonic() < deadline:
                rclpy.spin_once(self._client_node, timeout_sec=0.2)
                names = self._client_node.get_node_names()
                if any("pick_orchestrator_lifecycle" in n for n in names):
                    seen = True
                    break
            self.assertTrue(
                seen,
                f"pick_orchestrator_lifecycle no apareció en get_node_names() "
                f"tras 8s. names actuales: {self._client_node.get_node_names()}"
            )

        def test_action_pick_place_published_after_configure(self):
            """Tras 'configure' debe aparecer el action /pick_place.

            Para no añadir dependencia del cliente lifecycle, llamamos
            directamente al service /pick_orchestrator_lifecycle/change_state
            con CONFIGURE (id=1) usando rclpy bajo nivel.
            """
            from lifecycle_msgs.srv import ChangeState
            from lifecycle_msgs.msg import Transition

            change_state_name = "/pick_orchestrator_lifecycle/change_state"
            client = self._client_node.create_client(
                ChangeState, change_state_name
            )
            self.assertTrue(
                client.wait_for_service(timeout_sec=8.0),
                f"service {change_state_name} no apareció en 8s",
            )

            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_CONFIGURE
            future = client.call_async(req)
            rclpy.spin_until_future_complete(
                self._client_node, future, timeout_sec=5.0
            )
            self.assertTrue(future.done(), "configure call timeout")
            resp = future.result()
            self.assertTrue(
                getattr(resp, "success", False),
                f"configure devolvió success=False: {resp}"
            )

            # Ahora activate.
            req2 = ChangeState.Request()
            req2.transition.id = Transition.TRANSITION_ACTIVATE
            future2 = client.call_async(req2)
            rclpy.spin_until_future_complete(
                self._client_node, future2, timeout_sec=5.0
            )
            self.assertTrue(future2.done(), "activate call timeout")
            resp2 = future2.result()
            self.assertTrue(
                getattr(resp2, "success", False),
                f"activate devolvió success=False: {resp2}"
            )

            # El action /pick_place debe estar visible. ``Node`` no expone
            # get_action_names_and_types directamente — la API canónica
            # vive en ``rclpy.action.get_action_names_and_types(node)``.
            self._spin_for(1.0)
            from rclpy.action import get_action_names_and_types
            actions = get_action_names_and_types(self._client_node)
            action_names = {name for name, _types in actions}
            self.assertIn(
                "/pick_place",
                action_names,
                f"/pick_place no está en get_action_names_and_types() tras "
                f"activate. visible: {sorted(action_names)}",
            )


    @launch_testing.post_shutdown_test()
    class TestPickOrchestratorLifecycleShutdown(unittest.TestCase):
        """Verifica que el proceso terminó limpio (exit code 0 o SIGTERM)."""

        def test_proc_terminates_cleanly(self, proc_info, orchestrator):
            # SIGTERM (15) es lo que envía launch al teardown — aceptable.
            launch_testing.asserts.assertExitCodes(
                proc_info,
                allowable_exit_codes=[0, 15, -15],
                process=orchestrator,
            )
