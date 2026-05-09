#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_object_pose_resolver_launch.py
# Contenido: F7-step2 — launch_testing del object_pose_resolver_service.
"""F7-step2: launch_testing del object_pose_resolver_service (F5-step4).

Levanta el LifecycleNode con auto_activate=True (default), publica
manualmente un TFMessage emulando gz_pose_bridge, y verifica que el
service ResolveObjectPoseWorld responde con la pose esperada.

Esto cierra el loop end-to-end de F5-step3+4+5 sin depender de Gazebo:
  * Sub TFMessage funciona (recibe mensajes externos).
  * Cache se actualiza al recibir transforms.
  * Service responde con pose o success=False según freshness.

Skip auto si launch_testing/rclpy no disponibles.
"""

from __future__ import annotations

import time
import unittest

import pytest


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


pytestmark = pytest.mark.skipif(
    not _LAUNCH_TESTING_AVAILABLE,
    reason="launch_testing / rclpy no disponibles",
)


if _LAUNCH_TESTING_AVAILABLE:
    from launch import LaunchDescription
    from launch_ros.actions import Node as LaunchNode
    from launch_testing.actions import ReadyToTest


    def generate_test_description():
        resolver = LaunchNode(
            package="ur5_tools",
            executable="object_pose_resolver_service",
            name="object_pose_resolver_service",
            parameters=[
                # World name personalizado para que el sub topic sea
                # determinístico y no choque con un Gazebo real.
                {"world_name": "test_world"},
                {"world_frame": "world"},
                {"max_pose_age_sec": 5.0},
                {"auto_activate": True},
            ],
            output="screen",
            emulate_tty=True,
        )
        return (
            LaunchDescription([
                resolver,
                ReadyToTest(),
            ]),
            {"resolver": resolver},
        )


    class TestObjectPoseResolverLaunch(unittest.TestCase):
        """Smoke + service contract end-to-end."""

        @classmethod
        def setUpClass(cls):
            rclpy.init()
            cls._client_node = rclpy.create_node(
                "_test_object_pose_resolver_client"
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

        def _wait_for_service(self, service_name: str, srv_type, timeout_sec: float = 8.0):
            client = self._client_node.create_client(srv_type, service_name)
            self.assertTrue(
                client.wait_for_service(timeout_sec=timeout_sec),
                f"service {service_name} no apareció en {timeout_sec}s",
            )
            return client

        def test_service_appears_after_activate(self):
            """Tras auto_activate, /orchestrator/resolve_object_pose_world existe."""
            from ur5_panel_interfaces.srv import ResolveObjectPoseWorld
            self._wait_for_service(
                "/orchestrator/resolve_object_pose_world",
                ResolveObjectPoseWorld,
                timeout_sec=8.0,
            )

        def test_service_returns_failure_for_unknown_object(self):
            """Sin transforms publicados todavía, lookup devuelve success=False."""
            from ur5_panel_interfaces.srv import ResolveObjectPoseWorld
            client = self._wait_for_service(
                "/orchestrator/resolve_object_pose_world",
                ResolveObjectPoseWorld,
            )
            req = ResolveObjectPoseWorld.Request()
            req.object_name = "ghost_object"
            future = client.call_async(req)
            rclpy.spin_until_future_complete(
                self._client_node, future, timeout_sec=5.0
            )
            self.assertTrue(future.done())
            resp = future.result()
            self.assertFalse(resp.success)
            self.assertIn("object_not_in_cache", resp.detail)

        def test_service_returns_pose_after_publishing_tfmessage(self):
            """E2E: publicar TFMessage → cache se actualiza → service devuelve pose."""
            from geometry_msgs.msg import TransformStamped
            from tf2_msgs.msg import TFMessage
            from ur5_panel_interfaces.srv import ResolveObjectPoseWorld

            # Esperar al sub del resolver (publica el server con el sub
            # creado en on_activate; tarda ~1s tras el ready).
            pub = self._client_node.create_publisher(
                TFMessage,
                "/world/test_world/pose/info",
                10,
            )
            # Esperar a que el resolver tenga el sub.
            deadline = time.monotonic() + 5.0
            n_subs = 0
            while time.monotonic() < deadline:
                rclpy.spin_once(self._client_node, timeout_sec=0.1)
                n_subs = self._client_node.count_subscribers(
                    "/world/test_world/pose/info"
                )
                if n_subs > 0:
                    break
            self.assertGreater(
                n_subs, 0,
                "Resolver no se suscribió al topic /world/test_world/pose/info"
            )

            # Construir TFMessage con un objeto.
            tf_stamped = TransformStamped()
            tf_stamped.header.frame_id = "world"
            tf_stamped.header.stamp = self._client_node.get_clock().now().to_msg()
            tf_stamped.child_frame_id = "box_red"
            tf_stamped.transform.translation.x = 0.5
            tf_stamped.transform.translation.y = -0.1
            tf_stamped.transform.translation.z = 0.05
            tf_stamped.transform.rotation.x = 0.0
            tf_stamped.transform.rotation.y = 0.0
            tf_stamped.transform.rotation.z = 0.0
            tf_stamped.transform.rotation.w = 1.0
            msg = TFMessage(transforms=[tf_stamped])
            # Publicar varias veces y dar tiempo a que el callback corra.
            for _ in range(5):
                pub.publish(msg)
                self._spin_for(0.1)

            # Llamar al service.
            client = self._client_node.create_client(
                ResolveObjectPoseWorld,
                "/orchestrator/resolve_object_pose_world",
            )
            req = ResolveObjectPoseWorld.Request()
            req.object_name = "box_red"
            future = client.call_async(req)
            rclpy.spin_until_future_complete(
                self._client_node, future, timeout_sec=5.0
            )
            self.assertTrue(future.done(), "service call timeout")
            resp = future.result()
            self.assertTrue(
                resp.success,
                f"service devolvió success=False: detail={resp.detail}"
            )
            self.assertAlmostEqual(resp.pose_world.position.x, 0.5, places=4)
            self.assertAlmostEqual(resp.pose_world.position.y, -0.1, places=4)
            self.assertAlmostEqual(resp.pose_world.position.z, 0.05, places=4)
            self.assertAlmostEqual(resp.pose_world.orientation.w, 1.0, places=4)


    @launch_testing.post_shutdown_test()
    class TestObjectPoseResolverShutdown(unittest.TestCase):
        def test_proc_terminates_cleanly(self, proc_info, resolver):
            launch_testing.asserts.assertExitCodes(
                proc_info,
                allowable_exit_codes=[0, 15, -15],
                process=resolver,
            )
