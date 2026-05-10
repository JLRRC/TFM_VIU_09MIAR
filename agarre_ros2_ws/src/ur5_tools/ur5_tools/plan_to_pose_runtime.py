#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/plan_to_pose_runtime.py
# Contenido: F3.2 audit (2026-05-10) — runtime wiring de plan_to_pose_server.
"""F3.2 audit (2026-05-10): runtime wiring de plan_to_pose_server.

Punto de entrada ROS extraído del módulo principal para reducir LOC y
permitir tests del nodo sin ejecutar `main()`. Las funciones aquí son
puramente "plumbing" (rclpy.init, MultiThreadedExecutor, shutdown
defensivo) — la lógica del action server vive en
``plan_to_pose_server.PlanToPoseServer``.

Plan de evolución (NOTE F9 — refactor pendiente):
  La clase ``PlanToPoseServer`` aún concentra 3 modos (STUB,
  REAL_BRIDGE, MOVEIT_DIRECT) en métodos ``_execute_*`` con cascada
  ``if self._mode == ...``. La fase F9 del plan de auditoría propone
  migrar a Strategy pattern explícito::

      class PlanToPoseStrategy(Protocol):
          def execute(self, goal: PlanToPoseGoal, start_mono: float)
              -> PlanToPoseResult: ...

  Implementaciones por modo (StubStrategy, RealBridgeStrategy,
  MoveitDirectStrategy) ya tienen helpers extraídos en
  plan_to_pose_logic / plan_to_pose_real_bridge / plan_to_pose_moveit_direct.
"""
from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor


def run_plan_to_pose_server(args: Optional[list] = None) -> None:
    """Lanza ``PlanToPoseServer`` con MultiThreadedExecutor + cleanup defensivo."""
    # Import lazy para evitar dependencia circular en tests del módulo.
    from .plan_to_pose_server import PlanToPoseServer

    rclpy.init(args=args)
    node = PlanToPoseServer()
    executor = MultiThreadedExecutor(num_threads=2)  # iter4-bis: limitar threads (antes default = cpu_count = 8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


# Compat re-export (entry_point apunta a plan_to_pose_server:main).
main = run_plan_to_pose_server
