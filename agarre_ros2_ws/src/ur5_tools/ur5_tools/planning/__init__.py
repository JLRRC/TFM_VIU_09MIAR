"""F11 (auditoría 2026-05-10): namespace de planning del stack UR5.

Subpaquete que re-exporta los módulos del dominio "planning + trajectory
execution":
  * ``plan_to_pose_logic`` — dataclasses + pure helpers del action server.
  * ``plan_to_pose_helpers`` — helpers puros (extracción seed positions,
    selección duration/timeout, parsing request).
  * ``plan_to_pose_moveit_direct`` — builder de goals MoveIt direct.
  * ``plan_to_pose_real_bridge`` — interface al bridge real.
  * ``fjt_direct_helpers`` — builders FJT directos.
  * ``planning_scene_sync_helpers`` — helpers puros de planning scene.
  * ``trajectory_executor_contract`` — typed contract.

Los archivos físicos siguen en top-level; F11 iter 2 los moverá.
"""
from __future__ import annotations

from ..plan_to_pose_helpers import (  # noqa: F401
    extract_ordered_joint_positions,
    parse_plan_to_pose_request,
    select_traj_duration_and_timeout,
)
from ..plan_to_pose_logic import (  # noqa: F401
    PlanToPoseGoal,
    PlanToPoseResult,
)

__all__ = [
    "PlanToPoseGoal",
    "PlanToPoseResult",
    "extract_ordered_joint_positions",
    "parse_plan_to_pose_request",
    "select_traj_duration_and_timeout",
]
