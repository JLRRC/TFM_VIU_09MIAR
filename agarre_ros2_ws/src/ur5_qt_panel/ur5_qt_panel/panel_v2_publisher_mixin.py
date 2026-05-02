#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_publisher_mixin.py
# Contenido: F14-step2 (2026-05-01) — mixin de publishers MoveIt extraído de ControlPanelV2.
"""Mixin de publishers/auto-bridge de ``ControlPanelV2`` (F14-step2).

F14-step2 (2026-05-01) extrae el grupo cohesivo de métodos
relacionados con la creación y uso de publishers MoveIt
(``_init_moveit_publisher``, ``_ensure_moveit_node``,
``_publish_current_grasp_rect``) y los wrappers thin que delegan al
módulo ``panel_state_methods`` (``_get_traj_publisher``,
``_get_gripper_publisher``, ``_get_attach_publisher``,
``_publish_joint_trajectory``, ``_publish_moveit_pose``,
``_moveit_publish_context``, ``_request_auto_bridge_start``,
``_auto_bridge_tick``).

``ControlPanelV2`` lo incluye como mixin antes de ``QMainWindow``
para que los métodos sigan accesibles tal cual desde el panel y
desde el resto del código (panel_pick_demo, panel_ros, etc).

Este es el primer mixin extraído de ``panel_v2``. F14-step3/4
extraerán otros grupos cohesivos (motion helpers, base pose helpers,
attach helpers).
"""

from __future__ import annotations

import math

from std_msgs.msg import Float32MultiArray

from . import panel_state_methods as _stm
from .panel_moveit_publishers import init_moveit_publishers


# Constantes canónicas (mismas que en panel_v2). Replicadas aquí para
# que el mixin sea autosuficiente y no introduzca un import circular.
_MOVEIT_POSE_TOPIC = "/desired_grasp"
_MOVEIT_CARTESIAN_POSE_TOPIC = "/desired_grasp_cartesian"


class PanelV2PublisherMixin:
    """Métodos relacionados con publishers MoveIt y auto-bridge.

    Asume atributos en ``self`` (provistos por ControlPanelV2.__init__):

    * ``_moveit_node``, ``_moveit_pose_pub``, ``_moveit_pose_pub_cartesian``
    * ``_grasp_rect_pub``, ``_grasp_rect_topic``, ``_last_grasp_px``
    * ``_log``, ``_emit_log``, ``_audit_append``

    Usa la constante runtime ``USE_SIM_TIME`` propagada en self por el
    panel.
    """

    def _init_moveit_publisher(self) -> None:
        self._moveit_node, self._moveit_pose_pub, self._moveit_pose_pub_cartesian = (
            init_moveit_publishers(
                node=self._moveit_node,
                pose_pub=self._moveit_pose_pub,
                pose_pub_cartesian=self._moveit_pose_pub_cartesian,
                use_sim_time=bool(getattr(self, "_use_sim_time_flag", True)),
                pose_topic=_MOVEIT_POSE_TOPIC,
                cartesian_topic=_MOVEIT_CARTESIAN_POSE_TOPIC,
                logger=self._log,
            )
        )
        if self._moveit_node is not None and self._grasp_rect_pub is None:
            try:
                self._grasp_rect_pub = self._moveit_node.create_publisher(
                    Float32MultiArray,
                    self._grasp_rect_topic,
                    10,
                )
            except Exception as exc:
                self._emit_log(
                    f"[TFM] WARN: no se pudo crear publisher "
                    f"{self._grasp_rect_topic} ({exc})"
                )
                self._grasp_rect_pub = None

    def _ensure_moveit_node(self) -> None:
        if self._moveit_node is None:
            self._init_moveit_publisher()

    def _publish_current_grasp_rect(self) -> bool:
        if not self._last_grasp_px:
            return False
        self._ensure_moveit_node()
        if self._grasp_rect_pub is None:
            return False
        try:
            msg = Float32MultiArray()
            msg.data = [
                float(self._last_grasp_px.get("cx", 0.0)),
                float(self._last_grasp_px.get("cy", 0.0)),
                float(self._last_grasp_px.get("w", 0.0)),
                float(self._last_grasp_px.get("h", 0.0)),
                math.radians(float(self._last_grasp_px.get("angle_deg", 0.0))),
            ]
            self._grasp_rect_pub.publish(msg)
            self._audit_append(
                "logs/perception.log",
                "[TFM] grasp_rect tx "
                f"topic={self._grasp_rect_topic} "
                "type=std_msgs/msg/Float32MultiArray "
                f"cx={msg.data[0]:.1f} cy={msg.data[1]:.1f} "
                f"w={msg.data[2]:.1f} h={msg.data[3]:.1f} "
                f"theta_rad={msg.data[4]:.4f}",
            )
            return True
        except Exception as exc:
            self._emit_log(
                f"[TFM] WARN: fallo publicando {self._grasp_rect_topic} ({exc})"
            )
            self._audit_append(
                "logs/perception.log",
                f"[TFM] grasp_rect tx_fail topic={self._grasp_rect_topic} err={exc}",
            )
            return False

    # ------------------------------------------------------------------
    # Wrappers thin delegando a panel_state_methods (_stm)
    # ------------------------------------------------------------------

    def _moveit_publish_context(self, *args, **kwargs):
        return _stm._moveit_publish_context(self, *args, **kwargs)

    def _request_auto_bridge_start(self, *args, **kwargs):
        return _stm._request_auto_bridge_start(self, *args, **kwargs)

    def _auto_bridge_tick(self, *args, **kwargs):
        return _stm._auto_bridge_tick(self, *args, **kwargs)

    def _get_traj_publisher(self, *args, **kwargs):
        return _stm._get_traj_publisher(self, *args, **kwargs)

    def _get_gripper_publisher(self, *args, **kwargs):
        return _stm._get_gripper_publisher(self, *args, **kwargs)

    def _get_attach_publisher(self, *args, **kwargs):
        return _stm._get_attach_publisher(self, *args, **kwargs)

    def _publish_joint_trajectory(self, *args, **kwargs):
        return _stm._publish_joint_trajectory(self, *args, **kwargs)

    def _publish_moveit_pose(self, *args, **kwargs):
        return _stm._publish_moveit_pose(self, *args, **kwargs)
