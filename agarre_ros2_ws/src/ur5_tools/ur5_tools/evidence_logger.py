#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/evidence_logger.py
# Contenido: Nodo evidence_logger que graba JSON Lines + CSV por ciclo de pick (F7).
"""Evidence logger node: registro JSONL/CSV por sesion para defensa academica.

Suscribe a topics clave del pipeline pick & place y escribe eventos
estructurados en ``report/runs/<timestamp>/`` con dos formatos:

* ``events.jsonl`` — JSON Lines, una linea por evento, con campos
  ``ts_iso``, ``ts_mono``, ``ts_sim_ns``, ``topic``, ``kind``, ``data``.
* ``summary.csv`` — agregado de hitos por sesion: arranque, cada
  resultado de ``/desired_grasp/result``, cada attach/detach.

Topics suscritos:
- ``/desired_grasp/result`` (std_msgs/String) — resultado del bridge
  MoveIt: success/fail + reason por cada peticion.
- ``/system_state`` (std_msgs/String) — estado global del system_state_manager.
- ``/system_diag`` (std_msgs/String) — diagnosticos del system_state_manager.
- ``/gripper/<obj>/state`` (std_msgs/Bool) — attach/detach por objeto.

El directorio raiz por defecto es ``${WS_DIR}/report/runs/<timestamp>/``,
configurable con el parametro ROS ``output_root``.

Uso:
    ros2 run ur5_tools evidence_logger
    ros2 run ur5_tools evidence_logger --ros-args -p output_root:=/tmp/runs
"""

from __future__ import annotations

import csv
import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String


_DEFAULT_GRIPPER_OBJECTS = (
    "pick_demo",
    "box_blue",
    "box_green",
    "box_lightblue",
    "box_red",
    "box_yellow",
    "cross_cyan",
    "cyl_gray",
    "cyl_green",
    "cyl_orange",
    "cyl_purple",
)


# Helpers puros (re-exportados desde evidence_helpers para que sean
# testables sin rclpy).
from .evidence_helpers import now_iso as _now_iso, safe_unique_dir as _safe_unique_dir


class EvidenceLogger(Node):
    """ROS 2 node that records pick & place events to disk for academic review."""

    def __init__(self) -> None:
        super().__init__("evidence_logger")
        self.declare_parameter("output_root", "")
        self.declare_parameter("gripper_prefix", "/gripper")
        self.declare_parameter("gripper_objects", list(_DEFAULT_GRIPPER_OBJECTS))
        self.declare_parameter("desired_grasp_result_topic", "/desired_grasp/result")
        self.declare_parameter("system_state_topic", "/system_state")
        self.declare_parameter("system_diag_topic", "/system_diag")

        output_root_param = str(
            self.get_parameter("output_root").value or ""
        ).strip()
        if output_root_param:
            root = Path(output_root_param).expanduser().resolve()
        else:
            ws_dir = os.environ.get("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws"))
            root = Path(ws_dir) / "report" / "runs"
        root.mkdir(parents=True, exist_ok=True)
        self._session_dir = _safe_unique_dir(root)
        self._events_path = self._session_dir / "events.jsonl"
        self._summary_path = self._session_dir / "summary.csv"

        self._events_fp = self._events_path.open("a", encoding="utf-8")
        self._summary_fp = self._summary_path.open("a", encoding="utf-8", newline="")
        self._summary_writer = csv.writer(self._summary_fp)
        self._summary_writer.writerow(
            ["ts_iso", "kind", "object", "success", "reason"]
        )

        self._reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._subs: List[Any] = []
        self._setup_subscriptions()

        self._record(
            kind="session_started",
            data={
                "session_dir": str(self._session_dir),
                "events_path": str(self._events_path),
                "summary_path": str(self._summary_path),
            },
        )
        self.get_logger().info(
            f"[EVIDENCE] session={self._session_dir} events={self._events_path.name} "
            f"summary={self._summary_path.name}"
        )

    def _setup_subscriptions(self) -> None:
        result_topic = str(
            self.get_parameter("desired_grasp_result_topic").value
            or "/desired_grasp/result"
        ).strip()
        self._subs.append(
            self.create_subscription(
                String,
                result_topic,
                self._on_grasp_result,
                self._reliable_qos,
            )
        )

        state_topic = str(
            self.get_parameter("system_state_topic").value or "/system_state"
        ).strip()
        self._subs.append(
            self.create_subscription(
                String,
                state_topic,
                self._on_system_state,
                self._reliable_qos,
            )
        )

        diag_topic = str(
            self.get_parameter("system_diag_topic").value or "/system_diag"
        ).strip()
        self._subs.append(
            self.create_subscription(
                String,
                diag_topic,
                self._on_system_diag,
                self._best_effort_qos,
            )
        )

        prefix = str(
            self.get_parameter("gripper_prefix").value or "/gripper"
        ).strip().rstrip("/")
        objects_raw = self.get_parameter("gripper_objects").value or []
        objects = [str(o).strip() for o in objects_raw if str(o).strip()]
        for obj in objects:
            topic = f"{prefix}/{obj}/state"
            self._subs.append(
                self.create_subscription(
                    Bool,
                    topic,
                    lambda msg, name=obj: self._on_gripper_state(msg, name),
                    self._reliable_qos,
                )
            )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_grasp_result(self, msg: String) -> None:
        text = str(msg.data or "").strip()
        success, reason = self._parse_grasp_result(text)
        self._record(
            kind="grasp_result",
            data={"raw": text, "success": success, "reason": reason},
        )
        self._summary_writer.writerow(
            [
                _now_iso(),
                "grasp_result",
                "",
                str(bool(success)).lower(),
                reason,
            ]
        )
        self._summary_fp.flush()

    def _on_system_state(self, msg: String) -> None:
        self._record(kind="system_state", data={"raw": str(msg.data or "")})

    def _on_system_diag(self, msg: String) -> None:
        self._record(kind="system_diag", data={"raw": str(msg.data or "")})

    def _on_gripper_state(self, msg: Bool, name: str) -> None:
        attached = bool(getattr(msg, "data", False))
        self._record(
            kind="gripper_state",
            data={"object": name, "attached": attached},
        )
        self._summary_writer.writerow(
            [
                _now_iso(),
                "gripper_state",
                name,
                str(attached).lower(),
                "",
            ]
        )
        self._summary_fp.flush()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_grasp_result(text: str) -> tuple[Optional[bool], str]:
        """Extract (success, reason) from the bridge result payload.

        Bridge emits patterns like ``success=true reason=exec_ok ...`` or
        ``success=false reason=plan_failed:... request_uuid=...``. Falls back
        to ``(None, text)`` if parsing fails.
        """

        if not text:
            return None, ""
        success: Optional[bool] = None
        reason = text
        try:
            tokens = text.split()
            kv: Dict[str, str] = {}
            for tok in tokens:
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    kv[k] = v
            if "success" in kv:
                success_raw = kv["success"].strip().lower()
                if success_raw in ("true", "1", "yes"):
                    success = True
                elif success_raw in ("false", "0", "no"):
                    success = False
            if "reason" in kv:
                reason = kv["reason"]
        except Exception:
            pass
        return success, reason

    def _record(self, *, kind: str, data: Dict[str, Any]) -> None:
        try:
            sim_ns = int(self.get_clock().now().nanoseconds or 0)
        except Exception:
            sim_ns = 0
        try:
            mono = float(__import__("time").monotonic())
        except Exception:
            mono = 0.0
        entry = {
            "ts_iso": _now_iso(),
            "ts_mono": mono,
            "ts_sim_ns": sim_ns,
            "kind": kind,
            "data": data,
        }
        try:
            self._events_fp.write(json.dumps(entry, ensure_ascii=True) + "\n")
            self._events_fp.flush()
        except Exception as exc:
            self.get_logger().warning(
                f"[EVIDENCE] write_failed kind={kind} err={type(exc).__name__}:{exc}"
            )

    def shutdown(self) -> None:
        try:
            self._record(kind="session_finished", data={})
        except Exception:
            pass
        for fp in (self._events_fp, self._summary_fp):
            try:
                fp.close()
            except Exception:
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvidenceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
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
