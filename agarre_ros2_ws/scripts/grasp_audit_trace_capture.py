#!/usr/bin/env python3
"""Live, read-only trace capture for DIRECTO/DIRECTO2 grasp audits.

Captures the signals needed by the comparative benchmark without touching
production behavior:
- /world/<world>/pose/info
- /tf and /tf_static via tf2 buffer
- /joint_states
- /gripper_controller/commands
- /gripper/<object>/state
- /joint_trajectory_controller/controller_state
- /rosout backend events

The output is a JSONL trace designed for post-processing by
`grasp_audit_benchmark.py`.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from control_msgs.msg import JointTrajectoryControllerState
from rcl_interfaces.msg import Log
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


BACKEND_PATTERNS = (
    "attach_route_decision",
    "gazebo_attach_applied=true",
    "demo_transport_restore",
    "demo_transport_enabled",
    "attach_request_received",
    "detach_request_received",
    "carry_validation_ok",
    "demo_carry_validation_failed",
    "CARRY_NOT_ACQUIRED",
    "basket_release_done",
)


@dataclass
class PoseSample:
    frame_id: str
    position: Dict[str, float]
    orientation: Dict[str, float | list[float]]
    stamp: float


def _stamp_to_sec(stamp: Optional[TimeMsg]) -> float:
    if stamp is None:
        return 0.0
    return float(stamp.sec) + (float(stamp.nanosec) / 1_000_000_000.0)


def _quat_to_rpy(x: float, y: float, z: float, w: float) -> list[float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


def _pose_from_tf(parent: str, child: str, tf) -> dict:
    tr = tf.transform.translation
    rot = tf.transform.rotation
    return {
        "parent": parent,
        "child": child,
        "stamp": _stamp_to_sec(tf.header.stamp),
        "translation": {
            "x": float(tr.x),
            "y": float(tr.y),
            "z": float(tr.z),
        },
        "rotation": {
            "x": float(rot.x),
            "y": float(rot.y),
            "z": float(rot.z),
            "w": float(rot.w),
            "rpy": _quat_to_rpy(float(rot.x), float(rot.y), float(rot.z), float(rot.w)),
        },
    }


class AuditCapture(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("grasp_audit_trace_capture")
        self._args = args
        self._out_path = Path(args.output).expanduser().resolve()
        self._out_path.parent.mkdir(parents=True, exist_ok=True)
        self._out = self._out_path.open("w", encoding="utf-8")

        self._world_name = str(args.world_name)
        self._world_frame = str(args.world_frame)
        self._base_frame = str(args.base_frame)
        self._object_name = str(args.object_name)
        self._pose_topic = str(args.pose_topic or f"/world/{self._world_name}/pose/info")
        self._attach_topic = str(args.attach_topic or f"/gripper/{self._object_name}/state")
        self._frames = list(args.frames)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._pose_cache: Dict[str, PoseSample] = {}
        self._joint_states: Dict[str, float] = {}
        self._gripper_cmd: list[float] = []
        self._attach_state: Optional[bool] = None
        self._controller_state: dict = {}
        self._backend_event_count = 0

        self.create_subscription(TFMessage, self._pose_topic, self._on_pose_info, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(Float64MultiArray, "/gripper_controller/commands", self._on_gripper_cmd, 10)
        self.create_subscription(Bool, self._attach_topic, self._on_attach_state, 10)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/joint_trajectory_controller/controller_state",
            self._on_controller_state,
            10,
        )
        self.create_subscription(Log, "/rosout", self._on_rosout, 200)
        self._timer = self.create_timer(max(0.02, 1.0 / float(args.sample_hz)), self._on_sample)

        self._write_event(
            {
                "event": "observer_start",
                "type": "event",
                "label": args.label,
                "sim_time": self._now_sec(),
                "wall_time": self._wall_now(),
                "pose_topic": self._pose_topic,
                "attach_topic": self._attach_topic,
                "frames": self._frames,
            }
        )

    def destroy_node(self) -> bool:
        try:
            self._write_event(
                {
                    "event": "observer_stop",
                    "type": "event",
                    "label": self._args.label,
                    "sim_time": self._now_sec(),
                    "wall_time": self._wall_now(),
                    "backend_event_count": self._backend_event_count,
                }
            )
        finally:
            try:
                self._out.close()
            except Exception:
                pass
        return super().destroy_node()

    def _wall_now(self) -> float:
        import time

        return time.time()

    def _now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) / 1_000_000_000.0

    def _write_event(self, payload: dict) -> None:
        self._out.write(json.dumps(payload, ensure_ascii=True) + "\n")
        self._out.flush()

    def _on_pose_info(self, msg: TFMessage) -> None:
        for tf in getattr(msg, "transforms", []) or []:
            child = str(getattr(tf, "child_frame_id", "") or "").strip()
            if not child:
                continue
            header = getattr(tf, "header", None)
            frame_id = str(getattr(header, "frame_id", "") or "").strip() or self._world_frame
            tr = tf.transform.translation
            rot = tf.transform.rotation
            pose = PoseSample(
                frame_id=frame_id,
                position={
                    "x": float(tr.x),
                    "y": float(tr.y),
                    "z": float(tr.z),
                },
                orientation={
                    "x": float(rot.x),
                    "y": float(rot.y),
                    "z": float(rot.z),
                    "w": float(rot.w),
                    "rpy": _quat_to_rpy(float(rot.x), float(rot.y), float(rot.z), float(rot.w)),
                },
                stamp=_stamp_to_sec(getattr(header, "stamp", None)),
            )
            self._pose_cache[child] = pose
            if "::" in child:
                model = child.split("::", 1)[0].strip()
                if model:
                    self._pose_cache[model] = pose

    def _on_joint_states(self, msg: JointState) -> None:
        names = list(getattr(msg, "name", []) or [])
        pos = list(getattr(msg, "position", []) or [])
        self._joint_states = {
            str(name): float(value)
            for name, value in zip(names, pos)
        }

    def _on_gripper_cmd(self, msg: Float64MultiArray) -> None:
        self._gripper_cmd = [float(v) for v in list(getattr(msg, "data", []) or [])]

    def _on_attach_state(self, msg: Bool) -> None:
        new_state = bool(getattr(msg, "data", False))
        if self._attach_state is None or self._attach_state != new_state:
            self._attach_state = new_state
            self._write_event(
                {
                    "event": "attach_state_transition",
                    "type": "event",
                    "label": self._args.label,
                    "sim_time": self._now_sec(),
                    "wall_time": self._wall_now(),
                    "object_name": self._object_name,
                    "attach_state": new_state,
                }
            )
        else:
            self._attach_state = new_state

    def _on_controller_state(self, msg: JointTrajectoryControllerState) -> None:
        def _float_list(values: Iterable[float]) -> list[float]:
            return [float(v) for v in list(values or [])]

        self._controller_state = {
            "joint_names": [str(v) for v in list(getattr(msg, "joint_names", []) or [])],
            "reference": _float_list(getattr(msg.reference, "positions", []) if getattr(msg, "reference", None) else []),
            "feedback": _float_list(getattr(msg.feedback, "positions", []) if getattr(msg, "feedback", None) else []),
            "error": _float_list(getattr(msg.error, "positions", []) if getattr(msg, "error", None) else []),
        }

    def _on_rosout(self, msg: Log) -> None:
        name = str(getattr(msg, "name", "") or "")
        text = str(getattr(msg, "msg", "") or "")
        if "gripper_attach_backend" not in name and not any(p in text for p in BACKEND_PATTERNS):
            return
        if not any(p in text for p in BACKEND_PATTERNS):
            return
        self._backend_event_count += 1
        self._write_event(
            {
                "event": "backend_log",
                "type": "event",
                "label": self._args.label,
                "sim_time": self._now_sec(),
                "wall_time": self._wall_now(),
                "logger": name,
                "level": int(getattr(msg, "level", 0) or 0),
                "message": text,
            }
        )

    def _lookup_tf(self, parent: str, child: str) -> Optional[dict]:
        try:
            tf = self._tf_buffer.lookup_transform(
                parent,
                child,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.03),
            )
        except Exception:
            return None
        return _pose_from_tf(parent, child, tf)

    def _on_sample(self) -> None:
        tf_payload: Dict[str, Optional[dict]] = {}
        for child in self._frames:
            tf_payload[f"{self._world_frame}->{child}"] = self._lookup_tf(self._world_frame, child)
            if child != self._base_frame:
                tf_payload[f"{self._base_frame}->{child}"] = self._lookup_tf(self._base_frame, child)

        payload = {
            "type": "sample",
            "label": self._args.label,
            "sim_time": self._now_sec(),
            "wall_time": self._wall_now(),
            "object_name": self._object_name,
            "object_pose": asdict(self._pose_cache[self._object_name]) if self._object_name in self._pose_cache else {},
            "joint_states": dict(self._joint_states),
            "gripper_cmd": list(self._gripper_cmd),
            "attach_state": self._attach_state,
            "controller_state": dict(self._controller_state),
            "tf": tf_payload,
        }
        self._write_event(payload)


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="Output JSONL path")
    parser.add_argument("--label", default="GRASP_AUDIT", help="Run label")
    parser.add_argument("--object-name", default="pick_demo")
    parser.add_argument("--world-name", default="ur5_mesa_objetos")
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--pose-topic", default="")
    parser.add_argument("--attach-topic", default="")
    parser.add_argument("--sample-hz", type=float, default=15.0)
    parser.add_argument(
        "--frames",
        nargs="+",
        default=[
            "base_link",
            "tool0",
            "rg2_tcp",
            "rg2_pinch_center",
            "rg2_finger_link1",
            "rg2_finger_link2",
        ],
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(list(argv or sys.argv[1:]))
    rclpy.init(args=None)
    node = AuditCapture(args)

    stop = {"asked": False}

    def _shutdown(*_args) -> None:
        stop["asked"] = True

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        while rclpy.ok() and not stop["asked"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
