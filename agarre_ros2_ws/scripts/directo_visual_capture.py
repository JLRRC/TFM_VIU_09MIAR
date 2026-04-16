#!/usr/bin/env python3
"""Visual evidence capture for DIRECTO runs.

Observes helper.log plus camera/image, pose/info and attach_state topics to
save clear PNG evidence of the grasp lifecycle without touching runtime code.
"""

from __future__ import annotations

import argparse
import json
import re
import time
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


PRE_GRASP_RE = re.compile(r"\[PICK\]\[DIRECT\]\[TRANSITION\] from=PRE_CLOSE to=CLOSE ")
APPROACH_PREGRASP_RE = re.compile(r"\[PICK\]\[DIRECT\]\[DEBUG\] phase=APPROACH_COARSE result=ok ")
APPROACH_FAILURE_RE = re.compile(r"\[PICK\]\[DIRECT\]\[ANALYSIS\] code=(?P<code>[A-Z0-9_]+) phase=(?P<phase>[A-Z_]+) ")
BASKET_OK_RE = re.compile(r"\[PICK\]\[DEMO\] confirmacion cesta OK ")
SUCCESS_RE = re.compile(r"\[PICK\]\[DIRECT\] SECUENCIA COMPLETADA EXITOSAMENTE route=basket")


def _sanitize_label(topic: str) -> str:
    return topic.strip("/").replace("/", "_") or "camera"


class DirectoVisualCapture(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("directo_visual_capture")
        self._args = args
        self._bridge = CvBridge()
        self._output_dir = Path(args.output_dir).expanduser().resolve()
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._helper_log = Path(args.helper_log).expanduser().resolve()
        self._manifest_path = self._output_dir / "visual_capture_manifest.json"
        self._manifest: Dict[str, dict] = {}
        self._last_log_pos = 0
        self._pending: Dict[str, dict] = {}
        self._images: Dict[str, np.ndarray] = {}
        self._image_stamps: Dict[str, float] = {}
        self._attach_state: Optional[bool] = None
        self._attach_seen = False
        self._object_pose_world: Optional[tuple[float, float, float]] = None
        self._initial_object_pose_world: Optional[tuple[float, float, float]] = None
        self._done = False

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        misc_qos = QoSProfile(depth=20)

        for topic in args.camera_topics:
            self.create_subscription(
                Image,
                topic,
                lambda msg, topic_name=topic: self._on_image(topic_name, msg),
                image_qos,
            )
        self.create_subscription(Bool, args.attach_topic, self._on_attach_state, misc_qos)
        self.create_subscription(TFMessage, args.pose_topic, self._on_pose_info, misc_qos)
        self.create_timer(0.20, self._tick)

        self.get_logger().info(
            "visual capture started output_dir=%s helper_log=%s cameras=%s"
            % (self._output_dir, self._helper_log, ",".join(args.camera_topics))
        )

    def destroy_node(self) -> bool:
        try:
            self._poll_helper_log()
            self._flush_pending(force=True)
        except Exception as exc:
            self.get_logger().warning("final flush failed error=%s" % exc)
        return super().destroy_node()

    def _on_image(self, topic_name: str, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning("image decode failed topic=%s error=%s" % (topic_name, exc))
            return
        self._images[topic_name] = np.ascontiguousarray(frame)
        self._image_stamps[topic_name] = time.time()

    def _on_attach_state(self, msg: Bool) -> None:
        new_state = bool(msg.data)
        previous_state = self._attach_state
        self._attach_state = new_state
        if previous_state is False and new_state is True and "grasp_confirmed" not in self._manifest:
            self._queue_snapshot("grasp_confirmed", "attach_state_true")
        if previous_state is None and new_state is True and "grasp_confirmed" not in self._manifest:
            self._queue_snapshot("grasp_confirmed", "attach_state_true_initial")
        if new_state:
            self._attach_seen = True

    def _on_pose_info(self, msg: TFMessage) -> None:
        for tf in getattr(msg, "transforms", []) or []:
            child = str(getattr(tf, "child_frame_id", "") or "").strip()
            if not child:
                continue
            if child != self._args.object_name and not child.startswith(f"{self._args.object_name}::"):
                continue
            tr = tf.transform.translation
            pose = (float(tr.x), float(tr.y), float(tr.z))
            self._object_pose_world = pose
            if self._initial_object_pose_world is None:
                self._initial_object_pose_world = pose
            break

    def _tick(self) -> None:
        self._poll_helper_log()
        self._check_lift_condition()
        self._flush_pending()

    def _poll_helper_log(self) -> None:
        if not self._helper_log.exists():
            return
        with self._helper_log.open("r", encoding="utf-8", errors="replace") as handle:
            handle.seek(self._last_log_pos)
            for line in handle:
                self._handle_helper_line(line.rstrip("\n"))
            self._last_log_pos = handle.tell()

    def _handle_helper_line(self, line: str) -> None:
        if PRE_GRASP_RE.search(line) and "pre_grasp" not in self._manifest:
            self._queue_snapshot("pre_grasp", "pre_close_to_close_transition")
        elif APPROACH_PREGRASP_RE.search(line) and "pre_grasp" not in self._manifest:
            self._queue_snapshot("pre_grasp", "approach_coarse_result_ok")
        failure_match = APPROACH_FAILURE_RE.search(line)
        if failure_match and "approach_failure" not in self._manifest:
            code = str(failure_match.group("code") or "UNKNOWN")
            phase = str(failure_match.group("phase") or "UNKNOWN")
            self._queue_snapshot("approach_failure", f"{phase}:{code}")
        if BASKET_OK_RE.search(line) and "basket_drop" not in self._manifest:
            self._queue_snapshot("basket_drop", "basket_confirmation", delay_sec=self._args.basket_delay_sec)
        elif SUCCESS_RE.search(line) and "basket_drop" not in self._manifest:
            self._queue_snapshot("basket_drop", "sequence_success", delay_sec=self._args.basket_delay_sec)

    def _check_lift_condition(self) -> None:
        if "lift_with_object" in self._manifest:
            return
        if not self._attach_seen or not self._attach_state:
            return
        if self._initial_object_pose_world is None or self._object_pose_world is None:
            return
        lift_delta = self._object_pose_world[2] - self._initial_object_pose_world[2]
        if lift_delta >= self._args.lift_threshold_m:
            self._queue_snapshot("lift_with_object", f"lift_delta={lift_delta:.3f}m")

    def _queue_snapshot(self, name: str, reason: str, delay_sec: float = 0.0) -> None:
        if name in self._manifest:
            return
        current = self._pending.get(name)
        due_at = time.monotonic() + max(0.0, float(delay_sec))
        if current is None or due_at < float(current.get("due_at", due_at)):
            self._pending[name] = {"reason": reason, "due_at": due_at}
            self.get_logger().info("queued snapshot name=%s reason=%s" % (name, reason))

    def _flush_pending(self, force: bool = False) -> None:
        if not self._pending:
            return
        now = time.monotonic()
        for name in list(self._pending.keys()):
            pending = self._pending[name]
            if not force and now < float(pending["due_at"]):
                continue
            saved = self._save_snapshot(name, str(pending["reason"]))
            if saved:
                self._pending.pop(name, None)
        if len(self._manifest) >= 4 and not self._done:
            self._done = True
            self.get_logger().info("all requested snapshots captured")

    def _save_snapshot(self, name: str, reason: str) -> bool:
        canvas = self._compose_montage(name)
        if canvas is None:
            return False
        file_name = {
            "pre_grasp": "pre_grasp.png",
            "grasp_confirmed": "grasp_confirmed.png",
            "lift_with_object": "lift_with_object.png",
            "approach_failure": "approach_failure.png",
            "basket_drop": "basket_drop.png",
        }[name]
        file_path = self._output_dir / file_name
        cv2.imwrite(str(file_path), canvas)
        self._manifest[name] = {
            "path": str(file_path),
            "reason": reason,
            "saved_at": time.time(),
            "object_pose_world": self._object_pose_world,
            "attach_state": self._attach_state,
            "cameras": {
                topic: {"stamp": self._image_stamps.get(topic)}
                for topic in self._args.camera_topics
                if topic in self._images
            },
        }
        self._manifest_path.write_text(json.dumps(self._manifest, indent=2), encoding="utf-8")
        self.get_logger().info("saved snapshot name=%s path=%s reason=%s" % (name, file_path, reason))
        return True

    def _compose_montage(self, name: str) -> Optional[np.ndarray]:
        frames = []
        for topic in self._args.camera_topics:
            frame = self._images.get(topic)
            if frame is None:
                continue
            frames.append(self._render_labeled_frame(frame, topic))
        if not frames:
            return None
        target_height = min(self._args.frame_height_px, min(img.shape[0] for img in frames))
        resized = []
        for img in frames:
            scale = target_height / float(img.shape[0])
            width = max(1, int(round(float(img.shape[1]) * scale)))
            resized.append(cv2.resize(img, (width, target_height), interpolation=cv2.INTER_AREA))
        strip = cv2.hconcat(resized)
        banner_height = 44
        banner = np.zeros((banner_height, strip.shape[1], 3), dtype=np.uint8)
        cv2.putText(
            banner,
            f"{name}  object={self._args.object_name}  pose={self._fmt_pose(self._object_pose_world)}  attach={self._attach_state}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return cv2.vconcat([banner, strip])

    def _render_labeled_frame(self, frame: np.ndarray, topic: str) -> np.ndarray:
        canvas = frame.copy()
        overlay_h = 30
        canvas = cv2.copyMakeBorder(canvas, overlay_h, 0, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        cv2.putText(
            canvas,
            _sanitize_label(topic),
            (10, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return canvas

    @staticmethod
    def _fmt_pose(pose: Optional[tuple[float, float, float]]) -> str:
        if pose is None:
            return "none"
        return f"({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f})"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture visual evidence for DIRECTO runs")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--helper-log", required=True)
    parser.add_argument("--object-name", default="pick_demo")
    parser.add_argument("--pose-topic", default="/world/ur5_mesa_objetos/pose/info")
    parser.add_argument("--attach-topic", default="/gripper/pick_demo/state")
    parser.add_argument(
        "--camera-topics",
        nargs="+",
        default=["/camera_overhead/image", "/camera_north/image", "/camera_east/image"],
    )
    parser.add_argument("--lift-threshold-m", type=float, default=0.05)
    parser.add_argument("--basket-delay-sec", type=float, default=1.0)
    parser.add_argument("--frame-height-px", type=int, default=360)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    rclpy.init()
    node = DirectoVisualCapture(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())