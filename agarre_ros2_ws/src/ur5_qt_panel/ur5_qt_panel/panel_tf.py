#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tf.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""TF helper utilities for the panel."""
from __future__ import annotations

import os
import threading
import time
from typing import Optional, Set, Tuple

from PyQt5.QtCore import QObject, pyqtSignal, QThread

from .logging_utils import timestamped_line

from .panel_config import ROS_AVAILABLE, USE_SIM_TIME

try:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from geometry_msgs.msg import PointStamped, PoseStamped
    from builtin_interfaces.msg import Time as BuiltinTime
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, TransformRegistration, TypeException
except Exception:  # pragma: no cover
    rclpy = None  # type: ignore
    Duration = None  # type: ignore
    SingleThreadedExecutor = None  # type: ignore
    Parameter = None  # type: ignore
    QoSProfile = None  # type: ignore
    ReliabilityPolicy = None  # type: ignore
    DurabilityPolicy = None  # type: ignore
    HistoryPolicy = None  # type: ignore
    qos_profile_sensor_data = None  # type: ignore
    Time = None  # type: ignore
    PointStamped = None  # type: ignore
    PoseStamped = None  # type: ignore
    BuiltinTime = None  # type: ignore
    TFMessage = None  # type: ignore
    Buffer = None  # type: ignore
    TransformListener = None  # type: ignore
    LookupException = Exception  # type: ignore
    ConnectivityException = Exception  # type: ignore
    ExtrapolationException = Exception  # type: ignore
    TransformRegistration = None  # type: ignore
    TypeException = Exception  # type: ignore


def _duration_from_seconds(timeout_sec: float) -> Optional["Duration"]:
    if Duration is None:
        return None
    return Duration(seconds=float(timeout_sec))


def _extract_frames_from_yaml(yaml_text: str) -> Set[str]:
    if not yaml_text:
        return set()
    frames: Set[str] = set()
    for line in yaml_text.splitlines():
        if not line or ":" not in line:
            continue
        name = line.split(":", 1)[0].strip().strip("\"'")
        if name:
            frames.add(name)
    return frames


class _TfSpinWorker(QObject):
    finished = pyqtSignal()

    def __init__(self, helper: "TfHelper"):
        super().__init__()
        self._helper = helper

    def run(self) -> None:
        self._helper._spin()
        self.finished.emit()


class TfHelper:
    """Lightweight helper that keeps a TF2 buffer spinning for Panel transforms."""

    def __init__(self):
        self._lock = threading.Lock()
        self._node = None
        self._buffer = None
        self._listener = None
        self._executor = None
        self._thread: Optional[QThread] = None
        self._spin_worker: Optional[_TfSpinWorker] = None
        self._running = False
        self._stop_event = threading.Event()
        self._frame_event = threading.Event()
        self._tf_topic_sub = None
        self._tf_static_topic_sub = None
        self._tf_msg_count = 0
        self._tf_static_msg_count = 0
        self._frames_seen_tf: Set[str] = set()
        self._frames_seen_tf_static: Set[str] = set()
        self._tf_listener_logged = False
        self._debug_exceptions = os.environ.get("PANEL_DEBUG_EXCEPTIONS", "").strip() in ("1", "true", "True")
        if ROS_AVAILABLE and Buffer is not None:
            self._start()

    def _log_exception(self, context: str, exc: Exception) -> None:
        if not self._debug_exceptions:
            return
        print(timestamped_line(f"[TF][WARN] {context}: {exc}"), flush=True)

    def _start(self):
        with self._lock:
            if self._running or Buffer is None or rclpy is None:
                return
            try:
                if not rclpy.ok():
                    rclpy.init(args=None)
            except Exception:
                pass
            use_sim_time = bool(USE_SIM_TIME)
            overrides = None
            if Parameter is not None:
                overrides = [Parameter("use_sim_time", Parameter.Type.BOOL, bool(use_sim_time))]
            node_kwargs = {}
            if overrides:
                node_kwargs["parameter_overrides"] = overrides
            self._node = rclpy.create_node("panel_tf_helper", **node_kwargs)
            self._buffer = Buffer()
            self._listener = TransformListener(self._buffer, self._node)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            tf_qos = qos_profile_sensor_data
            tf_static_qos = qos_profile_sensor_data
            if (
                QoSProfile is not None
                and ReliabilityPolicy is not None
                and DurabilityPolicy is not None
                and HistoryPolicy is not None
            ):
                tf_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=200,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                )
                tf_static_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
            if TFMessage is not None:
                try:
                    self._tf_topic_sub = self._node.create_subscription(
                        TFMessage, "/tf", self._on_tf_msg, tf_qos
                    )
                    self._tf_static_topic_sub = self._node.create_subscription(
                        TFMessage, "/tf_static", self._on_tf_static, tf_static_qos
                    )
                except Exception as exc:
                    self._log_exception("create TF subscriptions", exc)
            self._running = True
            self._thread = QThread()
            self._spin_worker = _TfSpinWorker(self)
            self._spin_worker.moveToThread(self._thread)
            self._spin_worker.finished.connect(self._thread.quit)
            self._spin_worker.finished.connect(self._spin_worker.deleteLater)
            self._thread.finished.connect(self._thread.deleteLater)
            self._stop_event.clear()
            self._thread.started.connect(self._spin_worker.run)
            self._thread.start()

    def _spin(self):
        while not self._stop_event.is_set():
            with self._lock:
                if not self._running:
                    return
            try:
                if rclpy is not None and not rclpy.ok():
                    return
                if self._executor:
                    self._executor.spin_once(timeout_sec=0.05)
            except Exception as exc:
                self._log_exception("spin_once", exc)
            self._stop_event.wait(0.02)

    def shutdown(self) -> None:
        """Stop the helper cleanly."""
        with self._lock:
            if not self._running:
                self._stop_event.set()
                return
            self._running = False
            self._stop_event.set()
        if self._executor:
            try:
                self._executor.shutdown()
            except Exception as exc:
                self._log_exception("executor shutdown", exc)
        if self._thread and self._thread.isRunning():
            self._thread.quit()
            self._thread.wait(1000)
        if self._node:
            if self._tf_topic_sub:
                try:
                    self._node.destroy_subscription(self._tf_topic_sub)
                except Exception as exc:
                    self._log_exception("destroy /tf subscription", exc)
                self._tf_topic_sub = None
            if self._tf_static_topic_sub:
                try:
                    self._node.destroy_subscription(self._tf_static_topic_sub)
                except Exception as exc:
                    self._log_exception("destroy /tf_static subscription", exc)
                self._tf_static_topic_sub = None
        if self._executor and self._node:
            try:
                self._executor.remove_node(self._node)
            except Exception as exc:
                self._log_exception("remove node from executor", exc)
        if self._node:
            try:
                self._node.destroy_node()
            except Exception as exc:
                self._log_exception("destroy node", exc)
        self._node = None
        self._executor = None
        self._buffer = None
        self._listener = None
        self._thread = None
        self._spin_worker = None

    def _on_tf_msg(self, msg: "TFMessage") -> None:
        if not msg:
            return
        should_log = False
        with self._lock:
            transforms = list(getattr(msg, "transforms", []))
            self._tf_msg_count += len(transforms)
            for tf in transforms:
                if tf.header and tf.header.frame_id:
                    self._frames_seen_tf.add(tf.header.frame_id)
                if tf.child_frame_id:
                    self._frames_seen_tf.add(tf.child_frame_id)
            if not self._tf_listener_logged:
                self._tf_listener_logged = True
                should_log = True
        if should_log:
            self._log_tf_listener_active()
        if transforms:
            self._frame_event.set()

    def _on_tf_static(self, msg: "TFMessage") -> None:
        if not msg:
            return
        should_log = False
        with self._lock:
            transforms = list(getattr(msg, "transforms", []))
            self._tf_static_msg_count += len(transforms)
            for tf in transforms:
                if tf.header and tf.header.frame_id:
                    self._frames_seen_tf_static.add(tf.header.frame_id)
                if tf.child_frame_id:
                    self._frames_seen_tf_static.add(tf.child_frame_id)
            if not self._tf_listener_logged:
                self._tf_listener_logged = True
                should_log = True
        if should_log:
            self._log_tf_listener_active()
        if transforms:
            self._frame_event.set()

    def frames_snapshot(self) -> Tuple[Set[str], Set[str]]:
        return set(self._frames_seen_tf), set(self._frames_seen_tf_static)

    def wait_for_frames(self, timeout_sec: float) -> bool:
        frames, frames_static = self.frames_snapshot()
        if frames or frames_static:
            return True
        self._frame_event.clear()
        return self._frame_event.wait(timeout_sec)

    def tf_listener_stats(self) -> Tuple[int, int]:
        with self._lock:
            return self._tf_msg_count, self._tf_static_msg_count

    def tf_frames_seen(self) -> Tuple[Set[str], Set[str]]:
        with self._lock:
            return set(self._frames_seen_tf), set(self._frames_seen_tf_static)

    def _log_tf_listener_active(self) -> None:
        stats = self.tf_listener_stats()
        print(
            timestamped_line(
                f"[TRACE] TF listener active (tf_msgs={stats[0]} tf_static={stats[1]})"
            ),
            flush=True,
        )

    def transform_pose(self, pose, target_frame: str, timeout_sec: float = 0.8):
        """Transform a PoseStamped into *target_frame* (blocking until timeout)."""
        if not ROS_AVAILABLE or not self._buffer or rclpy is None:
            return None
        end = time.time() + timeout_sec
        timeout = _duration_from_seconds(timeout_sec)
        while time.time() < end:
            try:
                if timeout is not None:
                    return self._buffer.transform(pose, target_frame, timeout)
                return self._buffer.transform(pose, target_frame)
            except (LookupException, ConnectivityException, ExtrapolationException):
                time.sleep(0.05)
            except TypeException:
                return None
            except Exception:
                return None
        return None

    def transform_point(self, point: "PointStamped", target_frame: str, timeout_sec: float = 0.8):
        """Transform a PointStamped into *target_frame* (blocking until timeout)."""
        if not ROS_AVAILABLE or not self._buffer or point is None or PoseStamped is None:
            return None
        pose = PoseStamped()
        pose.header.frame_id = point.header.frame_id
        if point.header and point.header.stamp:
            pose.header.stamp = point.header.stamp
        else:
            if BuiltinTime is not None:
                pose.header.stamp = BuiltinTime(sec=0, nanosec=0)
            else:
                if rclpy is None:
                    return None
                pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x = point.point.x
        pose.pose.position.y = point.point.y
        pose.pose.position.z = point.point.z
        pose.pose.orientation.w = 1.0
        converted = self.transform_pose(pose, target_frame, timeout_sec)
        if not converted:
            return None
        out = PointStamped()
        out.header = converted.header
        out.point.x = converted.pose.position.x
        out.point.y = converted.pose.position.y
        out.point.z = converted.pose.position.z
        return out

    def list_frames(self) -> Set[str]:
        """Return the set of available TF frames."""
        if not ROS_AVAILABLE or not self._buffer:
            return set()
        try:
            yaml_text = self._buffer.all_frames_as_yaml()
        except Exception:
            return set()
        return _extract_frames_from_yaml(yaml_text)

    def frames_yaml(self) -> Optional[str]:
        """Return raw TF frames YAML (if available)."""
        if not ROS_AVAILABLE or not self._buffer:
            return None
        try:
            return self._buffer.all_frames_as_yaml()
        except Exception:
            return None

    def topic_names_and_types(self):
        """Return ROS graph topics if the helper node is available."""
        if not ROS_AVAILABLE or not self._node:
            return []
        try:
            return self._node.get_topic_names_and_types()
        except Exception:
            return []

    def can_transform(self, target_frame: str, source_frame: str, timeout_sec: float = 0.1) -> bool:
        """Check if TF is available between frames (blocking until timeout)."""
        if not ROS_AVAILABLE or not self._buffer or rclpy is None:
            return False
        end = time.time() + timeout_sec
        timeout = _duration_from_seconds(timeout_sec)
        while time.time() < end:
            try:
                ts = Time() if Time is not None else rclpy.time.Time()
                if timeout is not None:
                    return self._buffer.can_transform(target_frame, source_frame, ts, timeout)
                return self._buffer.can_transform(target_frame, source_frame, ts)
            except (LookupException, ConnectivityException, ExtrapolationException):
                time.sleep(0.02)
        return False

    def lookup_transform(self, target_frame: str, source_frame: str, timeout_sec: float = 1.0):
        """Lookup raw transform *target_frame* <- *source_frame*."""
        if not ROS_AVAILABLE or not self._buffer or rclpy is None:
            return None
        end = time.time() + timeout_sec
        while time.time() < end:
            try:
                return self._buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            except (LookupException, ConnectivityException, ExtrapolationException):
                time.sleep(0.05)
        return None


_TF_HELPER: Optional[TfHelper] = None


def get_tf_helper() -> Optional[TfHelper]:
    """Return singleton TfHelper (if ROS is available)."""
    global _TF_HELPER
    if not ROS_AVAILABLE or Buffer is None:
        return None
    if _TF_HELPER is None:
        _TF_HELPER = TfHelper()
    return _TF_HELPER


def shutdown_tf_helper() -> None:
    """Stop and destroy the TF helper if it exists."""
    global _TF_HELPER
    helper = _TF_HELPER
    if helper:
        helper.shutdown()
        _TF_HELPER = None
