#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_ros.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Implementacion del worker ROS para el panel."""
from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple

from PyQt5.QtCore import QObject, pyqtSignal, QThread
from PyQt5.QtGui import QImage

from .panel_config import (
    ROS_AVAILABLE,
    DEBUG_FRAME_LOG,
    DEPTH_PCTL_REFRESH_FRAMES,
    DEPTH_PCTL_STRIDE,
    DEPTH_FAST,
    CAMERA_MAX_SIZE,
    CAMERA_USE_BGR,
    CAMERA_COPY_FRAME,
)
from .panel_ui_params import get_panel_ui_params as _get_panel_ui_params
from .panel_ros_params import get_panel_ros_params as _get_panel_ros_params

CLOCK_MAX_AGE_SEC = 8.0


def _steady_time() -> float:
    """Devuelve un timestamp monotono local al proceso para watchdogs y frescura."""
    return time.monotonic()

try:
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import (
        qos_profile_sensor_data,
        QoSProfile,
        ReliabilityPolicy,
        DurabilityPolicy,
        HistoryPolicy,
    )
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String, Empty, Bool
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    from rosidl_runtime_py.utilities import get_message as ros_get_message
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover
    rclpy = None  # type: ignore
    ReentrantCallbackGroup = None  # type: ignore
    SingleThreadedExecutor = None  # type: ignore
    MultiThreadedExecutor = None  # type: ignore
    Parameter = None  # type: ignore
    qos_profile_sensor_data = None  # type: ignore
    QoSProfile = None  # type: ignore
    ReliabilityPolicy = None  # type: ignore
    DurabilityPolicy = None  # type: ignore
    HistoryPolicy = None  # type: ignore
    ExternalShutdownException = None  # type: ignore
    Image = None  # type: ignore
    JointState = None  # type: ignore
    String = None  # type: ignore
    Empty = None  # type: ignore
    Bool = None  # type: ignore
    Trigger = None  # type: ignore
    TFMessage = None  # type: ignore
    Clock = None  # type: ignore
    ros_get_message = None  # type: ignore
    CvBridge = None  # type: ignore

try:
    from ur5_panel_interfaces.srv import SelectObject
except Exception:  # pragma: no cover
    SelectObject = None  # type: ignore

if TYPE_CHECKING:
    from sensor_msgs.msg import CompressedImage
    from .panel_tf import TfHelper


class _RosWorkerThread(QThread):
    def __init__(self, worker: "RosWorker"):
        super().__init__()
        self._worker = worker

    def run(self):
        self._worker._thread_main()


class RosWorker(QObject):
    log = pyqtSignal(str)
    image = pyqtSignal(str, QImage, int, int, float)
    joint_state = pyqtSignal(object)
    grasp_rect = pyqtSignal(object)
    system_state = pyqtSignal(str, str)
    camera_connect_request = pyqtSignal(str)
    camera_disconnect_request = pyqtSignal(str)
    recover_request = pyqtSignal(str)
    tfm_infer_request = pyqtSignal(str)
    tfm_execute_request = pyqtSignal(str)
    pick_demo_request = pyqtSignal(str)
    pick_object_request = pyqtSignal(str)
    object_select_request = pyqtSignal(str, str)

    def __init__(self, force_realtime: bool = False):
        super().__init__()
        self._lock = threading.Lock()
        self._running = False
        self._debug_exceptions = _get_panel_ui_params().debug_exceptions
        self._node = None
        self._exec = None
        self._bridge = None
        self._thread: Optional[QThread] = None
        self._service_callback_group = None
        self._io_callback_group = None
        self._subs: Dict[str, object] = {}
        self._pose_sub = None
        self._pose_topic = ""
        self._pose_cache: Dict[str, Tuple[float, float, float]] = {}
        self._pose_last_wall = 0.0
        self._pose_msg_count = 0
        self._pose_last_entities = 0
        self._pose_info_empty_logged = False
        self._pose_event = threading.Event()
        self._pubs: Dict[str, object] = {}
        self._fps: Dict[str, List[float]] = {}
        self._frame_count: Dict[str, int] = {}
        self._image_msg_count: Dict[str, int] = {}
        self._frame_buffers: Dict[str, object] = {}
        self._latest_image_snapshot: Dict[str, Tuple[QImage, int, int, float, float]] = {}
        self._image_decode_error_logged: Dict[str, bool] = {}
        self._last_clock_wall = 0.0
        self._last_clock_stamp_ns: Optional[int] = None
        self._last_clock_advance_wall = 0.0
        self._clock_event = threading.Event()
        self._last_emit: Dict[str, float] = {}
        self._joint_sub = None
        self._joint_topic = ""
        self._last_joint_emit = 0.0
        self._joint_emit_interval = 0.1
        self._last_joint_payload: Optional[dict] = None
        self._last_joint_wall: float = 0.0
        self._depth_cache: Dict[str, Tuple[float, float, int]] = {}
        self._latest_depth_frame: Dict[str, Tuple[object, float]] = {}
        _ros_params = _get_panel_ros_params()
        self._camera_connect_service = _ros_params.camera_connect_trigger_service.strip() or "/panel/camera_connect"
        self._camera_disconnect_service = _ros_params.camera_disconnect_trigger_service.strip() or "/panel/camera_disconnect"
        self._recover_topic = _ros_params.recover_trigger_topic.strip() or "/panel/recover"
        self._recover_service = _ros_params.recover_trigger_service.strip() or "/panel/recover"
        self._tfm_infer_topic = _ros_params.tfm_infer_trigger_topic.strip() or "/panel/tfm_infer"
        self._tfm_infer_service = _ros_params.tfm_infer_trigger_service.strip() or "/panel/tfm_infer"
        self._tfm_execute_topic = _ros_params.tfm_execute_trigger_topic.strip() or "/panel/tfm_execute"
        self._tfm_execute_service = _ros_params.tfm_execute_trigger_service.strip() or "/panel/tfm_execute"
        self._pick_demo_service = _ros_params.pick_demo_trigger_service.strip() or "/panel/pick_demo"
        self._pick_object_topic = _ros_params.pick_object_trigger_topic.strip() or "/panel/pick_object"
        self._pick_object_service = _ros_params.pick_object_trigger_service.strip() or "/panel/pick_object"
        self._select_object_topic = _ros_params.select_object_topic.strip() or "/panel/select_object"
        self._select_object_service = _ros_params.select_object_service.strip() or "/panel/select_object"
        self._camera_connect_srv = None
        self._camera_disconnect_srv = None
        self._recover_sub = None
        self._recover_srv = None
        self._tfm_infer_sub = None
        self._tfm_infer_srv = None
        self._tfm_execute_sub = None
        self._tfm_execute_srv = None
        self._pick_demo_srv = None
        self._pick_object_sub = None
        self._pick_object_srv = None
        self._select_object_sub = None
        self._select_object_srv = None
        self._pick_demo_timeout_sec = _ros_params.pick_demo_service_timeout_sec
        self._pick_demo_pending: Dict[str, Tuple[threading.Event, Dict[str, object]]] = {}
        self._object_select_timeout_sec = _ros_params.select_object_service_timeout_sec
        self._object_select_pending: Dict[str, Tuple[threading.Event, Dict[str, object]]] = {}
        self._tfm_infer_timeout_sec = _ros_params.tfm_infer_service_timeout_sec
        self._tfm_execute_timeout_sec = _ros_params.tfm_execute_service_timeout_sec
        self._tfm_infer_pending: Dict[str, Tuple[threading.Event, Dict[str, object]]] = {}
        self._tfm_execute_pending: Dict[str, Tuple[threading.Event, Dict[str, object]]] = {}
        self._cleanup_done = False
        self._system_diag_reason: str = ""
        self._system_state_last: str = ""
        self._system_state_reason: str = ""
        self._system_state_wall: float = 0.0
        self._moveit_result_sub = None
        self._moveit_result_topic: str = ""
        self._moveit_result_last: str = ""
        self._moveit_result_wall: float = 0.0
        self._moveit_result_seq: int = 0
        self._moveit_result_event = threading.Event()
        self._moveit_result_qos = None
        self._moveit_bridge_hb_sub = None
        self._moveit_bridge_hb_topic: str = ""
        self._moveit_bridge_hb_wall: float = 0.0
        self._moveit_bridge_hb_event = threading.Event()
        self._moveit_bridge_hb_qos = None
        self._grasp_rect_sub = None
        self._grasp_rect_topic: str = ""
        self._grasp_rect_type: str = ""
        self._grasp_rect_last_wall: float = 0.0
        self._grasp_rect_count: int = 0
        if QoSProfile is not None:
            try:
                self._moveit_result_qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                )
            except Exception:
                self._moveit_result_qos = None
            try:
                self._moveit_bridge_hb_qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                )
            except Exception:
                self._moveit_bridge_hb_qos = None
        try:
            max_fps = float(os.environ.get("PANEL_MAX_FPS", "60"))
            if max_fps <= 0:
                max_fps = 60.0
        except ValueError:
            max_fps = 60.0
        self._min_emit_interval = 1.0 / max_fps
        self._force_realtime = force_realtime

    def _safe_log(self, msg: str) -> None:
        try:
            self.log.emit(msg)
        except RuntimeError:
            pass

    def _log_exception(self, context: str, exc: Exception) -> None:
        if not self._running and not self._debug_exceptions:
            return
        self._safe_log(f"[ROS] WARN {context}: {exc}")

    def start(self):
        if not ROS_AVAILABLE:
            self.log.emit("[ROS] No disponible (faltan deps).")
            return
        with self._lock:
            if self._running:
                return
            self._running = True
            self._cleanup_done = False
        thread = _RosWorkerThread(self)
        self._thread = thread
        self.moveToThread(thread)
        thread.start()

    def stop(self):
        with self._lock:
            self._running = False
        self._cleanup_ros()
        thread = self._thread
        if thread is not None:
            if QThread.currentThread() != thread:
                thread.quit()
                thread.wait(1000)
        self._thread = None

    def stop_and_join(self):
        self.stop()

    def list_topic_names(self) -> List[str]:
        with self._lock:
            node = self._node
        if node is None:
            return []
        try:
            return [name for name, _ in node.get_topic_names_and_types()]
        except Exception:
            return []

    def list_action_names(self) -> List[str]:
        with self._lock:
            node = self._node
        if node is None:
            return []
        try:
            return [name for name, _ in node.get_action_names_and_types()]
        except Exception:
            return []

    def topic_has_publishers(self, topic: str) -> bool:
        topic = (topic or "").strip()
        if not topic:
            return False
        with self._lock:
            node = self._node
        if node is None:
            return False
        try:
            return bool(node.get_publishers_info_by_topic(topic))
        except Exception:
            return False

    def topic_has_subscribers(self, topic: str) -> bool:
        topic = (topic or "").strip()
        if not topic:
            return False
        with self._lock:
            node = self._node
        if node is None:
            return False
        try:
            return bool(node.get_subscriptions_info_by_topic(topic))
        except Exception:
            return False

    def topic_subscriber_count(self, topic: str) -> int:
        topic = (topic or "").strip()
        if not topic:
            return 0
        with self._lock:
            node = self._node
        if node is None:
            return 0
        try:
            return len(node.get_subscriptions_info_by_topic(topic) or [])
        except Exception:
            return 0

    def topic_publisher_count(self, topic: str) -> int:
        topic = (topic or "").strip()
        if not topic:
            return 0
        with self._lock:
            node = self._node
        if node is None:
            return 0
        try:
            return len(node.get_publishers_info_by_topic(topic) or [])
        except Exception:
            return 0

    def node_name(self) -> str:
        with self._lock:
            node = self._node
        if node is None:
            return ""
        try:
            return node.get_name()
        except Exception:
            return ""

    def node_namespace(self) -> str:
        with self._lock:
            node = self._node
        if node is None:
            return ""
        try:
            return node.get_namespace()
        except Exception:
            return ""

    def publisher_nodes_by_topic(self, topic: str) -> List[str]:
        topic = (topic or "").strip()
        if not topic:
            return []
        with self._lock:
            node = self._node
        if node is None:
            return []
        try:
            infos = node.get_publishers_info_by_topic(topic)
        except Exception:
            return []
        names: List[str] = []
        for info in infos or []:
            name = getattr(info, "node_name", "") or ""
            ns = getattr(info, "node_namespace", "") or ""
            full = f"{ns}/{name}" if ns else name
            full = full.replace("//", "/").strip()
            if full:
                names.append(full)
        return names

    def topic_names_and_types(self) -> List[Tuple[str, List[str]]]:
        with self._lock:
            node = self._node
        if node is None:
            return []
        try:
            return node.get_topic_names_and_types()
        except Exception:
            return []

    def image_frame_snapshot(
        self, topic: str
    ) -> Optional[Tuple[QImage, int, int, float, float, int]]:
        topic = (topic or "").strip()
        if not topic:
            return None
        with self._lock:
            item = self._latest_image_snapshot.get(topic)
            count = int(self._frame_count.get(topic, 0))
        if not item:
            return None
        qimg, w, h, fps, wall = item
        try:
            return (qimg.copy(), int(w), int(h), float(fps), float(wall), count)
        except Exception:
            return None

    def service_names_and_types(self) -> List[Tuple[str, List[str]]]:
        with self._lock:
            node = self._node
        if node is None:
            return []
        try:
            return node.get_service_names_and_types()
        except Exception:
            return []

    def has_service(self, name: str) -> bool:
        target = (name or "").strip()
        if not target:
            return False
        for svc_name, _types in self.service_names_and_types():
            if svc_name == target:
                return True
        return False

    def system_state_snapshot(self) -> Tuple[str, str, float]:
        with self._lock:
            return (
                self._system_state_last,
                self._system_state_reason,
                float(self._system_state_wall),
            )

    def call_trigger_detail(
        self, service_name: str, timeout_sec: float = 3.0
    ) -> Tuple[bool, str]:
        if not ROS_AVAILABLE:
            return False, "ROS no disponible"
        target = (service_name or "").strip()
        if not target:
            return False, "nombre de servicio vacío"
        with self._lock:
            node = self._node
            exec_running = self._exec is not None and self._running
        if node is None:
            return False, "nodo ROS no listo"
        try:
            from std_srvs.srv import Trigger
        except Exception:
            return False, "std_srvs/Trigger no disponible"
        try:
            client = node.create_client(Trigger, target)
        except Exception:
            return False, "no se pudo crear cliente Trigger"
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                node.destroy_client(client)
                return False, f"timeout esperando servicio {target}"
            future = client.call_async(Trigger.Request())
            if exec_running:
                done_event = threading.Event()
                future.add_done_callback(lambda _fut: done_event.set())
                done_event.wait(timeout=max(0.1, timeout_sec))
            else:
                rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
            if not future.done():
                node.destroy_client(client)
                return False, f"timeout respuesta {target}"
            result = future.result()
            ok = bool(getattr(result, "success", False))
            message = str(getattr(result, "message", "") or "").strip()
            node.destroy_client(client)
            return ok, message
        except Exception:
            try:
                node.destroy_client(client)
            except Exception:
                pass
            return False, f"exception llamando {target}"

    def call_trigger(self, service_name: str, timeout_sec: float = 3.0) -> bool:
        ok, _msg = self.call_trigger_detail(service_name, timeout_sec=timeout_sec)
        return ok

    def get_latest_depth_frame(self, topic: str) -> Optional[Tuple[object, float]]:
        target = (topic or "").strip()
        if not target:
            return None
        with self._lock:
            frame = self._latest_depth_frame.get(target)
        if not frame:
            return None
        depth, ts = frame
        return depth, float(ts)

    def subscribe_pose_info(self, topic: str) -> bool:
        if not ROS_AVAILABLE or TFMessage is None:
            return False
        topic = (topic or "").strip()
        if not topic:
            return False
        with self._lock:
            if not self._node:
                self.log.emit("[ROS] Nodo no listo todavía.")
                return False
            if self._pose_sub is not None and self._pose_topic == topic:
                return True
            if self._pose_sub is not None:
                try:
                    self._node.destroy_subscription(self._pose_sub)
                except Exception as exc:
                    self._log_exception("destroy pose subscription", exc)
                self._pose_sub = None
            try:
                self._pose_topic = topic
                self._pose_sub = self._node.create_subscription(
                    TFMessage,
                    topic,
                    self._on_pose_info,
                    qos_profile_sensor_data,
                )
                self.log.emit(f"[ROS] Suscrito a {topic} (pose info)")
                return True
            except Exception as exc:
                self.log.emit(f"[ROS] ERROR suscribiendo pose info: {exc}")
                return False

    def subscribe_moveit_result(self, topic: str = "/desired_grasp/result") -> bool:
        if not ROS_AVAILABLE or String is None:
            return False
        topic = (topic or "").strip()
        if not topic:
            return False
        with self._lock:
            if not self._node:
                self.log.emit("[ROS] Nodo no listo todavía.")
                return False
            if self._moveit_result_sub is not None and self._moveit_result_topic == topic:
                return True
            if self._moveit_result_sub is not None:
                try:
                    self._node.destroy_subscription(self._moveit_result_sub)
                except Exception as exc:
                    self._log_exception("destroy moveit_result subscription", exc)
                self._moveit_result_sub = None
            try:
                self._moveit_result_topic = topic
                self._moveit_result_sub = self._node.create_subscription(
                    String,
                    topic,
                    self._on_moveit_result,
                    self._moveit_result_qos if self._moveit_result_qos is not None else 10,
                )
                self._moveit_result_last = ""
                self._moveit_result_wall = 0.0
                self._moveit_result_seq = 0
                self._moveit_result_event.clear()
                qos_summary = self._format_qos(self._moveit_result_qos)
                self.log.emit(f"[ROS] Suscrito a {topic} (moveit result, {qos_summary})")
                return True
            except Exception as exc:
                self.log.emit(f"[ROS] ERROR suscribiendo moveit result: {exc}")
                return False

    def moveit_result_snapshot(self) -> Tuple[str, float, int]:
        with self._lock:
            return self._moveit_result_last, self._moveit_result_wall, self._moveit_result_seq

    def wait_for_moveit_result(
        self,
        *,
        since_wall: float,
        since_seq: int = -1,
        timeout_sec: float,
    ) -> Tuple[bool, str, float, int]:
        timeout = max(0.1, float(timeout_sec))
        with self._lock:
            if self._moveit_result_seq > since_seq and self._moveit_result_wall >= (since_wall - 1e-3):
                return (
                    True,
                    self._moveit_result_last,
                    self._moveit_result_wall,
                    self._moveit_result_seq,
                )
            self._moveit_result_event.clear()
        deadline = _steady_time() + timeout
        while _steady_time() < deadline:
            remaining = max(0.01, deadline - _steady_time())
            self._moveit_result_event.wait(timeout=remaining)
            with self._lock:
                if self._moveit_result_seq > since_seq and self._moveit_result_wall >= (since_wall - 1e-3):
                    return (
                        True,
                        self._moveit_result_last,
                        self._moveit_result_wall,
                        self._moveit_result_seq,
                    )
                self._moveit_result_event.clear()
        with self._lock:
            return (
                False,
                self._moveit_result_last,
                self._moveit_result_wall,
                self._moveit_result_seq,
            )

    def drain_moveit_results(self, duration_sec: float = 0.2) -> int:
        """Consume durante una ventana corta los resultados viejos de MoveIt."""
        drained = 0
        timeout = max(0.0, float(duration_sec))
        deadline = _steady_time() + timeout
        _raw, cursor_wall, cursor_seq = self.moveit_result_snapshot()
        while _steady_time() < deadline:
            wait_chunk = min(0.05, max(0.01, deadline - _steady_time()))
            ok, _raw, wall, seq = self.wait_for_moveit_result(
                since_wall=cursor_wall,
                since_seq=cursor_seq,
                timeout_sec=wait_chunk,
            )
            if not ok:
                break
            drained += 1
            cursor_wall = max(cursor_wall, wall)
            cursor_seq = max(cursor_seq, int(seq))
        return drained

    def subscribe_moveit_bridge_heartbeat(
        self, topic: str = "/ur5_moveit_bridge/heartbeat"
    ) -> bool:
        if not ROS_AVAILABLE or Bool is None:
            return False
        topic = (topic or "").strip()
        if not topic:
            return False
        with self._lock:
            if not self._node:
                self.log.emit("[ROS] Nodo no listo todavía.")
                return False
            if (
                self._moveit_bridge_hb_sub is not None
                and self._moveit_bridge_hb_topic == topic
            ):
                return True
            if self._moveit_bridge_hb_sub is not None:
                try:
                    self._node.destroy_subscription(self._moveit_bridge_hb_sub)
                except Exception as exc:
                    self._log_exception("destroy moveit heartbeat subscription", exc)
                self._moveit_bridge_hb_sub = None
            try:
                self._moveit_bridge_hb_topic = topic
                self._moveit_bridge_hb_sub = self._node.create_subscription(
                    Bool,
                    topic,
                    self._on_moveit_bridge_heartbeat,
                    self._moveit_bridge_hb_qos
                    if self._moveit_bridge_hb_qos is not None
                    else 10,
                )
                self._moveit_bridge_hb_wall = 0.0
                self._moveit_bridge_hb_event.clear()
                qos_summary = self._format_qos(self._moveit_bridge_hb_qos)
                self.log.emit(
                    f"[ROS] Suscrito a {topic} (moveit heartbeat, {qos_summary})"
                )
                return True
            except Exception as exc:
                self.log.emit(f"[ROS] ERROR suscribiendo moveit heartbeat: {exc}")
                return False

    def subscribe_grasp_rect(self, topic: str = "/grasp_rect") -> bool:
        if not ROS_AVAILABLE:
            return False
        topic = (topic or "").strip()
        if not topic:
            return False
        with self._lock:
            if not self._node:
                self.log.emit("[ROS] Nodo no listo todavía.")
                return False
            if self._grasp_rect_sub is not None and self._grasp_rect_topic == topic:
                return True
            if self._grasp_rect_sub is not None:
                try:
                    self._node.destroy_subscription(self._grasp_rect_sub)
                except Exception as exc:
                    self._log_exception("destroy grasp_rect subscription", exc)
                self._grasp_rect_sub = None

            detected_type = ""
            try:
                for name, types in self._node.get_topic_names_and_types():
                    if name == topic and types:
                        detected_type = str(types[0] or "")
                        break
            except Exception as exc:
                self._log_exception("discover grasp_rect type", exc)
            if not detected_type:
                detected_type = "vision_msgs/msg/BoundingBox2D"

            msg_cls = self._resolve_ros_message_class(detected_type)
            if msg_cls is None:
                self.log.emit(
                    f"[ROS] ERROR suscribiendo {topic}: tipo no soportado ({detected_type})"
                )
                return False
            try:
                self._grasp_rect_topic = topic
                self._grasp_rect_type = detected_type
                self._grasp_rect_sub = self._node.create_subscription(
                    msg_cls,
                    topic,
                    self._on_grasp_rect,
                    qos_profile_sensor_data if qos_profile_sensor_data is not None else 10,
                )
                self._grasp_rect_last_wall = 0.0
                self._grasp_rect_count = 0
                self.log.emit(f"[ROS] Suscrito a {topic} (grasp rect, type={detected_type})")
                return True
            except Exception as exc:
                self.log.emit(f"[ROS] ERROR suscribiendo grasp rect: {exc}")
                return False

    def moveit_bridge_heartbeat_age(self) -> float:
        with self._lock:
            last = float(self._moveit_bridge_hb_wall)
        if last <= 0.0:
            return float("inf")
        return max(0.0, _steady_time() - last)

    def has_recent_moveit_bridge_heartbeat(self, max_age_sec: float = 1.0) -> bool:
        age = self.moveit_bridge_heartbeat_age()
        return age <= max(0.05, float(max_age_sec))

    def _on_moveit_result(self, msg: "String") -> None:
        try:
            data = getattr(msg, "data", "") or ""
        except Exception:
            data = ""
        req_id = -1
        req_uuid = ""
        success = "n/a"
        parse_status = "raw"
        try:
            payload = json.loads(str(data or ""))
            req_id = int(payload.get("request_id", -1) or -1)
            req_uuid = str(payload.get("request_uuid", "") or "")
            success = str(bool(payload.get("success", False))).lower()
            parse_status = "json"
        except Exception:
            pass
        with self._lock:
            self._moveit_result_last = str(data)
            self._moveit_result_wall = _steady_time()
            self._moveit_result_seq += 1
            self._moveit_result_event.set()
            seq = int(self._moveit_result_seq)
            topic = str(self._moveit_result_topic or "/desired_grasp/result")
            wall_ts = float(self._moveit_result_wall)
        self.log.emit(
            "[PICK_OBJ][RX_RESULT] "
            f"ts={wall_ts:.6f} req_id={req_id} req_uuid={req_uuid or 'n/a'} "
            f"success={success} match=unknown accepted=queued reason={parse_status} "
            f"topic={topic} seq={seq}"
        )

    def _on_moveit_bridge_heartbeat(self, _msg: "Bool") -> None:
        with self._lock:
            self._moveit_bridge_hb_wall = _steady_time()
            self._moveit_bridge_hb_event.set()

    def _resolve_ros_message_class(self, type_name: str):
        normalized = (type_name or "").strip()
        if not normalized or ros_get_message is None:
            return None
        candidates = [normalized]
        if "/msg/" in normalized:
            candidates.append(normalized.replace("/msg/", ".msg."))
        elif ".msg." in normalized:
            candidates.append(normalized.replace(".msg.", "/msg/"))
        for candidate in candidates:
            try:
                return ros_get_message(candidate)
            except Exception:
                continue
        return None

    def _extract_grasp_rect(self, msg: object) -> Optional[Dict[str, float]]:
        # vision_msgs/BoundingBox2D contract.
        try:
            center = getattr(msg, "center", None)
            position = getattr(center, "position", None) if center is not None else None
            if position is not None and hasattr(msg, "size_x") and hasattr(msg, "size_y"):
                cx = float(getattr(position, "x"))
                cy = float(getattr(position, "y"))
                width = float(getattr(msg, "size_x"))
                height = float(getattr(msg, "size_y"))
                theta_rad = float(getattr(center, "theta", 0.0))
                return {
                    "cx": cx,
                    "cy": cy,
                    "w": width,
                    "h": height,
                    "theta_rad": theta_rad,
                    "angle_deg": math.degrees(theta_rad),
                }
        except Exception:
            pass
        # Generic custom message contract: (c_x, c_y, w, h, theta).
        try:
            if all(hasattr(msg, attr) for attr in ("c_x", "c_y", "w", "h", "theta")):
                theta_rad = float(getattr(msg, "theta", 0.0))
                return {
                    "cx": float(getattr(msg, "c_x")),
                    "cy": float(getattr(msg, "c_y")),
                    "w": float(getattr(msg, "w")),
                    "h": float(getattr(msg, "h")),
                    "theta_rad": theta_rad,
                    "angle_deg": math.degrees(theta_rad),
                }
        except Exception:
            pass
        # Fallback: std_msgs/Float32MultiArray style [cx, cy, w, h, theta].
        try:
            data = list(getattr(msg, "data", []))
            if len(data) >= 5:
                theta_rad = float(data[4])
                return {
                    "cx": float(data[0]),
                    "cy": float(data[1]),
                    "w": float(data[2]),
                    "h": float(data[3]),
                    "theta_rad": theta_rad,
                    "angle_deg": math.degrees(theta_rad),
                }
        except Exception:
            pass
        return None

    def _on_grasp_rect(self, msg: object) -> None:
        payload = self._extract_grasp_rect(msg)
        if not payload:
            return
        now = _steady_time()
        payload["ts_wall"] = now
        with self._lock:
            payload["topic"] = self._grasp_rect_topic
            payload["msg_type"] = self._grasp_rect_type
            self._grasp_rect_last_wall = now
            self._grasp_rect_count += 1
        try:
            self.grasp_rect.emit(payload)
        except RuntimeError:
            pass

    def _emit_camera_disconnect_request(self, source: str) -> None:
        try:
            self.camera_disconnect_request.emit(source)
        except RuntimeError:
            pass

    def _emit_camera_connect_request(self, source: str) -> None:
        try:
            self.camera_connect_request.emit(source)
        except RuntimeError:
            pass

    def _emit_recover_request(self, source: str) -> None:
        try:
            self.recover_request.emit(source)
        except RuntimeError:
            pass

    def _emit_tfm_infer_request(self, source: str) -> None:
        try:
            self.tfm_infer_request.emit(source)
        except RuntimeError:
            pass

    def _emit_tfm_execute_request(self, source: str) -> None:
        try:
            self.tfm_execute_request.emit(source)
        except RuntimeError:
            pass

    def _emit_pick_object_request(self, source: str) -> None:
        try:
            self.pick_object_request.emit(source)
        except RuntimeError:
            pass

    def _emit_pick_demo_request(self, source: str) -> None:
        try:
            self.pick_demo_request.emit(source)
        except RuntimeError:
            pass

    def _emit_object_select_request(self, name: str, source: str) -> None:
        try:
            self.object_select_request.emit(name, source)
        except RuntimeError:
            pass

    def _on_camera_disconnect_service(self, _request: object, response: object) -> object:
        self._emit_camera_disconnect_request(f"service:{self._camera_disconnect_service}")
        try:
            response.success = True
            response.message = "camera_disconnect_triggered"
        except Exception:
            pass
        return response

    def _on_camera_connect_service(self, _request: object, response: object) -> object:
        self._emit_camera_connect_request(f"service:{self._camera_connect_service}")
        try:
            response.success = True
            response.message = "camera_connect_triggered"
        except Exception:
            pass
        return response

    def _on_recover_topic(self, _msg: "Empty") -> None:
        self._emit_recover_request(f"topic:{self._recover_topic}")

    def _on_recover_service(self, _request: object, response: object) -> object:
        self._emit_recover_request(f"service:{self._recover_service}")
        try:
            response.success = True
            response.message = "recover_triggered"
        except Exception:
            pass
        return response

    def _on_tfm_infer_topic(self, _msg: "Empty") -> None:
        self._emit_tfm_infer_request(f"topic:{self._tfm_infer_topic}")

    def _on_tfm_infer_service(self, _request: object, response: object) -> object:
        request_id = f"tfminfer-{time.monotonic_ns()}"
        done = threading.Event()
        result: Dict[str, object] = {
            "success": False,
            "message": "timeout esperando confirmación del panel",
        }
        with self._lock:
            self._tfm_infer_pending[request_id] = (done, result)
        self._safe_log(
            f"[ROS][TFM_INFER][SERVICE] request_id={request_id} waiting_ack=true"
        )
        self._emit_tfm_infer_request(f"service:{self._tfm_infer_service}#{request_id}")
        ack_ok = done.wait(timeout=max(0.1, self._tfm_infer_timeout_sec))
        with self._lock:
            _pending = self._tfm_infer_pending.pop(request_id, None)
        if _pending is not None:
            _done, payload = _pending
            result = payload
        self._safe_log(
            "[ROS][TFM_INFER][SERVICE] "
            f"request_id={request_id} ack={str(bool(ack_ok)).lower()} "
            f"success={str(bool(result.get('success', False))).lower()} "
            f"message={str(result.get('message', '') or 'n/a')}"
        )
        try:
            response.success = bool(result.get("success", False))
            response.message = str(result.get("message", "") or "")
        except Exception:
            pass
        return response

    def _on_tfm_execute_topic(self, _msg: "Empty") -> None:
        self._emit_tfm_execute_request(f"topic:{self._tfm_execute_topic}")

    def _on_tfm_execute_service(self, _request: object, response: object) -> object:
        request_id = f"tfmexec-{time.monotonic_ns()}"
        done = threading.Event()
        result: Dict[str, object] = {
            "success": False,
            "message": "timeout esperando confirmación del panel",
        }
        with self._lock:
            self._tfm_execute_pending[request_id] = (done, result)
        self._safe_log(
            f"[ROS][TFM_EXECUTE][SERVICE] request_id={request_id} waiting_ack=true"
        )
        self._emit_tfm_execute_request(f"service:{self._tfm_execute_service}#{request_id}")
        ack_ok = done.wait(timeout=max(0.1, self._tfm_execute_timeout_sec))
        with self._lock:
            _pending = self._tfm_execute_pending.pop(request_id, None)
        if _pending is not None:
            _done, payload = _pending
            result = payload
        self._safe_log(
            "[ROS][TFM_EXECUTE][SERVICE] "
            f"request_id={request_id} ack={str(bool(ack_ok)).lower()} "
            f"success={str(bool(result.get('success', False))).lower()} "
            f"message={str(result.get('message', '') or 'n/a')}"
        )
        try:
            response.success = bool(result.get("success", False))
            response.message = str(result.get("message", "") or "")
        except Exception:
            pass
        return response

    def _on_pick_object_topic(self, _msg: "Empty") -> None:
        self._emit_pick_object_request(f"topic:{self._pick_object_topic}")

    def _on_pick_object_service(self, _request: object, response: object) -> object:
        self._emit_pick_object_request(f"service:{self._pick_object_service}")
        try:
            response.success = True
            response.message = "pick_object_triggered"
        except Exception:
            pass
        return response

    def _on_pick_demo_service(self, _request: object, response: object) -> object:
        request_id = f"pickdemo-{time.monotonic_ns()}"
        done = threading.Event()
        result: Dict[str, object] = {
            "success": False,
            "message": "timeout esperando confirmación del panel",
        }
        with self._lock:
            self._pick_demo_pending[request_id] = (done, result)
        self._safe_log(
            f"[ROS][PICK_DEMO][SERVICE] request_id={request_id} waiting_ack=true"
        )
        self._emit_pick_demo_request(f"service:{self._pick_demo_service}#{request_id}")
        ack_ok = done.wait(timeout=max(0.1, self._pick_demo_timeout_sec))
        with self._lock:
            _pending = self._pick_demo_pending.pop(request_id, None)
        if _pending is not None:
            _done, payload = _pending
            result = payload
        self._safe_log(
            "[ROS][PICK_DEMO][SERVICE] "
            f"request_id={request_id} ack={str(bool(ack_ok)).lower()} "
            f"success={str(bool(result.get('success', False))).lower()} "
            f"message={str(result.get('message', '') or 'n/a')}"
        )
        try:
            response.success = bool(result.get("success", False))
            response.message = str(result.get("message", "") or "")
        except Exception:
            pass
        return response

    def _on_select_object_topic(self, msg: "String") -> None:
        try:
            name = str(getattr(msg, "data", "") or "").strip()
        except Exception:
            name = ""
        self._emit_object_select_request(name, f"topic:{self._select_object_topic}")

    def _on_select_object_service(self, request: object, response: object) -> object:
        try:
            name = str(getattr(request, "name", "") or "").strip()
        except Exception:
            name = ""
        request_id = f"objsel-{time.monotonic_ns()}"
        done = threading.Event()
        result: Dict[str, object] = {
            "success": False,
            "message": "timeout esperando confirmación del panel",
        }
        with self._lock:
            self._object_select_pending[request_id] = (done, result)
        self._safe_log(
            f"[ROS][SELECT_OBJECT][SERVICE] request_id={request_id} name={name or 'clear'} waiting_ack=true"
        )
        self._emit_object_select_request(name, f"service:{self._select_object_service}#{request_id}")
        wait_timeout = max(0.1, self._object_select_timeout_sec)
        remote_wait_timeout = max(
            2.0,
            _get_panel_ros_params().remote_select_on_table_wait_sec + 5.0,
        )
        ack_ok = done.wait(timeout=max(wait_timeout, remote_wait_timeout))
        with self._lock:
            _pending = self._object_select_pending.pop(request_id, None)
        if _pending is not None:
            _done, payload = _pending
            result = payload
        self._safe_log(
            "[ROS][SELECT_OBJECT][SERVICE] "
            f"request_id={request_id} ack={str(bool(ack_ok)).lower()} "
            f"success={str(bool(result.get('success', False))).lower()} "
            f"message={str(result.get('message', '') or 'n/a')}"
        )
        try:
            response.success = bool(result.get("success", False))
            response.message = str(result.get("message", "") or "")
        except Exception:
            pass
        return response

    def complete_pick_demo_request(self, request_id: str, success: bool, message: str) -> None:
        req_id = str(request_id or "").strip()
        if not req_id:
            return
        with self._lock:
            pending = self._pick_demo_pending.get(req_id)
        if pending is None:
            self._safe_log(
                f"[ROS][PICK_DEMO][ACK] request_id={req_id} pending=false success={str(bool(success)).lower()} message={message or 'n/a'}"
            )
            return
        done, payload = pending
        payload["success"] = bool(success)
        payload["message"] = str(message or "")
        self._safe_log(
            f"[ROS][PICK_DEMO][ACK] request_id={req_id} pending=true success={str(bool(success)).lower()} message={message or 'n/a'}"
        )
        done.set()

    def complete_object_select_request(self, request_id: str, success: bool, message: str) -> None:
        req_id = str(request_id or "").strip()
        if not req_id:
            return
        with self._lock:
            pending = self._object_select_pending.get(req_id)
        if pending is None:
            self._safe_log(
                f"[ROS][SELECT_OBJECT][ACK] request_id={req_id} pending=false success={str(bool(success)).lower()} message={message or 'n/a'}"
            )
            return
        done, payload = pending
        payload["success"] = bool(success)
        payload["message"] = str(message or "")
        self._safe_log(
            f"[ROS][SELECT_OBJECT][ACK] request_id={req_id} pending=true success={str(bool(success)).lower()} message={message or 'n/a'}"
        )
        done.set()

    def complete_tfm_infer_request(self, request_id: str, success: bool, message: str) -> None:
        req_id = str(request_id or "").strip()
        if not req_id:
            return
        with self._lock:
            pending = self._tfm_infer_pending.get(req_id)
        if pending is None:
            self._safe_log(
                f"[ROS][TFM_INFER][ACK] request_id={req_id} pending=false success={str(bool(success)).lower()} message={message or 'n/a'}"
            )
            return
        done, payload = pending
        payload["success"] = bool(success)
        payload["message"] = str(message or "")
        self._safe_log(
            f"[ROS][TFM_INFER][ACK] request_id={req_id} pending=true success={str(bool(success)).lower()} message={message or 'n/a'}"
        )
        done.set()

    def complete_tfm_execute_request(self, request_id: str, success: bool, message: str) -> None:
        req_id = str(request_id or "").strip()
        if not req_id:
            return
        with self._lock:
            pending = self._tfm_execute_pending.get(req_id)
        if pending is None:
            self._safe_log(
                f"[ROS][TFM_EXECUTE][ACK] request_id={req_id} pending=false success={str(bool(success)).lower()} message={message or 'n/a'}"
            )
            return
        done, payload = pending
        payload["success"] = bool(success)
        payload["message"] = str(message or "")
        self._safe_log(
            f"[ROS][TFM_EXECUTE][ACK] request_id={req_id} pending=true success={str(bool(success)).lower()} message={message or 'n/a'}"
        )
        done.set()

    def _on_pose_info(self, msg: "TFMessage") -> None:
        if not ROS_AVAILABLE:
            return
        try:
            transforms = getattr(msg, "transforms", [])
        except Exception:
            transforms = []
        data: Dict[str, Tuple[float, float, float]] = {}
        for tf in transforms:
            name = getattr(tf, "child_frame_id", "") or ""
            if not name:
                header = getattr(tf, "header", None)
                frame = getattr(header, "frame_id", "") if header else ""
                if frame and frame not in ("world", "/world"):
                    name = frame
            t = getattr(tf, "transform", None)
            if not t or not getattr(t, "translation", None):
                continue
            tr = t.translation
            if name:
                data[name] = (float(tr.x), float(tr.y), float(tr.z))
                if "::" in name:
                    base = name.split("::")[0]
                    data.setdefault(base, (float(tr.x), float(tr.y), float(tr.z)))
        if not transforms:
            return
        with self._lock:
            if data:
                self._pose_cache.update(data)
                self._pose_last_entities = len(data)
                self._pose_info_empty_logged = False
            else:
                # Accept pose/info heartbeat even if frame names are missing.
                self._pose_last_entities = 0
                if not self._pose_info_empty_logged and transforms:
                    sample = transforms[0]
                    header = getattr(sample, "header", None)
                    frame_id = getattr(header, "frame_id", "") if header else ""
                    child_id = getattr(sample, "child_frame_id", "") or ""
                    self.log.emit(
                        f"[PHYSICS][POSE_INFO][DIAG] TFMessage sin nombres: frame_id={frame_id or 'n/a'} child_frame_id={child_id or 'n/a'}"
                    )
                    self._pose_info_empty_logged = True
            self._pose_last_wall = _steady_time()
            self._pose_msg_count += 1
        self._pose_event.set()

    def pose_snapshot(self) -> Tuple[Dict[str, Tuple[float, float, float]], float]:
        with self._lock:
            return dict(self._pose_cache), float(self._pose_last_wall)

    def pose_info_status(self) -> Tuple[int, float]:
        """Return pose/info message count and last age (seconds)."""
        with self._lock:
            last = float(self._pose_last_wall)
            count = int(self._pose_msg_count)
        age = _steady_time() - last if last else float("inf")
        return count, age

    def pose_info_details(self) -> Tuple[int, float, int, str]:
        """Return pose/info count, last age, entities count, and topic."""
        with self._lock:
            last = float(self._pose_last_wall)
            count = int(self._pose_msg_count)
            entities = int(self._pose_last_entities)
            topic = str(self._pose_topic)
        age = _steady_time() - last if last else float("inf")
        return count, age, entities, topic

    def wait_for_pose_info(self, timeout_sec: float) -> bool:
        if self.pose_info_status()[0] > 0:
            return True
        self._pose_event.clear()
        return self._pose_event.wait(timeout_sec)

    def publish_empty(self, topic: str) -> bool:
        if not ROS_AVAILABLE or Empty is None:
            return False
        topic = (topic or "").strip()
        if not topic:
            return False
        with self._lock:
            if not self._node:
                return False
            pub = self._pubs.get(topic)
            if pub is None:
                try:
                    pub = self._node.create_publisher(Empty, topic, 10)
                    self._pubs[topic] = pub
                except Exception:
                    return False
        try:
            pub.publish(Empty())
            return True
        except Exception:
            return False
    def get_last_joint_state(self) -> Tuple[Optional[dict], float]:
        with self._lock:
            return self._last_joint_payload, self._last_joint_wall

    def joint_state_valid(self, timeout_sec: float = 2.0) -> Tuple[bool, str]:
        """
        FASE 2: Validar que tenemos JointState válido antes de enviar a MoveIt.
        Evita "Found empty JointState message" al inicio.
        """
        payload, wall = self.get_last_joint_state()
        if payload is None:
            return False, "no_joint_state_received"

        joint_names = payload.get("name", [])
        positions = payload.get("position", [])

        # Validar que joint_names no esté vacío
        if not joint_names or len(joint_names) == 0:
            return False, "empty_joint_names"

        # Validar que positions tenga el mismo tamaño que joint_names
        if not positions or len(positions) != len(joint_names):
            return False, f"position_mismatch:names={len(joint_names)},pos={len(positions)}"

        # Validar que el mensaje no sea muy antiguo
        age = _steady_time() - wall
        if age > timeout_sec:
            return False, f"stale_joint_state:age={age:.2f}s"

        return True, "ok"

    def subscribe_image(self, topic: str, msg_type: str = "image") -> bool:
        if not ROS_AVAILABLE:
            return False
        topic = topic.strip()
        if not topic:
            return False
        with self._lock:
            if not self._node:
                self._safe_log("[ROS] Nodo no listo todavía.")
                return False
            if topic in self._subs:
                try:
                    self._node.destroy_subscription(self._subs[topic])
                except Exception as exc:
                    self._log_exception("destroy image subscription", exc)
                self._subs.pop(topic, None)
            try:
                normalized = (msg_type or "").strip().lower()
                callback = self._on_image
                msg_cls = Image
                if "compressed" in normalized:
                    try:
                        from sensor_msgs.msg import CompressedImage

                        msg_cls = CompressedImage
                        callback = self._on_compressed_image
                    except Exception:
                        msg_cls = Image
                        callback = self._on_image
                kwargs = {}
                if self._io_callback_group is not None:
                    kwargs["callback_group"] = self._io_callback_group
                sub = self._node.create_subscription(
                    msg_cls,
                    topic,
                    lambda msg, t=topic: callback(msg, t),
                    qos_profile_sensor_data,
                    **kwargs,
                )
                self._subs[topic] = sub
                self._fps[topic] = [0.0, _steady_time(), 0.0]
                self._image_msg_count[topic] = 0
                self._image_decode_error_logged[topic] = False
                qos_summary = self._format_qos(getattr(sub, "qos_profile", None))
                self._safe_log(f"[ROS] Suscrito a {topic} ({qos_summary})")
                return True
            except Exception as exc:
                self._safe_log(f"[ROS] ERROR suscribiendo {topic}: {exc}")
                return False
        return False

    def unsubscribe_image(self, topic: str):
        if not ROS_AVAILABLE:
            return
        topic = topic.strip()
        with self._lock:
            if self._node and topic in self._subs:
                try:
                    self._node.destroy_subscription(self._subs[topic])
                except Exception as exc:
                    self._log_exception("destroy image subscription", exc)
                self._subs.pop(topic, None)
                self._safe_log(f"[ROS] Unsubscribe {topic}")

    def subscribe_joint_states(self, topic: str = "/joint_states"):
        if not ROS_AVAILABLE:
            return
        topic = (topic or "").strip()
        if not topic:
            return
        with self._lock:
            if not self._node:
                self._safe_log("[ROS] Nodo no listo todavia.")
                return
            if self._joint_sub is not None:
                try:
                    self._node.destroy_subscription(self._joint_sub)
                except Exception as exc:
                    self._log_exception("destroy joint_states subscription", exc)
                self._joint_sub = None
            try:
                self._joint_topic = topic
                # joint_state_broadcaster publica con RELIABLE (rclcpp::QoS(1)).
                # BEST_EFFORT subscriber puede ser incompatible en algunos RMW.
                _js_qos = 10  # RELIABLE VOLATILE KEEP_LAST(10)
                self._joint_sub = self._node.create_subscription(
                    JointState,
                    topic,
                    lambda msg, t=topic: self._on_joint_state(msg, t),
                    _js_qos,
                    callback_group=self._io_callback_group,
                )
                self._safe_log(f"[ROS] Suscrito a {topic} (joint_states, qos=RELIABLE)")
            except Exception as exc:
                self._safe_log(f"[ROS] ERROR suscribiendo joint_states: {exc}")

    def unsubscribe_joint_states(self):
        if not ROS_AVAILABLE:
            return
        with self._lock:
            if self._node and self._joint_sub is not None:
                try:
                    self._node.destroy_subscription(self._joint_sub)
                except Exception as exc:
                    self._log_exception("destroy joint_states subscription", exc)
                self._joint_sub = None
                self._joint_topic = ""
                self._safe_log("[ROS] Unsubscribe joint_states")

    def _format_qos(self, profile: Optional["QoSProfile"]) -> str:
        """Describe un perfil QoS para logging."""
        if profile is None:
            return "QoS=default"
        reliability = self._reliability_name(getattr(profile, "reliability", None))
        durability = self._durability_name(getattr(profile, "durability", None))
        history = self._history_name(getattr(profile, "history", None))
        depth = getattr(profile, "depth", None)
        depth_txt = f"depth={depth}" if depth is not None else "depth=?"
        return f"{reliability}/{durability}/{history}@{depth_txt}"

    def _reliability_name(self, value: Optional[int]) -> str:
        if value is None:
            return "reliability=?"
        if ReliabilityPolicy is not None:
            if value == ReliabilityPolicy.RELIABLE:
                return "RELIABLE"
            if value == ReliabilityPolicy.BEST_EFFORT:
                return "BEST_EFFORT"
        return str(value)

    def _durability_name(self, value: Optional[int]) -> str:
        if value is None:
            return "durability=?"
        if DurabilityPolicy is not None:
            if value == DurabilityPolicy.VOLATILE:
                return "VOLATILE"
            if value == DurabilityPolicy.TRANSIENT_LOCAL:
                return "TRANSIENT_LOCAL"
        return str(value)

    def _history_name(self, value: Optional[int]) -> str:
        if value is None:
            return "history=?"
        if HistoryPolicy is not None:
            if value == HistoryPolicy.KEEP_LAST:
                return "KEEP_LAST"
            if value == HistoryPolicy.KEEP_ALL:
                return "KEEP_ALL"
        return str(value)

    def _thread_main(self):
        if self._force_realtime:
            use_sim_time = False
        else:
            use_sim_time = os.environ.get("USE_SIM_TIME", "1") == "1"
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
        except Exception:
            try:
                rclpy.init(args=None)
            except Exception as exc:
                try:
                    self.log.emit(f"[ROS] ERROR rclpy.init: {exc}")
                except RuntimeError:
                    return
                return
        try:
            overrides = None
            if Parameter is not None:
                overrides = [
                    Parameter("use_sim_time", Parameter.Type.BOOL, bool(use_sim_time))
                ]
            node_kwargs = {}
            if overrides:
                node_kwargs["parameter_overrides"] = overrides
                node_kwargs["automatically_declare_parameters_from_overrides"] = True
            try:
                self._node = rclpy.create_node("panel_superpro", **node_kwargs)
            except TypeError:
                node_kwargs.pop("automatically_declare_parameters_from_overrides", None)
                self._node = rclpy.create_node("panel_superpro", **node_kwargs)
            try:
                effective_sim_time = bool(use_sim_time)
                if not self._node.has_parameter("use_sim_time"):
                    self._node.declare_parameter("use_sim_time", effective_sim_time)
                current_sim_time = bool(self._node.get_parameter("use_sim_time").value)
                if current_sim_time != effective_sim_time and Parameter is not None:
                    self._node.set_parameters([Parameter("use_sim_time", value=effective_sim_time)])
                    current_sim_time = bool(self._node.get_parameter("use_sim_time").value)
                self.log.emit(
                    "[ROS][SIM_TIME] "
                    f"requested={str(bool(use_sim_time)).lower()} "
                    f"effective={str(bool(current_sim_time)).lower()} "
                    f"force_realtime={str(bool(self._force_realtime)).lower()}"
                )
            except Exception as exc:
                self.log.emit(f"[ROS][SIM_TIME] WARN no se pudo confirmar use_sim_time: {exc}")
            self._bridge = CvBridge()
            if ReentrantCallbackGroup is not None:
                self._service_callback_group = ReentrantCallbackGroup()
                self._io_callback_group = ReentrantCallbackGroup()
            executor_threads = max(1, _get_panel_ros_params().ros_executor_threads)
            if MultiThreadedExecutor is not None and executor_threads > 1:
                self._exec = MultiThreadedExecutor(num_threads=executor_threads)
                self.log.emit(
                    f"[ROS] Executor multi-hilo activo (threads={executor_threads})"
                )
            else:
                self._exec = SingleThreadedExecutor()
                self.log.emit("[ROS] Executor monohilo activo")
            self._exec.add_node(self._node)
            self._node.create_subscription(
                Clock,
                "/clock",
                self._update_clock,
                qos_profile_sensor_data,
                callback_group=self._io_callback_group,
            )
            if String is not None:
                try:
                    self._subs["/system_state"] = self._node.create_subscription(
                        String,
                        "/system_state",
                        self._on_system_state,
                        10,
                        callback_group=self._io_callback_group,
                    )
                    self._subs["/system_diag"] = self._node.create_subscription(
                        String,
                        "/system_diag",
                        self._on_system_diag,
                        10,
                        callback_group=self._io_callback_group,
                    )
                except Exception:
                    pass
            if Empty is not None:
                try:
                    self._tfm_infer_sub = self._node.create_subscription(
                        Empty,
                        self._tfm_infer_topic,
                        self._on_tfm_infer_topic,
                        10,
                    )
                    self.log.emit(
                        f"[ROS] Trigger topic listo: {self._tfm_infer_topic} (std_msgs/Empty)"
                    )
                except Exception as exc:
                    self._log_exception("create tfm_infer topic", exc)
                try:
                    self._tfm_execute_sub = self._node.create_subscription(
                        Empty,
                        self._tfm_execute_topic,
                        self._on_tfm_execute_topic,
                        10,
                    )
                    self.log.emit(
                        f"[ROS] Trigger topic listo: {self._tfm_execute_topic} (std_msgs/Empty)"
                    )
                except Exception as exc:
                    self._log_exception("create tfm_execute topic", exc)
                try:
                    self._pick_object_sub = self._node.create_subscription(
                        Empty,
                        self._pick_object_topic,
                        self._on_pick_object_topic,
                        10,
                    )
                    self.log.emit(
                        f"[ROS] Trigger topic listo: {self._pick_object_topic} (std_msgs/Empty)"
                    )
                except Exception as exc:
                    self._log_exception("create pick_object topic", exc)
                try:
                    self._recover_sub = self._node.create_subscription(
                        Empty,
                        self._recover_topic,
                        self._on_recover_topic,
                        10,
                    )
                    self.log.emit(
                        f"[ROS] Trigger topic listo: {self._recover_topic} (std_msgs/Empty)"
                    )
                except Exception as exc:
                    self._log_exception("create recover topic", exc)
            if String is not None:
                try:
                    self._select_object_sub = self._node.create_subscription(
                        String,
                        self._select_object_topic,
                        self._on_select_object_topic,
                        10,
                    )
                    self.log.emit(
                        f"[ROS] Trigger topic listo: {self._select_object_topic} (std_msgs/String)"
                    )
                except Exception as exc:
                    self._log_exception("create select_object topic", exc)
            if Trigger is not None:
                try:
                    self._camera_connect_srv = self._node.create_service(
                        Trigger,
                        self._camera_connect_service,
                        self._on_camera_connect_service,
                        callback_group=self._service_callback_group,
                    )
                    self.log.emit(
                        f"[ROS] Trigger service listo: {self._camera_connect_service} (std_srvs/Trigger)"
                    )
                except Exception as exc:
                    self._log_exception("create camera_connect service", exc)
                try:
                    self._camera_disconnect_srv = self._node.create_service(
                        Trigger,
                        self._camera_disconnect_service,
                        self._on_camera_disconnect_service,
                        callback_group=self._service_callback_group,
                    )
                    self.log.emit(
                        f"[ROS] Trigger service listo: {self._camera_disconnect_service} (std_srvs/Trigger)"
                    )
                except Exception as exc:
                    self._log_exception("create camera_disconnect service", exc)
                try:
                    self._tfm_infer_srv = self._node.create_service(
                        Trigger,
                        self._tfm_infer_service,
                        self._on_tfm_infer_service,
                        callback_group=self._service_callback_group,
                    )
                    self.log.emit(
                        f"[ROS] Trigger service listo: {self._tfm_infer_service} (std_srvs/Trigger)"
                    )
                except Exception as exc:
                    self._log_exception("create tfm_infer service", exc)
                try:
                    self._tfm_execute_srv = self._node.create_service(
                        Trigger,
                        self._tfm_execute_service,
                        self._on_tfm_execute_service,
                        callback_group=self._service_callback_group,
                    )
                    self.log.emit(
                        f"[ROS] Trigger service listo: {self._tfm_execute_service} (std_srvs/Trigger)"
                    )
                except Exception as exc:
                    self._log_exception("create tfm_execute service", exc)
                try:
                    self._recover_srv = self._node.create_service(
                        Trigger,
                        self._recover_service,
                        self._on_recover_service,
                        callback_group=self._service_callback_group,
                    )
                    self.log.emit(
                        f"[ROS] Trigger service listo: {self._recover_service} (std_srvs/Trigger)"
                    )
                except Exception as exc:
                    self._log_exception("create recover service", exc)
                try:
                    self._pick_demo_srv = self._node.create_service(
                        Trigger,
                        self._pick_demo_service,
                        self._on_pick_demo_service,
                        callback_group=self._service_callback_group,
                    )
                    self.log.emit(
                        f"[ROS] Trigger service listo: {self._pick_demo_service} (std_srvs/Trigger)"
                    )
                except Exception as exc:
                    self._log_exception("create pick_demo service", exc)
                try:
                    self._pick_object_srv = self._node.create_service(
                        Trigger,
                        self._pick_object_service,
                        self._on_pick_object_service,
                        callback_group=self._service_callback_group,
                    )
                    self.log.emit(
                        f"[ROS] Trigger service listo: {self._pick_object_service} (std_srvs/Trigger)"
                    )
                except Exception as exc:
                    self._log_exception("create pick_object service", exc)
            if SelectObject is not None:
                try:
                    self._select_object_srv = self._node.create_service(
                        SelectObject,
                        self._select_object_service,
                        self._on_select_object_service,
                        callback_group=self._service_callback_group,
                    )
                    self.log.emit(
                        f"[ROS] Trigger service listo: {self._select_object_service} (ur5_panel_interfaces/SelectObject)"
                    )
                except Exception as exc:
                    self._log_exception("create select_object service", exc)
            try:
                self.log.emit("[ROS] OK: nodo listo.")
            except RuntimeError:
                return
        except Exception as exc:
            try:
                self.log.emit(f"[ROS] ERROR creando nodo/executor: {exc}")
            except RuntimeError:
                return
            return
        while True:
            with self._lock:
                if not self._running:
                    break
            try:
                if not rclpy.ok():
                    break
                self._exec.spin_once(timeout_sec=0.05)
            except Exception as exc:
                if ExternalShutdownException is not None and isinstance(exc, ExternalShutdownException):
                    break
                try:
                    # Evitar emitir si el QObject ya se destruyó durante el apagado
                    if self._running:
                        self.log.emit(f"[ROS] WARN spin_once: {exc}")
                    else:
                        break
                except RuntimeError:
                    break
                time.sleep(0.1)
        # Limpieza final (evita logs si ya estamos apagando)
        self._cleanup_ros()

    def _cleanup_ros(self):
        with self._lock:
            if self._cleanup_done:
                return
            self._cleanup_done = True
        try:
            if self._node and self._subs:
                for topic, sub in list(self._subs.items()):
                    try:
                        self._node.destroy_subscription(sub)
                    except Exception as exc:
                        self._log_exception(f"destroy subscription {topic}", exc)
                self._subs.clear()
            if self._node and self._joint_sub is not None:
                try:
                    self._node.destroy_subscription(self._joint_sub)
                except Exception as exc:
                    self._log_exception("destroy joint_states subscription", exc)
                self._joint_sub = None
                self._joint_topic = ""
            if self._node and self._pose_sub is not None:
                try:
                    self._node.destroy_subscription(self._pose_sub)
                except Exception as exc:
                    self._log_exception("destroy pose subscription", exc)
                self._pose_sub = None
                self._pose_topic = ""
                self._pose_cache.clear()
                self._pose_last_wall = 0.0
                self._pose_msg_count = 0
                self._pose_last_entities = 0
            if self._node and self._moveit_result_sub is not None:
                try:
                    self._node.destroy_subscription(self._moveit_result_sub)
                except Exception as exc:
                    self._log_exception("destroy moveit result subscription", exc)
                self._moveit_result_sub = None
                self._moveit_result_topic = ""
                self._moveit_result_last = ""
                self._moveit_result_wall = 0.0
                self._moveit_result_seq = 0
                self._moveit_result_event.clear()
            if self._node and self._moveit_bridge_hb_sub is not None:
                try:
                    self._node.destroy_subscription(self._moveit_bridge_hb_sub)
                except Exception as exc:
                    self._log_exception("destroy moveit heartbeat subscription", exc)
                self._moveit_bridge_hb_sub = None
                self._moveit_bridge_hb_topic = ""
                self._moveit_bridge_hb_wall = 0.0
                self._moveit_bridge_hb_event.clear()
            if self._node and self._grasp_rect_sub is not None:
                try:
                    self._node.destroy_subscription(self._grasp_rect_sub)
                except Exception as exc:
                    self._log_exception("destroy grasp_rect subscription", exc)
                self._grasp_rect_sub = None
                self._grasp_rect_topic = ""
                self._grasp_rect_type = ""
                self._grasp_rect_last_wall = 0.0
                self._grasp_rect_count = 0
            if self._node and self._camera_connect_srv is not None:
                try:
                    self._node.destroy_service(self._camera_connect_srv)
                except Exception as exc:
                    self._log_exception("destroy camera_connect service", exc)
                self._camera_connect_srv = None
            if self._node and self._camera_disconnect_srv is not None:
                try:
                    self._node.destroy_service(self._camera_disconnect_srv)
                except Exception as exc:
                    self._log_exception("destroy camera_disconnect service", exc)
                self._camera_disconnect_srv = None
            if self._node and self._tfm_infer_sub is not None:
                try:
                    self._node.destroy_subscription(self._tfm_infer_sub)
                except Exception as exc:
                    self._log_exception("destroy tfm_infer subscription", exc)
                self._tfm_infer_sub = None
            if self._node and self._tfm_infer_srv is not None:
                try:
                    self._node.destroy_service(self._tfm_infer_srv)
                except Exception as exc:
                    self._log_exception("destroy tfm_infer service", exc)
                self._tfm_infer_srv = None
            if self._node and self._tfm_execute_sub is not None:
                try:
                    self._node.destroy_subscription(self._tfm_execute_sub)
                except Exception as exc:
                    self._log_exception("destroy tfm_execute subscription", exc)
                self._tfm_execute_sub = None
            if self._node and self._tfm_execute_srv is not None:
                try:
                    self._node.destroy_service(self._tfm_execute_srv)
                except Exception as exc:
                    self._log_exception("destroy tfm_execute service", exc)
                self._tfm_execute_srv = None
            if self._node and self._pick_demo_srv is not None:
                try:
                    self._node.destroy_service(self._pick_demo_srv)
                except Exception as exc:
                    self._log_exception("destroy pick_demo service", exc)
                self._pick_demo_srv = None
            if self._node and self._select_object_sub is not None:
                try:
                    self._node.destroy_subscription(self._select_object_sub)
                except Exception as exc:
                    self._log_exception("destroy select_object subscription", exc)
                self._select_object_sub = None
            if self._node and self._select_object_srv is not None:
                try:
                    self._node.destroy_service(self._select_object_srv)
                except Exception as exc:
                    self._log_exception("destroy select_object service", exc)
                self._select_object_srv = None
            with self._lock:
                pending = list(self._pick_demo_pending.values())
                pending.extend(self._object_select_pending.values())
                pending.extend(self._tfm_infer_pending.values())
                pending.extend(self._tfm_execute_pending.values())
                self._pick_demo_pending.clear()
                self._object_select_pending.clear()
                self._tfm_infer_pending.clear()
                self._tfm_execute_pending.clear()
            for done, payload in pending:
                payload["success"] = False
                payload["message"] = str(payload.get("message") or "shutdown")
                done.set()
            if self._node and self._pubs:
                for _topic, pub in list(self._pubs.items()):
                    try:
                        self._node.destroy_publisher(pub)
                    except Exception as exc:
                        self._log_exception("destroy publisher", exc)
                self._pubs.clear()
            if self._exec and self._node:
                try:
                    self._exec.remove_node(self._node)
                except Exception as exc:
                    self._log_exception("remove node from executor", exc)
            if self._node:
                try:
                    self._node.destroy_node()
                except Exception as exc:
                    self._log_exception("destroy node", exc)
        except Exception:
            pass
        self._node = None
        self._exec = None
        self._bridge = None

    def _update_clock(self, msg: "Clock") -> None:
        now = _steady_time()
        try:
            stamp_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
        except Exception:
            stamp_ns = None
        with self._lock:
            self._last_clock_wall = now
            if stamp_ns is None:
                return
            if self._last_clock_stamp_ns is None:
                self._last_clock_stamp_ns = stamp_ns
                return
            if stamp_ns > self._last_clock_stamp_ns:
                self._last_clock_advance_wall = now
                self._clock_event.set()
            self._last_clock_stamp_ns = stamp_ns

    def clock_alive(self) -> Tuple[bool, float]:
        with self._lock:
            if self._last_clock_wall <= 0.0:
                return False, float("inf")
            age = _steady_time() - self._last_clock_wall
            if self._last_clock_advance_wall <= 0.0:
                return False, float("inf")
            advance_age = _steady_time() - self._last_clock_advance_wall
            ok = age < CLOCK_MAX_AGE_SEC and advance_age < CLOCK_MAX_AGE_SEC
            return ok, max(age, advance_age)

    def wait_for_clock(self, timeout_sec: float) -> bool:
        ok, _age = self.clock_alive()
        if ok:
            return True
        if not self._clock_event.wait(timeout_sec):
            return False
        return self.clock_alive()[0]

    def ros_now_ns(self) -> int:
        """Return the worker node ROS time in ns; 0 means sim clock is not valid yet."""
        with self._lock:
            node = self._node
            fallback = int(self._last_clock_stamp_ns or 0)
        if node is not None:
            try:
                return int(node.get_clock().now().nanoseconds or 0)
            except Exception:
                pass
        return fallback

    def wait_for_valid_ros_time(self, timeout_sec: float) -> bool:
        """Wait until node.get_clock().now().nanoseconds > 0."""
        deadline = _steady_time() + max(0.1, float(timeout_sec))
        while _steady_time() < deadline:
            if self.ros_now_ns() > 0:
                return True
            remaining = max(0.0, min(0.1, deadline - _steady_time()))
            self._clock_event.wait(remaining)
        return self.ros_now_ns() > 0

    def node_ready(self) -> bool:
        with self._lock:
            return self._node is not None

    def is_worker_running(self) -> bool:
        """Devuelve True si el bucle de spin sigue activo."""
        with self._lock:
            return bool(self._running)

    def _on_joint_state(self, msg: "JointState", topic: str):
        if not ROS_AVAILABLE:
            return

        now = _steady_time()
        wall_now = time.time()  # reloj de pared para checks de frescura en _wait_for_joint_target

        # Construir payload
        stamp = None
        try:
            if msg.header and (msg.header.stamp.sec or msg.header.stamp.nanosec):
                stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        except Exception:
            stamp = None

        payload = {
            "topic": topic,
            "stamp": stamp or now,
            "source": "ros",
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        }

        # Guardar SIEMPRE para el consumo interno del panel
        with self._lock:
            self._last_joint_payload = payload
            # Usamos reloj de pared (`time.time()`) para que _wait_for_joint_target
            # calcule bien state_age_sec = time.time() - ts. Si aqui se usara
            # time.monotonic(), apareceria un desfase enorme frente a time.time(),
            # state_age_sec saldria siempre gigante y _wait_for_joint_target
            # acabaria cayendo en _last_joint_time (camino de senal Qt), que se
            # queda viejo cuando el hilo principal de Qt se bloquea reconectando
            # camaras. Eso provocaba timeouts en HOME/MESA aunque el robot ya
            # estuviera en posicion.
            self._last_joint_wall = wall_now
            should_emit = (now - self._last_joint_emit) >= self._joint_emit_interval
            if should_emit:
                self._last_joint_emit = now

        if should_emit:
            try:
                self.joint_state.emit(payload)
            except RuntimeError:
                pass

    def _on_system_diag(self, msg: "String") -> None:
        try:
            raw = getattr(msg, "data", "") or ""
        except Exception:
            return
        reason = ""
        if raw:
            try:
                data = json.loads(raw)
                reason = str(data.get("reason") or "")
            except Exception:
                reason = ""
        with self._lock:
            self._system_diag_reason = reason

    def _on_system_state(self, msg: "String") -> None:
        try:
            state = getattr(msg, "data", "") or ""
        except Exception:
            return
        if not state:
            return
        wall_now = _steady_time()
        with self._lock:
            reason = self._system_diag_reason
            self._system_state_last = state
            self._system_state_reason = reason
            self._system_state_wall = wall_now
        try:
            # Emitimos siempre, incluso si el estado no cambia, para que
            # _external_state_last se mantenga fresco. Si aqui deduplicamos,
            # la ventana de liveness de 2 segundos se quedaria sin alimento y
            # bloquearia la entrada en ready del modo gestionado.
            self.system_state.emit(state, reason or "")
        except RuntimeError:
            pass

    def _decode_depth_to_bgr(self, cv_depth, topic: str):
        import numpy as np
        import cv2
        arr = cv_depth.astype("float32")
        arr[~np.isfinite(arr)] = 0.0
        cache = self._depth_cache.get(topic)
        lo = hi = None
        frame_idx = 1
        if cache:
            lo, hi, frame_idx = cache
            frame_idx += 1
        refresh = DEPTH_PCTL_REFRESH_FRAMES <= 1 or cache is None or (
            frame_idx % DEPTH_PCTL_REFRESH_FRAMES == 0
        )
        if refresh:
            stride = max(1, int(DEPTH_PCTL_STRIDE))
            sample = arr[::stride, ::stride]
            sample = sample[sample > 0.0]
            if sample.size:
                if DEPTH_FAST:
                    lo = float(sample.min())
                    hi = float(sample.max())
                else:
                    lo = float(np.percentile(sample, 1.0))
                    hi = float(np.percentile(sample, 99.0))
            else:
                lo = 0.0
                hi = 1.0
        if lo is None or hi is None:
            lo = 0.0
            hi = 1.0
        self._depth_cache[topic] = (lo, hi, frame_idx)
        if hi <= lo:
            hi = lo + 1e-3
        arr = (arr - lo) / (hi - lo)
        arr = np.clip(arr, 0.0, 1.0)
        u8 = (arr * 255.0).astype("uint8")
        return cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)

    def _on_image(self, msg: "Image", topic: str):
        if not ROS_AVAILABLE or not self._bridge:
            return
        now = _steady_time()
        last = self._last_emit.get(topic, 0.0)
        if (now - last) < self._min_emit_interval:
            return
        try:
            import cv2
            import numpy as np

            enc = (msg.encoding or "").lower()
            self._image_msg_count[topic] = self._image_msg_count.get(topic, 0) + 1
            if self._image_msg_count.get(topic, 0) == 1:
                self._safe_log(
                    f"[ROS][CAMERA] first_msg topic={topic} size={int(msg.width)}x{int(msg.height)} enc={enc or 'n/a'} step={int(msg.step)}"
                )
            if any(flag in enc for flag in ("32fc1", "16uc1", "mono16")) or "depth" in topic:
                try:
                    cv_depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                except Exception:
                    cv_depth = self._bridge.imgmsg_to_cv2(msg)
                if cv_depth.ndim == 3:
                    cv_depth = cv_depth[:, :, 0]
                with self._lock:
                    self._latest_depth_frame[topic] = (np.ascontiguousarray(cv_depth), now)
                bgr = self._decode_depth_to_bgr(cv_depth, topic)
            else:
                w = int(msg.width)
                h = int(msg.height)
                step = int(msg.step)
                data = msg.data
                row_bytes = max(0, w * 3)
                if enc in ("rgb8", "bgr8") and step >= row_bytes and len(data) >= h * step:
                    arr = np.frombuffer(data, dtype=np.uint8).reshape((h, step))
                    arr = arr[:, :row_bytes].reshape((h, w, 3))
                    if enc == "rgb8":
                        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                    else:
                        bgr = np.ascontiguousarray(arr)
                else:
                    try:
                        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    except Exception:
                        if step == row_bytes and len(data) >= h * step:
                            arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 3))
                            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                        else:
                            bgr = self._bridge.imgmsg_to_cv2(msg)
            self._emit_frame(topic, bgr)
        except Exception as exc:
            if not self._image_decode_error_logged.get(topic, False):
                self._image_decode_error_logged[topic] = True
                self._safe_log(f"[ROS][CAMERA] decode_error topic={topic}: {exc}")
            elif DEBUG_FRAME_LOG:
                self.log.emit(f"[ROS] ERROR frame {topic}: {exc}")

    def _emit_frame(self, topic: str, bgr):
        now = _steady_time()
        last = self._last_emit.get(topic, 0.0)
        if (now - last) < self._min_emit_interval:
            return
        self._last_emit[topic] = now
        self._frame_count[topic] = self._frame_count.get(topic, 0) + 1
        try:
            import cv2
            import numpy as np

            h, w = bgr.shape[:2]
            max_size = int(CAMERA_MAX_SIZE) if CAMERA_MAX_SIZE is not None else 0
            if max_size > 0 and max(h, w) > max_size:
                scale = max_size / float(max(h, w))
                new_w = max(1, int(w * scale))
                new_h = max(1, int(h * scale))
                bgr = cv2.resize(bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)
                h, w = bgr.shape[:2]
            rec = self._fps.get(topic, [0.0, _steady_time(), 0.0])
            rec[0] += 1.0
            dt = max(1e-6, _steady_time() - rec[1])
            if dt >= 1.0:
                rec[2] = rec[0] / dt
                rec[0] = 0.0
                rec[1] = _steady_time()
                if DEBUG_FRAME_LOG:
                    frames_total = self._frame_count.get(topic, 0)
                    self.log.emit(f"[CAMERA] {topic} frames={frames_total} fps={rec[2]:.1f}")
            self._fps[topic] = rec
            fps = float(rec[2])
            use_bgr = bool(CAMERA_USE_BGR) and hasattr(QImage, "Format_BGR888")
            copy_frame = bool(CAMERA_COPY_FRAME)
            if use_bgr:
                bgr = np.ascontiguousarray(bgr)
                qimg = QImage(bgr.data, w, h, bgr.strides[0], QImage.Format_BGR888)
                if copy_frame:
                    qimg = qimg.copy()
                else:
                    self._frame_buffers[topic] = bgr
            else:
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                qimg = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888)
                if copy_frame:
                    qimg = qimg.copy()
                else:
                    self._frame_buffers[topic] = rgb
            with self._lock:
                prev = self._latest_image_snapshot.get(topic)
                prev_wall = float(prev[4]) if prev is not None else 0.0
                if prev is None or (now - prev_wall) >= 0.20:
                    self._latest_image_snapshot[topic] = (qimg.copy(), int(w), int(h), fps, now)
            if self._frame_count.get(topic, 0) == 1:
                self._safe_log(
                    f"[ROS][CAMERA] first_frame topic={topic} size={int(w)}x{int(h)} fps={float(fps):.2f}"
                )
            self.image.emit(topic, qimg, w, h, fps)
        except Exception as exc:
            if DEBUG_FRAME_LOG:
                self.log.emit(f"[ROS] ERROR frame emit {topic}: {exc}")

    def _on_compressed_image(self, msg: "CompressedImage", topic: str):
        if not ROS_AVAILABLE:
            return
        now = _steady_time()
        last = self._last_emit.get(topic, 0.0)
        if (now - last) < self._min_emit_interval:
            return
        try:
            import cv2
            import numpy as np

            arr = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None:
                raise RuntimeError("No se pudo decodificar CompressedImage")
            self._emit_frame(topic, bgr)
        except Exception as exc:
            if DEBUG_FRAME_LOG:
                self.log.emit(f"[ROS] ERROR compressed frame {topic}: {exc}")


class _TfSpinWorker(QObject):
    finished = pyqtSignal()

    def __init__(self, helper: "TfHelper"):
        super().__init__()
        self._helper = helper

    def run(self) -> None:
        self._helper._spin()
        self.finished.emit()
