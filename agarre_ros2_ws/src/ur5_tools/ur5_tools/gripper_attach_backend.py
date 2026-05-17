#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_backend.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
"""Backend de attach para topics del gripper con movimiento fisico en Gazebo.

F3.3 NOTE (2026-05-10): el módulo ya está parcialmente extraído en 6
mixins (``attach_anchor``, ``attach_demo_transport``, ``attach_gz_cli``,
``attach_pose_lookup``, ``attach_pose_sub``, ``attach_set_pose``). El
LifecycleNode ``GripperAttachBackend`` aún concentra wiring (params +
service registration + lifecycle transitions). La separación final
propuesta por F9:

  * ``gripper_attach_node.py`` (≤500 LOC): solo Lifecycle + service
    registration + delegación a mixins.
  * Mixins existentes: sin cambios estructurales.

Mientras tanto NO añadir métodos nuevos a la clase; cualquier helper
debe vivir en uno de los mixins existentes o en ``attach_math``.
"""

from __future__ import annotations

from functools import partial
import math
import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64MultiArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

# F5-step6b: services /gripper/open|close + /orchestrator/attach|detach
from ur5_panel_interfaces.srv import (
    Attach as AttachSrv,
    Close as CloseSrv,
    Detach as DetachSrv,
    Open as OpenSrv,
)

try:
    from ros_gz_interfaces.msg import Contacts as GzContacts
    from ros_gz_interfaces.msg import Entity as GzEntity
    from ros_gz_interfaces.srv import SetEntityPose
except Exception:  # pragma: no cover - optional at runtime
    GzContacts = None
    GzEntity = None
    SetEntityPose = None

from .attach_anchor import AnchorMixin
from .attach_contact_gate import ContactGateMixin
from .attach_demo_transport import DemoTransportMixin
from .attach_gz_cli import GzCliMixin
from .attach_pose_lookup import PoseLookupMixin
from .attach_pose_sub import PoseSubscriberMixin
from .attach_set_pose import SetPoseMixin
from .attach_math import (  # noqa: F401  helpers reexportados (C.1 refactor)
    _dh_transform,
    _matmul3,
    _matvec3,
    _quat_from_rot3,
    _quat_inverse,
    _quat_multiply,
    _quat_multiply_raw,
    _quat_normalize,
    _rotate_vector,
)
from .gripper_geometry import (
    RG2_PINCH_CENTER_FRAME,
    RG2_TCP_FRAME,
    TOOL0_FRAME,
    load_gripper_geometry,
)


DEFAULT_OBJECTS = [
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
]

UR5_ARM_JOINT_ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

_UR5_DH_A = (0.0, -0.425, -0.39225, 0.0, 0.0, 0.0)
_UR5_DH_D = (0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823)
_UR5_DH_ALPHA = (
    math.pi / 2.0,
    0.0,
    0.0,
    math.pi / 2.0,
    -math.pi / 2.0,
    0.0,
)
_BASE_LINK_FIX_R = (
    (-1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
    (0.0, 0.0, 1.0),
)

_GRIPPER_GEOMETRY = load_gripper_geometry()
TCP_FALLBACKS = {
    RG2_PINCH_CENTER_FRAME: (
        TOOL0_FRAME,
        _GRIPPER_GEOMETRY.xyz_for_frame(RG2_PINCH_CENTER_FRAME),
    ),
    RG2_TCP_FRAME: (
        TOOL0_FRAME,
        _GRIPPER_GEOMETRY.xyz_for_frame(RG2_TCP_FRAME),
    ),
}


# F#13-step (2026-05-08): dataclasses + helpers movidos a
# gripper_attach_models.py (testeables offline). Reexportadas aquí para
# compatibilidad con consumers que hacían `from .gripper_attach_backend import
# PoseSample` (ej. mixins atajados).
from .gripper_attach_models import (  # noqa: F401
    AttachedTarget,
    DemoTransportState,
    PoseSample,
    coherence_breach_exceeded,
    increment_breach,
    reset_breach,
)


class GripperAttachBackend(
    AnchorMixin,
    DemoTransportMixin,
    GzCliMixin,
    PoseLookupMixin,
    PoseSubscriberMixin,
    SetPoseMixin,
    ContactGateMixin,
    LifecycleNode,
):
    """Backend que mantiene los objetos adheridos siguiendo fisicamente el TCP.

    F13 (2026-05-01): migrado a ``LifecycleNode``. La inicialización
    completa permanece en ``__init__`` (subscriptions, timers, clients
    creados al constructor, igual que antes) para preservar el
    comportamiento operativo. Las transiciones lifecycle se exponen
    como ``observable`` (configure/activate retornan SUCCESS sin
    re-crear recursos). Esto permite a ``system_state_manager``
    coordinar globalmente sin reescribir el backend. Una segregación
    estricta de recursos a on_configure/on_activate queda como F13b si
    se necesita destrucción granular de timers/subs.

    El parámetro ``auto_activate`` (default True) preserva el
    comportamiento de los launch existentes.
    """

    def _init_declare_parameters(self) -> None:
        """F3-step27a: declara los 40+ parámetros de GripperAttachBackend (~64 LOC)."""
        if not self.has_parameter("auto_activate"):
            self.declare_parameter("auto_activate", True)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("gripper_prefix", "/gripper")
        self.declare_parameter("drop_anchor_prefix", "/drop_anchor")
        self.declare_parameter("tool_anchor_prefix", "/gripper_anchor")
        self.declare_parameter("object_names", DEFAULT_OBJECTS)
        self.declare_parameter("attach_mode", "detachable_joint")
        self.declare_parameter("world_name", "ur5_mesa_objetos")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("tcp_frame", "rg2_tcp")
        self.declare_parameter("set_pose_service", "")
        self.declare_parameter("follow_rate_hz", 20.0)
        self.declare_parameter("max_pose_age_sec", 1.5)
        self.declare_parameter("service_timeout_sec", 0.2)
        self.declare_parameter("set_pose_future_timeout_sec", 1.0)
        self.declare_parameter("gz_cli_fallback", True)
        self.declare_parameter("gz_service_timeout_ms", 400)
        self.declare_parameter("gz_cmd_timeout_sec", 0.9)
        # 2026-05-17 fix_follow_tcp_rate_20260517_163003:
        # set_pose_async_cli=True usa subprocess.Popen fire-and-forget para
        # invocar gz CLI sin bloquear el timer follow_tcp. Latencia perceptible
        # del timer pasa de ~310ms+spikes a ~5ms (solo coste de Popen spawn).
        # Trade-off: no se conoce el éxito de cada llamada individual; se
        # confía en que el siguiente tick del follow loop reemplazará la pose.
        # Mantener False para tests síncronos o debug con confirmación.
        self.declare_parameter("set_pose_async_cli", True)
        self.declare_parameter("attach_initial_queue_retries", 4)
        self.declare_parameter("attach_retry_sleep_sec", 0.06)
        self.declare_parameter("attach_max_dist_m", 0.05)
        self.declare_parameter("require_contact_before_attach", True)
        self.declare_parameter("require_bilateral_contact", True)
        self.declare_parameter("contact_max_age_sec", 0.25)
        self.declare_parameter("left_contact_topic", "")
        self.declare_parameter("right_contact_topic", "")
        self.declare_parameter("equivalent_grasp_enable", True)
        self.declare_parameter("equivalent_grasp_max_opening_sum", 0.003)
        self.declare_parameter("equivalent_grasp_max_dist_m", 0.004)
        self.declare_parameter("equivalent_grasp_opening_max_age_sec", 0.50)
        self.declare_parameter("follow_break_dist_m", 0.18)
        self.declare_parameter("follow_break_consecutive", 3)
        self.declare_parameter("ws_dir", "")
        self.declare_parameter("startup_detach_tool_anchors", True)
        self.declare_parameter("startup_detach_max_attempts", 12)
        self.declare_parameter("startup_detach_period_sec", 0.5)
        self.declare_parameter("detachable_shadow_follow", True)
        string_array_param = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        self.declare_parameter("prefer_tool_anchor_objects", [""], string_array_param)
        self.declare_parameter("demo_transport_objects", [""], string_array_param)
        self.declare_parameter("demo_transport_rate_hz", 25.0)
        self.declare_parameter("demo_transport_min_step_m", 0.001)
        self.declare_parameter("demo_transport_respawn_sleep_sec", 0.08)
        self.declare_parameter("demo_transport_world_z_compensation_m", 0.065)
        self.declare_parameter("world_sdf", "")
        self.declare_parameter("gripper_cmd_topic", "/gripper_controller/commands")
        self.declare_parameter("gripper_open_rad", 0.0425)
        self.declare_parameter("gripper_closed_rad", 0.0)
        self.declare_parameter("gripper_joint2_sign", 1.0)
        self.declare_parameter("gripper_cmd_repeats", 6)
        self.declare_parameter("gripper_cmd_period_sec", 0.03)
        self.declare_parameter("orchestrator_attach_service", "/orchestrator/attach")
        self.declare_parameter("orchestrator_detach_service", "/orchestrator/detach")
        self.declare_parameter("gripper_open_service", "/gripper/open")
        self.declare_parameter("gripper_close_service", "/gripper/close")

    def _init_parse_parameters(self) -> None:
        """F3-step27b: parsea los parámetros declarados a self._<attr> (~115 LOC)."""
        self._gripper_prefix = str(
            self.get_parameter("gripper_prefix").value or "/gripper"
        ).strip("/")
        self._drop_anchor_prefix = str(
            self.get_parameter("drop_anchor_prefix").value or "/drop_anchor"
        ).strip("/")
        self._tool_anchor_prefix = str(
            self.get_parameter("tool_anchor_prefix").value or "/gripper_anchor"
        ).strip("/")
        names_raw = self.get_parameter("object_names").value
        object_names = [str(v).strip() for v in (names_raw or []) if str(v).strip()]
        self._object_names = sorted(set(object_names))
        self._attach_mode = str(
            self.get_parameter("attach_mode").value or "detachable_joint"
        ).strip()
        self._world_name = str(
            self.get_parameter("world_name").value or "ur5_mesa_objetos"
        ).strip()
        self._world_frame = str(
            self.get_parameter("world_frame").value or "world"
        ).strip() or "world"
        self._base_frame = str(
            self.get_parameter("base_frame").value or "base_link"
        ).strip() or "base_link"
        self._tcp_frame = str(self.get_parameter("tcp_frame").value or "rg2_tcp").strip()
        self._pose_topic = str(self.get_parameter("pose_topic").value or "").strip()
        self._joint_states_topic = str(
            self.get_parameter("joint_states_topic").value or "/joint_states"
        ).strip() or "/joint_states"
        if not self._pose_topic:
            self._pose_topic = f"/world/{self._world_name}/pose/info"
        self._set_pose_service = str(
            self.get_parameter("set_pose_service").value or ""
        ).strip()
        self._follow_rate_hz = max(
            2.0, float(self.get_parameter("follow_rate_hz").value or 20.0)
        )
        self._max_pose_age_sec = max(
            0.05, float(self.get_parameter("max_pose_age_sec").value or 0.6)
        )
        self._service_timeout_sec = max(
            0.05, float(self.get_parameter("service_timeout_sec").value or 0.2)
        )
        self._set_pose_future_timeout_sec = max(
            0.2, float(self.get_parameter("set_pose_future_timeout_sec").value or 1.0)
        )
        self._gz_cli_fallback = bool(self.get_parameter("gz_cli_fallback").value)
        # 2026-05-17 fix_follow_tcp_rate_20260517_163003: flag para Popen async
        self._set_pose_async_cli = bool(
            self.get_parameter("set_pose_async_cli").value
        )
        self._async_set_pose_procs: list = []
        self._gz_service_timeout_ms = max(
            100, int(self.get_parameter("gz_service_timeout_ms").value or 400)
        )
        self._gz_cmd_timeout_sec = max(
            0.3, float(self.get_parameter("gz_cmd_timeout_sec").value or 2.0)
        )
        self._attach_initial_queue_retries = max(
            1, int(self.get_parameter("attach_initial_queue_retries").value or 4)
        )
        self._attach_retry_sleep_sec = max(
            0.01, float(self.get_parameter("attach_retry_sleep_sec").value or 0.06)
        )
        self._attach_max_dist_m = max(
            0.01, float(self.get_parameter("attach_max_dist_m").value or 0.05)
        )
        self._require_contact_before_attach = bool(
            self.get_parameter("require_contact_before_attach").value
        )
        self._require_bilateral_contact = bool(
            self.get_parameter("require_bilateral_contact").value
        )
        self._contact_max_age_sec = max(
            0.01, float(self.get_parameter("contact_max_age_sec").value or 0.25)
        )
        self._left_contact_topic = str(
            self.get_parameter("left_contact_topic").value or ""
        ).strip()
        self._right_contact_topic = str(
            self.get_parameter("right_contact_topic").value or ""
        ).strip()
        if not self._left_contact_topic:
            self._left_contact_topic = (
                f"/world/{self._world_name}/model/ur5_rg2/link/"
                "rg2_finger_link1/sensor/rg2_left_contact_sensor/contact"
            )
        if not self._right_contact_topic:
            self._right_contact_topic = (
                f"/world/{self._world_name}/model/ur5_rg2/link/"
                "rg2_finger_link2/sensor/rg2_right_contact_sensor/contact"
            )
        self._equivalent_grasp_enable = bool(
            self.get_parameter("equivalent_grasp_enable").value
        )
        self._equivalent_grasp_max_opening_sum = max(
            0.0,
            float(
                self.get_parameter("equivalent_grasp_max_opening_sum").value or 0.003
            ),
        )
        self._equivalent_grasp_max_dist_m = max(
            0.0,
            float(self.get_parameter("equivalent_grasp_max_dist_m").value or 0.004),
        )
        self._equivalent_grasp_opening_max_age_sec = max(
            0.01,
            float(
                self.get_parameter("equivalent_grasp_opening_max_age_sec").value
                or 0.50
            ),
        )
        self._follow_break_dist_m = max(
            self._attach_max_dist_m,
            float(self.get_parameter("follow_break_dist_m").value or 0.18),
        )
        self._follow_break_consecutive = max(
            1, int(self.get_parameter("follow_break_consecutive").value or 3)
        )
        self._ws_dir = str(self.get_parameter("ws_dir").value or "").strip()
        if not self._ws_dir:
            from ur5_tools.workspace_paths import get_ws_dir
            self._ws_dir = get_ws_dir(default="~/TFM/agarre_ros2_ws")
        self._startup_detach_tool_anchors = bool(
            self.get_parameter("startup_detach_tool_anchors").value
        )
        self._startup_detach_max_attempts = max(
            1, int(self.get_parameter("startup_detach_max_attempts").value or 12)
        )
        self._startup_detach_period_sec = max(
            0.1, float(self.get_parameter("startup_detach_period_sec").value or 0.5)
        )
        self._detachable_shadow_follow = bool(
            self.get_parameter("detachable_shadow_follow").value
        )
        prefer_tool_anchor_raw = self.get_parameter("prefer_tool_anchor_objects").value
        self._prefer_tool_anchor_objects = {
            str(v).strip()
            for v in (prefer_tool_anchor_raw or [])
            if str(v).strip()
        }
        demo_transport_raw = self.get_parameter("demo_transport_objects").value
        self._demo_transport_objects = {
            str(v).strip()
            for v in (demo_transport_raw or [])
            if str(v).strip()
        }
        self._demo_transport_period_sec = 1.0 / max(
            1.0, float(self.get_parameter("demo_transport_rate_hz").value or 25.0)
        )
        self._demo_transport_min_step_m = max(
            0.0005, float(self.get_parameter("demo_transport_min_step_m").value or 0.001)
        )
        self._demo_transport_respawn_sleep_sec = max(
            0.0,
            float(self.get_parameter("demo_transport_respawn_sleep_sec").value or 0.02),
        )
        self._demo_transport_world_z_compensation_m = float(
            self.get_parameter("demo_transport_world_z_compensation_m").value or 0.065
        )
        self._world_sdf = str(self.get_parameter("world_sdf").value or "").strip()
        if not self._world_sdf:
            # F5 audit (2026-05-10): resolver via share/ur5_gazebo o source tree.
            from .workspace_paths import resolve_world_file
            resolved = resolve_world_file(self._world_name)
            if resolved:
                self._world_sdf = resolved
            else:
                candidate = os.path.join(
                    self._ws_dir, "src", "ur5_gazebo", "worlds",
                    f"{self._world_name}.sdf",
                )
                self._world_sdf = candidate if os.path.exists(candidate) else ""

    def _init_state_pubs_subs_timers(self) -> None:
        """F3-step27c: state init + qos + pubs/subs/timers (~135 LOC)."""
        self._qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._gripper_state_pubs: Dict[str, object] = {}
        self._drop_detach_pubs: Dict[str, object] = {}
        self._drop_attach_pubs: Dict[str, object] = {}
        self._tool_detach_pubs: Dict[str, object] = {}
        self._tool_attach_pubs: Dict[str, object] = {}
        self._drop_anchor_states: Dict[str, bool] = {}
        # F5-step6b: publisher al gripper_controller para los services
        # /gripper/open|close. Creado lazy en on_activate.
        self._gripper_cmd_topic = str(
            self.get_parameter("gripper_cmd_topic").value
            or "/gripper_controller/commands"
        ).strip()
        self._gripper_open_rad = float(
            self.get_parameter("gripper_open_rad").value
        )
        self._gripper_closed_rad = float(
            self.get_parameter("gripper_closed_rad").value
        )
        self._gripper_joint2_sign = float(
            self.get_parameter("gripper_joint2_sign").value
        )
        self._gripper_cmd_repeats = max(
            1, int(self.get_parameter("gripper_cmd_repeats").value or 6)
        )
        self._gripper_cmd_period_sec = max(
            0.0, float(self.get_parameter("gripper_cmd_period_sec").value or 0.03)
        )
        self._gripper_cmd_pub = None
        # F5-step6b: handles a los 4 services creados en on_activate.
        self._attach_srv = None
        self._detach_srv = None
        self._open_srv = None
        self._close_srv = None

        self._subs = []
        for name in self._object_names:
            grip_attach = f"/{self._gripper_prefix}/{name}/attach".replace("//", "/")
            grip_detach = f"/{self._gripper_prefix}/{name}/detach".replace("//", "/")
            grip_state = f"/{self._gripper_prefix}/{name}/state".replace("//", "/")
            self._gripper_state_pubs[name] = self.create_publisher(
                Bool, grip_state, self._qos
            )
            self._subs.append(
                self.create_subscription(
                    Empty,
                    grip_attach,
                    partial(self._on_gripper_attach, name=name, src_topic=grip_attach),
                    self._qos,
                )
            )
            self._subs.append(
                self.create_subscription(
                    Empty,
                    grip_detach,
                    partial(self._on_gripper_detach, name=name, src_topic=grip_detach),
                    self._qos,
                )
            )
            drop_attach = (
                f"/{self._drop_anchor_prefix}/{name}/attach".replace("//", "/")
            )
            drop_detach = (
                f"/{self._drop_anchor_prefix}/{name}/detach".replace("//", "/")
            )
            self._drop_attach_pubs[name] = self.create_publisher(
                Empty, drop_attach, self._qos
            )
            self._drop_detach_pubs[name] = self.create_publisher(
                Empty, drop_detach, self._qos
            )
            tool_attach = (
                f"/{self._tool_anchor_prefix}/{name}/attach".replace("//", "/")
            )
            tool_detach = (
                f"/{self._tool_anchor_prefix}/{name}/detach".replace("//", "/")
            )
            self._tool_attach_pubs[name] = self.create_publisher(
                Empty, tool_attach, self._qos
            )
            self._tool_detach_pubs[name] = self.create_publisher(
                Empty, tool_detach, self._qos
            )
            drop_state = f"/{self._drop_anchor_prefix}/{name}/state".replace("//", "/")
            self._subs.append(
                self.create_subscription(
                    Bool,
                    drop_state,
                    partial(self._on_drop_anchor_state, name=name),
                    self._qos,
                )
            )

        self._pose_cache: Dict[str, PoseSample] = {}
        self._pose_sub = self.create_subscription(
            TFMessage,
            self._pose_topic,
            self._on_pose_info,
            self._qos,
        )
        self._joint_state_sub = self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._on_joint_states,
            10,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        # El temporizador de follow puede bloquearse esperando poses de Gazebo;
        # mantenemos TF en un hilo dedicado para que el buffer no se quede viejo
        # durante los lifts.
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._attached: Dict[str, AttachedTarget] = {}
        self._set_pose_client = None
        self._set_pose_future = None
        self._set_pose_pending: Optional[Tuple[str, PoseSample]] = None
        self._set_pose_future_start_ts = 0.0
        self._contact_last_ns: Dict[Tuple[str, str], int] = {}
        self._last_contact_update_ns = 0
        self._left_contact_sub = None
        self._right_contact_sub = None
        self._contact_gate_enabled = False
        self._contacts_msg_type = GzContacts
        if self._require_contact_before_attach:
            if self._contacts_msg_type is None:
                self.get_logger().warning(
                    "[ATTACH_BACKEND] contact_gate_disabled detail=ros_gz_interfaces_missing"
                )
            else:
                try:
                    self._left_contact_sub = self.create_subscription(
                        self._contacts_msg_type,
                        self._left_contact_topic,
                        partial(self._on_contact_msg, topic=self._left_contact_topic),
                        self._qos,
                    )
                    self._right_contact_sub = self.create_subscription(
                        self._contacts_msg_type,
                        self._right_contact_topic,
                        partial(self._on_contact_msg, topic=self._right_contact_topic),
                        self._qos,
                    )
                    self._subs.append(self._left_contact_sub)
                    self._subs.append(self._right_contact_sub)
                    self._contact_gate_enabled = True
                except Exception as exc:
                    self.get_logger().warning(
                        "[ATTACH_BACKEND] contact_gate_disabled "
                        f"detail=create_subscription_failed:{type(exc).__name__}:{exc}"
                    )
        self._last_apply_log_ts = 0.0
        self._last_exception_log_ts = 0.0
        self._last_stale_warn_ts = 0.0
        self._last_coherence_warn_ts = 0.0
        self._last_set_pose_timeout_log_ts = 0.0
        self._last_demo_transport_log_ts = 0.0
        self._gz_set_pose_service: Optional[str] = None
        self._gz_spawn_service: Optional[str] = None
        self._gz_delete_service: Optional[str] = None
        self._last_tcp_pose_source = "none"
        self._last_tcp_pose_diag: Dict[str, float | str | bool] = {}
        # En este workspace la base del UR5 esta fija a world. Guardamos la ultima
        # pose fresca world->base para que el carry-follow siga siendo coherente
        # aunque el buffer local de TF deje de refrescar momentaneamente esa arista.
        self._stable_world_base_pose: Optional[PoseSample] = None
        self._joint_state_positions: Optional[Tuple[float, float, float, float, float, float]] = None
        self._joint_state_stamp_ns = 0
        self._gripper_opening_joint1: Optional[float] = None
        self._gripper_opening_joint2: Optional[float] = None
        self._gripper_opening_sum: Optional[float] = None
        self._gripper_opening_stamp_ns = 0
        self._startup_detach_attempts_left = int(self._startup_detach_max_attempts)
        self._startup_detach_sent = 0
        self._demo_transport_active: Dict[str, DemoTransportState] = {}
        self._demo_transport_dynamic_sdf: Dict[str, str] = {}
        self._demo_transport_carry_sdf: Dict[str, str] = {}

        self._follow_timer = self.create_timer(
            1.0 / self._follow_rate_hz,
            self._follow_attached_objects,
        )
        self._startup_detach_timer = None
        if self._startup_detach_tool_anchors and self._tool_detach_pubs:
            self._startup_detach_timer = self.create_timer(
                self._startup_detach_period_sec,
                self._startup_detach_tool_anchors_once_ready,
            )

        self.get_logger().info(
            "[ATTACH_BACKEND] ready "
            f"objects={','.join(self._object_names)} "
            f"mode={self._attach_mode} "
            f"gripper_prefix=/{self._gripper_prefix} "
            f"tool_anchor_prefix=/{self._tool_anchor_prefix} "
            f"prefer_tool_anchor={','.join(sorted(self._prefer_tool_anchor_objects)) or 'none'} "
            f"demo_transport={','.join(sorted(self._demo_transport_objects)) or 'none'} "
            f"pose_topic={self._pose_topic} joint_states_topic={self._joint_states_topic} "
            f"base_frame={self._base_frame} tcp_frame={self._tcp_frame} "
            f"contact_gate={'on' if self._require_contact_before_attach else 'off'} "
            f"contact_mode={'bilateral' if self._require_bilateral_contact else 'unilateral'} "
            f"contact_topics={self._left_contact_topic},{self._right_contact_topic} "
            f"equivalent_grasp={'on' if self._equivalent_grasp_enable else 'off'} "
            f"eq_max_opening_sum={self._equivalent_grasp_max_opening_sum:.4f} "
            f"eq_max_dist={self._equivalent_grasp_max_dist_m:.4f}m "
            f"eq_opening_max_age={self._equivalent_grasp_opening_max_age_sec:.3f}s"
        )

    def __init__(self) -> None:
        super().__init__("gripper_attach_backend")
        # F13 lifecycle: auto-activate por defecto preserva backward-compat.
        # F3-step27 (2026-05-03): __init__ split en 3 sub-helpers.
        self._init_declare_parameters()
        self._init_parse_parameters()
        self._init_state_pubs_subs_timers()

    def _drop_attached_target(
        self,
        name: str,
        *,
        detail: str,
        dist: Optional[float] = None,
    ) -> None:
        if name in self._demo_transport_objects:
            self._attached.pop(name, None)
            self._demo_transport_restore_dynamic(
                name,
                detail=f"drop_attached_target:{detail}",
            )
            dist_txt = "n/a" if dist is None else f"{dist:.4f}m"
            self.get_logger().warning(
                "[ATTACH_BACKEND] follow_detached "
                f"object={name} detail={detail} dist={dist_txt} "
                f"max={self._follow_break_dist_m:.4f}m "
                f"breaches_required={self._follow_break_consecutive}"
            )
            return
        self._attached.pop(name, None)
        if self._attach_mode == "detachable_joint" or name in self._prefer_tool_anchor_objects:
            pub = self._tool_detach_pubs.get(name)
            if pub is not None:
                pub.publish(Empty())
        self._publish_state(name, False)
        dist_txt = "n/a" if dist is None else f"{dist:.4f}m"
        self.get_logger().warning(
            "[ATTACH_BACKEND] follow_detached "
            f"object={name} detail={detail} dist={dist_txt} "
            f"max={self._follow_break_dist_m:.4f}m "
            f"breaches_required={self._follow_break_consecutive}"
        )

    def _on_set_pose_done(self, future) -> None:
        pending = self._set_pose_pending
        self._set_pose_pending = None
        self._set_pose_future = None
        self._set_pose_future_start_ts = 0.0
        if pending is None:
            return
        name, _pose = pending
        ok = False
        detail = "unknown"
        try:
            result = future.result()
            ok = bool(getattr(result, "success", False))
            detail = str(getattr(result, "status_message", "") or "n/a")
        except Exception as exc:
            detail = f"exception:{exc}"
            now = time.time()
            if (now - self._last_exception_log_ts) >= 1.0:
                self.get_logger().exception(
                    f"[ATTACH_BACKEND] [BRIDGE][EXCEPTION] object={name} err={exc}"
                )
                self._last_exception_log_ts = now
        now = time.time()
        if (now - self._last_apply_log_ts) >= 0.8:
            self.get_logger().info(
                f"[ATTACH_BACKEND] gazebo_attach_applied={str(ok).lower()} "
                f"object={name} method={self._attach_mode} detail={detail}"
            )
            self._last_apply_log_ts = now

    def _follow_attached_objects(self) -> None:
        has_shadow_follow = (
            self._attach_mode == "follow_tcp"
            or (self._attach_mode == "detachable_joint" and self._detachable_shadow_follow)
        )
        has_demo_work = bool(self._demo_transport_active)
        # detachable_shadow_follow=False solo desactiva el camino de physics-joint/
        # shadow. NO debe impedir que siga corriendo el bucle cinematico de
        # demo_transport; son dos mecanismos ortogonales.
        if not has_shadow_follow and not has_demo_work:
            return
        if not self._attached:
            return
        tcp_pose = self._lookup_tcp_pose()
        if tcp_pose is None:
            return
        tcp_age = self._pose_age_sec(tcp_pose)
        hard_age = max(3.0, self._max_pose_age_sec * 3.0)
        if tcp_age > hard_age:
            return
        soft_stale = tcp_age > self._max_pose_age_sec
        if tcp_age > self._max_pose_age_sec:
            now = time.time()
            if (now - self._last_stale_warn_ts) >= 0.7:
                diag = dict(self._last_tcp_pose_diag or {})
                diag_txt = (
                    f"src={self._last_tcp_pose_source} "
                    f"joint_state_base_ok={diag.get('joint_state_base_ok')} joint_state_age={diag.get('joint_state_age')} "
                    f"joint_state_chain_ok={diag.get('joint_state_chain_ok')} joint_state_chain_age={diag.get('joint_state_chain_age')} "
                    f"base_chain_ok={diag.get('base_chain_ok')} base_chain_age={diag.get('base_chain_age')} "
                    f"world_tcp_ok={diag.get('world_tcp_ok')} world_tcp_age={diag.get('world_tcp_age')} "
                    f"world_base_age={diag.get('world_base_age')} base_tcp_age={diag.get('base_tcp_age')} "
                    f"stable_world_base_ok={diag.get('stable_world_base_ok')} "
                    f"stable_world_base_age={diag.get('stable_world_base_age')} "
                    f"stable_world_base_mode={diag.get('stable_world_base_mode')} "
                    f"cache_ok={diag.get('cache_pose_ok')} cache_age={diag.get('cache_pose_age')} "
                    f"tool_fallback_ok={diag.get('tool_fallback_ok')} tool_fallback_age={diag.get('tool_fallback_age')} "
                    f"world_base_ok={diag.get('world_base_ok')} base_tcp_ok={diag.get('base_tcp_ok')}"
                )
                self.get_logger().warning(
                    f"[ATTACH_BACKEND] stale_tcp_pose_soft_follow age={tcp_age:.3f}s "
                    f"max={self._max_pose_age_sec:.3f}s hard={hard_age:.3f}s {diag_txt}"
                )
                self._last_stale_warn_ts = now
        for name, target in list(self._attached.items()):
            demo_transport_active = name in self._demo_transport_active
            # Los objetos que no son del demo no deben recibir actualizaciones de
            # pose cuando shadow_follow esta desactivado; solo demo_transport salta
            # esa restriccion.
            if not demo_transport_active and not has_shadow_follow:
                continue
            obj_pose = self._lookup_pose(name)
            if demo_transport_active:
                target.coherence_breach_count = 0
            elif (not soft_stale) and obj_pose is not None and self._pose_age_ok(obj_pose):
                obj_tcp_dist = math.sqrt(
                    (obj_pose.x - tcp_pose.x) ** 2
                    + (obj_pose.y - tcp_pose.y) ** 2
                    + (obj_pose.z - tcp_pose.z) ** 2
                )
                if obj_tcp_dist > self._follow_break_dist_m:
                    target.coherence_breach_count += 1
                    now = time.time()
                    if (
                        target.coherence_breach_count >= self._follow_break_consecutive
                    ):
                        self._drop_attached_target(
                            name,
                            detail="coherence_lost",
                            dist=obj_tcp_dist,
                        )
                        continue
                    if (now - self._last_coherence_warn_ts) >= 0.5:
                        self.get_logger().warning(
                            "[ATTACH_BACKEND] follow_coherence_warning "
                            f"object={name} dist={obj_tcp_dist:.4f}m "
                            f"max={self._follow_break_dist_m:.4f}m "
                            f"breach={target.coherence_breach_count}/"
                            f"{self._follow_break_consecutive}"
                        )
                        self._last_coherence_warn_ts = now
                else:
                    target.coherence_breach_count = 0
            else:
                target.coherence_breach_count = 0
            desired = PoseSample(
                x=0.0,
                y=0.0,
                z=0.0,
                qx=0.0,
                qy=0.0,
                qz=0.0,
                qw=1.0,
                stamp_ns=int(tcp_pose.stamp_ns),
            )
            if demo_transport_active:
                demo_state = self._demo_transport_active.get(name)
                if demo_state is not None and demo_state.use_world_locked_pose:
                    desired.x = float(tcp_pose.x + demo_state.world_offset_x)
                    desired.y = float(tcp_pose.y + demo_state.world_offset_y)
                    desired.z = float(tcp_pose.z + demo_state.world_offset_z)
                    desired.qx = float(demo_state.world_qx)
                    desired.qy = float(demo_state.world_qy)
                    desired.qz = float(demo_state.world_qz)
                    desired.qw = float(demo_state.world_qw)
                    now = time.time()
                    if (now - self._last_demo_transport_log_ts) >= 1.0:
                        self.get_logger().info(
                            "[ATTACH_BACKEND] demo_transport_follow_tick "
                            f"object={name} mode=world_locked "
                            f"desired=({desired.x:.3f},{desired.y:.3f},{desired.z:.3f}) "
                            f"tcp=({tcp_pose.x:.3f},{tcp_pose.y:.3f},{tcp_pose.z:.3f}) "
                            f"tcp_src={self._last_tcp_pose_source}"
                        )
                        self._last_demo_transport_log_ts = now
                    self._demo_transport_update(name, desired)
                    continue
            tcp_q = _quat_normalize(
                (float(tcp_pose.qx), float(tcp_pose.qy), float(tcp_pose.qz), float(tcp_pose.qw))
            )
            rel_pos_world = _rotate_vector(
                tcp_q,
                (float(target.offset_x), float(target.offset_y), float(target.offset_z)),
            )
            rel_q_world = _quat_multiply(
                tcp_q,
                (float(target.qx), float(target.qy), float(target.qz), float(target.qw)),
            )
            desired.x = float(tcp_pose.x + rel_pos_world[0])
            desired.y = float(tcp_pose.y + rel_pos_world[1])
            desired.z = float(tcp_pose.z + rel_pos_world[2])
            desired.qx = float(rel_q_world[0])
            desired.qy = float(rel_q_world[1])
            desired.qz = float(rel_q_world[2])
            desired.qw = float(rel_q_world[3])
            if demo_transport_active:
                now = time.time()
                if (now - self._last_demo_transport_log_ts) >= 1.0:
                    self.get_logger().info(
                        "[ATTACH_BACKEND] demo_transport_follow_tick "
                        f"object={name} mode=relative_offset "
                        f"desired=({desired.x:.3f},{desired.y:.3f},{desired.z:.3f}) "
                        f"tcp=({tcp_pose.x:.3f},{tcp_pose.y:.3f},{tcp_pose.z:.3f})"
                    )
                    self._last_demo_transport_log_ts = now
                self._demo_transport_update(name, desired)
                continue
            if self._queue_set_pose(name, desired):
                break

    def _activate_follow_attachment(self, name: str, *, method: str) -> bool:
        obj_pose = self._lookup_pose(name)
        tcp_pose = self._lookup_tcp_pose()
        if obj_pose is None or tcp_pose is None:
            self.get_logger().error(
                "[ATTACH_BACKEND] gazebo_attach_applied=false "
                f"object={name} method={method} detail=missing_pose "
                f"obj_pose={str(obj_pose is not None).lower()} tcp_pose={str(tcp_pose is not None).lower()}"
            )
            self._publish_state(name, False)
            return False
        obj_age = self._pose_age_sec(obj_pose)
        tcp_age = self._pose_age_sec(tcp_pose)
        hard_age = max(3.0, self._max_pose_age_sec * 3.0)
        if obj_age > hard_age or tcp_age > hard_age:
            self.get_logger().error(
                "[ATTACH_BACKEND] gazebo_attach_applied=false "
                f"object={name} method={method} detail=stale_pose_hard "
                f"obj_age={obj_age:.3f}s tcp_age={tcp_age:.3f}s hard={hard_age:.3f}s"
            )
            self._publish_state(name, False)
            return False
        if obj_age > self._max_pose_age_sec or tcp_age > self._max_pose_age_sec:
            self.get_logger().warning(
                "[ATTACH_BACKEND] stale_pose_soft_accept "
                f"object={name} obj_age={obj_age:.3f}s tcp_age={tcp_age:.3f}s "
                f"max={self._max_pose_age_sec:.3f}s"
            )
        if (not self._ensure_set_pose_client()) and (not self._gz_cli_fallback):
            self.get_logger().error(
                "[ATTACH_BACKEND] gazebo_attach_applied=false "
                f"object={name} method={method} detail=set_pose_backend_unavailable"
            )
            self._publish_state(name, False)
            return False
        attach_dist = math.sqrt(
            (obj_pose.x - tcp_pose.x) ** 2
            + (obj_pose.y - tcp_pose.y) ** 2
            + (obj_pose.z - tcp_pose.z) ** 2
        )
        if attach_dist > self._attach_max_dist_m:
            self.get_logger().error(
                "[ATTACH_BACKEND] gazebo_attach_applied=false "
                f"object={name} method={method} detail=distance_too_large "
                f"dist={attach_dist:.4f}m max={self._attach_max_dist_m:.4f}m "
                f"tcp=({tcp_pose.x:.3f},{tcp_pose.y:.3f},{tcp_pose.z:.3f}) "
                f"obj=({obj_pose.x:.3f},{obj_pose.y:.3f},{obj_pose.z:.3f})"
            )
            self._publish_state(name, False)
            return False
        tcp_q = _quat_normalize(
            (float(tcp_pose.qx), float(tcp_pose.qy), float(tcp_pose.qz), float(tcp_pose.qw))
        )
        obj_q = _quat_normalize(
            (float(obj_pose.qx), float(obj_pose.qy), float(obj_pose.qz), float(obj_pose.qw))
        )
        tcp_q_inv = _quat_inverse(tcp_q)
        rel_pos = _rotate_vector(
            tcp_q_inv,
            (
                float(obj_pose.x - tcp_pose.x),
                float(obj_pose.y - tcp_pose.y),
                float(obj_pose.z - tcp_pose.z),
            ),
        )
        rel_q = _quat_multiply(tcp_q_inv, obj_q)
        attached = AttachedTarget(
            name=name,
            offset_x=float(rel_pos[0]),
            offset_y=float(rel_pos[1]),
            offset_z=float(rel_pos[2]),
            qx=float(rel_q[0]),
            qy=float(rel_q[1]),
            qz=float(rel_q[2]),
            qw=float(rel_q[3]),
            attach_stamp_ns=int(self.get_clock().now().nanoseconds),
        )
        self._attached[name] = attached
        self._publish_state(name, True)
        self.get_logger().info(
            f"[ATTACH_BACKEND] gazebo_attach_applied=true object={name} method={method} "
            f"dist={attach_dist:.4f}m max={self._attach_max_dist_m:.4f}m "
            f"rel_pos=({attached.offset_x:.3f},{attached.offset_y:.3f},{attached.offset_z:.3f}) "
            f"rel_q=({attached.qx:.3f},{attached.qy:.3f},{attached.qz:.3f},{attached.qw:.3f})"
        )
        rel_pos_world = _rotate_vector(
            tcp_q,
            (float(attached.offset_x), float(attached.offset_y), float(attached.offset_z)),
        )
        rel_q_world = _quat_multiply(
            tcp_q,
            (float(attached.qx), float(attached.qy), float(attached.qz), float(attached.qw)),
        )
        desired = PoseSample(
            x=float(tcp_pose.x + rel_pos_world[0]),
            y=float(tcp_pose.y + rel_pos_world[1]),
            z=float(tcp_pose.z + rel_pos_world[2]),
            qx=float(rel_q_world[0]),
            qy=float(rel_q_world[1]),
            qz=float(rel_q_world[2]),
            qw=float(rel_q_world[3]),
            stamp_ns=int(tcp_pose.stamp_ns),
        )
        if not self._queue_set_pose_with_retry(name, desired):
            self.get_logger().warning(
                "[ATTACH_BACKEND] initial_set_pose_not_queued "
                f"object={name} method={method} "
                f"retries={self._attach_initial_queue_retries} "
                "detail=will_retry_on_follow_timer"
            )
        return True

    def _on_gripper_attach(self, _msg: Empty, *, name: str, src_topic: str) -> bool:
        self.get_logger().info(
            f"[ATTACH_BACKEND] attach_request_received object={name} src={src_topic} mode={self._attach_mode}"
        )
        # TRACE-ATTACH-ROUTE: trazamos la distancia TCP<->objeto y la ruta elegida
        # antes de bifurcar. Asi cada decision de attach queda auditable sin
        # herramientas adicionales.
        _ta_obj = self._lookup_pose(name)
        _ta_tcp = self._lookup_tcp_pose()
        if _ta_obj is not None and _ta_tcp is not None:
            _ta_dist = math.sqrt(
                (_ta_obj.x - _ta_tcp.x) ** 2
                + (_ta_obj.y - _ta_tcp.y) ** 2
                + (_ta_obj.z - _ta_tcp.z) ** 2
            )
            _ta_route = (
                "demo_transport" if name in self._demo_transport_objects
                else "tool_anchor" if name in self._prefer_tool_anchor_objects
                else "tool_anchor_relay" if self._attach_mode != "follow_tcp"
                else "follow_tcp"
            )
            _ta_will_pass = _ta_dist <= self._attach_max_dist_m
            self.get_logger().info(
                f"[ATTACH_BACKEND] attach_route_decision object={name} "
                f"route={_ta_route} "
                f"dist={_ta_dist:.4f}m max={self._attach_max_dist_m:.4f}m "
                f"geometry_ok={str(_ta_will_pass).lower()} "
                f"tcp_src={self._last_tcp_pose_source} "
                f"tcp=({_ta_tcp.x:.3f},{_ta_tcp.y:.3f},{_ta_tcp.z:.3f}) "
                f"obj=({_ta_obj.x:.3f},{_ta_obj.y:.3f},{_ta_obj.z:.3f})"
            )
            if not _ta_will_pass:
                self.get_logger().warning(
                    f"[ATTACH_BACKEND] attach_blocked object={name} "
                    f"route={_ta_route} "
                    f"detail=distance_too_large dist={_ta_dist:.4f}m max={self._attach_max_dist_m:.4f}m"
                )
                self._publish_state(name, False)
                return False
        else:
            self.get_logger().warning(
                f"[ATTACH_BACKEND] attach_route_decision object={name} "
                f"detail=pose_unavailable "
                f"obj_available={str(_ta_obj is not None).lower()} "
                f"tcp_available={str(_ta_tcp is not None).lower()}"
            )
            self._publish_state(name, False)
            return False
        _contact_ok, _contact_detail = self._validate_contact_gate(name)
        if not _contact_ok:
            self.get_logger().warning(
                f"[ATTACH_BACKEND] attach_blocked object={name} "
                "route=contact_gate "
                f"detail={_contact_detail}"
            )
            self._publish_state(name, False)
            return False
        if name in self._demo_transport_objects:
            self._force_drop_anchor_detach(name)
            self._relay_tool_anchor_detach(
                name,
                detail="demo_transport_preclean",
            )
            if not self._activate_demo_transport_attachment(
                name,
                method="demo_controlled_carry",
            ):
                self.get_logger().error(
                    f"[ATTACH_BACKEND] demo_transport_attach_failed object={name}"
                )
                self._publish_state(name, False)
                return False
            return True
        if name in self._prefer_tool_anchor_objects:
            relayed = self._relay_tool_anchor_attach(
                name,
                detail="prefer_tool_anchor_object",
            )
            if (not relayed) or (not self._detachable_shadow_follow):
                return bool(relayed)
            self.get_logger().info(
                f"[ATTACH_BACKEND] tool_anchor relay object={name} detail=starting_shadow_follow"
            )
            if not self._activate_follow_attachment(name, method="tool_anchor_shadow_follow"):
                self._relay_tool_anchor_detach(
                    name,
                    detail="shadow_follow_activation_failed",
                )
                return False
            return True
        if self._attach_mode != "follow_tcp":
            self._force_drop_anchor_detach(name)
            pub = self._tool_attach_pubs.get(name)
            if pub is None:
                self.get_logger().error(
                    f"[ATTACH_BACKEND] missing tool_attach publisher object={name}"
                )
                self._publish_state(name, False)
                return False
            pub.publish(Empty())
            self.get_logger().info(
                f"[ATTACH_BACKEND] relay attach object={name} "
                f"dst=/{self._tool_anchor_prefix}/{name}/attach method=tool_anchor_relay"
            )
            if self._detachable_shadow_follow:
                self.get_logger().info(
                    f"[ATTACH_BACKEND] tool_anchor relay object={name} "
                    "detail=starting_shadow_follow"
                )
                return bool(
                    self._activate_follow_attachment(
                        name, method="detachable_joint_shadow_follow",
                    )
                )
            else:
                self._publish_state(name, True)
            return True

        # El drop anchor puede seguir fisicamente unido aunque su topic ya reporte
        # false. Lo liberamos de forma proactiva antes del follow attach.
        self._force_drop_anchor_detach(name)
        return bool(self._activate_follow_attachment(name, method="follow_tcp"))

    def _on_gripper_detach(self, _msg: Empty, *, name: str, src_topic: str) -> None:
        self.get_logger().info(
            f"[ATTACH_BACKEND] detach_request_received object={name} src={src_topic} mode={self._attach_mode}"
        )
        if name in self._demo_transport_objects:
            self._attached.pop(name, None)
            if not self._demo_transport_restore_dynamic(
                name,
                detail="gripper_detach",
            ):
                self._publish_state(name, False)
            return
        if name in self._prefer_tool_anchor_objects:
            self._relay_tool_anchor_detach(
                name,
                detail="prefer_tool_anchor_object",
            )
            return
        if self._attach_mode != "follow_tcp":
            self._attached.pop(name, None)
            pub = self._tool_detach_pubs.get(name)
            if pub is None:
                self.get_logger().error(
                    f"[ATTACH_BACKEND] missing tool_detach publisher object={name}"
                )
                self._publish_state(name, False)
                return
            pub.publish(Empty())
            self.get_logger().info(
                f"[ATTACH_BACKEND] relay detach object={name} "
                f"dst=/{self._tool_anchor_prefix}/{name}/detach method=tool_anchor_relay"
            )
            self._publish_state(name, False)
            return
        self._attached.pop(name, None)
        self._publish_state(name, False)
        self.get_logger().info(
            f"[ATTACH_BACKEND] gazebo_detach_applied=true object={name} method=follow_tcp"
        )

    # ------------------------------------------------------------------
    # Lifecycle transitions (F13 — observable, sin re-creación de recursos)
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # F5-step6b: services /gripper/open|close + /orchestrator/attach|detach
    # ------------------------------------------------------------------

    def _command_gripper_position(self, target_rad: float, *, label: str) -> tuple[bool, str]:
        """Publica comando al gripper_controller (Float64MultiArray con 2 joints).

        Devuelve (success, detail). Lazy-init del publisher si no existe.
        """
        if self._gripper_cmd_pub is None:
            try:
                self._gripper_cmd_pub = self.create_publisher(
                    Float64MultiArray, self._gripper_cmd_topic, 10,
                )
            except Exception as exc:
                msg = f"create_publisher_failed:{type(exc).__name__}:{exc}"
                self.get_logger().error(
                    f"[GRIPPER_SVC] {label} {msg} topic={self._gripper_cmd_topic}"
                )
                return False, msg
        msg = Float64MultiArray()
        msg.data = [float(target_rad), float(target_rad) * self._gripper_joint2_sign]
        try:
            try:
                repeats = max(1, int(getattr(self, "_gripper_cmd_repeats", 1)))
            except Exception:
                repeats = 1
            try:
                period = max(0.0, float(getattr(self, "_gripper_cmd_period_sec", 0.0)))
            except Exception:
                period = 0.0
            for idx in range(repeats):
                self._gripper_cmd_pub.publish(msg)
                if idx + 1 < repeats and period > 0.0:
                    time.sleep(period)
        except Exception as exc:
            err = f"publish_failed:{type(exc).__name__}:{exc}"
            self.get_logger().error(
                f"[GRIPPER_SVC] {label} {err} topic={self._gripper_cmd_topic}"
            )
            return False, err
        detail = (
            f"target_rad={target_rad:.4f} joint2_sign={self._gripper_joint2_sign:.2f} "
            f"topic={self._gripper_cmd_topic} repeats={int(repeats)} "
            f"period={float(period):.3f}s"
        )
        self.get_logger().info(f"[GRIPPER_SVC] {label} ok {detail}")
        return True, detail

    def _on_open_service(self, _request, response):
        """Handler /gripper/open (Open.srv): publica gripper_open_rad."""
        ok, detail = self._command_gripper_position(
            self._gripper_open_rad, label="OPEN",
        )
        response.success = ok
        response.message = detail
        return response

    def _on_close_service(self, _request, response):
        """Handler /gripper/close (Close.srv): publica gripper_closed_rad."""
        ok, detail = self._command_gripper_position(
            self._gripper_closed_rad, label="CLOSE",
        )
        response.success = ok
        response.message = detail
        return response

    def _on_attach_service(self, request, response):
        """Handler /orchestrator/attach (Attach.srv): delega en _on_gripper_attach.

        El response.method y response.tcp_obj_dist_m los proveemos best-effort
        (la lógica interna actual no devuelve esos valores explícitamente;
        usamos heurísticas seguras).
        """
        name = str(request.object_name or "").strip()
        if not name:
            response.success = False
            response.message = "object_name_empty"
            response.method = ""
            response.tcp_obj_dist_m = 0.0
            return response
        # Calcular distancia TCP-objeto antes para reportarla.
        tcp_obj_dist = 0.0
        try:
            obj_pose = self._lookup_pose(name)
            tcp_pose = self._lookup_tcp_pose()
            if obj_pose is not None and tcp_pose is not None:
                tcp_obj_dist = math.sqrt(
                    (obj_pose.x - tcp_pose.x) ** 2
                    + (obj_pose.y - tcp_pose.y) ** 2
                    + (obj_pose.z - tcp_pose.z) ** 2
                )
        except Exception:
            pass
        # Determinar método como en _on_gripper_attach.
        if name in self._demo_transport_objects:
            method = "demo_transport"
        elif name in self._prefer_tool_anchor_objects:
            method = "tool_anchor"
        else:
            method = "follow_tcp" if self._attach_mode == "follow_tcp" else "tool_anchor_relay"
        try:
            attached_ok = bool(
                self._on_gripper_attach(
                Empty(), name=name, src_topic="/orchestrator/attach",
                )
            )
            response.success = attached_ok
            response.message = "attach_applied" if attached_ok else "attach_rejected"
            response.method = method
            response.tcp_obj_dist_m = float(tcp_obj_dist)
            return response
        except Exception as exc:
            response.success = False
            response.message = f"attach_exception:{type(exc).__name__}:{exc}"
            response.method = method
            response.tcp_obj_dist_m = float(tcp_obj_dist)
            return response

    def _on_detach_service(self, request, response):
        """Handler /orchestrator/detach (Detach.srv): delega en _on_gripper_detach.

        Si object_name es "" (vacío), libera todos los objetos actualmente
        adheridos (contrato de Detach.srv).
        """
        name = str(request.object_name or "").strip()
        targets = [name] if name else list(self._attached.keys())
        detached_count = 0
        errors = []
        for target in targets:
            try:
                self._on_gripper_detach(
                    Empty(), name=target, src_topic="/orchestrator/detach",
                )
                detached_count += 1
            except Exception as exc:
                errors.append(f"{target}:{type(exc).__name__}:{exc}")
        response.detached_count = int(detached_count)
        if errors:
            response.success = False
            response.message = "detach_errors:" + ",".join(errors)
        else:
            response.success = True
            response.message = (
                f"detached_all (count={detached_count})" if not name
                else f"detached={name}"
            )
        return response

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, _state) -> TransitionCallbackReturn:
        self.get_logger().info("[LIFECYCLE] GripperAttachBackend configured")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state) -> TransitionCallbackReturn:
        # F5-step6b: crear los 4 services en activate.
        attach_name = str(self.get_parameter("orchestrator_attach_service").value)
        detach_name = str(self.get_parameter("orchestrator_detach_service").value)
        open_name = str(self.get_parameter("gripper_open_service").value)
        close_name = str(self.get_parameter("gripper_close_service").value)
        try:
            self._attach_srv = self.create_service(
                AttachSrv, attach_name, self._on_attach_service,
            )
            self._detach_srv = self.create_service(
                DetachSrv, detach_name, self._on_detach_service,
            )
            self._open_srv = self.create_service(
                OpenSrv, open_name, self._on_open_service,
            )
            self._close_srv = self.create_service(
                CloseSrv, close_name, self._on_close_service,
            )
            self.get_logger().info(
                "[LIFECYCLE] GripperAttachBackend activated — services up: "
                f"{attach_name} | {detach_name} | {open_name} | {close_name}"
            )
        except Exception as exc:
            self.get_logger().error(
                f"[LIFECYCLE] failed to create orchestrator/gripper services: {exc}"
            )
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state) -> TransitionCallbackReturn:
        # F5-step6b: destruir los 4 services + el publisher cmd.
        for attr in ("_attach_srv", "_detach_srv", "_open_srv", "_close_srv"):
            srv = getattr(self, attr, None)
            if srv is not None:
                try:
                    self.destroy_service(srv)
                except Exception:
                    pass
                setattr(self, attr, None)
        self.get_logger().info("[LIFECYCLE] GripperAttachBackend deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state) -> TransitionCallbackReturn:
        # Cleanup también destruye los services + el publisher cmd.
        for attr in ("_attach_srv", "_detach_srv", "_open_srv", "_close_srv"):
            srv = getattr(self, attr, None)
            if srv is not None:
                try:
                    self.destroy_service(srv)
                except Exception:
                    pass
                setattr(self, attr, None)
        if self._gripper_cmd_pub is not None:
            try:
                self.destroy_publisher(self._gripper_cmd_pub)
            except Exception:
                pass
            self._gripper_cmd_pub = None
        self.get_logger().info("[LIFECYCLE] GripperAttachBackend cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state) -> TransitionCallbackReturn:
        return self.on_cleanup(_state)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GripperAttachBackend()
    # F13 lifecycle: auto-activate por defecto preserva backward-compat.
    if bool(node.get_parameter("auto_activate").value):
        try:
            node.trigger_configure()
            node.trigger_activate()
        except Exception as exc:
            node.get_logger().error(f"[LIFECYCLE] auto_activate failed: {exc}")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("[ATTACH_BACKEND] stopped")
    finally:
        try:
            node.destroy_node()
        except BaseException:
            pass
        try:
            rclpy.try_shutdown()
        except BaseException:
            pass


if __name__ == "__main__":
    main()
