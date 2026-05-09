# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/attach_pose_sub.py
# Contenido: PoseSubscriberMixin: callbacks de /pose/info + /joint_states + age helpers (C.7).
"""PoseSubscriberMixin: pose subscribers + age helpers (refactor C.7).

Extraido de ``gripper_attach_backend.py`` (lineas 461-571 originales).
Callbacks de los topicos /pose/info y /joint_states + computacion FK
para tcp_base_pose desde joint state + verificacion de edad de pose.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from .attach_math import (  # noqa: F401
    _dh_transform,
    _matmul3,
    _matvec3,
    _quat_from_rot3,
)

if TYPE_CHECKING:
    from .gripper_attach_backend import PoseSample  # noqa: F401


def _get_PoseSample_class():
    from .gripper_attach_backend import PoseSample
    return PoseSample


def _get_dh_constants():
    from .gripper_attach_backend import (
        UR5_ARM_JOINT_ORDER,
        _UR5_DH_A,
        _UR5_DH_ALPHA,
        _UR5_DH_D,
        _BASE_LINK_FIX_R,
        _GRIPPER_GEOMETRY,
    )
    return UR5_ARM_JOINT_ORDER, _UR5_DH_A, _UR5_DH_ALPHA, _UR5_DH_D, _BASE_LINK_FIX_R, _GRIPPER_GEOMETRY


class PoseSubscriberMixin:
    """/pose/info + /joint_states callbacks + FK + age helpers."""
    def _on_pose_info(self, msg: TFMessage) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        PoseSampleCls = _get_PoseSample_class()
        for tf in getattr(msg, "transforms", []) or []:
            name = str(getattr(tf, "child_frame_id", "") or "").strip()
            if not name:
                continue
            stamp = getattr(tf, "header", None)
            stamp_msg = getattr(stamp, "stamp", None)
            if stamp_msg is not None:
                stamp_ns = int(stamp_msg.sec) * 1_000_000_000 + int(stamp_msg.nanosec)
            else:
                stamp_ns = now_ns
            tr = tf.transform.translation
            rot = tf.transform.rotation
            sample = PoseSampleCls(
                x=float(tr.x),
                y=float(tr.y),
                z=float(tr.z),
                qx=float(rot.x),
                qy=float(rot.y),
                qz=float(rot.z),
                qw=float(rot.w),
                stamp_ns=stamp_ns,
            )
            self._pose_cache[name] = sample
            if "::" in name:
                model = name.split("::")[0].strip()
                if model:
                    # Mantenemos el alias del modelo en la muestra mas reciente;
                    # setdefault dejaria congelados valores viejos.
                    self._pose_cache[model] = sample

    def _on_joint_states(self, msg: JointState) -> None:
        UR5_ARM_JOINT_ORDER, _UR5_DH_A, _UR5_DH_ALPHA, _UR5_DH_D, _BASE_LINK_FIX_R, _ = _get_dh_constants()
        names = list(getattr(msg, "name", []) or [])
        positions = list(getattr(msg, "position", []) or [])
        if not names or len(names) != len(positions):
            return
        index = {str(name): idx for idx, name in enumerate(names)}
        try:
            ordered = tuple(
                float(positions[index[joint_name]])
                for joint_name in UR5_ARM_JOINT_ORDER
            )
        except Exception:
            return
        stamp = getattr(msg, "header", None)
        stamp_msg = getattr(stamp, "stamp", None)
        stamp_ns = 0
        if stamp_msg is not None:
            stamp_ns = int(stamp_msg.sec) * 1_000_000_000 + int(stamp_msg.nanosec)
        if stamp_ns <= 0:
            stamp_ns = int(self.get_clock().now().nanoseconds)
        self._joint_state_positions = ordered
        self._joint_state_stamp_ns = int(stamp_ns)

    def _joint_state_tcp_base_pose(self) -> Optional[PoseSample]:
        UR5_ARM_JOINT_ORDER, _UR5_DH_A, _UR5_DH_ALPHA, _UR5_DH_D, _BASE_LINK_FIX_R, _GRIPPER_GEOMETRY = _get_dh_constants()
        joints = self._joint_state_positions
        stamp_ns = int(self._joint_state_stamp_ns)
        if joints is None or len(joints) != len(UR5_ARM_JOINT_ORDER) or stamp_ns <= 0:
            return None
        rot = (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
        pos = (0.0, 0.0, 0.0)
        for joint_idx, theta in enumerate(joints):
            step_rot, step_pos = _dh_transform(
                float(_UR5_DH_A[joint_idx]),
                float(_UR5_DH_D[joint_idx]),
                float(_UR5_DH_ALPHA[joint_idx]),
                float(theta),
            )
            pos = tuple(
                float(pos[i]) + float(_matvec3(rot, step_pos)[i])
                for i in range(3)
            )
            rot = _matmul3(rot, step_rot)
        base_pos = (-float(pos[0]), -float(pos[1]), float(pos[2]))
        base_rot = _matmul3(_BASE_LINK_FIX_R, rot)
        offset = _GRIPPER_GEOMETRY.xyz_for_frame(self._tcp_frame)
        offset_base = _matvec3(
            base_rot,
            (float(offset[0]), float(offset[1]), float(offset[2])),
        )
        tcp_base_pos = tuple(
            float(base_pos[i]) + float(offset_base[i])
            for i in range(3)
        )
        qx, qy, qz, qw = _quat_from_rot3(base_rot)
        return _get_PoseSample_class()(
            x=float(tcp_base_pos[0]),
            y=float(tcp_base_pos[1]),
            z=float(tcp_base_pos[2]),
            qx=float(qx),
            qy=float(qy),
            qz=float(qz),
            qw=float(qw),
            stamp_ns=stamp_ns,
        )

    def _pose_age_ok(self, pose: PoseSample) -> bool:
        age = self._pose_age_sec(pose)
        return -0.25 <= age <= self._max_pose_age_sec

    def _pose_age_sec(self, pose: PoseSample) -> float:
        now_ns = int(self.get_clock().now().nanoseconds)
        if pose.stamp_ns <= 0 or now_ns <= 0:
            return float("inf")
        return (now_ns - pose.stamp_ns) / 1_000_000_000.0
