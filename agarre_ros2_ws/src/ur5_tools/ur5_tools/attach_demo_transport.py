# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/attach_demo_transport.py
# Contenido: DemoTransportMixin extraido de gripper_attach_backend (C.3).
"""DemoTransportMixin: SDFs dinamicos + spawn/delete + activacion (refactor C.3).

Extraido de ``gripper_attach_backend.py`` (lineas 459-768 originales).
Maneja la teletransportacion del objeto durante demos (delete/spawn
con SDFs dinamicos, follow del TCP, restore al estado dinamico
original al detach).

Sin estado propio. Requiere que la clase concreta herede de ``Node``,
``GzCliMixin`` (provee ``_gz_delete_entity_cli`` y ``_gz_spawn_entity_cli``)
y tenga inicializados ``_demo_transport_*``, ``_attached``, etc.
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Optional

from .attach_math import (  # noqa: F401  used by extracted methods
    _quat_inverse,
    _quat_multiply,
    _quat_normalize,
    _rotate_vector,
)

if TYPE_CHECKING:
    from .gripper_attach_backend import (  # noqa: F401
        AttachedTarget,
        PoseSample,
    )


class DemoTransportMixin:
    """Demo transport: dynamic SDFs + spawn/delete via gz CLI + follow TCP."""

    @staticmethod
    def _import_demo_transport_state():
        """Lazy import to avoid circular dependency with gripper_attach_backend."""

        from .gripper_attach_backend import DemoTransportState
        return DemoTransportState

    @staticmethod
    def _import_attached_target():
        """Lazy import to avoid circular dependency with gripper_attach_backend."""

        from .gripper_attach_backend import AttachedTarget  # noqa: F811  rebound at runtime to avoid circular import
        return AttachedTarget

    def _demo_dynamic_sdf(self, name: str) -> Optional[str]:
        if name != "pick_demo":
            return None
        return f"""
<sdf version="1.10">
  <model name="{name}">
    <static>false</static>
    <allow_auto_disable>true</allow_auto_disable>
    <link name="link">
      <inertial>
        <mass>0.08</mass>
        <inertia>
          <ixx>0.0002</ixx><iyy>0.0002</iyy><izz>0.0002</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><cylinder><radius>0.025</radius><length>0.05</length></cylinder></geometry>
        <surface>
          <friction>
            <ode><mu>2.0</mu><mu2>2.0</mu2></ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <ode><kp>200000</kp><kd>50.0</kd><max_vel>0.1</max_vel><min_depth>0.001</min_depth></ode>
          </contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>0.025</radius><length>0.05</length></cylinder></geometry>
        <material><diffuse>0.95 0.70 0.10 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
""".strip()

    def _demo_carry_sdf(self, name: str) -> Optional[str]:
        if name != "pick_demo":
            return None
        return f"""
<sdf version="1.10">
  <model name="{name}">
    <static>false</static>
    <allow_auto_disable>false</allow_auto_disable>
    <link name="link">
      <inertial>
        <mass>0.08</mass>
        <inertia>
          <ixx>0.0002</ixx><iyy>0.0002</iyy><izz>0.0002</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <visual name="visual">
        <geometry><cylinder><radius>0.025</radius><length>0.05</length></cylinder></geometry>
        <material><diffuse>0.95 0.70 0.10 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
""".strip()

    def _ensure_demo_transport_sdfs(self, name: str) -> bool:
        if name in self._demo_transport_dynamic_sdf and name in self._demo_transport_carry_sdf:
            return True
        dyn = self._demo_dynamic_sdf(name)
        carry = self._demo_carry_sdf(name)
        if not dyn or not carry:
            return False
        self._demo_transport_dynamic_sdf[name] = dyn
        self._demo_transport_carry_sdf[name] = carry
        return True

    def _demo_transport_pose_delta(self, a: Optional[PoseSample], b: PoseSample) -> float:
        if a is None:
            return float("inf")
        return math.sqrt(
            ((float(a.x) - float(b.x)) ** 2)
            + ((float(a.y) - float(b.y)) ** 2)
            + ((float(a.z) - float(b.z)) ** 2)
        )

    def _demo_transport_enable(self, name: str, pose: PoseSample) -> bool:
        if not self._ensure_demo_transport_sdfs(name):
            self.get_logger().warning(
                f"[ATTACH_BACKEND] demo_transport_missing_sdf object={name}"
            )
            return False
        delete_service, spawn_service, env_prefix = self._ensure_demo_transport_services()
        if not delete_service or not spawn_service:
            self.get_logger().warning(
                "[ATTACH_BACKEND] demo_transport_services_unavailable "
                f"object={name} delete={delete_service} spawn={spawn_service}"
            )
            return False
        ok = False
        for attempt in range(1, 4):
            self._gz_delete_entity_cli(delete_service, env_prefix, name, allow_missing=True)
            if self._demo_transport_respawn_sleep_sec > 0.0:
                time.sleep(self._demo_transport_respawn_sleep_sec)
            ok = self._gz_spawn_entity_cli(
                spawn_service,
                env_prefix,
                name,
                self._demo_transport_carry_sdf[name],
                pose,
            )
            if ok:
                break
            self.get_logger().warning(
                "[ATTACH_BACKEND] demo_transport_enable_retry "
                f"object={name} attempt={attempt}/3"
            )
            time.sleep(max(0.02, self._demo_transport_respawn_sleep_sec))
        if not ok:
            self.get_logger().warning(
                f"[ATTACH_BACKEND] demo_transport_enable_failed object={name}"
            )
            return False
        self._demo_transport_active[name] = self._import_demo_transport_state()(
            name=name,
            last_pose=pose,
            last_spawn_ts=time.time(),
        )
        self.get_logger().info(
            "[ATTACH_BACKEND] demo_transport_enabled "
            f"object={name} mode=respawn_kinematic_no_collision "
            f"pose=({pose.x:.3f},{pose.y:.3f},{pose.z:.3f})"
        )
        return True

    def _demo_transport_update(self, name: str, pose: PoseSample) -> bool:
        state = self._demo_transport_active.get(name)
        if state is None:
            return False
        now = time.time()
        if (
            (now - float(state.last_spawn_ts)) < self._demo_transport_period_sec
            and self._demo_transport_pose_delta(state.last_pose, pose) < self._demo_transport_min_step_m
        ):
            return True
        ok = self._queue_set_pose_with_retry(name, pose)
        if ok:
            state.last_pose = pose
            state.last_spawn_ts = now
            if (now - self._last_demo_transport_log_ts) >= 2.0:
                self.get_logger().info(
                    "[ATTACH_BACKEND] demo_transport_set_pose_ok "
                    f"object={name} pose=({pose.x:.3f},{pose.y:.3f},{pose.z:.3f})"
                )
                self._last_demo_transport_log_ts = now
        elif (now - self._last_demo_transport_log_ts) >= 0.5:
            self.get_logger().warning(
                "[ATTACH_BACKEND] demo_transport_set_pose_fail "
                f"object={name} pose=({pose.x:.3f},{pose.y:.3f},{pose.z:.3f}) "
                "detail=set_pose_queue_failed"
            )
            self._last_demo_transport_log_ts = now
        return ok

    def _demo_transport_restore_dynamic(
        self,
        name: str,
        *,
        detail: str,
        pose: Optional[PoseSample] = None,
    ) -> bool:
        state = self._demo_transport_active.pop(name, None)
        if not self._ensure_demo_transport_sdfs(name):
            self._publish_state(name, False)
            return False
        delete_service, spawn_service, env_prefix = self._ensure_demo_transport_services()
        if not delete_service or not spawn_service:
            self._publish_state(name, False)
            return False
        restore_pose = pose or (state.last_pose if state is not None else None) or self._lookup_pose(name)
        if restore_pose is None:
            self._publish_state(name, False)
            return False
        self._gz_delete_entity_cli(delete_service, env_prefix, name, allow_missing=True)
        if self._demo_transport_respawn_sleep_sec > 0.0:
            time.sleep(self._demo_transport_respawn_sleep_sec)
        ok = self._gz_spawn_entity_cli(
            spawn_service,
            env_prefix,
            name,
            self._demo_transport_dynamic_sdf[name],
            restore_pose,
        )
        self._publish_state(name, False)
        self.get_logger().info(
            "[ATTACH_BACKEND] demo_transport_restore "
            f"object={name} detail={detail} success={str(ok).lower()} "
            f"pose=({restore_pose.x:.3f},{restore_pose.y:.3f},{restore_pose.z:.3f})"
        )
        return ok

    def _activate_demo_transport_attachment(self, name: str, *, method: str) -> bool:
        obj_pose = self._lookup_pose(name)
        tcp_pose = self._lookup_tcp_pose()
        if obj_pose is None or tcp_pose is None:
            self.get_logger().warning(
                "[ATTACH_BACKEND] demo_transport_attach_blocked "
                f"object={name} detail=missing_pose obj_pose={str(obj_pose is not None).lower()} "
                f"tcp_pose={str(tcp_pose is not None).lower()}"
            )
            self._publish_state(name, False)
            return False
        obj_age = self._pose_age_sec(obj_pose)
        tcp_age = self._pose_age_sec(tcp_pose)
        hard_age = max(3.0, self._max_pose_age_sec * 3.0)
        if obj_age > hard_age or tcp_age > hard_age:
            self.get_logger().warning(
                "[ATTACH_BACKEND] demo_transport_attach_blocked "
                f"object={name} detail=stale_pose_hard "
                f"obj_age={obj_age:.3f}s tcp_age={tcp_age:.3f}s "
                f"hard={hard_age:.3f}s tcp_src={self._last_tcp_pose_source}"
            )
            self._publish_state(name, False)
            return False
        if obj_age > self._max_pose_age_sec or tcp_age > self._max_pose_age_sec:
            self.get_logger().warning(
                "[ATTACH_BACKEND] demo_transport_attach_soft_stale "
                f"object={name} obj_age={obj_age:.3f}s tcp_age={tcp_age:.3f}s "
                f"max={self._max_pose_age_sec:.3f}s tcp_src={self._last_tcp_pose_source}"
            )
        attach_dist = math.sqrt(
            (obj_pose.x - tcp_pose.x) ** 2
            + (obj_pose.y - tcp_pose.y) ** 2
            + (obj_pose.z - tcp_pose.z) ** 2
        )
        if attach_dist > self._attach_max_dist_m:
            self.get_logger().warning(
                "[ATTACH_BACKEND] demo_transport_attach_blocked "
                f"object={name} detail=distance_too_large dist={attach_dist:.4f} "
                f"max={self._attach_max_dist_m:.4f}"
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
        AttachedTargetCls = self._import_attached_target()
        attached = AttachedTargetCls(
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
        if not self._demo_transport_enable(name, obj_pose):
            self._attached.pop(name, None)
            self._publish_state(name, False)
            return False
        demo_state = self._demo_transport_active.get(name)
        if demo_state is not None:
            # En el carry del demo priorizamos un lift visualmente rigido frente a
            # preservar la orientacion local del objeto respecto al TCP inclinado.
            # Mantener el offset inicial en world evita que el objeto se quede mas
            # descolgado a medida que la herramienta rota durante el lift.
            demo_state.world_offset_x = float(obj_pose.x - tcp_pose.x)
            demo_state.world_offset_y = float(obj_pose.y - tcp_pose.y)
            raw_world_offset_z = float(obj_pose.z - tcp_pose.z)
            demo_state.world_offset_z = min(
                -0.015,
                raw_world_offset_z + float(self._demo_transport_world_z_compensation_m),
            )
            demo_state.world_qx = float(obj_pose.qx)
            demo_state.world_qy = float(obj_pose.qy)
            demo_state.world_qz = float(obj_pose.qz)
            demo_state.world_qw = float(obj_pose.qw)
            demo_state.use_world_locked_pose = True
        self._publish_state(name, True)
        demo_offset_txt = "n/a"
        if demo_state is not None:
            demo_offset_txt = (
                f"({demo_state.world_offset_x:.3f},{demo_state.world_offset_y:.3f},"
                f"{demo_state.world_offset_z:.3f})"
            )
        self.get_logger().info(
            f"[ATTACH_BACKEND] gazebo_attach_applied=true object={name} method={method} "
            f"dist={attach_dist:.4f}m max={self._attach_max_dist_m:.4f}m "
            f"rel_pos=({attached.offset_x:.3f},{attached.offset_y:.3f},{attached.offset_z:.3f}) "
            f"rel_q=({attached.qx:.3f},{attached.qy:.3f},{attached.qz:.3f},{attached.qw:.3f}) "
            f"demo_world_offset={demo_offset_txt}"
        )
        return True
