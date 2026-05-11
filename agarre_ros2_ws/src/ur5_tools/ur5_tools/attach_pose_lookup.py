# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/attach_pose_lookup.py
# Contenido: PoseLookupMixin con TF lookups + pose composition (C.5).
"""PoseLookupMixin: TF lookups + pose composition (refactor C.5).

Extraido de ``gripper_attach_backend.py`` (lineas 461-708 originales).
6 metodos para lookup de pose en world/base + composicion de pose
desde TF/joint_state + fallback chain para TCP.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Dict, Optional

from rclpy.duration import Duration
from rclpy.time import Time

from .attach_math import _quat_multiply, _quat_normalize, _rotate_vector  # noqa: F401

if TYPE_CHECKING:
    from .gripper_attach_backend import PoseSample  # noqa: F401


def _get_PoseSample_class():
    from .gripper_attach_backend import PoseSample
    return PoseSample


def _get_TCP_FALLBACKS():
    from .gripper_attach_backend import TCP_FALLBACKS
    return TCP_FALLBACKS


class PoseLookupMixin:
    """TF pose lookup + composition + fallback chain."""
    def _lookup_pose(self, key: str) -> Optional[PoseSample]:
        if key in self._pose_cache:
            return self._pose_cache[key]
        alt = key.lstrip("/")
        if alt in self._pose_cache:
            return self._pose_cache[alt]
        scoped = f"ur5_rg2::{alt}"
        if scoped in self._pose_cache:
            return self._pose_cache[scoped]
        suffix = f"::{alt}"
        for name, pose in self._pose_cache.items():
            if name.endswith(suffix):
                return pose
        return None

    def _lookup_tf_pose(
        self,
        parent_frame: str,
        child_frame: str,
        *,
        timeout_sec: float = 0.05,
    ) -> Optional[PoseSample]:
        try:
            tf = self._tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                Time(),
                timeout=Duration(seconds=timeout_sec),
            )
            tr = tf.transform.translation
            rot = tf.transform.rotation
            stamp = tf.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            return _get_PoseSample_class()(
                x=float(tr.x),
                y=float(tr.y),
                z=float(tr.z),
                qx=float(rot.x),
                qy=float(rot.y),
                qz=float(rot.z),
                qw=float(rot.w),
                stamp_ns=stamp_ns,
            )
        except Exception:
            return None

    def _compose_pose(
        self,
        parent_pose: PoseSample,
        child_pose: PoseSample,
    ) -> PoseSample:
        parent_q = _quat_normalize(
            (
                float(parent_pose.qx),
                float(parent_pose.qy),
                float(parent_pose.qz),
                float(parent_pose.qw),
            )
        )
        rel_q = _quat_normalize(
            (
                float(child_pose.qx),
                float(child_pose.qy),
                float(child_pose.qz),
                float(child_pose.qw),
            )
        )
        rel_pos_world = _rotate_vector(
            parent_q,
            (float(child_pose.x), float(child_pose.y), float(child_pose.z)),
        )
        world_q = _quat_multiply(parent_q, rel_q)
        return _get_PoseSample_class()(
            x=float(parent_pose.x + rel_pos_world[0]),
            y=float(parent_pose.y + rel_pos_world[1]),
            z=float(parent_pose.z + rel_pos_world[2]),
            qx=float(world_q[0]),
            qy=float(world_q[1]),
            qz=float(world_q[2]),
            qw=float(world_q[3]),
            stamp_ns=max(int(parent_pose.stamp_ns), int(child_pose.stamp_ns)),
        )

    def _pose_with_stamp(self, pose: PoseSample, stamp_ns: int) -> PoseSample:
        return _get_PoseSample_class()(
            x=float(pose.x),
            y=float(pose.y),
            z=float(pose.z),
            qx=float(pose.qx),
            qy=float(pose.qy),
            qz=float(pose.qz),
            qw=float(pose.qw),
            stamp_ns=int(stamp_ns),
        )

    def _fallback_tcp_pose(self) -> Optional[PoseSample]:
        fallback = _get_TCP_FALLBACKS().get(self._tcp_frame)
        if not fallback:
            return None
        parent_frame, offset = fallback
        # Gazebo `/pose/info` es consistente para objetos libres, pero los links del
        # robot en este workspace no comparten necesariamente esa misma convencion
        # en world. Derivamos los fallbacks del TCP desde TF vivo para mantener
        # coherente la geometria objeto-vs-TCP.
        parent_pose = self._lookup_tf_pose(self._world_frame, parent_frame)
        if parent_pose is None:
            return None
        child_pose = _get_PoseSample_class()(
            x=float(offset[0]),
            y=float(offset[1]),
            z=float(offset[2]),
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0,
            stamp_ns=int(parent_pose.stamp_ns),
        )
        try:
            return self._compose_pose(parent_pose, child_pose)
        except Exception:
            return None

    def _lookup_tcp_pose(self) -> Optional[PoseSample]:
        tf_pose = self._lookup_tf_pose(self._world_frame, self._tcp_frame)
        base_chain_pose: Optional[PoseSample] = None
        joint_state_chain_pose: Optional[PoseSample] = None
        world_base: Optional[PoseSample] = None
        base_tcp: Optional[PoseSample] = None
        joint_state_base_tcp: Optional[PoseSample] = None
        world_base_age = float("inf")
        base_tcp_age = float("inf")
        joint_state_age = float("inf")
        stable_world_base_mode = "none"
        if (
            self._base_frame
            and self._base_frame != self._world_frame
            and self._base_frame != self._tcp_frame
        ):
            world_base = self._lookup_tf_pose(self._world_frame, self._base_frame)
            base_tcp = self._lookup_tf_pose(self._base_frame, self._tcp_frame)
            if self._base_frame == "base_link":
                joint_state_base_tcp = self._joint_state_tcp_base_pose()
            if world_base is not None:
                world_base_age = float(self._pose_age_sec(world_base))
                if self._pose_age_ok(world_base):
                    self._stable_world_base_pose = world_base
                    stable_world_base_mode = "live"
            stable_world_base = None
            if world_base is not None and self._pose_age_ok(world_base):
                stable_world_base = world_base
            elif self._stable_world_base_pose is not None and base_tcp is not None:
                # world->base_link es fijo en el world cargado, asi que un ancla de
                # base buena conocida sigue siendo valida geometricamente mientras
                # base_link->tcp aporta el movimiento vivo del manipulador.
                stable_world_base = self._pose_with_stamp(
                    self._stable_world_base_pose,
                    int(base_tcp.stamp_ns),
                )
                stable_world_base_mode = "cached_fixed_base"
            if base_tcp is not None:
                base_tcp_age = float(self._pose_age_sec(base_tcp))
            if stable_world_base is not None and base_tcp is not None:
                try:
                    base_chain_pose = self._compose_pose(stable_world_base, base_tcp)
                except Exception:
                    base_chain_pose = None
            if joint_state_base_tcp is not None:
                joint_state_age = float(self._pose_age_sec(joint_state_base_tcp))
            if stable_world_base is not None and joint_state_base_tcp is not None:
                try:
                    joint_state_chain_pose = self._compose_pose(
                        stable_world_base,
                        joint_state_base_tcp,
                    )
                except Exception:
                    joint_state_chain_pose = None
        tool_fallback_pose = self._fallback_tcp_pose()
        # No usamos directamente la cache de poses de Gazebo para el TCP. En links
        # del robot puede parecer fresca pero ser geometricamente inconsistente con
        # las poses world de los objetos.
        named_candidates = [
            ("world_tcp", tf_pose),
            ("joint_state_chain", joint_state_chain_pose),
            ("tool_fallback", tool_fallback_pose),
            ("base_chain", base_chain_pose),
        ]
        diag: Dict[str, float | str | bool] = {
            "world_base_ok": world_base is not None,
            "base_tcp_ok": base_tcp is not None,
            "joint_state_base_ok": joint_state_base_tcp is not None,
            "pose_cache_policy": "disabled_for_tcp_lookup",
            "world_base_age": world_base_age,
            "base_tcp_age": base_tcp_age,
            "joint_state_age": joint_state_age,
            "stable_world_base_ok": self._stable_world_base_pose is not None,
            "stable_world_base_age": (
                float("inf")
                if self._stable_world_base_pose is None
                else float(self._pose_age_sec(self._stable_world_base_pose))
            ),
            "stable_world_base_mode": stable_world_base_mode,
            "world_base_policy": "reuse_fixed_base_cache_when_live_stale",
        }
        for name, pose in named_candidates:
            diag[f"{name}_ok"] = pose is not None
            diag[f"{name}_age"] = float("inf") if pose is None else float(self._pose_age_sec(pose))
        self._last_tcp_pose_diag = diag
        # Priorizamos TF world->tcp directo cuando está fresco. En este stack
        # es la referencia más confiable para coherencia física con Gazebo.
        priority_order = {
            "world_tcp": 0,
            "joint_state_chain": 1,
            "tool_fallback": 2,
            "base_chain": 3,
        }
        fresh_candidates = []
        for name, pose in named_candidates:
            if pose is None:
                continue
            age = float(self._pose_age_sec(pose))
            if -0.25 <= age <= self._max_pose_age_sec:
                fresh_candidates.append((age, priority_order.get(name, 99), name, pose))
        if fresh_candidates:
            _age, _priority, best_name, best_pose = min(fresh_candidates)
            self._last_tcp_pose_source = str(best_name)
            return best_pose
        # Cuando todos los candidatos parecen viejos, priorizamos las cadenas
        # geometricamente coherentes de FK por joint-state / `world->base_link +
        # base_link->tcp` frente a un world->tcp directo envejecido. En nuestro
        # stack world->base_link es fijo, asi que base cacheada + cinematica viva
        # relativa a base sigue siendo coherente durante el carry aunque la muestra
        # directa world->tcp vaya con varios segundos de retraso.
        stale_priority_order = {
            "world_tcp": 0,
            "joint_state_chain": 1,
            "base_chain": 2,
            "tool_fallback": 3,
        }
        stale_candidates = [
            (stale_priority_order.get(name, 99), name, pose)
            for name, pose in named_candidates
            if pose is not None
        ]
        if not stale_candidates:
            self._last_tcp_pose_source = "none"
            return None
        _priority, best_name, best_pose = min(stale_candidates)
        self._last_tcp_pose_source = str(best_name)
        return best_pose
