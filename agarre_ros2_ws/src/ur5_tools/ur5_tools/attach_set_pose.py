# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/attach_set_pose.py
# Contenido: SetPoseMixin con resolucion + invocacion del SetEntityPose service (C.6).
"""SetPoseMixin: SetEntityPose service + gz CLI fallback (refactor C.6).

Extraido de ``gripper_attach_backend.py`` (lineas 572-760 originales).
6 metodos para invocar SetEntityPose con queue + future + fallback CLI.
"""

from __future__ import annotations

import os
import subprocess
import time
from typing import TYPE_CHECKING, List, Optional, Tuple

try:
    from ros_gz_interfaces.msg import Entity as GzEntity
    from ros_gz_interfaces.srv import SetEntityPose
except Exception:  # pragma: no cover
    GzEntity = None
    SetEntityPose = None

if TYPE_CHECKING:
    from .gripper_attach_backend import PoseSample  # noqa: F401


class SetPoseMixin:
    """SetEntityPose service + queue + gz CLI fallback."""
    def _resolve_set_pose_service(self) -> Optional[str]:
        if self._set_pose_service:
            return self._set_pose_service
        expected = f"/world/{self._world_name}/set_entity_pose"
        services = self.get_service_names_and_types()
        for name, types in services:
            if name == expected and any("SetEntityPose" in t for t in (types or [])):
                self._set_pose_service = expected
                return self._set_pose_service
        for name, types in services:
            if f"/world/{self._world_name}/" not in name:
                continue
            if any("SetEntityPose" in t for t in (types or [])):
                self._set_pose_service = name
                return self._set_pose_service
        return None

    def _ensure_set_pose_client(self) -> bool:
        if SetEntityPose is None:
            return False
        # Re-resolvemos dinamicamente porque los servicios de Gazebo pueden aparecer
        # despues de arrancar este nodo.
        if not self._set_pose_service:
            resolved = self._resolve_set_pose_service()
            if resolved:
                self._set_pose_service = resolved
                self.get_logger().info(
                    f"[ATTACH_BACKEND] set_pose_service resolved={resolved}"
                )
        if not self._set_pose_service:
            return False
        if (
            self._set_pose_client is None
            or getattr(self._set_pose_client, "srv_name", "") != self._set_pose_service
        ):
            self._set_pose_client = self.create_client(
                SetEntityPose, self._set_pose_service
            )
        return self._set_pose_client.wait_for_service(timeout_sec=self._service_timeout_sec)

    def _gz_env_prefix(self) -> str:
        exports: List[str] = []
        gz_partition = os.environ.get("GZ_PARTITION", "").strip()
        if not gz_partition:
            part_file = os.path.join(self._ws_dir, "log", "gz_partition.txt")
            try:
                with open(part_file, "r", encoding="utf-8") as f:
                    gz_partition = f.read().strip()
            except Exception:
                gz_partition = ""
        if gz_partition:
            exports.append(f"export GZ_PARTITION='{gz_partition}'")
        gz_ip = os.environ.get("GZ_IP", "").strip()
        if gz_ip:
            exports.append(f"export GZ_IP='{gz_ip}'")
        if not exports:
            return ""
        return " ; ".join(exports) + " ; "

    def _set_pose_via_gz_cli(self, name: str, pose: PoseSample) -> Tuple[bool, str]:
        req = (
            f'name: "{name}" '
            f"position {{x: {float(pose.x)} y: {float(pose.y)} z: {float(pose.z)}}} "
            f"orientation {{x: {float(pose.qx)} y: {float(pose.qy)} z: {float(pose.qz)} w: {float(pose.qw)}}}"
        )
        env_prefix = self._gz_env_prefix()
        world_scope = f"/world/{self._world_name}"
        candidates: List[str] = []
        preferred = [
            f"{world_scope}/set_entity_pose",
            f"{world_scope}/set_pose/blocking",
            f"{world_scope}/set_pose",
        ]
        cached = (self._gz_set_pose_service or "").strip()
        if cached.endswith("/set_entity_pose") or cached.endswith("/set_pose/blocking"):
            candidates.append(cached)
        candidates.extend(preferred)
        if cached and cached not in candidates:
            candidates.append(cached)
        dedup: List[str] = []
        for svc in candidates:
            if svc and svc not in dedup:
                dedup.append(svc)
        last_detail = "gz_set_pose_service_unavailable"
        for svc in dedup:
            cmd = (
                f"{env_prefix}gz service -s {svc} "
                "--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean "
                f"--timeout {int(self._gz_service_timeout_ms)} --req '{req}'"
            )
            try:
                res = subprocess.run(
                    ["bash", "-lc", cmd],
                    text=True,
                    capture_output=True,
                    timeout=self._gz_cmd_timeout_sec,
                )
            except Exception as exc:
                last_detail = f"gz_service_exception:{exc}"
                continue
            if res.returncode != 0:
                err = (res.stderr or "").strip()
                last_detail = f"gz_service_rc={res.returncode}:{err or 'n/a'}"
                continue
            out = (res.stdout or "").strip()
            ok = ("data: true" in out.lower()) or ("true" == out.lower())
            if not ok:
                err = (res.stderr or "").strip()
                last_detail = f"gz_service_false:{err or out or 'empty'}"
                continue
            if self._gz_set_pose_service != svc:
                self._gz_set_pose_service = svc
                self.get_logger().info(
                    f"[ATTACH_BACKEND] gz_set_pose_service resolved={svc}"
                )
            return True, "gz_service_ok"
        return False, last_detail

    def _queue_set_pose(self, name: str, pose: PoseSample) -> bool:
        if self._ensure_set_pose_client():
            if self._set_pose_future is not None and not self._set_pose_future.done():
                age = 0.0
                if self._set_pose_future_start_ts > 0.0:
                    age = max(0.0, time.time() - self._set_pose_future_start_ts)
                if age < self._set_pose_future_timeout_sec:
                    return False
                pending_name = name
                if self._set_pose_pending is not None:
                    pending_name = self._set_pose_pending[0]
                try:
                    self._set_pose_future.cancel()
                except Exception:
                    pass
                self._set_pose_future = None
                self._set_pose_pending = None
                self._set_pose_future_start_ts = 0.0
                now = time.time()
                if (now - self._last_set_pose_timeout_log_ts) >= 0.8:
                    self.get_logger().warning(
                        "[ATTACH_BACKEND] set_pose_future_stale "
                        f"object={pending_name} age={age:.3f}s "
                        f"timeout={self._set_pose_future_timeout_sec:.3f}s "
                        "action=cancel_and_retry"
                    )
                    self._last_set_pose_timeout_log_ts = now
                if self._gz_cli_fallback:
                    ok, detail = self._set_pose_via_gz_cli(name, pose)
                    if (now - self._last_apply_log_ts) >= 0.8:
                        self.get_logger().info(
                            f"[ATTACH_BACKEND] gazebo_attach_applied={str(ok).lower()} "
                            f"object={name} method={self._attach_mode} detail={detail}"
                        )
                        self._last_apply_log_ts = now
                    if ok:
                        return True
            req = SetEntityPose.Request()
            req.entity.name = name
            req.entity.type = GzEntity.MODEL if GzEntity is not None else 2
            req.pose.position.x = float(pose.x)
            req.pose.position.y = float(pose.y)
            req.pose.position.z = float(pose.z)
            req.pose.orientation.x = float(pose.qx)
            req.pose.orientation.y = float(pose.qy)
            req.pose.orientation.z = float(pose.qz)
            req.pose.orientation.w = float(pose.qw)
            self._set_pose_pending = (name, pose)
            self._set_pose_future = self._set_pose_client.call_async(req)
            self._set_pose_future_start_ts = time.time()
            self._set_pose_future.add_done_callback(self._on_set_pose_done)
            return True
        ok, detail = self._set_pose_via_gz_cli(name, pose)
        now = time.time()
        if (now - self._last_apply_log_ts) >= 0.8:
            self.get_logger().info(
                f"[ATTACH_BACKEND] gazebo_attach_applied={str(ok).lower()} "
                f"object={name} method={self._attach_mode} detail={detail}"
            )
            self._last_apply_log_ts = now
        return ok

    def _queue_set_pose_with_retry(self, name: str, pose: PoseSample) -> bool:
        attempts = max(1, int(self._attach_initial_queue_retries))
        for idx in range(attempts):
            if self._queue_set_pose(name, pose):
                return True
            if idx + 1 < attempts:
                time.sleep(self._attach_retry_sleep_sec)
        return False
