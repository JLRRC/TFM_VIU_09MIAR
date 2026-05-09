# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/attach_gz_cli.py
# Contenido: GzCliMixin con helpers de gz service / spawn / delete entity (C.2).
"""GzCliMixin: helpers de gz service / spawn / delete entity (refactor C.2).

Extraido de ``gripper_attach_backend.py`` (lineas 536-650 originales).
Cuatro metodos que invocan el CLI ``gz`` para introspectar y mutar
entidades en Gazebo Sim moderno.

Sin estado propio. Requiere que la clase concreta herede de ``Node``
(usa ``self.get_logger()``).
"""

from __future__ import annotations

import subprocess
from typing import TYPE_CHECKING, List, Optional, Tuple

if TYPE_CHECKING:
    from .gripper_attach_backend import PoseSample  # noqa: F401


class GzCliMixin:
    """gz service exists / resolve + delete/spawn entity via CLI."""
    def _gz_service_exists(self, env_prefix: str, service: str) -> bool:
        cmd = f"{env_prefix}gz service -s {service} -i"
        try:
            result = subprocess.run(
                ["bash", "-lc", cmd],
                text=True,
                capture_output=True,
                timeout=max(0.5, self._gz_cmd_timeout_sec),
            )
        except Exception:
            return False
        return result.returncode == 0

    def _resolve_gz_service(self, candidates: List[str], env_prefix: str) -> Optional[str]:
        for svc in candidates:
            if svc and self._gz_service_exists(env_prefix, svc):
                return svc
        return None

    def _ensure_demo_transport_services(self) -> Tuple[Optional[str], Optional[str], str]:
        env_prefix = self._gz_env_prefix()
        if not self._gz_delete_service:
            # En este camino del demo, los servicios blocking del world son la via
            # estable en runtime. Evitamos la pre-introspeccion: `gz service -i`
            # ha sido poco fiable incluso cuando la llamada directa si funciona.
            self._gz_delete_service = f"/world/{self._world_name}/remove/blocking"
            if not self._gz_delete_service:
                self.get_logger().warning(
                    f"[ATTACH_BACKEND] demo_transport_missing_delete_service world={self._world_name}"
                )
        if not self._gz_spawn_service:
            self._gz_spawn_service = f"/world/{self._world_name}/create/blocking"
            if not self._gz_spawn_service:
                self.get_logger().warning(
                    f"[ATTACH_BACKEND] demo_transport_missing_spawn_service world={self._world_name}"
                )
        return self._gz_delete_service, self._gz_spawn_service, env_prefix

    def _gz_delete_entity_cli(
        self,
        service: str,
        env_prefix: str,
        name: str,
        *,
        allow_missing: bool = False,
    ) -> bool:
        req = f'name: "{name}" type: MODEL'
        cmd = (
            f"{env_prefix}gz service -s {service} "
            "--reqtype gz.msgs.Entity --reptype gz.msgs.Boolean "
            f"--timeout {int(self._gz_service_timeout_ms)} --req '{req}'"
        )
        try:
            result = subprocess.run(
                ["bash", "-lc", cmd],
                text=True,
                capture_output=True,
                timeout=max(0.5, self._gz_cmd_timeout_sec),
            )
        except Exception as exc:
            self.get_logger().warning(
                f"[ATTACH_BACKEND] demo_transport_delete_exception object={name} err={exc}"
            )
            return False
        if result.returncode != 0:
            self.get_logger().warning(
                "[ATTACH_BACKEND] demo_transport_delete_failed "
                f"object={name} rc={result.returncode} stdout={result.stdout.strip()} stderr={result.stderr.strip()}"
            )
            return False if not allow_missing else True
        ok = "data: true" in (result.stdout or "")
        return ok or allow_missing

    def _gz_spawn_entity_cli(
        self,
        service: str,
        env_prefix: str,
        name: str,
        sdf: str,
        pose: "PoseSample",
    ) -> bool:
        sdf_escaped = sdf.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "")
        req = (
            f'name: "{name}" '
            "allow_renaming: false "
            f'sdf: "{sdf_escaped}" '
            "pose {"
            f"position {{x: {float(pose.x)} y: {float(pose.y)} z: {float(pose.z)}}} "
            f"orientation {{x: {float(pose.qx)} y: {float(pose.qy)} z: {float(pose.qz)} w: {float(pose.qw)}}}"
            "}"
        )
        cmd = (
            f"{env_prefix}gz service -s {service} "
            "--reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean "
            f"--timeout {int(max(600, self._gz_service_timeout_ms))} --req '{req}'"
        )
        try:
            result = subprocess.run(
                ["bash", "-lc", cmd],
                text=True,
                capture_output=True,
                timeout=max(1.5, self._gz_cmd_timeout_sec * 2.0),
            )
        except Exception as exc:
            self.get_logger().warning(
                f"[ATTACH_BACKEND] demo_transport_spawn_exception object={name} err={exc}"
            )
            return False
        ok = result.returncode == 0 and "data: true" in (result.stdout or "")
        if not ok:
            self.get_logger().warning(
                "[ATTACH_BACKEND] demo_transport_spawn_failed "
                f"object={name} rc={result.returncode} stdout={result.stdout.strip()} stderr={result.stderr.strip()}"
            )
        return ok
