#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/release_objects_service.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
"""ROS 2 service to release DROP objects in Gazebo (spawn as dynamic on demand)."""

from __future__ import annotations

import copy
import math
import os
import shutil
import subprocess
import time
from typing import Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Pose
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from std_srvs.srv import Trigger

from .param_utils import (
    read_float_param,
    read_int_param,
    read_str_list_param,
    read_str_param,
)

DROP_OBJECTS = [
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
DROP_ANCHOR_NAME = "drop_anchor"


class ReleaseObjectsService(Node):
    def __init__(self) -> None:
        super().__init__("release_objects_service")
        self.declare_parameter("object_names", DROP_OBJECTS)
        self.declare_parameter("attempts", 1)
        self.declare_parameter("attempt_sleep", 0.0)
        self.declare_parameter("world_sdf", "")
        self.declare_parameter("world_name", "")
        self.declare_parameter("delete_service", "")
        self.declare_parameter("spawn_service", "")
        self.declare_parameter("use_gz_transport", True)
        self.declare_parameter("service_timeout_sec", 0.35)
        self.declare_parameter("service_wait_sec", 0.2)
        self.declare_parameter("service_wait_total_sec", 0.8)
        self.declare_parameter("gz_list_wait_sec", 0.15)
        self.declare_parameter("gz_list_total_sec", 0.6)
        self.declare_parameter("gz_cmd_timeout_sec", 3.5)
        self.declare_parameter("gz_delete_timeout_ms", 300)
        self.declare_parameter("gz_spawn_timeout_ms", 600)
        self.declare_parameter("drop_anchor_name", DROP_ANCHOR_NAME)
        self._service = self.create_service(
            Trigger, "release_objects", self._handle_release
        )
        self._world_name: Optional[str] = None
        self._drop_cache_loaded = False
        self._drop_sdf: Dict[str, str] = {}
        self._drop_pose: Dict[str, Pose] = {}
        self._drop_anchor_name = str(
            self.get_parameter("drop_anchor_name").value or DROP_ANCHOR_NAME
        ).strip() or DROP_ANCHOR_NAME
        self._drop_anchor_sdf: str = ""
        self._drop_anchor_pose: Optional[Pose] = None
        self._last_error: str = ""
        self._last_release_ok_ts: float = 0.0
        self._delete_client: Optional[object] = None
        self._spawn_client: Optional[object] = None

    def _handle_release(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self._last_error = ""
        ready, reason = self._backend_ready()
        if not ready:
            response.success = False
            response.message = f"NOT_READY: {reason}"
            self.get_logger().warn(f"[PHYSICS][DROP] {response.message}")
            return response
        names = read_str_list_param(self, "object_names", default=DROP_OBJECTS)
        if not names:
            response.success = False
            response.message = "object_names vacío"
            return response
        if (time.monotonic() - self._last_release_ok_ts) < 0.8:
            response.success = True
            response.message = "DROP ya liberados (idempotente)"
            self.get_logger().info("[PHYSICS][DROP] idempotent short-circuit")
            return response

        attempts = read_int_param(self, "attempts", 1, min_value=1)
        sleep_s = read_float_param(self, "attempt_sleep", 0.0, min_value=0.0)
        ok = False
        for _idx in range(attempts):
            ok = self._spawn_drop_objects(names)
            if ok:
                break
            if sleep_s > 0.0:
                time.sleep(sleep_s)
        response.success = ok
        if ok:
            response.message = f"DROP liberados ({len(names)} objs)"
            self._last_release_ok_ts = time.monotonic()
        else:
            response.message = self._last_error or "Fallo liberando DROP (spawn)"
        return response

    def _set_error(self, message: str) -> None:
        self._last_error = str(message or "").strip()

    def _backend_ready(self) -> Tuple[bool, str]:
        try:
            world_name = self._resolve_world_name()
            if not world_name:
                return False, "world_name no disponible"
            use_gz = bool(self.get_parameter("use_gz_transport").value)
            if use_gz:
                # Fail-fast: avoid expensive `gz service -l` here.
                return True, "ok"
            if not self._ensure_clients(world_name):
                return False, "ros_gz services no disponibles"
            return True, "ok"
        except Exception as exc:
            return False, f"backend exception: {exc}"

    def _spawn_drop_objects(self, names: List[str]) -> bool:
        if not self._drop_cache_loaded:
            if not self._load_drop_models():
                return False
        world_name = self._resolve_world_name()
        if not world_name:
            self.get_logger().error("[PHYSICS][DROP] world_name no disponible")
            self._set_error("world_name no disponible")
            return False
        if not self._guard_pose_info(world_name, names):
            return False
        use_gz = bool(self.get_parameter("use_gz_transport").value)
        if use_gz:
            return self._spawn_drop_objects_gz(world_name, names)
        if not self._ensure_clients(world_name):
            self.get_logger().warn(
                "[PHYSICS][DROP] ros services no disponibles; usando gz transport"
            )
            return self._spawn_drop_objects_gz(world_name, names)

        timeout = read_float_param(self, "service_timeout_sec", 1.0, min_value=0.1)
        if not self._reset_drop_anchor_ros(timeout):
            self.get_logger().error("[PHYSICS][DROP] reset drop_anchor falló (ros)")
            self._set_error("reset drop_anchor falló (ros)")
            return False
        delete_ok = True
        for name in names:
            if name not in self._drop_sdf:
                self.get_logger().warn(f"[PHYSICS][DROP] sin SDF para {name}")
                delete_ok = False
                continue
            if not self._call_delete(name, timeout):
                delete_ok = False
        spawn_ok = True
        for name in names:
            sdf = self._drop_sdf.get(name)
            pose = self._drop_pose.get(name)
            if not sdf or pose is None:
                spawn_ok = False
                continue
            if not self._call_spawn(name, sdf, pose, timeout):
                spawn_ok = False
        return delete_ok and spawn_ok

    def _guard_pose_info(self, world_name: str, names: List[str]) -> bool:
        env_prefix = self._gz_env_prefix()
        pose_names = self._list_pose_info_entity_names(env_prefix, world_name)
        if not pose_names:
            msg = "pose/info vacío o Gazebo no disponible"
            self.get_logger().error(f"[PHYSICS][DROP] {msg}")
            self._set_error(msg)
            return False
        required = [self._drop_anchor_name] + list(names)
        pose_name_set = set(pose_names)
        missing_required = [name for name in required if name not in pose_name_set]
        if missing_required:
            msg = (
                "pose/info vacío o Gazebo no disponible: "
                f"faltan entidades {missing_required}"
            )
            self.get_logger().error(f"[PHYSICS][DROP] {msg}")
            self._set_error(msg)
            return False
        return True

    def _resolve_world_name(self) -> str:
        if self._world_name:
            return self._world_name
        world_name = read_str_param(self, "world_name", "").strip()
        if world_name:
            self._world_name = world_name
            return world_name
        world_sdf = read_str_param(self, "world_sdf", "").strip()
        if not world_sdf:
            return ""
        try:
            tree = ET.parse(world_sdf)
            root = tree.getroot()
            world_elem = root.find("world")
            if world_elem is not None:
                self._world_name = world_elem.get("name") or ""
        except Exception as exc:
            self.get_logger().error(f"[PHYSICS][DROP] world_sdf invalido: {exc}")
        return self._world_name or ""

    def _ensure_clients(self, world_name: str) -> bool:
        delete_service = read_str_param(self, "delete_service", "").strip()
        spawn_service = read_str_param(self, "spawn_service", "").strip()
        if not delete_service:
            delete_service = (
                self._find_service_by_type(world_name, "DeleteEntity")
                or f"/world/{world_name}/remove"
            )
        if not spawn_service:
            spawn_service = (
                self._find_service_by_type(world_name, "SpawnEntity")
                or f"/world/{world_name}/create"
            )
        if (
            self._delete_client is None
            or getattr(
                self._delete_client,
                "srv_name",
                None,
            )
            != delete_service
        ):
            self._delete_client = self.create_client(DeleteEntity, delete_service)
        if (
            self._spawn_client is None
            or getattr(
                self._spawn_client,
                "srv_name",
                None,
            )
            != spawn_service
        ):
            self._spawn_client = self.create_client(SpawnEntity, spawn_service)
        wait_sec = read_float_param(self, "service_wait_sec", 1.5, min_value=0.1)
        wait_total = read_float_param(
            self, "service_wait_total_sec", 4.0, min_value=0.2
        )
        deadline = time.monotonic() + wait_total
        while time.monotonic() < deadline:
            delete_ready = self._delete_client.wait_for_service(timeout_sec=wait_sec)
            spawn_ready = self._spawn_client.wait_for_service(timeout_sec=wait_sec)
            if delete_ready and spawn_ready:
                return True
            time.sleep(0.1)
        if not self._delete_client.service_is_ready():
            self.get_logger().error(
                f"[PHYSICS][DROP] servicio delete no disponible: {delete_service}"
            )
        if not self._spawn_client.service_is_ready():
            self.get_logger().error(
                f"[PHYSICS][DROP] servicio spawn no disponible: {spawn_service}"
            )
        return False

    def _call_delete(
        self,
        name: str,
        timeout: float,
        *,
        entity_type: int = Entity.MODEL,
        allow_missing: bool = False,
        log_context: str = "delete",
    ) -> bool:
        req = DeleteEntity.Request()
        req.entity = Entity(name=name, type=entity_type)
        future = self._delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            self.get_logger().warn(f"[PHYSICS][DROP] {log_context} timeout {name}")
            return False
        result = future.result()
        if not result.success:
            status = str(getattr(result, "status_message", "") or "").strip()
            if allow_missing:
                self.get_logger().info(
                    f"[PHYSICS][DROP] {log_context} skipped (missing) {name} {status}".strip()
                )
                return True
            self.get_logger().warn(
                f"[PHYSICS][DROP] {log_context} fallo {name} {status}".strip()
            )
            return False
        return True

    def _call_spawn(self, name: str, sdf: str, pose: Pose, timeout: float) -> bool:
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.allow_renaming = False
        req.entity_factory.sdf = sdf
        req.entity_factory.pose = pose
        req.entity_factory.relative_to = "world"
        future = self._spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            self.get_logger().warn(f"[PHYSICS][DROP] spawn timeout {name}")
            return False
        if not future.result().success:
            self.get_logger().warn(f"[PHYSICS][DROP] spawn fallo {name}")
            return False
        return True

    def _drop_anchor_cleanup_targets(
        self,
        *,
        include_primary: bool,
    ) -> List[Tuple[str, str]]:
        anchor_name = self._drop_anchor_name
        model_names: List[str] = []
        if include_primary:
            model_names.append(anchor_name)
        for idx in (1, 2):
            model_names.append(f"{anchor_name}({idx})")
        targets: List[Tuple[str, str]] = []
        for model_name in model_names:
            targets.append((model_name, "MODEL"))
            targets.append((f"{model_name}::link", "LINK"))
            targets.append((f"{model_name}/link", "LINK"))
        return targets

    def _reset_drop_anchor_ros(self, timeout: float) -> bool:
        anchor_name = self._drop_anchor_name
        if not self._drop_anchor_sdf or self._drop_anchor_pose is None:
            self.get_logger().error("[PHYSICS][DROP] sin SDF/pose de drop_anchor")
            self._set_error("sin SDF/pose de drop_anchor")
            return False
        link_type = int(getattr(Entity, "LINK", Entity.MODEL))
        self.get_logger().info("[PHYSICS][DROP] drop_anchor pre-clean (ros)")
        for target_name, target_type in self._drop_anchor_cleanup_targets(
            include_primary=True
        ):
            entity_type = Entity.MODEL if target_type == "MODEL" else link_type
            self._call_delete(
                target_name,
                timeout,
                entity_type=entity_type,
                allow_missing=True,
                log_context="drop_anchor remove",
            )
        if not self._call_spawn(
            anchor_name, self._drop_anchor_sdf, self._drop_anchor_pose, timeout
        ):
            self.get_logger().error("[PHYSICS][DROP] drop_anchor spawn fallo (ros)")
            self._set_error("drop_anchor spawn fallo (ros)")
            return False
        self.get_logger().info(
            "[PHYSICS][DROP] drop_anchor spawn ok name=drop_anchor (allow_renaming=false)"
        )
        return True

    def _reset_drop_anchor_gz(
        self,
        *,
        env_prefix: str,
        delete_service: str,
        spawn_service: str,
    ) -> bool:
        anchor_name = self._drop_anchor_name
        if not self._drop_anchor_sdf or self._drop_anchor_pose is None:
            self.get_logger().error("[PHYSICS][DROP] sin SDF/pose de drop_anchor")
            self._set_error("sin SDF/pose de drop_anchor")
            return False
        world_name = self._resolve_world_name()
        if world_name:
            names = self._list_pose_info_entity_names(env_prefix, world_name)
            if anchor_name in names:
                self.get_logger().info(
                    "[PHYSICS][DROP] drop_anchor presente; se conserva sin respawn"
                )
                return True
        self.get_logger().info("[PHYSICS][DROP] drop_anchor pre-clean (gz)")
        for target_name, target_type in self._drop_anchor_cleanup_targets(
            include_primary=True
        ):
            self._gz_delete_entity(
                env_prefix,
                delete_service,
                target_name,
                entity_type=target_type,
                allow_missing=True,
                log_context="drop_anchor remove",
            )
        if not self._gz_spawn_entity(
            env_prefix,
            spawn_service,
            anchor_name,
            self._drop_anchor_sdf,
            self._drop_anchor_pose,
        ):
            self.get_logger().error("[PHYSICS][DROP] drop_anchor spawn fallo (gz)")
            self._set_error("drop_anchor spawn fallo (gz)")
            return False
        if not self._validate_drop_anchor_name_gz(env_prefix, anchor_name):
            return False
        self.get_logger().info(
            "[PHYSICS][DROP] drop_anchor spawn ok name=drop_anchor (allow_renaming=false)"
        )
        return True

    def _validate_drop_anchor_name_gz(self, env_prefix: str, anchor_name: str) -> bool:
        world_name = self._resolve_world_name()
        if not world_name:
            msg = "validacion drop_anchor falló: world_name vacío"
            self.get_logger().error(f"[PHYSICS][DROP] {msg}")
            self._set_error(msg)
            return False
        names = self._list_pose_info_entity_names(env_prefix, world_name)
        if not names:
            msg = "pose/info vacío o Gazebo no disponible"
            self.get_logger().error(f"[PHYSICS][DROP] {msg}")
            self._set_error(msg)
            return False
        has_exact = anchor_name in names
        duplicates = [n for n in names if n.startswith(f"{anchor_name}(")]
        if not has_exact:
            msg = f"validacion drop_anchor falló: '{anchor_name}' no presente"
            self.get_logger().error(f"[PHYSICS][DROP] {msg}")
            self._set_error(msg)
            return False
        if duplicates:
            msg = f"validacion drop_anchor falló: duplicados {duplicates}"
            self.get_logger().error(f"[PHYSICS][DROP] {msg}")
            self._set_error(msg)
            return False
        return True

    def _list_pose_info_entity_names(self, env_prefix: str, world_name: str) -> List[str]:
        cmd_timeout = read_float_param(self, "gz_cmd_timeout_sec", 3.0, min_value=0.5)
        topic = f"/world/{world_name}/pose/info"
        cmd = f"{env_prefix}gz topic -t {topic} -n 1 -e"
        try:
            result = subprocess.run(
                ["bash", "-lc", cmd],
                text=True,
                capture_output=True,
                timeout=max(0.25, cmd_timeout),
            )
        except Exception:
            return []
        if result.returncode != 0:
            return []
        names: List[str] = []
        for line in (result.stdout or "").splitlines():
            line = line.strip()
            if not line.startswith("name:"):
                continue
            try:
                _, value = line.split(":", 1)
            except ValueError:
                continue
            value = value.strip().strip('"').strip("'")
            if value:
                names.append(value)
        return names

    def _load_drop_models(self) -> bool:
        world_sdf = read_str_param(self, "world_sdf", "").strip()
        if not world_sdf:
            self.get_logger().error("[PHYSICS][DROP] world_sdf sin configurar")
            return False
        if not os.path.exists(world_sdf):
            self.get_logger().error(f"[PHYSICS][DROP] world_sdf no existe: {world_sdf}")
            return False
        try:
            tree = ET.parse(world_sdf)
            root = tree.getroot()
        except Exception as exc:
            self.get_logger().error(f"[PHYSICS][DROP] error leyendo world_sdf: {exc}")
            return False

        world_elem = root.find("world")
        if world_elem is None:
            self.get_logger().error("[PHYSICS][DROP] world no encontrado en SDF")
            return False

        drop_set = set(DROP_OBJECTS)
        anchor_name = self._drop_anchor_name
        for model in world_elem.findall("model"):
            name = model.get("name", "")
            if name not in drop_set and name != anchor_name:
                continue
            pose = self._pose_from_text(model.findtext("pose", default="0 0 0 0 0 0"))
            model_copy = copy.deepcopy(model)
            pose_elem = model_copy.find("pose")
            if pose_elem is not None:
                model_copy.remove(pose_elem)
            static_elem = model_copy.find("static")
            if static_elem is None:
                static_elem = ET.SubElement(model_copy, "static")
            static_elem.text = "false"
            for link in model_copy.findall("link"):
                gravity = link.find("gravity")
                if gravity is None:
                    gravity = ET.SubElement(link, "gravity")
                gravity.text = "true"
                kinematic = link.find("kinematic")
                if kinematic is None:
                    kinematic = ET.SubElement(link, "kinematic")
                kinematic.text = "false"
            sdf_root = ET.Element("sdf", {"version": "1.10"})
            sdf_root.append(model_copy)
            sdf_text = ET.tostring(sdf_root, encoding="unicode")
            if name == anchor_name:
                self._drop_anchor_sdf = sdf_text
                self._drop_anchor_pose = pose
            else:
                self._drop_sdf[name] = sdf_text
                self._drop_pose[name] = pose

        missing = [name for name in DROP_OBJECTS if name not in self._drop_sdf]
        if missing:
            self.get_logger().warn(f"[PHYSICS][DROP] faltan modelos: {missing}")
        if not self._drop_anchor_sdf or self._drop_anchor_pose is None:
            self.get_logger().error(
                f"[PHYSICS][DROP] drop_anchor no encontrado en world_sdf ({anchor_name})"
            )
            return False
        self._drop_cache_loaded = True
        return True

    def _spawn_drop_objects_gz(self, world_name: str, names: List[str]) -> bool:
        gz_bin = shutil.which("gz")
        if not gz_bin:
            self.get_logger().error("[PHYSICS][DROP] gz no disponible para fallback")
            self._set_error("gz no disponible para fallback")
            return False
        env_prefix = self._gz_env_prefix()
        delete_service = self._resolve_delete_service(world_name, env_prefix)
        if not delete_service:
            return False
        spawn_service = self._resolve_spawn_service(world_name, env_prefix)
        if not spawn_service:
            return False
        self.get_logger().info(
            f"[PHYSICS][DROP] gz_services delete={delete_service} spawn={spawn_service}"
        )
        if not self._reset_drop_anchor_gz(
            env_prefix=env_prefix,
            delete_service=delete_service,
            spawn_service=spawn_service,
        ):
            self.get_logger().error("[PHYSICS][DROP] reset drop_anchor falló (gz)")
            self._set_error("reset drop_anchor falló (gz)")
            return False
        delete_ok = True
        for name in names:
            if not self._gz_delete_entity(env_prefix, delete_service, name):
                delete_ok = False
        spawn_ok = True
        for name in names:
            sdf = self._drop_sdf.get(name)
            pose = self._drop_pose.get(name)
            if not sdf or pose is None:
                self.get_logger().warn(
                    f"[PHYSICS][DROP] sin datos de spawn para {name}"
                )
                spawn_ok = False
                continue
            if not self._gz_spawn_entity(env_prefix, spawn_service, name, sdf, pose):
                spawn_ok = False
        if not delete_ok:
            self.get_logger().warn(
                "[PHYSICS][DROP] delete incompleto; continuando con spawn"
            )
        if not spawn_ok:
            self._set_error("Fallo liberando DROP (spawn/delete gz)")
        return spawn_ok

    def _gz_service_exists(self, env_prefix: str, service: str) -> bool:
        cmd_timeout = read_float_param(self, "gz_cmd_timeout_sec", 3.5, min_value=0.2)
        cmd = f"{env_prefix}gz service -s {service} -i"
        try:
            result = subprocess.run(
                ["bash", "-lc", cmd],
                text=True,
                capture_output=True,
                timeout=max(0.5, cmd_timeout),
            )
        except Exception:
            return False
        return result.returncode == 0

    def _resolve_delete_service(self, world_name: str, env_prefix: str) -> Optional[str]:
        explicit = read_str_param(self, "delete_service", "").strip()
        if explicit:
            self.get_logger().info(f"[PHYSICS][DROP] delete_service (explicit): {explicit}")
            return explicit
        candidates = [
            f"/world/{world_name}/remove/blocking",
            f"/world/{world_name}/remove",
            f"/world/{world_name}/remove_entity/blocking",
            f"/world/{world_name}/remove_entity",
            f"/world/{world_name}/delete",
        ]
        return self._resolve_gz_service(candidates, env_prefix, "delete")

    def _resolve_spawn_service(self, world_name: str, env_prefix: str) -> Optional[str]:
        explicit = read_str_param(self, "spawn_service", "").strip()
        if explicit:
            self.get_logger().info(f"[PHYSICS][DROP] spawn_service (explicit): {explicit}")
            return explicit
        candidates = [
            f"/world/{world_name}/create/blocking",
            f"/world/{world_name}/create",
            f"/world/{world_name}/spawn/blocking",
            f"/world/{world_name}/spawn",
        ]
        return self._resolve_gz_service(candidates, env_prefix, "spawn")

    def _resolve_gz_service(
        self,
        candidates: List[str],
        env_prefix: str,
        role: str,
    ) -> Optional[str]:
        for svc in candidates:
            if not svc:
                continue
            if self._gz_service_exists(env_prefix, svc):
                return svc
        msg = f"servicio gz no disponible ({role}): {candidates}"
        self.get_logger().error(f"[PHYSICS][DROP] {msg}")
        self._set_error(msg)
        return None

    def _gz_env_prefix(self) -> str:
        exports = []
        gz_partition = os.environ.get("GZ_PARTITION", "").strip()
        if not gz_partition:
            ws_dir = os.environ.get(
                "WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws")
            )
            part_file = os.path.join(ws_dir, "log", "gz_partition.txt")
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

    def _list_gz_services(self, env_prefix: str) -> List[str]:
        wait_sec = read_float_param(self, "gz_list_wait_sec", 3.0, min_value=0.1)
        wait_total = read_float_param(self, "gz_list_total_sec", 10.0, min_value=0.2)
        deadline = time.monotonic() + wait_total
        last_error = ""
        while time.monotonic() < deadline:
            try:
                result = subprocess.run(
                    ["bash", "-lc", f"{env_prefix}gz service -l"],
                    text=True,
                    capture_output=True,
                    timeout=max(3.0, wait_sec + 0.5),
                )
            except Exception as exc:
                last_error = str(exc)
                time.sleep(wait_sec)
                continue
            if result.returncode == 0:
                services = [
                    line.strip()
                    for line in (result.stdout or "").splitlines()
                    if line.strip()
                ]
                if services:
                    return services
            last_error = (result.stderr or "").strip()
            time.sleep(wait_sec)
        if last_error:
            self.get_logger().error(
                f"[PHYSICS][DROP] gz service list error: {last_error}"
            )
        return []

    def _pick_gz_service(
        self,
        services: List[str],
        world_name: str,
        suffixes: Tuple[str, ...],
    ) -> Optional[str]:
        scoped = [s for s in services if f"/world/{world_name}/" in s]
        for suffix in suffixes:
            for name in scoped:
                if name.endswith(f"/{suffix}"):
                    return name
        if scoped:
            for name in scoped:
                for suffix in suffixes:
                    if name.endswith(suffix):
                        return name
        return None

    def _gz_delete_entity(
        self,
        env_prefix: str,
        service: str,
        name: str,
        *,
        entity_type: str = "MODEL",
        allow_missing: bool = False,
        log_context: str = "delete",
    ) -> bool:
        timeout_ms = read_int_param(self, "gz_delete_timeout_ms", 300, min_value=100)
        cmd_timeout = read_float_param(self, "gz_cmd_timeout_sec", 3.5, min_value=0.2)
        req = f'name: "{name}" type: {entity_type}'
        cmd = (
            f"{env_prefix}gz service -s {service} "
            "--reqtype gz.msgs.Entity --reptype gz.msgs.Boolean "
            f"--timeout {timeout_ms} --req '{req}'"
        )
        try:
            result = subprocess.run(
                ["bash", "-lc", cmd],
                text=True,
                capture_output=True,
                timeout=cmd_timeout,
            )
        except Exception:
            return False
        if result.returncode != 0:
            self.get_logger().warn(
                f"[PHYSICS][DROP] gz {log_context} rc={result.returncode} name={name} type={entity_type}"
            )
            return False
        ok = "data: true" in (result.stdout or "")
        if not ok:
            if allow_missing:
                self.get_logger().info(
                    f"[PHYSICS][DROP] gz {log_context} skipped (missing) name={name} type={entity_type}"
                )
                return True
            self.get_logger().warn(
                f"[PHYSICS][DROP] gz {log_context} false name={name} type={entity_type}"
            )
            return False
        self.get_logger().info(
            f"[PHYSICS][DROP] gz {log_context} ok name={name} type={entity_type}"
        )
        return True

    def _gz_spawn_entity(
        self,
        env_prefix: str,
        service: str,
        name: str,
        sdf: str,
        pose: Pose,
    ) -> bool:
        timeout_ms = read_int_param(self, "gz_spawn_timeout_ms", 600, min_value=150)
        cmd_timeout = read_float_param(self, "gz_cmd_timeout_sec", 3.5, min_value=0.2)
        sdf_escaped = sdf.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "")
        req = (
            f'name: "{name}" '
            "allow_renaming: false "
            f'sdf: "{sdf_escaped}" '
            "pose {"
            f"position {{x: {pose.position.x} y: {pose.position.y} z: {pose.position.z}}} "
            f"orientation {{x: {pose.orientation.x} y: {pose.orientation.y} "
            f"z: {pose.orientation.z} w: {pose.orientation.w}}}"
            "}"
        )
        cmd = (
            f"{env_prefix}gz service -s {service} "
            "--reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean "
            f"--timeout {timeout_ms} --req '{req}'"
        )
        try:
            result = subprocess.run(
                ["bash", "-lc", cmd],
                text=True,
                capture_output=True,
                timeout=cmd_timeout,
            )
        except Exception:
            return False
        if result.returncode != 0:
            self.get_logger().warn(
                f"[PHYSICS][DROP] gz spawn rc={result.returncode} {name}"
            )
            return False
        ok = "data: true" in (result.stdout or "")
        if not ok:
            self.get_logger().warn(
                f"[PHYSICS][DROP] gz spawn false {name} out={result.stdout.strip()}"
            )
        return ok

    def _find_service_by_type(self, world_name: str, type_hint: str) -> Optional[str]:
        candidates = []
        for name, types in self.get_service_names_and_types():
            if f"/world/{world_name}/" not in name:
                continue
            if any(type_hint in t for t in types):
                candidates.append(name)
        if not candidates:
            return None
        for suffix in ("/remove", "/remove_entity", "/delete", "/create"):
            for name in candidates:
                if name.endswith(suffix):
                    return name
        return candidates[0]

    def _pose_from_text(self, text: str) -> Pose:
        pose = Pose()
        parts = [p for p in text.strip().split() if p]
        values = [float(p) for p in parts[:6]]
        while len(values) < 6:
            values.append(0.0)
        x, y, z, roll, pitch, yaw = values
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        qx, qy, qz, qw = self._quat_from_rpy(roll, pitch, yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def _quat_from_rpy(
        self,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> Tuple[float, float, float, float]:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main() -> None:
    rclpy.init()
    node = ReleaseObjectsService()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        node.get_logger().warn(f"[PHYSICS][DROP] shutdown race ignorada: {exc}")
    finally:
        try:
            node.destroy_node()
        except BaseException:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
