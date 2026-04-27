"""ControllerManagementMixin: gestión del FJT ActionClient y query a controller_manager.

Extraído de ``ur5_tools/ur5_moveit_bridge.py`` (líneas 1053-1212 originales).
Sin estado propio; todo accede a atributos del nodo principal vía ``self``.
La clase concreta debe heredar también de ``rclpy.node.Node``.
"""

from __future__ import annotations

import time

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient

from ..moveit_bridge_utils import normalize_action_name


class ControllerManagementMixin:
    """FJT ActionClient lifecycle + ListControllers queries."""

    def _available_action_names(self) -> list[str]:
        try:
            names = self.get_action_names_and_types()
            return [str(name) for name, _types in names]
        except Exception:
            return []

    @staticmethod
    def _normalize_action_name(name: str) -> str:
        return normalize_action_name(name)

    def _destroy_fjt_action_client(self) -> None:
        with self._action_client_lock:
            client = self._fjt_client
            self._fjt_client = None
            self._fjt_action_name = ""
        if client is None:
            return
        try:
            client.destroy()
        except Exception:
            pass

    def _ensure_fjt_action_client(self) -> ActionClient | None:
        action_name = self._normalize_action_name(self._controller_action_name)
        if not action_name:
            return None
        with self._action_client_lock:
            if self._fjt_client is not None and self._fjt_action_name == action_name:
                return self._fjt_client
            old_client = self._fjt_client
            self._fjt_client = None
            self._fjt_action_name = ""
            if old_client is not None:
                try:
                    old_client.destroy()
                except Exception:
                    pass
            try:
                client = ActionClient(self, FollowJointTrajectory, action_name)
            except Exception as exc:
                self.get_logger().warning(
                    "[BRIDGE_EXEC] no se pudo crear ActionClient "
                    f"action={action_name}: {type(exc).__name__}: {exc}"
                )
                return None
            self._fjt_client = client
            self._fjt_action_name = action_name
            return client

    def _prime_fjt_action_client(self) -> None:
        try:
            self._ensure_fjt_action_client()
            timer = self._fjt_prime_timer
            if timer is not None:
                try:
                    timer.cancel()
                except Exception:
                    pass
                self._fjt_prime_timer = None
        except Exception as exc:
            self.get_logger().warning(
                "[BRIDGE_EXEC] prime ActionClient fallido "
                f"type={type(exc).__name__} err={exc}"
            )

    def _controller_manager_service_name(self, service: str) -> str:
        base = (self._controller_manager_name or "/controller_manager").strip()
        if not base:
            base = "/controller_manager"
        if not base.startswith("/"):
            base = f"/{base}"
        base = base.rstrip("/")
        return f"{base}/{service}"

    def _controller_action_candidates(self, action_names: list[str]) -> list[str]:
        candidates: list[str] = []
        seen: set[str] = set()
        controller = str(self._controller_name or "joint_trajectory_controller").strip("/")
        action_ns = str(self._controller_action_ns or "follow_joint_trajectory").strip("/")
        expected = str(self._controller_action_name or "").strip()
        base_cm = (self._controller_manager_name or "/controller_manager").strip().rstrip("/")
        if not base_cm.startswith("/"):
            base_cm = f"/{base_cm}"
        base_cm_ns = base_cm.removesuffix("/controller_manager")
        for raw in (
            expected,
            f"/{controller}/{action_ns}",
            f"/controller_manager/{controller}/{action_ns}",
            f"{base_cm_ns}/{controller}/{action_ns}" if base_cm_ns else "",
        ):
            name = str(raw or "").strip()
            if not name:
                continue
            if not name.startswith("/"):
                name = f"/{name}"
            name = name.replace("//", "/")
            if name in seen:
                continue
            seen.add(name)
            candidates.append(name)
        suffix = f"/{controller}/{action_ns}"
        for name in action_names:
            norm = f"/{name.lstrip('/')}"
            if norm.endswith(suffix) and norm not in seen:
                seen.add(norm)
                candidates.append(norm)
        return candidates

    def _active_controllers(self, timeout_sec: float = 0.8) -> list[str]:
        client = self._list_controllers_client
        if client is None:
            return []
        if not client.wait_for_service(timeout_sec=min(0.2, timeout_sec)):
            return []
        req = ListControllers.Request()
        fut = client.call_async(req)
        deadline = time.monotonic() + max(0.1, timeout_sec)
        while time.monotonic() < deadline:
            if fut.done():
                break
            time.sleep(0.05)
        if not fut.done():
            return []
        try:
            resp = fut.result()
        except Exception:
            return []
        out: list[str] = []
        for c in getattr(resp, "controller", []) or []:
            try:
                if str(getattr(c, "state", "")).lower() == "active":
                    out.append(str(getattr(c, "name", "")))
            except Exception:
                continue
        return out

    def _wait_for_expected_controller_action(
        self, timeout_sec: float = 2.0
    ) -> tuple[bool, str, list[str], list[str]]:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        last_actions: list[str] = []
        last_candidates: list[str] = []
        while time.monotonic() <= deadline:
            last_actions = self._available_action_names()
            last_candidates = self._controller_action_candidates(last_actions)
            client = self._ensure_fjt_action_client()
            if client is not None:
                try:
                    if client.wait_for_server(timeout_sec=0.2):
                        matched = self._normalize_action_name(self._controller_action_name)
                        return True, matched, last_actions, last_candidates
                except Exception as exc:
                    self.get_logger().warning(
                        "[BRIDGE_EXEC] wait_for_server fallido "
                        f"action={self._controller_action_name}: {type(exc).__name__}: {exc}"
                    )
                    return False, "", last_actions, last_candidates
            time.sleep(0.1)
        return False, "", last_actions, last_candidates
