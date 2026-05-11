# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/attach_contact_gate.py
# Contenido: ContactGateMixin: validacion de contacto bilateral previa a attach.
"""ContactGateMixin: gate fisico basado en contactos de dedos RG2.

Este mixin consume mensajes ``ros_gz_interfaces/msg/Contacts`` ya puenteados
a ROS desde los sensores del modelo SDF y mantiene un cache de ultimo contacto
por lado (left/right) y por objeto.

Objetivo:
  * Bloquear ``attach`` cuando no hay evidencia reciente de contacto fisico.
  * Permitir modo bilateral estricto (left && right) o unilateral (left || right).
"""

from __future__ import annotations

import math
from typing import Optional


class ContactGateMixin:
    """Helpers para gate de contacto pre-attach."""

    @staticmethod
    def _contact_side_from_topic(topic: str) -> str:
        t = str(topic or "")
        if "left" in t:
            return "left"
        if "right" in t:
            return "right"
        return "unknown"

    def _extract_object_from_contact(self, collision_a: str, collision_b: str) -> Optional[str]:
        """Devuelve object_name reconocido si aparece en los nombres de collision."""
        c1 = str(collision_a or "")
        c2 = str(collision_b or "")
        combined = f"{c1}\n{c2}"
        for name in self._object_names:
            n = str(name)
            if not n:
                continue
            if (
                f"/model/{n}/" in combined
                or f"{n}::" in combined
                or combined.endswith(f"/{n}")
                or combined.endswith(f"::{n}")
            ):
                return n
        return None

    def _on_contact_msg(self, msg, *, topic: str) -> None:
        """Actualiza cache de contacto por objeto/lado con timestamp ROS."""
        side = self._contact_side_from_topic(topic)
        if side not in ("left", "right"):
            return
        now_ns = int(self.get_clock().now().nanoseconds)
        contacts = getattr(msg, "contacts", None) or []
        updated = 0
        for contact in contacts:
            c1 = str(getattr(getattr(contact, "collision1", None), "name", "") or "")
            c2 = str(getattr(getattr(contact, "collision2", None), "name", "") or "")
            object_name = self._extract_object_from_contact(c1, c2)
            if not object_name:
                continue
            self._contact_last_ns[(side, object_name)] = now_ns
            updated += 1
        if updated:
            self._last_contact_update_ns = now_ns

    def _contact_recent_for(self, side: str, object_name: str) -> bool:
        stamp_ns = int(self._contact_last_ns.get((side, object_name), 0))
        if stamp_ns <= 0:
            return False
        now_ns = int(self.get_clock().now().nanoseconds)
        if now_ns <= 0:
            return False
        age = (now_ns - stamp_ns) / 1_000_000_000.0
        return 0.0 <= age <= float(self._contact_max_age_sec)

    def _validate_contact_gate(self, object_name: str) -> tuple[bool, str]:
        """Valida criterio de contacto previo al attach para ``object_name``."""
        if not bool(self._require_contact_before_attach):
            return True, "contact_gate_disabled"
        if not bool(self._contact_gate_enabled):
            return False, "contact_gate_unavailable"
        left_ok = self._contact_recent_for("left", object_name)
        right_ok = self._contact_recent_for("right", object_name)
        if bool(self._require_bilateral_contact):
            ok = left_ok and right_ok
            mode = "bilateral"
        else:
            ok = left_ok or right_ok
            mode = "unilateral"
        detail = (
            f"mode={mode} left={str(left_ok).lower()} right={str(right_ok).lower()} "
            f"max_age={float(self._contact_max_age_sec):.3f}s"
        )
        if ok:
            return True, detail
        eq_ok, eq_detail = self._validate_equivalent_grasp_gate(object_name)
        if eq_ok:
            return True, f"{detail} equivalent={eq_detail}"
        return False, f"{detail} equivalent={eq_detail}"

    def _validate_equivalent_grasp_gate(self, object_name: str) -> tuple[bool, str]:
        """Fallback cuando no hay contactos: valida un grasp estable equivalente.

        Criterio:
          1) Gripper realmente cerrado (opening_sum <= threshold).
          2) Objeto y TCP prácticamente coincidentes (dist <= threshold).
          3) Poses frescas según ``max_pose_age_sec``.
        """
        if not bool(getattr(self, "_equivalent_grasp_enable", False)):
            return False, "disabled"
        opening_sum = getattr(self, "_gripper_opening_sum", None)
        if opening_sum is None:
            return False, "no_opening_measurement"
        try:
            opening = float(opening_sum)
        except Exception:
            return False, "invalid_opening_measurement"
        opening_stamp_ns = int(getattr(self, "_gripper_opening_stamp_ns", 0))
        now_ns = int(self.get_clock().now().nanoseconds)
        if opening_stamp_ns <= 0 or now_ns <= 0:
            return False, "opening_stamp_unavailable"
        opening_age = (now_ns - opening_stamp_ns) / 1_000_000_000.0
        max_opening_age = float(
            getattr(self, "_equivalent_grasp_opening_max_age_sec", 0.50)
        )
        if opening_age > max_opening_age:
            return False, (
                f"stale_opening age={opening_age:.3f}s "
                f"max={max_opening_age:.3f}s"
            )
        max_opening = float(getattr(self, "_equivalent_grasp_max_opening_sum", 0.003))
        if opening > max_opening:
            return False, f"opening={opening:.4f}>{max_opening:.4f}"

        obj_pose = self._lookup_pose(object_name)
        tcp_pose = self._lookup_tcp_pose()
        if obj_pose is None or tcp_pose is None:
            return False, "pose_unavailable"
        obj_age = self._pose_age_sec(obj_pose)
        tcp_age = self._pose_age_sec(tcp_pose)
        max_age = float(getattr(self, "_max_pose_age_sec", 1.5))
        if obj_age > max_age or tcp_age > max_age:
            return False, (
                f"stale_pose obj_age={obj_age:.3f}s tcp_age={tcp_age:.3f}s "
                f"max={max_age:.3f}s"
            )
        dist = math.sqrt(
            (float(obj_pose.x) - float(tcp_pose.x)) ** 2
            + (float(obj_pose.y) - float(tcp_pose.y)) ** 2
            + (float(obj_pose.z) - float(tcp_pose.z)) ** 2
        )
        max_dist = float(getattr(self, "_equivalent_grasp_max_dist_m", 0.003))
        if dist > max_dist:
            return False, f"dist={dist:.4f}>{max_dist:.4f}"
        return True, (
            f"dist={dist:.4f}<={max_dist:.4f} "
            f"opening={opening:.4f}<={max_opening:.4f} "
            f"opening_age={opening_age:.3f}s<={max_opening_age:.3f}s"
        )
