# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/attach_anchor.py
# Contenido: AnchorMixin con publish/relay anchor handlers (C.4).
"""AnchorMixin: drop anchor + tool anchor relay handlers (refactor C.4).

Extraido de ``gripper_attach_backend.py`` (lineas 460-568 originales).
Maneja la publicacion de estado de attach + el relay de mensajes
tool_anchor / drop_anchor a otros suscriptores del topico.

Sin estado propio. Requiere atributos del Node ya inicializados:
``_gripper_state_pubs``, ``_drop_anchor_states``, ``_tool_*``,
``_startup_*``, etc.
"""

from __future__ import annotations

import time

from std_msgs.msg import Bool, Empty


class AnchorMixin:
    """Drop anchor + tool anchor relay + startup detach helpers."""

    def _publish_state(self, name: str, attached: bool) -> None:
        pub = self._gripper_state_pubs.get(name)
        if pub is None:
            return
        msg = Bool()
        msg.data = bool(attached)
        pub.publish(msg)

    def _on_drop_anchor_state(self, msg: Bool, *, name: str) -> None:
        self._drop_anchor_states[name] = bool(getattr(msg, "data", False))

    def _force_drop_anchor_detach(self, name: str) -> None:
        pub = self._drop_detach_pubs.get(name)
        if pub is None:
            self.get_logger().warning(
                f"[ATTACH_BACKEND] drop_anchor detach publisher missing object={name}"
            )
            return
        state_hint = bool(self._drop_anchor_states.get(name, False))
        # El topic de estado de drop_anchor puede dar false aunque la junta siga
        # sujetando el objeto sobre la mesa. Publicamos detach de forma proactiva.
        for _idx in range(2):
            pub.publish(Empty())
            time.sleep(0.05)
        self.get_logger().info(
            f"[ATTACH_BACKEND] relay detach object={name} "
            f"dst=/{self._drop_anchor_prefix}/{name}/detach "
            "method=drop_anchor_force_release "
            f"state_hint={str(state_hint).lower()}"
        )

    def _relay_tool_anchor_attach(self, name: str, *, detail: str) -> bool:
        self._force_drop_anchor_detach(name)
        self._attached.pop(name, None)
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
            f"dst=/{self._tool_anchor_prefix}/{name}/attach "
            f"method=tool_anchor_preferred detail={detail}"
        )
        self._publish_state(name, True)
        return True

    def _relay_tool_anchor_detach(self, name: str, *, detail: str) -> bool:
        self._attached.pop(name, None)
        pub = self._tool_detach_pubs.get(name)
        if pub is None:
            self.get_logger().error(
                f"[ATTACH_BACKEND] missing tool_detach publisher object={name}"
            )
            self._publish_state(name, False)
            return False
        pub.publish(Empty())
        self.get_logger().info(
            f"[ATTACH_BACKEND] relay detach object={name} "
            f"dst=/{self._tool_anchor_prefix}/{name}/detach "
            f"method=tool_anchor_preferred detail={detail}"
        )
        self._publish_state(name, False)
        return True

    def _startup_detach_tool_anchors_once_ready(self) -> None:
        timer = self._startup_detach_timer
        if timer is None:
            return
        if self._startup_detach_attempts_left <= 0:
            timer.cancel()
            self._startup_detach_timer = None
            if self._startup_detach_sent <= 0:
                self.get_logger().warning(
                    "[ATTACH_BACKEND] startup_tool_detach skipped "
                    "detail=no_tool_anchor_subscribers"
                )
            return

        self._startup_detach_attempts_left -= 1
        ready = []
        for name, pub in self._tool_detach_pubs.items():
            try:
                sub_count = int(pub.get_subscription_count())
            except Exception:
                sub_count = 0
            if sub_count > 0:
                ready.append((name, pub))
        if not ready:
            return

        detach_sent = 0
        for name, pub in ready:
            pub.publish(Empty())
            self._publish_state(name, False)
            detach_sent += 1
        self._startup_detach_sent += detach_sent
        self.get_logger().info(
            "[ATTACH_BACKEND] startup_tool_detach "
            f"sent={detach_sent} ready={len(ready)}/{len(self._tool_detach_pubs)} "
            f"attempts_left={self._startup_detach_attempts_left}"
        )
        if len(ready) >= len(self._tool_detach_pubs):
            timer.cancel()
            self._startup_detach_timer = None
