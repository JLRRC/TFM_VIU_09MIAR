#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/tfm_grasping/grasp_selector_node.py
"""F17-step2 (2026-05-08) — Nodo ROS2 wrapper sobre grasp_selector.

Suscribe a un topic de poses candidatas (PoseArray + scores opcional)
y publica el mejor candidato seleccionado por la lógica pura
``grasp_selector.select_best_grasp`` (filtros workspace + TCP-down +
ranking score+distance).

Topics:
- Sub: ``~/grasp_candidates`` (geometry_msgs/PoseArray)  — candidatos en base_link.
- Sub: ``~/grasp_candidate_scores`` (std_msgs/Float32MultiArray)
        — opcional, scores [0..1] por candidato (mismo orden que poses).
- Pub: ``~/best_grasp`` (geometry_msgs/PoseStamped)  — mejor candidato.
- Pub: ``~/grasp_selector_status`` (std_msgs/String)  — JSON con
        n_input/n_after_workspace/n_after_tcp_down/reason.

Parámetros declarados:
- ``ws_radius_m`` (default 0.85): radio máximo del workspace UR5+RG2.
- ``ws_min_radius_m`` (default 0.20): radio mínimo (zona singular).
- ``tcp_down_tolerance_rad`` (default 0.20): tolerancia TCP-down (~11.5°).
- ``require_tcp_down`` (default true): aplicar filtro TCP-down.
- ``base_xyz`` (default [0.0,0.0,0.0]): coords del base_link en el frame
        de los candidatos.

Diseño: la **decisión** vive en ``grasp_selector.select_best_grasp``
(función pura testeada offline). Este nodo sólo es shim ROS — convertir
ROS msgs → GraspCandidate, llamar la función pura, publicar el resultado.
NO contiene lógica de selección que no esté en el helper puro.

Status: NO se ejecuta en E2E del proyecto todavía. Add al stack
(ur5_stack.launch.py) cuando se valide live (queda pendiente de F17-step3).
"""

from __future__ import annotations

import json
from typing import List, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseArray, PoseStamped
    from std_msgs.msg import Float32MultiArray, String
except ImportError as e:
    raise SystemExit(f"ROS 2 imports faltantes: {e}")

from tfm_grasping.grasp_selector import (
    DEFAULT_TCP_DOWN_TOLERANCE_RAD,
    DEFAULT_WS_MIN_RADIUS_M,
    DEFAULT_WS_RADIUS_M,
    GraspCandidate,
    select_best_grasp,
)


def pose_array_to_candidates(
    msg: PoseArray,
    scores: List[float] | None = None,
) -> List[GraspCandidate]:
    """Convierte PoseArray + scores opcional a lista de GraspCandidate.

    Si ``scores`` es None o tiene longitud distinta, usa 0.0 como score.
    """
    candidates: List[GraspCandidate] = []
    poses = list(msg.poses or [])
    n = len(poses)
    use_scores = scores is not None and len(scores) == n
    for i, pose in enumerate(poses):
        score = float(scores[i]) if use_scores and scores is not None else 0.0
        candidates.append(GraspCandidate(
            xyz=(float(pose.position.x), float(pose.position.y), float(pose.position.z)),
            quat_xyzw=(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
            score=score,
            label=f"cand_{i:03d}",
        ))
    return candidates


class GraspSelectorNode(Node):
    """Wrapper ROS thin sobre grasp_selector.select_best_grasp."""

    def __init__(self) -> None:
        super().__init__("grasp_selector_node")

        # Params
        self.declare_parameter("ws_radius_m", DEFAULT_WS_RADIUS_M)
        self.declare_parameter("ws_min_radius_m", DEFAULT_WS_MIN_RADIUS_M)
        self.declare_parameter(
            "tcp_down_tolerance_rad", DEFAULT_TCP_DOWN_TOLERANCE_RAD
        )
        self.declare_parameter("require_tcp_down", True)
        self.declare_parameter("base_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("base_frame", "base_link")

        self._scores_cache: List[float] | None = None

        # Publishers
        self._best_pub = self.create_publisher(PoseStamped, "~/best_grasp", 10)
        self._status_pub = self.create_publisher(String, "~/grasp_selector_status", 10)

        # Subscribers
        self.create_subscription(
            Float32MultiArray, "~/grasp_candidate_scores",
            self._on_scores, 10,
        )
        self.create_subscription(
            PoseArray, "~/grasp_candidates",
            self._on_candidates, 10,
        )

        self.get_logger().info(
            "[GRASP_SELECTOR] node started — sub /grasp_candidates, pub /best_grasp"
        )

    def _on_scores(self, msg: Float32MultiArray) -> None:
        """Cachea los scores para emparejarlos con la siguiente PoseArray."""
        self._scores_cache = [float(v) for v in msg.data]

    def _on_candidates(self, msg: PoseArray) -> None:
        candidates = pose_array_to_candidates(msg, scores=self._scores_cache)
        self._scores_cache = None  # consumed

        # Read params
        ws_radius_m = float(self.get_parameter("ws_radius_m").value)
        ws_min_radius_m = float(self.get_parameter("ws_min_radius_m").value)
        tcp_tol = float(self.get_parameter("tcp_down_tolerance_rad").value)
        require_tcp_down = bool(self.get_parameter("require_tcp_down").value)
        base_xyz_raw = self.get_parameter("base_xyz").value or [0.0, 0.0, 0.0]
        base_xyz: Tuple[float, float, float] = (
            float(base_xyz_raw[0]),
            float(base_xyz_raw[1]),
            float(base_xyz_raw[2]),
        )
        base_frame = str(self.get_parameter("base_frame").value or "base_link")

        result = select_best_grasp(
            candidates,
            base_xyz=base_xyz,
            ws_radius_m=ws_radius_m,
            ws_min_radius_m=ws_min_radius_m,
            tcp_down_tolerance_rad=tcp_tol,
            require_tcp_down=require_tcp_down,
        )

        # Publish status JSON
        status = String()
        status.data = json.dumps({
            "n_input": result.n_input,
            "n_after_workspace": result.n_after_workspace,
            "n_after_tcp_down": result.n_after_tcp_down,
            "reason": result.reason,
            "best_label": result.best.label if result.best else None,
        })
        self._status_pub.publish(status)

        # Publish best (si hay)
        if result.best is not None:
            ps = PoseStamped()
            ps.header = msg.header
            ps.header.frame_id = base_frame
            ps.pose.position.x = float(result.best.xyz[0])
            ps.pose.position.y = float(result.best.xyz[1])
            ps.pose.position.z = float(result.best.xyz[2])
            ps.pose.orientation.x = float(result.best.quat_xyzw[0])
            ps.pose.orientation.y = float(result.best.quat_xyzw[1])
            ps.pose.orientation.z = float(result.best.quat_xyzw[2])
            ps.pose.orientation.w = float(result.best.quat_xyzw[3])
            self._best_pub.publish(ps)
            self.get_logger().info(
                f"[GRASP_SELECTOR] best={result.best.label} "
                f"score={result.best.score:.3f} reason={result.reason}"
            )
        else:
            self.get_logger().warning(
                f"[GRASP_SELECTOR] no best candidate — reason={result.reason} "
                f"n_input={result.n_input} ws={result.n_after_workspace} "
                f"tcp_down={result.n_after_tcp_down}"
            )


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GraspSelectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
