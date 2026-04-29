"""TrajectoryPrepMixin: preparación de JointTrajectory para el controller.

Extraído de ``ur5_tools/ur5_moveit_bridge.py`` (líneas 2174-2573 originales).

Cinco métodos cohesivos sobre ``trajectory_msgs.msg.JointTrajectory``:

* ``_joint_trajectory_initial_segment_max_delta`` (staticmethod, helper).
* ``_prepare_joint_trajectory_for_controller`` (~334 L) — el método grande:
  normaliza header.stamp, desplega joints continuos, alinea start con el
  estado actual, inserta cold-hold/midpoint y formatea preview.
* ``_extract_joint_trajectory_msg`` (~55 L) — extracción robusta a varios
  formatos de RobotTrajectory.
* ``_scale_joint_trajectory_timing`` y ``_joint_trajectory_duration_sec``
  (staticmethods que delegan en ``moveit_bridge_utils``).

La clase concreta debe heredar también de ``rclpy.node.Node`` y de
``JointStateHelpersMixin`` (para ``_WRAPAROUND_JOINTS`` y
``_normalize_joint_position``).
"""

from __future__ import annotations

import math
from copy import deepcopy

from trajectory_msgs.msg import JointTrajectory

from ..moveit_bridge_utils import (
    joint_trajectory_duration_sec,
    joint_trajectory_initial_segment_max_delta,
    scale_joint_trajectory_timing,
)


class TrajectoryPrepMixin:
    """Preparación / sanity / scaling de JointTrajectory para el FJT controller."""

    @staticmethod
    def _joint_trajectory_initial_segment_max_delta(jt: JointTrajectory) -> float:
        return joint_trajectory_initial_segment_max_delta(jt)

    def _prepare_joint_trajectory_for_controller(
        self,
        jt: JointTrajectory,
        *,
        force_cold_start_hold: bool = False,
    ) -> JointTrajectory:
        prepared = deepcopy(jt)
        try:
            stamp = prepared.header.stamp
            stamp_sec = int(getattr(stamp, "sec", 0) or 0)
            stamp_nsec = int(getattr(stamp, "nanosec", 0) or 0)
        except Exception:
            stamp_sec = 0
            stamp_nsec = 0
        if stamp_sec != 0 or stamp_nsec != 0:
            try:
                now_ns = int(self.get_clock().now().nanoseconds or 0)
            except Exception:
                now_ns = 0
            stamp_ns = stamp_sec * 1_000_000_000 + stamp_nsec
            skew_txt = "n/a"
            if now_ns > 0:
                skew_txt = f"{(stamp_ns - now_ns) / 1_000_000_000.0:.3f}s"
            self.get_logger().warning(
                "[BRIDGE_EXEC] normalizing JointTrajectory header.stamp "
                f"from={stamp_sec}.{stamp_nsec:09d} to=0 sim_time={str(self._use_sim_time).lower()} "
                f"skew={skew_txt}"
            )
            prepared.header.stamp.sec = 0
            prepared.header.stamp.nanosec = 0
        joint_names = list(getattr(prepared, "joint_names", []) or [])
        points = list(getattr(prepared, "points", []) or [])
        raw_names = list(self._joint_state_last_names or [])
        raw_positions = list(self._joint_state_last_positions or [])
        if joint_names and points and raw_names and len(raw_positions) >= len(raw_names):
            current_map = {
                str(name): float(pos)
                for name, pos in zip(raw_names, raw_positions)
                if str(name or "").strip()
            }
            adjusted_counts: dict[str, int] = {}
            references: dict[str, float] = {}
            for jname in joint_names:
                if jname in current_map:
                    references[jname] = current_map[jname]
            explicit_unwrap = bool(getattr(self, "_unwrap_continuous_joints", False))
            for point in points:
                positions = list(getattr(point, "positions", []) or [])
                if len(positions) != len(joint_names):
                    continue
                adjusted_positions = list(positions)
                for idx, jname in enumerate(joint_names):
                    if jname not in self._WRAPAROUND_JOINTS:
                        continue
                    target = float(adjusted_positions[idx])
                    if explicit_unwrap:
                        ref = references.get(jname)
                        if ref is None:
                            adjusted = self._normalize_joint_position(jname, target)
                        else:
                            adjusted = target + (math.tau * round((ref - target) / math.tau))
                    else:
                        adjusted = self._normalize_joint_position(jname, target)
                    if abs(adjusted - target) > 1e-6:
                        adjusted_counts[jname] = adjusted_counts.get(jname, 0) + 1
                    adjusted_positions[idx] = adjusted
                    references[jname] = adjusted
                point.positions = tuple(adjusted_positions)
            if adjusted_counts:
                details = ",".join(
                    f"{name}:{count}" for name, count in sorted(adjusted_counts.items())
                )
                mode = "explicit" if self._unwrap_continuous_joints else "auto"
                self.get_logger().info(
                    "[BRIDGE_EXEC] normalized continuous joints for controller "
                    f"mode={mode} joints={details}"
                )

            # Align start of trajectory with current state to avoid immediate
            # path-tolerance violations at controller startup.
            first_positions = list(getattr(points[0], "positions", []) or []) if points else []
            if first_positions and len(first_positions) == len(joint_names):
                current_vec = [
                    float(current_map.get(jname, first_positions[idx]))
                    for idx, jname in enumerate(joint_names)
                ]
                max_start_err = 0.0
                start_err_details: list[tuple[str, float, float, float]] = []
                for idx, jname in enumerate(joint_names):
                    cur = current_vec[idx]
                    tgt = float(first_positions[idx])
                    if jname in self._WRAPAROUND_JOINTS:
                        cur = self._normalize_joint_position(jname, cur)
                        tgt = self._normalize_joint_position(jname, tgt)
                        err = abs(math.atan2(math.sin(cur - tgt), math.cos(cur - tgt)))
                    else:
                        err = abs(cur - tgt)
                    max_start_err = max(max_start_err, err)
                    start_err_details.append((jname, cur, tgt, err))

                lead_gain = max(
                    0.20,
                    self._env_float("PANEL_MOVEIT_BRIDGE_START_LEAD_GAIN", 0.8),
                )
                lead_max_sec = max(
                    0.50,
                    self._env_float("PANEL_MOVEIT_BRIDGE_START_LEAD_MAX_SEC", 2.5),
                )
                blend_err_rad = max(
                    0.30,
                    self._env_float("PANEL_MOVEIT_BRIDGE_START_BLEND_ERR_RAD", 1.0),
                )
                blend_ratio = min(
                    0.80,
                    max(
                        0.20,
                        self._env_float("PANEL_MOVEIT_BRIDGE_START_BLEND_RATIO", 0.55),
                    ),
                )
                lead_sec = max(0.20, min(lead_max_sec, max_start_err / lead_gain))
                if max_start_err > 0.05:
                    lead_sec = max(0.50, lead_sec)
                    top_errs = sorted(
                        start_err_details,
                        key=lambda item: item[3],
                        reverse=True,
                    )[:6]
                    details = ",".join(
                        f"{jname}:cur={cur:.3f}->tgt={tgt:.3f}:err={err:.3f}"
                        for jname, cur, tgt, err in top_errs
                    )
                    self.get_logger().info(
                        "[BRIDGE_EXEC] start-state error detail "
                        f"max_start_err={max_start_err:.4f}rad joints={details}"
                    )
                try:
                    t0 = points[0].time_from_start
                    first_time_sec = float(t0.sec) + float(t0.nanosec) / 1_000_000_000.0
                except Exception:
                    first_time_sec = 0.0
                shift_sec = max(0.0, lead_sec - max(0.0, first_time_sec))
                if shift_sec > 1e-6:
                    for point in points:
                        tfs = point.time_from_start
                        total = (
                            float(getattr(tfs, "sec", 0) or 0)
                            + float(getattr(tfs, "nanosec", 0) or 0) / 1_000_000_000.0
                            + shift_sec
                        )
                        sec = int(total)
                        nsec = int(round((total - sec) * 1_000_000_000.0))
                        if nsec >= 1_000_000_000:
                            sec += 1
                            nsec -= 1_000_000_000
                        tfs.sec = sec
                        tfs.nanosec = nsec

                initial_segment_max_delta = max(
                    0.0,
                    float(self._joint_trajectory_initial_segment_max_delta(prepared)),
                )
                auto_short_goal_hold = False
                short_goal_first_time_sec = max(0.0, float(first_time_sec))
                if not force_cold_start_hold and short_goal_first_time_sec <= max(
                    0.15,
                    self._env_float(
                        "PANEL_MOVEIT_BRIDGE_SHORT_GOAL_FIRST_POINT_MAX_SEC",
                        0.25,
                    ),
                ):
                    auto_short_goal_hold = True
                cold_hold_needed = bool(force_cold_start_hold)
                if cold_hold_needed:
                    cold_hold_needed = bool(
                        max_start_err > max(
                            0.05,
                            self._env_float(
                                "PANEL_MOVEIT_BRIDGE_FIRST_GOAL_HOLD_MIN_START_ERR_RAD",
                                0.08,
                            ),
                        )
                        or short_goal_first_time_sec <= max(
                            0.10,
                            self._env_float(
                                "PANEL_MOVEIT_BRIDGE_FIRST_GOAL_HOLD_MIN_FIRST_POINT_SEC",
                                0.18,
                            ),
                        )
                        or initial_segment_max_delta >= max(
                            0.05,
                            self._env_float(
                                "PANEL_MOVEIT_BRIDGE_FIRST_GOAL_HOLD_MIN_SEGMENT_DELTA_RAD",
                                0.12,
                            ),
                        )
                    )
                insert_cold_hold = bool(cold_hold_needed or auto_short_goal_hold)
                cold_hold_sec = 0.0
                if insert_cold_hold:
                    cold_hold_sec = max(
                        0.25,
                        self._env_float(
                            (
                                "PANEL_MOVEIT_BRIDGE_FIRST_GOAL_HOLD_SEC"
                                if force_cold_start_hold
                                else "PANEL_MOVEIT_BRIDGE_SHORT_GOAL_HOLD_SEC"
                            ),
                            0.60 if force_cold_start_hold else 0.35,
                        ),
                    )
                if max_start_err > 0.02 or insert_cold_hold:
                    first_point_time = 0.0
                    try:
                        first_tfs = points[0].time_from_start
                        first_point_time = (
                            float(getattr(first_tfs, "sec", 0) or 0)
                            + float(getattr(first_tfs, "nanosec", 0) or 0)
                            / 1_000_000_000.0
                        )
                    except Exception:
                        first_point_time = max(0.0, float(lead_sec))
                    inserted_midpoint = False
                    if max_start_err >= blend_err_rad and first_point_time >= 0.60:
                        mid_time = min(
                            max(0.25, first_point_time * blend_ratio),
                            max(0.25, first_point_time - 0.10),
                        )
                        if mid_time >= 0.20:
                            midpoint = deepcopy(points[0])
                            midpoint_positions: list[float] = []
                            for idx, jname in enumerate(joint_names):
                                cur = float(current_vec[idx])
                                tgt = float(first_positions[idx])
                                if jname in self._WRAPAROUND_JOINTS:
                                    delta = math.atan2(
                                        math.sin(tgt - cur),
                                        math.cos(tgt - cur),
                                    )
                                    interp = cur + (delta * blend_ratio)
                                else:
                                    interp = cur + ((tgt - cur) * blend_ratio)
                                midpoint_positions.append(float(interp))
                            midpoint.positions = tuple(midpoint_positions)
                            midpoint.velocities = tuple(0.0 for _ in joint_names)
                            if getattr(midpoint, "accelerations", None):
                                midpoint.accelerations = tuple(0.0 for _ in joint_names)
                            mid_sec = int(mid_time)
                            mid_nsec = int(round((mid_time - mid_sec) * 1_000_000_000.0))
                            if mid_nsec >= 1_000_000_000:
                                mid_sec += 1
                                mid_nsec -= 1_000_000_000
                            midpoint.time_from_start.sec = mid_sec
                            midpoint.time_from_start.nanosec = mid_nsec
                            points.insert(0, midpoint)
                            inserted_midpoint = True
                    if cold_hold_sec > 1e-6:
                        for point in points:
                            tfs = point.time_from_start
                            total = (
                                float(getattr(tfs, "sec", 0) or 0)
                                + float(getattr(tfs, "nanosec", 0) or 0) / 1_000_000_000.0
                                + cold_hold_sec
                            )
                            sec = int(total)
                            nsec = int(round((total - sec) * 1_000_000_000.0))
                            if nsec >= 1_000_000_000:
                                sec += 1
                                nsec -= 1_000_000_000
                            tfs.sec = sec
                            tfs.nanosec = nsec
                    start_point = deepcopy(points[0])
                    start_point.positions = tuple(current_vec)
                    start_point.velocities = tuple(0.0 for _ in joint_names)
                    if getattr(start_point, "accelerations", None):
                        start_point.accelerations = tuple(0.0 for _ in joint_names)
                    start_point.time_from_start.sec = 0
                    start_point.time_from_start.nanosec = 0
                    points.insert(0, start_point)
                    if cold_hold_sec > 1e-6:
                        hold_point = deepcopy(start_point)
                        hold_sec = int(cold_hold_sec)
                        hold_nsec = int(round((cold_hold_sec - hold_sec) * 1_000_000_000.0))
                        if hold_nsec >= 1_000_000_000:
                            hold_sec += 1
                            hold_nsec -= 1_000_000_000
                        hold_point.time_from_start.sec = hold_sec
                        hold_point.time_from_start.nanosec = hold_nsec
                        points.insert(1, hold_point)
                    prepared.points = points
                    self.get_logger().info(
                        "[BRIDGE_EXEC] inserted start-state waypoint for controller "
                        f"max_start_err={max_start_err:.4f}rad lead_sec={lead_sec:.2f} "
                        f"midpoint={str(bool(inserted_midpoint)).lower()} "
                        f"cold_hold_sec={cold_hold_sec:.2f} "
                        f"short_goal_hold={str(bool(auto_short_goal_hold)).lower()} "
                        f"initial_segment_max_delta={initial_segment_max_delta:.4f}"
                    )
                elif force_cold_start_hold:
                    self.get_logger().info(
                        "[BRIDGE_EXEC] skipped cold-start hold for controller "
                        f"max_start_err={max_start_err:.4f}rad "
                        f"first_point_sec={short_goal_first_time_sec:.3f} "
                        f"initial_segment_max_delta={initial_segment_max_delta:.4f}"
                    )
                try:
                    preview_points = list(points[:2])
                    preview_lines: list[str] = []
                    for idx, point in enumerate(preview_points):
                        tfs = getattr(point, "time_from_start", None)
                        t_sec = (
                            float(getattr(tfs, "sec", 0) or 0)
                            + float(getattr(tfs, "nanosec", 0) or 0) / 1_000_000_000.0
                        ) if tfs is not None else 0.0
                        pos_vals = list(getattr(point, "positions", []) or [])
                        vel_vals = list(getattr(point, "velocities", []) or [])
                        pos_txt = ",".join(
                            f"{jname}={float(pos_vals[jdx]):.3f}"
                            for jdx, jname in enumerate(joint_names[: min(len(joint_names), len(pos_vals))])
                        )
                        vel_txt = "none"
                        if vel_vals:
                            vel_txt = ",".join(
                                f"{jname}={float(vel_vals[jdx]):.3f}"
                                for jdx, jname in enumerate(joint_names[: min(len(joint_names), len(vel_vals))])
                            )
                        preview_lines.append(
                            f"p{idx}:t={t_sec:.3f}:pos[{pos_txt}]:vel[{vel_txt}]"
                        )
                    if preview_lines:
                        self.get_logger().info(
                            "[BRIDGE_EXEC] controller trajectory preview "
                            + " | ".join(preview_lines)
                        )
                except Exception as preview_exc:
                    self.get_logger().warning(
                        "[BRIDGE_EXEC] no se pudo generar preview de trayectoria: "
                        f"{type(preview_exc).__name__}: {preview_exc}"
                    )
        return prepared

    def _extract_joint_trajectory_msg(self, trajectory) -> JointTrajectory | None:
        if trajectory is None:
            return None
        queue = [trajectory]
        visited = set()
        while queue:
            item = queue.pop(0)
            if item is None:
                continue
            item_id = id(item)
            if item_id in visited:
                continue
            visited.add(item_id)

            if isinstance(item, JointTrajectory):
                if getattr(item, "points", None):
                    return item
                continue

            jt = getattr(item, "joint_trajectory", None)
            if isinstance(jt, JointTrajectory) and getattr(jt, "points", None):
                return jt

            for attr in (
                "trajectory",
                "_trajectory",
                "robot_trajectory",
                "_robot_trajectory",
                "_msg",
                "msg",
            ):
                try:
                    nested = getattr(item, attr, None)
                except Exception:
                    nested = None
                if nested is not None:
                    queue.append(nested)

            for meth in ("get_robot_trajectory_msg", "to_msg"):
                fn = getattr(item, meth, None)
                if callable(fn):
                    try:
                        nested = fn()
                    except Exception:
                        nested = None
                    if nested is not None:
                        queue.append(nested)

        self.get_logger().warning(
            "Fallback execution: no se pudo extraer JointTrajectory "
            f"(trajectory_type={type(trajectory).__name__})"
        )
        return None

    @staticmethod
    def _scale_joint_trajectory_timing(jt: JointTrajectory, scale: float = 2.0) -> JointTrajectory:
        return scale_joint_trajectory_timing(jt, scale)

    @staticmethod
    def _joint_trajectory_duration_sec(jt: JointTrajectory | None) -> float:
        return joint_trajectory_duration_sec(jt)
