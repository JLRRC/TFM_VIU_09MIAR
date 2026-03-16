#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/autonomous_touch_probe.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""AUTONOMOUS MOVEIT TOUCH TUNER (CLI): ensayo/error para TOUCH en base_link."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time
import uuid
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray, String
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _str2bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class MoveItResult:
    request_id: int
    success: bool
    plan_ok: bool
    exec_ok: bool
    message: str
    frame_id: str
    ee_link: str
    raw: str


class AutonomousTouchProbe(Node):
    def __init__(self, args: argparse.Namespace, out_dir: Path) -> None:
        super().__init__("autonomous_touch_probe")
        self.args = args
        self.out_dir = out_dir
        self.base_frame = str(args.base_frame)
        self.ee_frame = str(args.ee_frame)
        self._request_seq = 0
        self._result_seq = 0
        self._last_result: Optional[MoveItResult] = None
        # FASE 3: mutex para no disparar goals en paralelo.
        self._motion_in_progress = False
        self._consecutive_timeouts = 0
        self._max_consecutive_timeouts = 3

        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, "/desired_grasp", 10)
        self.pose_pub_cartesian = self.create_publisher(PoseStamped, "/desired_grasp_cartesian", 10)
        self.result_sub = self.create_subscription(String, "/desired_grasp/result", self._on_result, 20)
        self.gripper_pub = self.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)
        self.jt_pub = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)

        self.csv_path = self.out_dir / "tuning.csv"
        self.log_path = self.out_dir / "run.log"
        self._log_file = self.log_path.open("w", encoding="utf-8")
        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            [
                "iter",
                "target_x",
                "target_y",
                "target_z",
                "tcp_x",
                "tcp_y",
                "tcp_z",
                "dxy",
                "dz",
                "plan_ok",
                "exec_ok",
                "collision",
                "notes",
            ]
        )

    def _log(self, msg: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"
        print(line, flush=True)
        self._log_file.write(line + "\n")
        self._log_file.flush()

    def close(self) -> None:
        try:
            self._csv_file.close()
        except Exception:
            pass
        try:
            self._log_file.close()
        except Exception:
            pass

    def _spin_for(self, sec: float) -> None:
        deadline = time.time() + max(0.0, sec)
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def _on_result(self, msg: String) -> None:
        self._result_seq += 1
        raw = str(msg.data or "")
        try:
            data = json.loads(raw)
            self._last_result = MoveItResult(
                request_id=int(data.get("request_id", -1)),
                success=bool(data.get("success", False)),
                plan_ok=bool(data.get("plan_ok", False)),
                exec_ok=bool(data.get("exec_ok", False)),
                message=str(data.get("message", "")),
                frame_id=str(data.get("frame_id", "")),
                ee_link=str(data.get("ee_link", "")),
                raw=raw,
            )
        except Exception:
            self._last_result = MoveItResult(
                request_id=-1,
                success=False,
                plan_ok=False,
                exec_ok=False,
                message="invalid_result_json",
                frame_id="",
                ee_link="",
                raw=raw,
            )

    def _wait_result(self, expected_request_id: int, timeout_sec: float) -> Optional[MoveItResult]:
        start_seq = self._result_seq
        deadline = time.time() + max(0.1, timeout_sec)
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._result_seq <= start_seq:
                continue
            if self._last_result is None:
                continue
            if self._last_result.request_id == expected_request_id:
                return self._last_result
        return None

    def get_tcp_pose_base_tf(self, timeout_sec: float = 0.2) -> tuple[Optional[PoseStamped], Optional[tuple[float, float, float]], str]:
        """Source of truth TCP: TF directo base_link -> rg2_tcp."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                Time(),
                timeout=Duration(seconds=max(0.01, timeout_sec)),
            )
        except Exception as exc:
            return None, None, f"TF_NOT_READY:{exc}"

        out = PoseStamped()
        out.header.frame_id = str(tf.header.frame_id or self.base_frame)
        out.header.stamp = tf.header.stamp
        out.pose.position.x = float(tf.transform.translation.x)
        out.pose.position.y = float(tf.transform.translation.y)
        out.pose.position.z = float(tf.transform.translation.z)
        out.pose.orientation.x = float(tf.transform.rotation.x)
        out.pose.orientation.y = float(tf.transform.rotation.y)
        out.pose.orientation.z = float(tf.transform.rotation.z)
        out.pose.orientation.w = float(tf.transform.rotation.w)
        rpy = _quat_to_rpy(
            out.pose.orientation.x,
            out.pose.orientation.y,
            out.pose.orientation.z,
            out.pose.orientation.w,
        )
        return out, rpy, "ok"

    def _transform_pose_to_base(self, pose: PoseStamped, timeout_sec: float = 0.2) -> tuple[Optional[PoseStamped], str]:
        src = str(pose.header.frame_id or "").strip() or self.base_frame
        if src == self.base_frame:
            return pose, "ok"
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                src,
                Time(),
                timeout=Duration(seconds=max(0.01, timeout_sec)),
            )
            transformed = do_transform_pose_stamped(pose, tf)
            transformed.header.frame_id = self.base_frame
            return transformed, "ok"
        except Exception as exc:
            return None, f"FRAME/TF MISMATCH:{exc}"

    def _request_frame(self) -> tuple[str, int, str]:
        self._request_seq += 1
        rid = int(self._request_seq)
        uid = uuid.uuid4().hex
        return f"{self.base_frame}|rid={rid}|uid={uid}", rid, uid

    def _publish_goal(self, *, label: str, x: float, y: float, z: float, orientation: tuple[float, float, float, float], cartesian: bool) -> tuple[bool, str, Optional[MoveItResult]]:
        # FASE 3: Esperar a que termine el movimiento anterior antes de enviar uno nuevo.
        if self._motion_in_progress:
            self._log(f"[MUTEX] waiting for previous motion to finish before {label}")
            drain_deadline = time.time() + 5.0
            while self._motion_in_progress and time.time() < drain_deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if self._motion_in_progress:
                self._log(f"[MUTEX] previous motion did not finish; forcing release for {label}")
            self._motion_in_progress = False

        self._motion_in_progress = True
        frame_id, rid, uid = self._request_frame()
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = float(orientation[0])
        msg.pose.orientation.y = float(orientation[1])
        msg.pose.orientation.z = float(orientation[2])
        msg.pose.orientation.w = float(orientation[3])
        pub = self.pose_pub_cartesian if cartesian else self.pose_pub
        pub.publish(msg)
        self._log(
            "[GOAL] "
            f"label={label} frame_id={msg.header.frame_id} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
            f"target_base=({x:.4f},{y:.4f},{z:.4f}) "
            f"ori=({orientation[0]:.4f},{orientation[1]:.4f},{orientation[2]:.4f},{orientation[3]:.4f}) "
            f"cartesian={str(bool(cartesian)).lower()} rid={rid} uid={uid}"
        )
        result = self._wait_result(rid, timeout_sec=float(self.args.result_timeout))
        self._motion_in_progress = False
        if result is None:
            self._consecutive_timeouts += 1
            self._log(
                f"[TIMEOUT] {label} execute_timeout:no_result "
                f"consecutive={self._consecutive_timeouts}/{self._max_consecutive_timeouts}"
            )
            if self._consecutive_timeouts >= self._max_consecutive_timeouts:
                self._log(
                    f"[ABORT] {self._max_consecutive_timeouts} consecutive timeouts; "
                    "bridge may be unresponsive"
                )
            return False, "execute_timeout:no_result", None
        # Reset consecutive timeout counter on any response.
        self._consecutive_timeouts = 0
        if result.frame_id.split("|", 1)[0].strip() != self.base_frame:
            self._log(
                "[WARNING] FRAME/TF MISMATCH "
                f"result_frame={result.frame_id} expected={self.base_frame}"
            )
        return bool(result.success), str(result.message), result

    def _classify_failure(self, message: str) -> str:
        txt = str(message or "").lower()
        if "tf" in txt or "frame" in txt:
            return "tf_missing_or_mismatch"
        if "collision" in txt:
            return "collision"
        if "timeout" in txt:
            return "execute_timeout"
        if "plan" in txt and "fail" in txt:
            return "planning_failed"
        if "exec" in txt and "fail" in txt:
            return "execute_failed"
        if "unreachable" in txt or "no_trajectory" in txt:
            return "unreachable"
        return "unknown"

    def _open_gripper(self) -> None:
        msg = Float64MultiArray()
        msg.data = [float(self.args.gripper_open_rad), float(self.args.gripper_open_rad)]
        self.gripper_pub.publish(msg)
        self._log(f"[GRIPPER] open cmd={msg.data}")
        self._spin_for(0.4)

    def _go_home(self) -> None:
        joints = [float(v) for v in str(self.args.home_joints).split(",") if str(v).strip()]
        if len(joints) != 6:
            joints = [0.0] * 6
        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start = DurationMsg(sec=max(1, int(round(float(self.args.home_move_sec)))), nanosec=0)
        traj.points = [point]
        self.jt_pub.publish(traj)
        self._log(f"[HOME] published joints={joints}")
        self._spin_for(float(self.args.home_move_sec) + 0.4)

    def _target_from_name(self) -> tuple[float, float, float, str]:
        name = str(self.args.target_name).strip().upper()
        source_note = "target_from_args"
        if self.args.target_x is not None and self.args.target_y is not None and self.args.target_z is not None:
            return float(self.args.target_x), float(self.args.target_y), float(self.args.target_z), source_note

        table_center_x = float(self.args.table_center_x)
        table_center_y = float(self.args.table_center_y)
        table_size_x = float(self.args.table_size_x)
        table_size_y = float(self.args.table_size_y)
        inset = float(self.args.corner_inset_m)
        table_z_world = float(self.args.table_z_world)

        x_min = table_center_x - table_size_x * 0.5
        x_max = table_center_x + table_size_x * 0.5
        y_min = table_center_y - table_size_y * 0.5
        y_max = table_center_y + table_size_y * 0.5
        inset_x = min(max(0.02, inset), max(0.02, (x_max - x_min) * 0.24))
        inset_y = min(max(0.02, inset), max(0.02, (y_max - y_min) * 0.24))

        edge_name = "x_min"
        try:
            tf_world_base = self.tf_buffer.lookup_transform(
                "world",
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            bx = float(tf_world_base.transform.translation.x)
            by = float(tf_world_base.transform.translation.y)
            edge_centers = {
                "x_min": (x_min, table_center_y),
                "x_max": (x_max, table_center_y),
                "y_min": (table_center_x, y_min),
                "y_max": (table_center_x, y_max),
            }
            edge_name = min(
                edge_centers.keys(),
                key=lambda key: math.hypot(edge_centers[key][0] - bx, edge_centers[key][1] - by),
            )
        except Exception:
            edge_name = "x_min"

        if edge_name == "x_min":
            raw_points = [
                (x_min + inset_x, y_min + inset_y, table_z_world),
                (x_min + inset_x, y_max - inset_y, table_z_world),
            ]
        elif edge_name == "x_max":
            raw_points = [
                (x_max - inset_x, y_min + inset_y, table_z_world),
                (x_max - inset_x, y_max - inset_y, table_z_world),
            ]
        elif edge_name == "y_min":
            raw_points = [
                (x_min + inset_x, y_min + inset_y, table_z_world),
                (x_max - inset_x, y_min + inset_y, table_z_world),
            ]
        else:
            raw_points = [
                (x_min + inset_x, y_max - inset_y, table_z_world),
                (x_max - inset_x, y_max - inset_y, table_z_world),
            ]

        converted: list[tuple[float, float, float]] = []
        tf_deadline = time.time() + max(1.0, float(self.args.tf_wait_sec))
        tf_fail_detail = ""
        while time.time() < tf_deadline and len(converted) < 2:
            converted = []
            tf_fail_detail = ""
            for wx, wy, wz in raw_points:
                p = PoseStamped()
                p.header.frame_id = "world"
                p.pose.position.x = float(wx)
                p.pose.position.y = float(wy)
                p.pose.position.z = float(wz)
                p.pose.orientation.w = 1.0
                pb, status = self._transform_pose_to_base(p, timeout_sec=0.15)
                if pb is None:
                    tf_fail_detail = str(status)
                    converted = []
                    break
                converted.append(
                    (float(pb.pose.position.x), float(pb.pose.position.y), float(pb.pose.position.z))
                )
            if len(converted) < 2:
                self._spin_for(0.08)

        if len(converted) < 2:
            # Fallback explícito en base_link para robustez de arranque.
            fallback_left = [float(v) for v in str(self.args.fallback_front_left_base).split(",")]
            fallback_right = [float(v) for v in str(self.args.fallback_front_right_base).split(",")]
            if len(fallback_left) != 3 or len(fallback_right) != 3:
                raise RuntimeError(
                    f"FRAME/TF MISMATCH converting world->base_link: {tf_fail_detail}"
                )
            marker_map_fb = {
                "FRONT_LEFT": (fallback_left[0], fallback_left[1], fallback_left[2]),
                "FRONT_RIGHT": (fallback_right[0], fallback_right[1], fallback_right[2]),
            }
            if name not in marker_map_fb:
                raise RuntimeError(
                    f"FRAME/TF MISMATCH converting world->base_link: {tf_fail_detail}"
                )
            tx, ty, tz = marker_map_fb[name]
            self._log(
                "[WARNING] FRAME/TF MISMATCH world->base_link; "
                f"using fallback_base target_name={name} "
                f"target_base=({tx:.4f},{ty:.4f},{tz:.4f}) detail={tf_fail_detail}"
            )
            return tx, ty, tz, f"target_fallback_base:{name}"

        ordered = sorted(converted, key=lambda row: row[1], reverse=True)
        marker_map = {
            "FRONT_LEFT": ordered[0],
            "FRONT_RIGHT": ordered[1],
        }
        if name not in marker_map:
            raise RuntimeError(f"target_name no soportado: {name}")
        tx, ty, tz = marker_map[name]
        return tx, ty, tz, f"target_from_panel_formula:{name}:edge={edge_name}:source=world"

    def run(self) -> int:
        self._log("[START] AUTONOMOUS MOVEIT TOUCH TUNER")
        self._log(
            "[CONFIG] "
            f"base_frame={self.base_frame} ee_frame={self.ee_frame} target_name={self.args.target_name} "
            f"tol_xy={self.args.tol_xy:.4f} tol_z={self.args.tol_z:.4f} max_iters={self.args.max_iters} "
            f"z_start_offset={self.args.z_start_offset:.4f} z_step={self.args.z_step:.4f} "
            f"cartesian_descent={str(self.args.cartesian_descent).lower()} "
            f"constraint_orientation={str(self.args.constraint_orientation).lower()} "
            f"allowed_collision_table={str(self.args.allowed_collision_table).lower()}"
        )

        self._spin_for(0.6)

        # FASE 4: Verificar que el ee_frame (rg2_tcp) es alcanzable por TF.
        tcp_check, _, tcp_status = self.get_tcp_pose_base_tf(timeout_sec=1.0)
        if tcp_check is not None:
            self._log(
                f"[FASE4] tip_frame={self.ee_frame} verified ok "
                f"pos=({tcp_check.pose.position.x:.4f},"
                f"{tcp_check.pose.position.y:.4f},"
                f"{tcp_check.pose.position.z:.4f})"
            )
        else:
            self._log(f"[FASE4] WARNING tip_frame={self.ee_frame} NOT reachable: {tcp_status}")

        target_x, target_y, target_z, target_source = self._target_from_name()
        self._log(
            f"[TARGET] source={target_source} target_base=({target_x:.4f},{target_y:.4f},{target_z:.4f}) frame={self.base_frame}"
        )

        self._go_home()
        self._open_gripper()

        tcp0, rpy0, tcp0_status = self.get_tcp_pose_base_tf(timeout_sec=0.2)
        if tcp0 is None:
            self._log(f"[ABORT] TF_NOT_READY before loop: {tcp0_status}")
            return 3
        ori_fixed = (
            float(tcp0.pose.orientation.x),
            float(tcp0.pose.orientation.y),
            float(tcp0.pose.orientation.z),
            float(tcp0.pose.orientation.w),
        )
        self._log(
            "[TCP_INIT] "
            f"frame={tcp0.header.frame_id} stamp={tcp0.header.stamp.sec}.{tcp0.header.stamp.nanosec:09d} "
            f"pos=({tcp0.pose.position.x:.4f},{tcp0.pose.position.y:.4f},{tcp0.pose.position.z:.4f}) "
            f"rpy=({math.degrees(rpy0[0]):.2f},{math.degrees(rpy0[1]):.2f},{math.degrees(rpy0[2]):.2f})"
        )

        bias_x = 0.0
        bias_y = 0.0
        bias_z = 0.0
        collision_mode = False

        for it in range(1, int(self.args.max_iters) + 1):
            if not rclpy.ok():
                self._log("[ABORT] ROS no disponible")
                return 4

            tcp_before, rpy_before, before_status = self.get_tcp_pose_base_tf(timeout_sec=0.2)
            if tcp_before is None:
                self._log(f"[ITER {it}] TF_NOT_READY before move: {before_status}")
                continue

            orientation = ori_fixed if self.args.constraint_orientation else (
                float(tcp_before.pose.orientation.x),
                float(tcp_before.pose.orientation.y),
                float(tcp_before.pose.orientation.z),
                float(tcp_before.pose.orientation.w),
            )
            orientation_candidates = [orientation]
            # Fallbacks robustos (usados típicamente en el panel para pick/touch).
            for cand in (
                (0.0, 0.707, 0.707, 0.0),
                (0.0, 0.0, 1.0, 0.0),
                (0.0, 1.0, 0.0, 0.0),
            ):
                if cand not in orientation_candidates:
                    orientation_candidates.append(cand)

            tx = float(target_x + bias_x)
            ty = float(target_y + bias_y)
            tz_touch = float(target_z + bias_z)
            tz_pre = float(tz_touch + self.args.z_start_offset)

            self._log(
                f"[ITER {it}] pre target_base=({tx:.4f},{ty:.4f},{tz_pre:.4f}) touch target_base=({tx:.4f},{ty:.4f},{tz_touch:.4f})"
            )

            self._open_gripper()
            ok_pre = False
            msg_pre = "pre_not_attempted"
            res_pre = None
            selected_orientation = orientation
            for idx_ori, ori_try in enumerate(orientation_candidates, start=1):
                ok_pre, msg_pre, res_pre = self._publish_goal(
                    label=f"AUTO_TOUCH_PRE_{it}_ORI{idx_ori}",
                    x=tx,
                    y=ty,
                    z=tz_pre,
                    orientation=ori_try,
                    cartesian=False,
                )
                selected_orientation = ori_try
                if ok_pre:
                    if idx_ori > 1:
                        self._log(
                            "[ITER %d] PRETOUCH orientation_fallback_ok idx=%d ori=(%.4f,%.4f,%.4f,%.4f)"
                            % (it, idx_ori, ori_try[0], ori_try[1], ori_try[2], ori_try[3])
                        )
                    break
                reason_pre_try = self._classify_failure(msg_pre)
                if reason_pre_try not in ("unreachable", "planning_failed"):
                    break
                self._log(
                    "[ITER %d] PRETOUCH orientation_retry idx=%d reason=%s msg=%s"
                    % (it, idx_ori, reason_pre_try, msg_pre)
                )

            if not ok_pre:
                reason = self._classify_failure(msg_pre)
                self._log(f"[ITER {it}] PRETOUCH fail reason={reason} msg={msg_pre}")
                self._csv.writerow([it, tx, ty, tz_touch, "", "", "", "", "", False, False, reason == "collision", f"PRETOUCH:{reason}:{msg_pre}"])
                self._csv_file.flush()
                # FASE 3: Abortar si hay demasiados timeouts consecutivos.
                if self._consecutive_timeouts >= self._max_consecutive_timeouts:
                    self._log("[ABORT] too many consecutive timeouts in PRETOUCH; stopping")
                    return 5
                continue

            touch_cartesian = bool(self.args.cartesian_descent)
            ok_touch, msg_touch, res_touch = self._publish_goal(
                label=f"AUTO_TOUCH_TOUCH_{it}",
                x=tx,
                y=ty,
                z=tz_touch,
                orientation=selected_orientation,
                cartesian=touch_cartesian,
            )

            note = ""
            if (not ok_touch) and touch_cartesian:
                reason = self._classify_failure(msg_touch)
                self._log(f"[ITER {it}] TOUCH cartesian fail reason={reason} msg={msg_touch}; fallback planner")
                ok_touch, msg_touch, res_touch = self._publish_goal(
                    label=f"AUTO_TOUCH_TOUCH_FALLBACK_{it}",
                    x=tx,
                    y=ty,
                    z=tz_touch,
                    orientation=selected_orientation,
                    cartesian=False,
                )
                note = f"cartesian_fallback:{reason}"

            if (not ok_touch) and self.args.allowed_collision_table:
                reason = self._classify_failure(msg_touch)
                if reason == "collision":
                    collision_mode = True
                    note = (note + "|" if note else "") + "collision_mitigation:shorter_descent"
                    self._log(
                        "[MITIGATION] collision detectada; allowed_collision_table=true solicitado. "
                        "Aplicando mitigación conservadora (descenso más corto/fallback)."
                    )

            tcp_touch, rpy_touch, touch_status = self.get_tcp_pose_base_tf(timeout_sec=0.2)
            if tcp_touch is None:
                self._log(f"[ITER {it}] TF_NOT_READY after touch: {touch_status}")
                self._csv.writerow([it, tx, ty, tz_touch, "", "", "", "", "", bool(res_touch and res_touch.plan_ok), bool(res_touch and res_touch.exec_ok), "tf_missing" in touch_status, f"TOUCH:{msg_touch}:{touch_status}"])
                self._csv_file.flush()
                continue

            _ok_retreat, _msg_retreat, _res_retreat = self._publish_goal(
                label=f"AUTO_TOUCH_RETREAT_{it}",
                x=tx,
                y=ty,
                z=tz_pre,
                orientation=selected_orientation,
                cartesian=False,
            )

            dx = float(tx - tcp_touch.pose.position.x)
            dy = float(ty - tcp_touch.pose.position.y)
            dz_signed = float(tz_touch - tcp_touch.pose.position.z)
            dxy = float(math.hypot(dx, dy))
            dz = float(abs(dz_signed))

            reason = self._classify_failure(msg_touch) if not ok_touch else "ok"
            plan_ok = bool(res_touch.plan_ok) if res_touch else False
            exec_ok = bool(res_touch.exec_ok) if res_touch else False
            collision = reason == "collision"
            notes = f"reason={reason}|msg={msg_touch}"
            if note:
                notes = f"{notes}|{note}"

            self._csv.writerow(
                [
                    it,
                    tx,
                    ty,
                    tz_touch,
                    float(tcp_touch.pose.position.x),
                    float(tcp_touch.pose.position.y),
                    float(tcp_touch.pose.position.z),
                    dxy,
                    dz,
                    plan_ok,
                    exec_ok,
                    collision,
                    notes,
                ]
            )
            self._csv_file.flush()

            self._log(
                "[ITER_RESULT] "
                f"iter={it} frame_target={self.base_frame} frame_tcp={tcp_touch.header.frame_id} "
                f"stamp_tcp={tcp_touch.header.stamp.sec}.{tcp_touch.header.stamp.nanosec:09d} "
                f"target_base=({tx:.4f},{ty:.4f},{tz_touch:.4f}) "
                f"tcp_base_real=({tcp_touch.pose.position.x:.4f},{tcp_touch.pose.position.y:.4f},{tcp_touch.pose.position.z:.4f}) "
                f"error_xy={dxy:.4f} error_z={dz:.4f} err=({dx:+.4f},{dy:+.4f},{dz_signed:+.4f}) "
                f"plan_ok={str(plan_ok).lower()} exec_ok={str(exec_ok).lower()} reason={reason}"
            )

            if dxy <= float(self.args.tol_xy) and dz <= float(self.args.tol_z):
                self._log(
                    f"[SUCCESS] TOUCH_PASS iter={it} dxy={dxy:.4f} dz={dz:.4f} tol_xy={self.args.tol_xy:.4f} tol_z={self.args.tol_z:.4f}"
                )
                return 0

            # Ensayo/error: priorizar corrección de Z cuando XY ya está bien.
            if dz > float(self.args.tol_z):
                step = float(self.args.z_step)
                if tcp_touch.pose.position.z > tz_touch:
                    bias_z -= step
                else:
                    bias_z += step

            if dxy > float(self.args.tol_xy):
                gain_xy = 0.35
                cap_xy = 0.003
                if not collision_mode:
                    bias_x += _clamp(dx * gain_xy, -cap_xy, cap_xy)
                    bias_y += _clamp(dy * gain_xy, -cap_xy, cap_xy)

            # Mitigación de colisión: acortar pre-touch para evitar barridos.
            if collision_mode:
                self.args.z_start_offset = max(0.03, float(self.args.z_start_offset) - 0.01)

        self._log(
            f"[FAIL] no convergió en {self.args.max_iters} iteraciones (tol_xy={self.args.tol_xy:.4f}, tol_z={self.args.tol_z:.4f})"
        )
        return 2


def _default_reports_dir() -> Path:
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return Path("/home/laboratorio/TFM/agarre_ros2_ws/reports/touch_tuner") / stamp


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Autonomous MoveIt touch tuner")
    p.add_argument("--base_frame", default="base_link")
    p.add_argument("--ee_frame", default="rg2_tcp")
    p.add_argument("--target_name", default="FRONT_LEFT")
    p.add_argument("--target_x", type=float, default=None)
    p.add_argument("--target_y", type=float, default=None)
    p.add_argument("--target_z", type=float, default=None)
    p.add_argument("--tol_xy", type=float, default=0.02)
    p.add_argument("--tol_z", type=float, default=0.02)
    p.add_argument("--max_iters", type=int, default=50)
    p.add_argument("--z_start_offset", type=float, default=0.12)
    p.add_argument("--z_step", type=float, default=0.01)
    p.add_argument("--cartesian_descent", type=_str2bool, default=True)
    p.add_argument("--constraint_orientation", type=_str2bool, default=True)
    p.add_argument("--allowed_collision_table", type=_str2bool, default=False)
    p.add_argument("--result_timeout", type=float, default=28.0)
    p.add_argument("--gripper_open_rad", type=float, default=float(os.environ.get("PANEL_GRIPPER_OPEN_RAD", "1.0")))
    p.add_argument("--home_joints", default="0,0,0,0,0,0")
    p.add_argument("--home_move_sec", type=float, default=3.0)
    p.add_argument("--table_size_x", type=float, default=0.768)
    p.add_argument("--table_size_y", type=float, default=0.80)
    p.add_argument("--table_center_x", type=float, default=-0.17)
    p.add_argument("--table_center_y", type=float, default=0.0)
    p.add_argument("--table_z_world", type=float, default=0.075)
    p.add_argument("--corner_inset_m", type=float, default=0.06)
    p.add_argument("--tf_wait_sec", type=float, default=3.0)
    p.add_argument("--fallback_front_left_base", default="0.356,0.340,-0.700")
    p.add_argument("--fallback_front_right_base", default="0.356,-0.340,-0.700")
    p.add_argument("--reports_dir", default=str(_default_reports_dir()))
    return p


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    out_dir = Path(args.reports_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = AutonomousTouchProbe(args, out_dir)
    try:
        rc = node.run()
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return int(rc)


if __name__ == "__main__":
    raise SystemExit(main())
