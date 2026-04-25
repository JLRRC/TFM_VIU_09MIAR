"""visual_autopick_node: rclpy node that executes the 9-phase autopick sequence.

Phases:
  0 — WAIT_IMAGE
  1 — READ_OBJECT
  2 — TF_CHECK
  3 — COMPUTE_TARGETS
  4 — MOVE_APPROACH
  5 — MOVE_GRASP
  6 — CLOSE_GRIPPER
  7 — LIFT
  8 — DONE
"""
from __future__ import annotations

import math
import threading
import time
from typing import Callable, Dict, Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from .gripper_client import GripperClient
from .image_monitor import ImageMonitor
from .moveit_tcp_client import MoveItTcpClient
from .object_pose_reader import ObjectPoseReader
from .safety_gate import check_all_preflight, check_object_on_table, check_workspace
from .tf_utils import lookup_transform, transform_point


PHASE_NAMES = [
    "IDLE",
    "WAIT_IMAGE",
    "READ_OBJECT",
    "TF_CHECK",
    "COMPUTE_TARGETS",
    "MOVE_APPROACH",
    "MOVE_GRASP",
    "CLOSE_GRIPPER",
    "LIFT",
    "DONE",
    "FAIL",
    "STOPPED",
]


class VisualAutopickNode(Node):
    """Main node for visual autopick. Runs the pick sequence in a background thread."""

    # ------------------------------------------------------------------ init --
    def __init__(self) -> None:
        super().__init__("visual_autopick_node")

        # ---- Parameters (declared with defaults, can be overridden via YAML) ----
        self.declare_parameter("camera_topic", "/camera_overhead/image")
        self.declare_parameter("pose_info_topic", "/world/ur5_mesa_objetos/pose/info")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tcp_frame", "rg2_pinch_center")
        self.declare_parameter("object_name", "pick_demo")
        self.declare_parameter("tf_max_age_sec", 0.5)
        self.declare_parameter("image_min_frames", 3)
        self.declare_parameter("image_max_age_sec", 1.0)
        self.declare_parameter("object_pose_max_age_sec", 1.0)
        self.declare_parameter("approach_z_offset_m", 0.035)
        self.declare_parameter("grasp_z_offset_m", 0.005)
        self.declare_parameter("lift_z_offset_m", 0.080)
        self.declare_parameter("approach_tolerance_m", 0.020)
        self.declare_parameter("grasp_xy_tolerance_m", 0.020)
        self.declare_parameter("grasp_z_tolerance_m", 0.020)
        self.declare_parameter("grasp_dist_tolerance_m", 0.030)
        self.declare_parameter("lift_min_delta_z_m", 0.050)
        self.declare_parameter("moveit_timeout_sec", 10.0)
        self.declare_parameter("gripper_close_timeout_sec", 4.0)
        self.declare_parameter("gripper_min_close_delta_sum", 0.01)

        # Workspace limits (nested in YAML, flat params here)
        self.declare_parameter("workspace_limits.base_x_min", 0.20)
        self.declare_parameter("workspace_limits.base_x_max", 0.70)
        self.declare_parameter("workspace_limits.base_y_min", -0.30)
        self.declare_parameter("workspace_limits.base_y_max", 0.30)
        self.declare_parameter("workspace_limits.base_z_min", 0.00)
        self.declare_parameter("workspace_limits.base_z_max", 0.30)

        # ---- Read parameters ----
        self._camera_topic = str(self.get_parameter("camera_topic").value)
        self._pose_info_topic = str(self.get_parameter("pose_info_topic").value)
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tcp_frame = str(self.get_parameter("tcp_frame").value)
        self._object_name = str(self.get_parameter("object_name").value)
        self._tf_max_age = float(self.get_parameter("tf_max_age_sec").value)
        self._img_min_frames = int(self.get_parameter("image_min_frames").value)
        self._img_max_age = float(self.get_parameter("image_max_age_sec").value)
        self._obj_max_age = float(self.get_parameter("object_pose_max_age_sec").value)
        self._approach_z = float(self.get_parameter("approach_z_offset_m").value)
        self._grasp_z = float(self.get_parameter("grasp_z_offset_m").value)
        self._lift_z = float(self.get_parameter("lift_z_offset_m").value)
        self._approach_tol = float(self.get_parameter("approach_tolerance_m").value)
        self._grasp_xy_tol = float(self.get_parameter("grasp_xy_tolerance_m").value)
        self._grasp_z_tol = float(self.get_parameter("grasp_z_tolerance_m").value)
        self._grasp_dist_tol = float(self.get_parameter("grasp_dist_tolerance_m").value)
        self._lift_min_delta_z = float(self.get_parameter("lift_min_delta_z_m").value)
        self._moveit_timeout = float(self.get_parameter("moveit_timeout_sec").value)
        self._gripper_close_timeout = float(self.get_parameter("gripper_close_timeout_sec").value)
        self._gripper_min_delta = float(
            self.get_parameter("gripper_min_close_delta_sum").value
        )

        self._workspace_limits = {
            "base_x_min": float(self.get_parameter("workspace_limits.base_x_min").value),
            "base_x_max": float(self.get_parameter("workspace_limits.base_x_max").value),
            "base_y_min": float(self.get_parameter("workspace_limits.base_y_min").value),
            "base_y_max": float(self.get_parameter("workspace_limits.base_y_max").value),
            "base_z_min": float(self.get_parameter("workspace_limits.base_z_min").value),
            "base_z_max": float(self.get_parameter("workspace_limits.base_z_max").value),
        }

        # ---- State ----
        self._phase = "IDLE"
        self._phase_lock = threading.Lock()
        self._one_shot_done = False
        self._stop_requested = False
        self._armed = False
        self._last_log: str = ""
        self._status_callbacks: list = []

        # ---- TF ----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- Sub-components ----
        self._image_monitor = ImageMonitor(self, self._camera_topic)
        self._pose_reader = ObjectPoseReader(self, self._pose_info_topic)
        self._moveit_client = MoveItTcpClient(self)
        self._gripper_client = GripperClient(self)

        # ---- Timers ----
        self.create_timer(0.5, self._status_timer)

        self.get_logger().info(
            "[VISUAL_AUTOPICK] node initialised. "
            f"camera={self._camera_topic} "
            f"object={self._object_name} "
            f"base={self._base_frame} tcp={self._tcp_frame}"
        )

    # ---------------------------------------------------------------- status --
    def register_status_callback(self, cb: Callable) -> None:
        self._status_callbacks.append(cb)

    def _status_timer(self) -> None:
        status = self.get_full_status()
        for cb in self._status_callbacks:
            try:
                cb(status)
            except Exception:
                pass

    def get_full_status(self) -> Dict:
        img = self._image_monitor.get_status()
        obj_pose, obj_age, obj_err = self._pose_reader.get_object_pose(
            self._object_name, self._obj_max_age
        )
        tf_tcp, tf_age, tf_err = lookup_transform(
            self._tf_buffer,
            self._world_frame,
            self._tcp_frame,
            max_age_sec=self._tf_max_age,
        )
        with self._phase_lock:
            phase = self._phase

        return {
            "phase": phase,
            "image": img,
            "object_detected": obj_pose is not None,
            "object_pos_world": obj_pose,
            "object_age_sec": obj_age,
            "object_error": obj_err,
            "tf_ok": tf_tcp is not None and not tf_err,
            "tf_age_sec": tf_age,
            "tf_error": tf_err,
            "bridge_alive": self._moveit_client.is_bridge_alive(),
            "one_shot_done": self._one_shot_done,
            "stop_requested": self._stop_requested,
            "armed": self._armed,
            "last_log": self._last_log,
        }

    # ---------------------------------------------------------------- control --
    def arm_autopick(self) -> Tuple[bool, str]:
        """Called by the panel when ARM AUTO PICK is pressed."""
        status = self.get_full_status()
        ok, reason = check_all_preflight(
            image_ok=self._image_monitor.is_valid(self._img_min_frames, self._img_max_age),
            object_ok=status["object_detected"],
            tf_ok=status["tf_ok"],
            bridge_ok=status["bridge_alive"],
            one_shot_done=self._one_shot_done,
            stop_requested=self._stop_requested,
        )
        if not ok:
            return False, f"preflight failed: {reason}"

        self._armed = True
        self._stop_requested = False
        t = threading.Thread(target=self._run_sequence, daemon=True, name="autopick_seq")
        t.start()
        return True, "armed — sequence started"

    def stop(self) -> None:
        self._stop_requested = True
        self._armed = False
        with self._phase_lock:
            self._phase = "STOPPED"
        self._log("[VISUAL_AUTOPICK] STOP requested by user")

    def reset(self) -> None:
        self._one_shot_done = False
        self._stop_requested = False
        self._armed = False
        with self._phase_lock:
            self._phase = "IDLE"
        self._log("[VISUAL_AUTOPICK] RESET")

    # ---------------------------------------------------------------- logging --
    def _log(self, msg: str) -> None:
        self._last_log = msg
        self.get_logger().info(msg)

    def _fail(self, reason: str) -> None:
        with self._phase_lock:
            self._phase = "FAIL"
        self._one_shot_done = True
        self._log(f"[VISUAL_AUTOPICK][DONE] result=FAIL reason={reason} object={self._object_name}")

    # ---------------------------------------------------------------- sequence --
    def _set_phase(self, phase: str) -> None:
        with self._phase_lock:
            self._phase = phase
        self._log(f"[VISUAL_AUTOPICK][PHASE] -> {phase}")

    def _check_stop(self) -> bool:
        return self._stop_requested

    def _run_sequence(self) -> None:
        try:
            self._sequence_impl()
        except Exception as exc:
            self._fail(f"unhandled exception: {exc}")

    def _sequence_impl(self) -> None:
        # ---- FASE 0: WAIT_IMAGE ----
        self._set_phase("WAIT_IMAGE")
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if self._check_stop():
                return
            if self._image_monitor.is_valid(self._img_min_frames, self._img_max_age):
                break
            time.sleep(0.2)
        else:
            self._fail("timeout waiting for valid image")
            return

        img_status = self._image_monitor.get_status()
        self._log(
            f"[VISUAL_AUTOPICK][IMAGE] "
            f"received=true "
            f"width={img_status.get('width',0)} "
            f"height={img_status.get('height',0)} "
            f"age_sec={img_status.get('age_sec', 0.0):.3f}"
        )

        if self._check_stop():
            return

        # ---- FASE 1: READ_OBJECT ----
        self._set_phase("READ_OBJECT")
        obj_world, obj_age, obj_err = self._pose_reader.get_object_pose(
            self._object_name, self._obj_max_age
        )
        if obj_world is None:
            self._fail(f"READ_OBJECT failed: {obj_err}")
            return

        ox_w, oy_w, oz_w = obj_world
        # Validate on-table
        ok_table, table_err = check_object_on_table(oz_w)
        if not ok_table:
            self._log(
                f"[VISUAL_AUTOPICK][OBJECT] WARNING table check: {table_err}"
            )

        obj_base, tf_err_obj = transform_point(
            self._tf_buffer,
            self._base_frame,
            self._world_frame,
            obj_world,
            max_age_sec=self._tf_max_age,
        )
        if obj_base is None:
            self._fail(f"READ_OBJECT TF world→base failed: {tf_err_obj}")
            return

        self._log(
            f"[VISUAL_AUTOPICK][OBJECT] "
            f"name={self._object_name} "
            f"world=({ox_w:.3f},{oy_w:.3f},{oz_w:.3f}) "
            f"base=({obj_base[0]:.3f},{obj_base[1]:.3f},{obj_base[2]:.3f}) "
            f"age_sec={obj_age:.3f} "
            f"source=pose_info"
        )

        if self._check_stop():
            return

        # ---- FASE 2: TF_CHECK ----
        self._set_phase("TF_CHECK")
        tf_w_b, age_wb, err_wb = lookup_transform(
            self._tf_buffer, self._world_frame, self._base_frame,
            max_age_sec=self._tf_max_age
        )
        tf_w_tcp, age_wtcp, err_wtcp = lookup_transform(
            self._tf_buffer, self._world_frame, self._tcp_frame,
            max_age_sec=self._tf_max_age
        )
        tf_b_tcp, age_btcp, err_btcp = lookup_transform(
            self._tf_buffer, self._base_frame, self._tcp_frame,
            max_age_sec=self._tf_max_age
        )

        if tf_w_tcp is None or tf_b_tcp is None:
            verdict = "FAIL"
            self._log(
                f"[VISUAL_AUTOPICK][TF] "
                f"tcp_world=N/A tcp_base=N/A "
                f"age_sec=N/A verdict={verdict} "
                f"err_world_tcp={err_wtcp} err_base_tcp={err_btcp}"
            )
            self._fail(f"TF_CHECK failed: {err_wtcp or err_btcp}")
            return

        def _tf_pos(tf):
            t = tf.transform.translation
            return (t.x, t.y, t.z)

        tcp_world = _tf_pos(tf_w_tcp)
        tcp_base = _tf_pos(tf_b_tcp)
        tf_stale = bool(err_wtcp) or bool(err_btcp)
        verdict = "STALE" if tf_stale else "OK"

        self._log(
            f"[VISUAL_AUTOPICK][TF] "
            f"tcp_world=({tcp_world[0]:.3f},{tcp_world[1]:.3f},{tcp_world[2]:.3f}) "
            f"tcp_base=({tcp_base[0]:.3f},{tcp_base[1]:.3f},{tcp_base[2]:.3f}) "
            f"age_sec={age_btcp:.3f if age_btcp else 'N/A'} "
            f"verdict={verdict}"
        )

        if tf_stale:
            self._fail(f"TF STALE: {err_wtcp or err_btcp}")
            return

        if self._check_stop():
            return

        # ---- FASE 3: COMPUTE_TARGETS ----
        self._set_phase("COMPUTE_TARGETS")
        approach_w = (ox_w, oy_w, oz_w + self._approach_z)
        grasp_w = (ox_w, oy_w, oz_w + self._grasp_z)
        lift_w = (ox_w, oy_w, oz_w + self._lift_z)

        approach_b, e1 = transform_point(
            self._tf_buffer, self._base_frame, self._world_frame,
            approach_w, max_age_sec=self._tf_max_age
        )
        grasp_b, e2 = transform_point(
            self._tf_buffer, self._base_frame, self._world_frame,
            grasp_w, max_age_sec=self._tf_max_age
        )
        lift_b, e3 = transform_point(
            self._tf_buffer, self._base_frame, self._world_frame,
            lift_w, max_age_sec=self._tf_max_age
        )

        if approach_b is None or grasp_b is None or lift_b is None:
            self._fail(f"COMPUTE_TARGETS TF failed: {e1 or e2 or e3}")
            return

        # Workspace check on grasp target
        ok_ws, ws_err = check_workspace(grasp_b, self._workspace_limits)
        if not ok_ws:
            self._fail(f"COMPUTE_TARGETS workspace check failed: {ws_err}")
            return

        def _fmt3(pt):
            return f"({pt[0]:.3f},{pt[1]:.3f},{pt[2]:.3f})"

        self._log(
            f"[VISUAL_AUTOPICK][TARGETS] "
            f"object_world={_fmt3(obj_world)} "
            f"approach_world={_fmt3(approach_w)} "
            f"grasp_world={_fmt3(grasp_w)} "
            f"lift_world={_fmt3(lift_w)} "
            f"approach_base={_fmt3(approach_b)} "
            f"grasp_base={_fmt3(grasp_b)} "
            f"lift_base={_fmt3(lift_b)}"
        )

        if self._check_stop():
            return

        # ---- FASE 4: MOVE_APPROACH ----
        self._set_phase("MOVE_APPROACH")
        self._log(
            f"[VISUAL_AUTOPICK][MOVE_REQUEST] "
            f"phase=APPROACH tcp_frame={self._tcp_frame} "
            f"target_base={_fmt3(approach_b)}"
        )
        ok, msg = self._moveit_client.move_tcp(
            *approach_b, frame_id=self._base_frame, timeout_sec=self._moveit_timeout
        )
        # Read TCP position after move
        tcp_after = self._get_tcp_base()
        dist_approach = _dist(tcp_after, approach_b) if tcp_after else None
        self._log(
            f"[VISUAL_AUTOPICK][MOVE_RESULT] "
            f"phase=APPROACH success={str(ok).lower()} error={msg} "
            f"tcp_after={_fmt3(tcp_after) if tcp_after else 'N/A'} "
            f"dist_to_target={dist_approach:.4f if dist_approach is not None else 'N/A'}"
        )
        if not ok:
            self._fail(f"MOVE_APPROACH failed: {msg}")
            return
        if dist_approach is not None and dist_approach > self._approach_tol:
            self._log(
                f"[VISUAL_AUTOPICK][MOVE_RESULT] WARNING approach dist {dist_approach:.3f} "
                f"> tol {self._approach_tol:.3f} but continuing"
            )

        if self._check_stop():
            return

        # ---- FASE 5: MOVE_GRASP ----
        self._set_phase("MOVE_GRASP")
        self._log(
            f"[VISUAL_AUTOPICK][MOVE_REQUEST] "
            f"phase=GRASP tcp_frame={self._tcp_frame} "
            f"target_base={_fmt3(grasp_b)}"
        )
        ok, msg = self._moveit_client.move_tcp(
            *grasp_b, frame_id=self._base_frame, timeout_sec=self._moveit_timeout
        )
        tcp_after = self._get_tcp_base()
        dist_grasp = _dist(tcp_after, grasp_b) if tcp_after else None
        xy_err = _xy_dist(tcp_after, grasp_b) if tcp_after else None
        z_err = abs(tcp_after[2] - grasp_b[2]) if tcp_after else None
        dist_obj = _dist(tcp_after, obj_base) if tcp_after else None

        self._log(
            f"[VISUAL_AUTOPICK][MOVE_RESULT] "
            f"phase=GRASP success={str(ok).lower()} error={msg} "
            f"tcp_after={_fmt3(tcp_after) if tcp_after else 'N/A'} "
            f"dist_to_target={dist_grasp:.4f if dist_grasp is not None else 'N/A'}"
        )
        if not ok:
            self._fail(f"MOVE_GRASP failed: {msg}")
            return

        if self._check_stop():
            return

        # ---- FASE 6: CLOSE_GRIPPER ----
        self._set_phase("CLOSE_GRIPPER")
        g_ok, pre_sum, post_sum, delta = self._gripper_client.close_gripper(
            timeout_sec=self._gripper_close_timeout,
            min_delta_sum=self._gripper_min_delta,
        )
        verdict_g = "CLOSED" if g_ok else "FAILED"
        self._log(
            f"[VISUAL_AUTOPICK][GRIPPER] "
            f"command=close "
            f"pre_opening_sum={pre_sum:.4f} "
            f"post_opening_sum={post_sum:.4f} "
            f"delta={delta:.4f} "
            f"verdict={verdict_g}"
        )
        if not g_ok:
            self._fail(f"CLOSE_GRIPPER failed: delta={delta:.4f} < min={self._gripper_min_delta:.4f}")
            return

        if self._check_stop():
            return

        # ---- FASE 7: LIFT ----
        self._set_phase("LIFT")
        tcp_pre_lift = self._get_tcp_base()
        self._log(
            f"[VISUAL_AUTOPICK][MOVE_REQUEST] "
            f"phase=LIFT tcp_frame={self._tcp_frame} "
            f"target_base={_fmt3(lift_b)}"
        )
        ok, msg = self._moveit_client.move_tcp(
            *lift_b, frame_id=self._base_frame, timeout_sec=self._moveit_timeout
        )
        tcp_after = self._get_tcp_base()
        lift_delta_z = (tcp_after[2] - tcp_pre_lift[2]) if (tcp_after and tcp_pre_lift) else None
        self._log(
            f"[VISUAL_AUTOPICK][MOVE_RESULT] "
            f"phase=LIFT success={str(ok).lower()} error={msg} "
            f"tcp_after={_fmt3(tcp_after) if tcp_after else 'N/A'} "
            f"lift_delta_z={lift_delta_z:.4f if lift_delta_z is not None else 'N/A'}"
        )
        if not ok:
            self._fail(f"LIFT failed: {msg}")
            return

        min_lift = self._lift_z - 0.020
        if lift_delta_z is not None and lift_delta_z < min_lift:
            self._log(
                f"[VISUAL_AUTOPICK][LIFT] WARNING delta_z={lift_delta_z:.3f} < min={min_lift:.3f}"
            )

        if self._check_stop():
            return

        # ---- FASE 8: DONE ----
        self._set_phase("DONE")
        self._one_shot_done = True
        self._log(
            f"[VISUAL_AUTOPICK][DONE] result=SUCCESS reason=sequence_complete "
            f"object={self._object_name}"
        )

    # ---------------------------------------------------------------- helpers --
    def _get_tcp_base(self) -> Optional[Tuple[float, float, float]]:
        t, _, err = lookup_transform(
            self._tf_buffer, self._base_frame, self._tcp_frame,
            max_age_sec=self._tf_max_age * 2
        )
        if t is None:
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, tr.z)


def _dist(a, b) -> float:
    if a is None or b is None:
        return float("inf")
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def _xy_dist(a, b) -> float:
    if a is None or b is None:
        return float("inf")
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def main() -> None:
    rclpy.init()
    node = VisualAutopickNode()

    # Launch PyQt panel in the main thread using a separate process or same thread.
    # We use a QApplication approach: spin node in background thread.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy_spin")
    spin_thread.start()

    # Import panel here to allow the node to be used without a display
    try:
        import sys
        from PyQt5.QtWidgets import QApplication
        from .visual_autopick_panel import VisualAutopickPanel

        app = QApplication(sys.argv)
        panel = VisualAutopickPanel(node)
        panel.show()
        app.exec_()
    except ImportError as exc:
        node.get_logger().warning(f"PyQt5 not available; running headless: {exc}")
        spin_thread.join()
    finally:
        node.get_logger().info("[VISUAL_AUTOPICK] shutting down")
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
