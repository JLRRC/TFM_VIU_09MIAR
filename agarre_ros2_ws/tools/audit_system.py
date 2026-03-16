#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/audit_system.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""
Non-intrusive ROS 2 system audit script.
Runs layered checks using only ros2 CLI commands (no rclpy).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from typing import Dict, Optional, Tuple, List


def _run_cmd(cmd: str, timeout: float) -> Tuple[bool, str, str, str]:
    start = time.time()
    try:
        res = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return False, "", "", f"timeout>{timeout:.1f}s"
    except Exception as exc:
        return False, "", "", f"error:{exc}"
    dt = time.time() - start
    ok = res.returncode == 0
    return ok, res.stdout or "", res.stderr or "", f"{dt:.2f}s"


def _parse_joint_names(output: str) -> int:
    if not output:
        return 0
    match = re.search(r"^name:\s*\[(.*)\]$", output, re.MULTILINE)
    if match:
        inside = match.group(1).strip()
        if not inside:
            return 0
        return len([p.strip() for p in inside.split(",") if p.strip()])
    lines = output.splitlines()
    count = 0
    in_name = False
    for line in lines:
        if line.strip().startswith("name:"):
            in_name = True
            continue
        if in_name:
            if re.match(r"^\s*-\s*\S+", line):
                count += 1
            elif line.strip() and not line.strip().startswith("-"):
                break
    return count


def _parse_controllers(output: str) -> Dict[str, str]:
    state_map: Dict[str, str] = {}
    for line in output.splitlines():
        line = line.strip()
        if not line:
            continue
        match = re.match(r"^([^\s\[]+)\s*\[([^\]]+)\]$", line)
        if match:
            name, state = match.group(1), match.group(2)
        elif ":" in line:
            name, state = [p.strip() for p in line.split(":", 1)]
        else:
            parts = line.split()
            if len(parts) >= 2:
                name, state = parts[0], parts[-1]
            else:
                continue
        state_map[name] = state
    return state_map


def _parse_tf_edges(output: str) -> List[Tuple[str, str]]:
    edges: List[Tuple[str, str]] = []
    parent = None
    for line in output.splitlines():
        line = line.strip()
        if line.startswith("frame_id:"):
            parent = line.split("frame_id:", 1)[1].strip()
        elif line.startswith("child_frame_id:"):
            child = line.split("child_frame_id:", 1)[1].strip()
            if parent:
                edges.append((parent, child))
    return edges


def _parse_pose_info(output: str) -> List[Dict[str, object]]:
    messages: List[Dict[str, object]] = []
    current = {"zs": [], "named": {}}
    current_name: Optional[str] = None
    in_position = False
    in_translation = False
    for line in output.splitlines():
        if line.strip() == "---":
            if current["zs"] or current["named"]:
                messages.append(current)
            current = {"zs": [], "named": {}}
            current_name = None
            in_position = False
            in_translation = False
            continue
        name_match = re.match(r"^\s*name:\s*([^\s]+)\s*$", line)
        if name_match:
            current_name = name_match.group(1)
            continue
        child_match = re.match(r"^\s*child_frame_id:\s*([^\s]+)\s*$", line)
        if child_match:
            current_name = child_match.group(1)
            continue
        frame_match = re.match(r"^\s*frame_id:\s*([^\s]+)\s*$", line)
        if frame_match:
            current_name = frame_match.group(1)
            continue
        if line.strip().startswith("position:"):
            in_position = True
            in_translation = False
            continue
        if line.strip().startswith("translation:"):
            in_translation = True
            in_position = False
            continue
        if in_position or in_translation:
            z_match = re.match(r"^\s*z:\s*([-+0-9eE\.]+)\s*$", line)
            if z_match:
                try:
                    z_val = float(z_match.group(1))
                except ValueError:
                    z_val = None
                if z_val is not None:
                    current["zs"].append(z_val)
                    if current_name:
                        current["named"].setdefault(current_name, []).append(z_val)
                current_name = None
                in_position = False
                in_translation = False
    if current["zs"] or current["named"]:
        messages.append(current)
    return messages


def _has_path(edges: List[Tuple[str, str]], src: str, dst: str) -> bool:
    if src == dst:
        return True
    graph: Dict[str, List[str]] = {}
    for a, b in edges:
        graph.setdefault(a, []).append(b)
    seen = set([src])
    stack = [src]
    while stack:
        node = stack.pop()
        for nxt in graph.get(node, []):
            if nxt == dst:
                return True
            if nxt not in seen:
                seen.add(nxt)
                stack.append(nxt)
    return False


def _retry_until(deadline_sec: float, interval_sec: float = 0.4):
    end = time.monotonic() + max(0.1, deadline_sec)
    while time.monotonic() < end:
        yield
        time.sleep(interval_sec)


def _log_line(label: str, status: str, reason: Optional[str] = None) -> None:
    if reason:
        print(f"[AUDIT] {label:<26} {status} ({reason})")
    else:
        print(f"[AUDIT] {label:<26} {status}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit ROS 2 system readiness.")
    parser.add_argument("--json", action="store_true", help="Output JSON report.")
    parser.add_argument("--timeout", type=float, default=5.0, help="Default timeout seconds.")
    parser.add_argument("--clock-timeout", type=float, default=5.0, help="Timeout for /clock checks.")
    parser.add_argument("--tf-timeout", type=float, default=5.0, help="Timeout for TF checks.")
    parser.add_argument("--pose-timeout", type=float, default=4.0, help="Timeout for /pose/info sampling.")
    parser.add_argument("--pose-samples", type=int, default=2, help="Number of pose/info samples to compare.")
    parser.add_argument(
        "--calib-path",
        type=str,
        default="scripts/table_pixel_map.json",
        help="Calibration file path hint (optional).",
    )
    args = parser.parse_args()

    report: Dict[str, object] = {"layers": {}, "state": "", "checks": {}}

    # CAPA 0 — Infraestructura ROS
    domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
    try:
        int(domain_id)
        domain_ok = True
    except ValueError:
        domain_ok = False
    daemon_ok, daemon_out, daemon_err, daemon_dt = _run_cmd("ros2 daemon status", args.timeout)
    nodes_ok, nodes_out, nodes_err, nodes_dt = _run_cmd("ros2 node list", args.timeout)
    layer0_ok = domain_ok and daemon_ok and nodes_ok
    report["layers"]["infra"] = {
        "domain_id": domain_id,
        "domain_ok": domain_ok,
        "daemon_ok": daemon_ok,
        "nodes_ok": nodes_ok,
        "daemon_dt": daemon_dt,
        "nodes_dt": nodes_dt,
    }
    if not nodes_ok:
        _log_line("CAPA 0 ROS", "FAIL", nodes_err.strip() or nodes_dt)
    else:
        _log_line("CAPA 0 ROS", "PASS")

    # CAPA 1 — Tiempo y simulacion
    clock_topic_ok, topics_out, topics_err, topics_dt = _run_cmd("ros2 topic list", args.timeout)
    has_clock = clock_topic_ok and "/clock" in topics_out.splitlines()
    clock_msg_ok = False
    clock_reason = ""
    if has_clock:
        cmd = "ros2 topic echo --once /clock"
        clock_msg_ok, clock_out, clock_err, clock_dt = _run_cmd(cmd, args.clock_timeout)
        clock_reason = clock_err.strip() or clock_dt
    else:
        clock_reason = "no /clock topic"
    if has_clock and clock_msg_ok:
        _log_line("CAPA 1 CLOCK", "PASS")
        report["layers"]["clock"] = {"has_clock": True, "msg_ok": True}
        clock_result = "PASS_CLOCK"
    else:
        _log_line("CAPA 1 CLOCK", "FAIL", clock_reason)
        report["layers"]["clock"] = {"has_clock": has_clock, "msg_ok": False, "reason": clock_reason}
        clock_result = "FAIL_CLOCK_TIMEOUT"

    # CAPA 2 — ros2_control
    services_ok, services_out, services_err, services_dt = _run_cmd("ros2 service list", args.timeout)
    has_list_srv = services_ok and "/controller_manager/list_controllers" in services_out.splitlines()
    ctrl_result = "FAIL_CONTROLLERS_MISSING"
    ctrl_reason = "service missing"
    ctrl_active = False
    if has_list_srv:
        cmd = "ros2 control list_controllers -c /controller_manager"
        ctrl_ok, ctrl_out, ctrl_err, ctrl_dt = _run_cmd(cmd, args.timeout)
        if not ctrl_ok:
            ctrl_result = "FAIL_CONTROLLERS_TIMEOUT"
            ctrl_reason = ctrl_err.strip() or ctrl_dt
        else:
            states = _parse_controllers(ctrl_out)
            ctrl_active = states.get("joint_state_broadcaster", "") == "active"
            if ctrl_active:
                ctrl_result = "PASS_CONTROLLERS_ACTIVE"
                ctrl_reason = ""
            else:
                ctrl_result = "FAIL_CONTROLLERS_MISSING"
                ctrl_reason = "joint_state_broadcaster not active"
    report["layers"]["controllers"] = {
        "service_ok": has_list_srv,
        "active": ctrl_active,
        "result": ctrl_result,
        "reason": ctrl_reason,
    }
    if ctrl_result == "PASS_CONTROLLERS_ACTIVE":
        _log_line("CAPA 2 CONTROLLERS", "PASS")
    else:
        _log_line("CAPA 2 CONTROLLERS", "FAIL", ctrl_reason)

    # CAPA 3 — Estado articular
    has_joint_states = clock_topic_ok and "/joint_states" in topics_out.splitlines()
    js_ok = False
    js_reason = ""
    names_count = 0
    if has_joint_states:
        for _ in _retry_until(args.timeout, interval_sec=0.4):
            cmd = "ros2 topic echo --qos-reliability best_effort --once /joint_states"
            js_ok, js_out, js_err, js_dt = _run_cmd(cmd, min(2.0, args.timeout))
            names_count = _parse_joint_names(js_out)
            if js_ok and names_count > 0:
                break
        js_ok = bool(js_ok and names_count > 0)
        js_reason = js_err.strip() or (f"names={names_count}" if js_ok else "empty")
    else:
        js_reason = "no /joint_states topic"
    report["layers"]["joint_states"] = {
        "has_topic": has_joint_states,
        "ok": js_ok,
        "reason": js_reason,
    }
    if js_ok:
        _log_line("CAPA 3 JOINT STATES", "PASS")
        js_result = "PASS_JOINT_STATES"
    else:
        _log_line("CAPA 3 JOINT STATES", "FAIL", js_reason)
        js_result = "FAIL_NO_JOINT_STATES"

    # CAPA 3.5 — Pose info / física
    pose_topics = []
    if clock_topic_ok:
        pose_topics = [t for t in topics_out.splitlines() if t.startswith("/world/") and t.endswith("/pose/info")]
    pose_topic = pose_topics[0] if pose_topics else ""
    pose_ok = False
    pose_reason = ""
    pose_stats = {}
    if pose_topic:
        messages: List[Dict[str, object]] = []
        pose_err = ""
        pose_dt = ""
        for _ in range(max(1, args.pose_samples)):
            cmd = f"timeout {args.pose_timeout:.1f}s ros2 topic echo --qos-reliability best_effort --once {pose_topic}"
            pose_cmd_ok, pose_out, pose_err, pose_dt = _run_cmd(cmd, max(1.0, args.pose_timeout + 1.0))
            messages.extend(_parse_pose_info(pose_out))
            time.sleep(0.2)
        if not messages:
            pose_reason = pose_err.strip() or f"no pose samples ({pose_dt})"
        else:
            per_msg_min = [min(m["zs"]) for m in messages if m["zs"]]
            per_msg_max = [max(m["zs"]) for m in messages if m["zs"]]
            min_z = min(per_msg_min) if per_msg_min else None
            max_z = max(per_msg_max) if per_msg_max else None
            min_range = (max(per_msg_min) - min(per_msg_min)) if len(per_msg_min) >= 2 else 0.0
            max_range = (max(per_msg_max) - min(per_msg_max)) if len(per_msg_max) >= 2 else 0.0
            pieza_z = []
            named_count = 0
            for msg in messages:
                pieza_z.extend(msg["named"].get("pick_demo", []))
                named_count += len(msg["named"])
            pieza_min = min(pieza_z) if pieza_z else None
            pieza_max = max(pieza_z) if pieza_z else None
            pose_stats = {
                "topic": pose_topic,
                "messages": len(messages),
                "min_z": min_z,
                "max_z": max_z,
                "min_z_range": min_range,
                "max_z_range": max_range,
                "pick_demo_min": pieza_min,
                "pick_demo_max": pieza_max,
                "named_frames": named_count,
            }
            pose_ok = True
    else:
        pose_reason = "no /world/*/pose/info topic"
    report["layers"]["pose_info"] = {
        "topic": pose_topic,
        "ok": pose_ok,
        "reason": pose_reason,
        **pose_stats,
    }
    if pose_ok:
        detail = None
        if pose_stats.get("min_z") is not None:
            detail = (
                f"min_z={pose_stats.get('min_z'):.3f} max_z={pose_stats.get('max_z'):.3f} "
                f"range_min={pose_stats.get('min_z_range'):.3f} range_max={pose_stats.get('max_z_range'):.3f} "
                f"named={pose_stats.get('named_frames')}"
            )
        _log_line("CAPA 3.5 POSE INFO", "PASS", detail)
    else:
        _log_line("CAPA 3.5 POSE INFO", "FAIL", pose_reason)

    # CAPA 4 — TF y cinematica
    tf_ok = False
    tf_reason = ""
    tf_edges: List[Tuple[str, str]] = []
    last_tf_err = ""
    last_tf_static_err = ""
    for _ in _retry_until(args.tf_timeout, interval_sec=0.5):
        tf_ok_cmd, tf_out, tf_err, _tf_dt = _run_cmd(
            "timeout 2s ros2 topic echo --qos-reliability best_effort /tf",
            min(4.0, args.tf_timeout + 1.0),
        )
        if tf_out:
            tf_edges.extend(_parse_tf_edges(tf_out))
        elif not tf_ok_cmd:
            last_tf_err = tf_err.strip() or last_tf_err
        tf_static_ok, tf_static_out, tf_static_err, _tf_sdt = _run_cmd(
            "timeout 3s ros2 topic echo --qos-durability transient_local --qos-reliability reliable /tf_static",
            min(4.0, args.tf_timeout + 1.0),
        )
        if tf_static_out:
            tf_edges.extend(_parse_tf_edges(tf_static_out))
        elif not tf_static_ok:
            last_tf_static_err = tf_static_err.strip() or last_tf_static_err
        if tf_edges:
            base_to_tool = _has_path(tf_edges, "base_link", "tool0")
            base_to_wrist = _has_path(tf_edges, "base_link", "wrist_3_link")
            wrist_to_tool = _has_path(tf_edges, "wrist_3_link", "tool0")
            world_to_base = _has_path(tf_edges, "world", "base_link")
            if world_to_base and (base_to_tool or (base_to_wrist and wrist_to_tool)):
                tf_ok = True
                break
    if not tf_ok:
        tf_topic_present = "/tf" in topics_out.splitlines()
        static_edges = _parse_tf_edges(tf_static_out) if tf_static_out else []
        world_to_base_static = _has_path(static_edges, "world", "base_link")
        wrist_to_tool_static = _has_path(static_edges, "wrist_3_link", "tool0")
        if tf_topic_present and world_to_base_static and wrist_to_tool_static:
            tf_ok = True
            tf_reason = ""
        else:
            all_edges = tf_edges + static_edges
            base_seen = any(a == "base_link" or b == "base_link" for a, b in all_edges)
            tool_seen = any(a == "tool0" or b == "tool0" for a, b in all_edges)
            if tf_topic_present and base_seen and tool_seen:
                tf_ok = True
                tf_reason = ""
            elif tf_edges:
                tf_reason = "missing world->base_link or base_link->tool0"
            else:
                tf_reason = last_tf_err or last_tf_static_err or "tf timeout"
    if not tf_ok:
        tf2_ok, tf2_out, tf2_err, _tf2_dt = _run_cmd(
            "timeout 2s ros2 run tf2_ros tf2_echo base_link tool0",
            min(3.0, args.tf_timeout + 1.0),
        )
        tf2_text = (tf2_out or "") + (tf2_err or "")
        if "Translation" in tf2_text or "Rotation" in tf2_text:
            tf_ok = True
            tf_reason = ""
    report["layers"]["tf"] = {"ok": tf_ok, "reason": tf_reason}
    if tf_ok:
        _log_line("CAPA 4 TF", "PASS")
        tf_result = "PASS_TF_CHAIN"
    else:
        _log_line("CAPA 4 TF", "FAIL", tf_reason)
        tf_result = "FAIL_TF_INCOMPLETE"

    # CAPA 5 — Estado logico
    calib_ok = os.path.isfile(args.calib_path)
    state = "STATE_BOOTING"
    if not nodes_ok:
        state = "STATE_BOOTING"
    elif clock_result == "FAIL_CLOCK_TIMEOUT":
        state = "STATE_WAIT_CLOCK"
    elif ctrl_result != "PASS_CONTROLLERS_ACTIVE":
        state = "STATE_WAIT_CONTROLLERS"
    elif js_result != "PASS_JOINT_STATES":
        state = "STATE_NO_JOINT_STATES"
    elif tf_result != "PASS_TF_CHAIN":
        state = "STATE_ERROR"
    else:
        state = "STATE_READY_FOR_TEST"
        if calib_ok:
            state = "STATE_READY_FOR_PICK"
        else:
            state = "STATE_READY_FOR_CALIBRATION"
    report["state"] = state
    report["checks"] = {
        "clock": clock_result,
        "controllers": ctrl_result,
        "joint_states": js_result,
        "tf": tf_result,
        "calibration_ok": calib_ok,
    }

    _log_line("ESTADO GLOBAL", state, None)

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))

    ok_states = {"STATE_READY_FOR_TEST", "STATE_READY_FOR_CALIBRATION", "STATE_READY_FOR_PICK"}
    return 0 if state in ok_states else 1


if __name__ == "__main__":
    sys.exit(main())
