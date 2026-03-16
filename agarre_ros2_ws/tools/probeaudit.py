#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/probeaudit.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""
Headless audit of panel_v2 logic (static analysis only).
No ROS, no GUI, no imports from project modules.
"""
from __future__ import annotations

import argparse
import ast
import json
import os
import re
from typing import Dict, List, Optional, Tuple


class FuncSpan:
    def __init__(self, name: str, start: int, end: int) -> None:
        self.name = name
        self.start = start
        self.end = end

    def contains(self, line_no: int) -> bool:
        return self.start <= line_no <= self.end


def _read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def _collect_functions(tree: ast.AST, source_lines: List[str]) -> List[FuncSpan]:
    spans: List[FuncSpan] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            start = node.lineno
            end = getattr(node, "end_lineno", None)
            if end is None:
                end = start
                for idx in range(start - 1, len(source_lines)):
                    if source_lines[idx].startswith("def ") and idx + 1 > start:
                        end = idx
                        break
                else:
                    end = len(source_lines)
            spans.append(FuncSpan(node.name, start, end))
    return spans


def _find_function(funcs: List[FuncSpan], line_no: int) -> str:
    for fn in funcs:
        if fn.contains(line_no):
            return fn.name
    return "<module>"


def _enum_members(tree: ast.AST) -> Dict[str, List[str]]:
    enums: Dict[str, List[str]] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            if not any(isinstance(b, ast.Name) and b.id == "Enum" for b in node.bases):
                continue
            members: List[str] = []
            for stmt in node.body:
                if isinstance(stmt, ast.Assign) and len(stmt.targets) == 1:
                    target = stmt.targets[0]
                    if isinstance(target, ast.Name):
                        members.append(target.id)
            enums[node.name] = members
    return enums


def _collect_calls(tree: ast.AST, target: str) -> List[Tuple[int, Optional[str]]]:
    calls: List[Tuple[int, Optional[str]]] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if isinstance(func, ast.Attribute) and func.attr == target:
            state_name = None
            if node.args:
                arg = node.args[0]
                if isinstance(arg, ast.Attribute) and isinstance(arg.value, ast.Name):
                    state_name = f"{arg.value.id}.{arg.attr}"
            calls.append((node.lineno, state_name))
    return calls


def _extract_dependencies(fn_node: ast.FunctionDef) -> Tuple[List[str], Dict[str, List[int]]]:
    deps = set()
    locations: Dict[str, List[int]] = {}
    for node in ast.walk(fn_node):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self":
            name = f"self.{node.attr}"
            deps.add(name)
            locations.setdefault(name, []).append(node.lineno)
    return sorted(deps), locations


def _slice_function_text(source_lines: List[str], func: FuncSpan) -> str:
    return "\n".join(source_lines[func.start - 1 : func.end])


def _find_wait_loops(source_lines: List[str], funcs: List[FuncSpan]) -> List[Dict[str, str]]:
    loops = []
    for fn in funcs:
        body = source_lines[fn.start - 1 : fn.end]
        for offset, line in enumerate(body):
            if re.match(r"\\s*while\\s+.*:", line):
                window = "\n".join(body[offset : offset + 12])
                has_deadline = "deadline" in window or "timeout" in window
                has_break = "break" in window
                loops.append(
                    {
                        "function": fn.name,
                        "line": str(fn.start + offset),
                        "line_text": line.strip(),
                        "timeout_hint": "yes" if has_deadline else "no",
                        "break_hint": "yes" if has_break else "no",
                    }
                )
    return loops


def _derive_state_diagram(transitions: List[Dict[str, str]]) -> List[str]:
    lines: List[str] = []
    for t in transitions:
        lhs = t.get("from") or "?"
        rhs = t.get("to") or "?"
        ctx = t.get("function") or "?"
        lines.append(f"{lhs} -> {rhs}  [{ctx}]")
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description="Static audit of panel_v2 logic.")
    parser.add_argument(
        "--panel",
        default="src/ur5_qt_panel/ur5_qt_panel/panel_v2.py",
        help="Path to panel_v2.py",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON report.")
    parser.add_argument("--report", type=str, default="", help="Write a text report to this path.")
    args = parser.parse_args()

    panel_path = args.panel
    if not os.path.isfile(panel_path):
        print(f"[AUDIT] ERROR: panel file not found: {panel_path}")
        return 2

    text = _read_text(panel_path)
    lines = text.splitlines()
    tree = ast.parse(text)

    funcs = _collect_functions(tree, lines)
    enums = _enum_members(tree)

    flow_calls = _collect_calls(tree, "_set_flow_state")
    sys_calls = _collect_calls(tree, "_set_system_state")
    error_calls = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
            if node.value.id == "SystemState" and node.attr == "ERROR":
                error_calls.append((node.lineno, "SystemState.ERROR"))

    transitions: List[Dict[str, str]] = []
    for line_no, state_name in flow_calls:
        state = state_name or "UNKNOWN"
        transitions.append(
            {
                "type": "flow",
                "to": state,
                "function": _find_function(funcs, line_no),
                "line": str(line_no),
                "line_text": state or "",
            }
        )

    for line_no, state_name in sys_calls:
        state = state_name or "UNKNOWN"
        transitions.append(
            {
                "type": "system",
                "to": state,
                "function": _find_function(funcs, line_no),
                "line": str(line_no),
                "line_text": state or "",
            }
        )

    func_nodes = {node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
    test_node = func_nodes.get("_run_robot_test")
    ready_node = func_nodes.get("_system_ready_for_test")
    refresh_node = func_nodes.get("_refresh_controls")

    test_deps, test_loc = _extract_dependencies(test_node) if test_node else ([], {})
    ready_deps, ready_loc = _extract_dependencies(ready_node) if ready_node else ([], {})
    refresh_deps, refresh_loc = _extract_dependencies(refresh_node) if refresh_node else ([], {})

    required_groups = {
        "controllers": ["self._controllers_ok"],
        "joint_states": ["self._joint_states_ready", "self._joint_active"],
        "tf_base_tool": ["self._tf_base_to_tool_stable", "self._tf_ready_state"],
        "pose_info": ["self._pose_info_ready", "self._pose_info_ok"],
    }
    forbidden_test = [
        "self._calibration_ready",
        "self._camera_stream_ok",
        "self._moveit_state",
        "self._moveit_ready",
        "self._moveit_required",
        "self._pickable_map_override",
    ]
    missing_required = []
    for label, choices in required_groups.items():
        if not any(choice in test_deps or choice in ready_deps for choice in choices):
            missing_required.append(label)
    present_forbidden = [f for f in forbidden_test if f in test_deps or f in ready_deps]

    loops = _find_wait_loops(lines, funcs)

    state_names = {
        "FlowState": enums.get("FlowState", []),
        "SystemState": enums.get("SystemState", []),
        "MoveItState": enums.get("MoveItState", []),
    }

    redundancy = []
    if "READY" in state_names.get("FlowState", []) and "READY" in state_names.get("SystemState", []):
        redundancy.append("READY appears in FlowState and SystemState (ambiguous semantics).")
    if "WAIT_TF" in state_names.get("FlowState", []) and "WAITING_TF" in state_names.get("SystemState", []):
        redundancy.append("WAIT_TF (FlowState) vs WAITING_TF (SystemState) overlap.")

    defects = []
    if missing_required:
        defects.append(f"TEST missing required deps: {', '.join(missing_required)}")
    if present_forbidden:
        defects.append(f"TEST includes forbidden deps: {', '.join(present_forbidden)}")

    error_routes = []
    for line_no, _ in error_calls:
        segment = ast.get_source_segment(text, next(
            (n for n in ast.walk(tree) if isinstance(n, ast.Attribute)
             and isinstance(n.value, ast.Name) and n.value.id == "SystemState"
             and n.attr == "ERROR" and getattr(n, "lineno", None) == line_no),
            None,
        ))
        error_routes.append(
            {
                "line": str(line_no),
                "function": _find_function(funcs, line_no),
                "line_text": (segment or "SystemState.ERROR").strip(),
            }
        )

    error_classified = []
    for item in error_routes:
        line_txt = item["line_text"].lower()
        if any(k in line_txt for k in ("tf", "moveit", "camera", "pose", "clock", "controller")):
            kind = "transient_risk"
        else:
            kind = "unknown"
        error_classified.append({**item, "class": kind})

    report = {
        "states": state_names,
        "transitions": transitions,
        "redundant_or_ambiguous": redundancy,
        "test_dependencies": {
            "system_ready_for_test": ready_deps,
            "run_robot_test": test_deps,
            "refresh_controls": refresh_deps,
            "missing_required": missing_required,
            "forbidden_present": present_forbidden,
            "locations": {
                "system_ready_for_test": ready_loc,
                "run_robot_test": test_loc,
                "refresh_controls": refresh_loc,
            },
        },
        "error_routes": error_classified,
        "wait_loops": loops,
    }

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
        if not args.report:
            return 0

    lines_out: List[str] = []
    lines_out.append("=== AUDIT REPORT (STATIC) ===")
    lines_out.append("Estados detectados:")
    for key, vals in state_names.items():
        lines_out.append(f"- {key}: {', '.join(vals) if vals else 'none'}")

    if redundancy:
        lines_out.append("")
        lines_out.append("Estados redundantes/ambiguos:")
        for item in redundancy:
            lines_out.append(f"- {item}")

    lines_out.append("")
    lines_out.append("Diagrama textual (transiciones encontradas):")
    for line in _derive_state_diagram(transitions):
        lines_out.append(f"- {line}")

    lines_out.append("")
    lines_out.append("Dependencias TEST (debe ser cinemático):")
    lines_out.append(f"- system_ready_for_test deps: {', '.join(ready_deps) if ready_deps else 'none'}")
    lines_out.append(f"- run_robot_test deps: {', '.join(test_deps) if test_deps else 'none'}")
    if missing_required:
        lines_out.append(f"- DEFECTO: faltan grupos requeridos: {', '.join(missing_required)}")
    if present_forbidden:
        lines_out.append(f"- DEFECTO: deps prohibidas presentes: {', '.join(present_forbidden)}")

    lines_out.append("")
    lines_out.append("Rutas a ERROR:")
    if not error_classified:
        lines_out.append("- none")
    else:
        for item in error_classified:
            lines_out.append(f"- {item['function']} @L{item['line']}: {item['line_text']} [{item['class']}]")

    lines_out.append("")
    lines_out.append("Posibles bloqueos (while sin timeout evidente):")
    if not loops:
        lines_out.append("- none")
    else:
        for loop in loops:
            if loop["timeout_hint"] == "no":
                lines_out.append(
                    f"- {loop['function']} @L{loop['line']}: {loop['line_text']} "
                    f"(timeout={loop['timeout_hint']}, break={loop['break_hint']})"
                )

    lines_out.append("")
    lines_out.append("Recomendaciones localizadas:")
    lines_out.append("- Revisar funciones que fuerzan ERROR por condiciones recuperables.")
    lines_out.append("- Consolidar semantica READY/WAIT_TF entre FlowState y SystemState.")
    lines_out.append("- Validar deps de TEST contra criterios cinemáticos (controllers/joint_states/TF/pose_info).")

    if args.report:
        with open(args.report, "w", encoding="utf-8") as f:
            f.write("\n".join(lines_out) + "\n")

    if not args.json:
        print("\n".join(lines_out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
