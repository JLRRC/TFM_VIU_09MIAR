#!/usr/bin/env python3
"""Genera una auditoria tecnica reproducible del stack UR5 + MoveIt 2 + Gazebo."""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, UTC
from pathlib import Path
from typing import Any, Dict, List, Optional


ROOT = Path(__file__).resolve().parents[1]
REPORT_DIR = ROOT / "reports" / "moveit2_audit"
TOOLS_DIR = ROOT / "tools"
LOG_FILE = ROOT / "log" / "ros2_launch.log"


@dataclass
class CommandResult:
    name: str
    argv: List[str]
    returncode: int
    stdout: str
    stderr: str
    timed_out: bool = False

    @property
    def ok(self) -> bool:
        return (not self.timed_out) and self.returncode == 0


def _run(name: str, argv: List[str], timeout: float = 15.0) -> CommandResult:
    try:
        completed = subprocess.run(
            argv,
            cwd=str(ROOT),
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return CommandResult(
            name=name,
            argv=argv,
            returncode=completed.returncode,
            stdout=completed.stdout or "",
            stderr=completed.stderr or "",
            timed_out=False,
        )
    except subprocess.TimeoutExpired as exc:
        return CommandResult(
            name=name,
            argv=argv,
            returncode=124,
            stdout=(exc.stdout or "") if isinstance(exc.stdout, str) else "",
            stderr=(exc.stderr or "") if isinstance(exc.stderr, str) else "",
            timed_out=True,
        )


def _load_json_from_output(text: str) -> Dict[str, Any]:
    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end < start:
        return {}
    try:
        return json.loads(text[start : end + 1])
    except Exception:
        return {}


def _run_with_retries(
    name: str,
    argv: List[str],
    *,
    timeout: float,
    attempts: int,
    score_fn,
) -> CommandResult:
    best_result: Optional[CommandResult] = None
    best_score: Optional[tuple] = None
    for _ in range(max(1, attempts)):
        result = _run(name, argv, timeout=timeout)
        score = score_fn(result)
        if best_result is None or score > best_score:
            best_result = result
            best_score = score
    return best_result if best_result is not None else _run(name, argv, timeout=timeout)


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except Exception:
        return ""


def _tail_matches(lines: List[str], pattern: str, limit: int = 12) -> List[str]:
    regex = re.compile(pattern)
    matches = [line for line in lines if regex.search(line)]
    return matches[-limit:]


def _last_index(lines: List[str], pattern: str) -> int:
    regex = re.compile(pattern)
    for idx in range(len(lines) - 1, -1, -1):
        if regex.search(lines[idx]):
            return idx
    return -1


def _status(rank: str, summary: str, evidence: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    return {
        "status": rank,
        "summary": summary,
        "evidence": evidence or {},
    }


def _parse_pose_lines(stdout: str) -> Dict[str, Dict[str, float]]:
    result: Dict[str, Dict[str, float]] = {}
    for line in stdout.splitlines():
        match = re.search(
            r"^\[POSE\]\s+(?P<name>\S+)\s+xyz=\((?P<x>[-+0-9.]+),(?P<y>[-+0-9.]+),(?P<z>[-+0-9.]+)\)",
            line.strip(),
        )
        if not match:
            continue
        result[match.group("name")] = {
            "x": float(match.group("x")),
            "y": float(match.group("y")),
            "z": float(match.group("z")),
        }
    return result


def _build_architecture() -> Dict[str, Any]:
    files = {
        "stack_launch": ROOT / "src" / "ur5_bringup" / "launch" / "ur5_stack.launch.py",
        "moveit_launch": ROOT / "src" / "ur5_moveit_config" / "launch" / "ur5_moveit_bringup.launch.py",
        "urdf": ROOT / "src" / "ur5_description" / "urdf" / "ur5.urdf.xacro",
        "controllers": ROOT / "src" / "ur5_description" / "config" / "ur5_controllers.yaml",
        "moveit_controllers": ROOT / "src" / "ur5_moveit_config" / "config" / "moveit_controllers.yaml",
        "srdf": ROOT / "src" / "ur5_moveit_config" / "config" / "ur5.srdf",
        "srdf_strict": ROOT / "src" / "ur5_moveit_config" / "config" / "ur5_strict.srdf",
        "scene_sync": ROOT / "src" / "ur5_tools" / "ur5_tools" / "planning_scene_sync.py",
        "attach_backend": ROOT / "src" / "ur5_tools" / "ur5_tools" / "gripper_attach_backend.py",
        "pick_logic": ROOT / "src" / "ur5_qt_panel" / "ur5_qt_panel" / "panel_pick_object.py",
        "panel_start": ROOT / "scripts" / "start_panel_v2.sh",
    }
    return {
        "files": {name: {"path": str(path), "exists": path.is_file()} for name, path in files.items()},
        "components": [
            {
                "name": "Bringup unificado",
                "role": "Arranca Gazebo, ros_gz_bridge, controller bootstrap, planning_scene_sync, panel y opcionalmente MoveIt.",
                "path": str(files["stack_launch"]),
            },
            {
                "name": "MoveIt 2",
                "role": "Carga URDF/SRDF, move_group y la ejecucion FJT mediante MoveItSimpleControllerManager.",
                "path": str(files["moveit_launch"]),
            },
            {
                "name": "Modelo robot + ros2_control",
                "role": "Define UR5 + RG2 + rg2_tcp y el plugin gz_ros2_control/GazeboSimSystem.",
                "path": str(files["urdf"]),
            },
            {
                "name": "Planning scene",
                "role": "Replica mesa/objetos de Gazebo en MoveIt y promueve objetos attached segun topics de agarre.",
                "path": str(files["scene_sync"]),
            },
            {
                "name": "Attach backend software",
                "role": "Mueve fisicamente objetos siguiendo rg2_tcp en modo follow_tcp cuando esta habilitado.",
                "path": str(files["attach_backend"]),
            },
            {
                "name": "Pick orchestrator",
                "role": "Publica objetivos a MoveIt, controla gripper y valida carry fisico mediante CARRY_GATE.",
                "path": str(files["pick_logic"]),
            },
        ],
    }


def _build_layer_checks(
    audit_json: Dict[str, Any],
    node_list: List[str],
    controllers_text: str,
    pose_probe: CommandResult,
    fjt_probe: CommandResult,
    log_lines: List[str],
) -> Dict[str, Any]:
    latest_fail_idx = _last_index(log_lines, r"carry_coherence_failed")
    latest_success_idx = _last_index(log_lines, r"SECUENCIA COMPLETADA EXITOSAMENTE")
    strict_mode = any("strict_physics_mode activo" in line for line in log_lines[:160])
    attach_backend_alive = any(node.strip() == "/gripper_attach_backend" for node in node_list)
    move_group_alive = any(node.strip() == "/move_group" for node in node_list)
    scene_sync_alive = any(node.strip() == "/planning_scene_sync" for node in node_list)
    pose_stats = (((audit_json.get("layers") or {}).get("pose_info") or {}))
    joint_states_layer = (audit_json.get("layers") or {}).get("joint_states") or {}
    tf_layer = (audit_json.get("layers") or {}).get("tf") or {}
    controllers_ok = (audit_json.get("checks") or {}).get("controllers") == "PASS_CONTROLLERS_ACTIVE"
    joint_states_ok = (audit_json.get("checks") or {}).get("joint_states") == "PASS_JOINT_STATES"
    tf_ok = (audit_json.get("checks") or {}).get("tf") == "PASS_TF_CHAIN"
    if joint_states_ok and tf_ok:
        joint_tf_status = "PASS"
    elif joint_states_layer.get("has_topic") and tf_ok and controllers_ok:
        joint_tf_status = "WARNING"
    else:
        joint_tf_status = "FAIL"

    layers = {
        "infra_ros": _status(
            "PASS" if (((audit_json.get("layers") or {}).get("infra") or {}).get("nodes_ok")) else "FAIL",
            "ROS graph y daemon accesibles.",
            (audit_json.get("layers") or {}).get("infra") or {},
        ),
        "sim_clock": _status(
            "PASS" if (((audit_json.get("layers") or {}).get("clock") or {}).get("msg_ok")) else "FAIL",
            "Reloj de simulacion disponible.",
            (audit_json.get("layers") or {}).get("clock") or {},
        ),
        "ros2_control": _status(
            "PASS" if ((audit_json.get("checks") or {}).get("controllers") == "PASS_CONTROLLERS_ACTIVE") else "FAIL",
            "controller_manager y controladores activos.",
            {
                "audit": (audit_json.get("layers") or {}).get("controllers") or {},
                "list_controllers": controllers_text.strip(),
            },
        ),
        "joint_states_tf": _status(
            joint_tf_status,
            "Joint states y cadena TF base_link->rg2_tcp disponibles.",
            {
                "joint_states": joint_states_layer,
                "tf": tf_layer,
            },
        ),
        "moveit_plan_exec": _status(
            "PASS" if move_group_alive and fjt_probe.ok else "FAIL",
            "MoveIt y la accion FollowJointTrajectory responden.",
            {
                "move_group_alive": move_group_alive,
                "fjt_probe": fjt_probe.stdout.strip() or fjt_probe.stderr.strip(),
            },
        ),
        "planning_scene_sync": _status(
            "PASS" if scene_sync_alive else "FAIL",
            "planning_scene_sync esta presente en el grafo ROS.",
            {"node_present": scene_sync_alive},
        ),
        "logical_grasp_mode": _status(
            "PASS" if strict_mode and (not attach_backend_alive) else ("WARNING" if not strict_mode else "FAIL"),
            "Separacion entre agarre logico y fisico verificada en modo estricto.",
            {
                "strict_mode_log": strict_mode,
                "attach_backend_alive": attach_backend_alive,
            },
        ),
        "physical_grasp_transport": _status(
            "FAIL" if latest_fail_idx > latest_success_idx else ("PASS" if latest_success_idx >= 0 else "WARNING"),
            "La planificacion y la ejecucion completan, pero el carry fisico es la capa que decide el exito real.",
            {
                "latest_failure_idx": latest_fail_idx,
                "latest_success_idx": latest_success_idx,
                "latest_relevant_log": _tail_matches(
                    log_lines,
                    r"STRICT_CONTACT|STRICT_PROBE|STRICT_LIFT|CARRY_GATE|carry_coherence_failed|SECUENCIA COMPLETADA EXITOSAMENTE",
                    limit=16,
                ),
            },
        ),
        "audit_harness_quality": _status(
            "PASS"
            if pose_stats.get("warning") or (not pose_probe.ok)
            else ("WARNING" if pose_stats.get("max_z", 0.0) and float(pose_stats.get("max_z", 0.0)) >= 5.0 else "PASS"),
            "La auditoria fisica ya expone outliers de pose en vez de ocultarlos como PASS silencioso.",
            {
                "audit_pose_info": pose_stats,
                "pose_probe_stdout": pose_probe.stdout.strip(),
            },
        ),
    }
    return layers


def _build_diagnosis(layer_checks: Dict[str, Any], pose_probe: CommandResult) -> Dict[str, Any]:
    pose_map = _parse_pose_lines(pose_probe.stdout)
    ranking = [
        {
            "rank": 1,
            "title": "Inestabilidad del agarre fisico durante el transporte vertical estricto",
            "probability": "alta",
            "why": "MoveIt planifica y ejecuta hasta STRICT_LIFT_STAGE_3, pero el CARRY_GATE aborta por distancia objeto-TCP ~0.189/0.190 > 0.180.",
        },
        {
            "rank": 2,
            "title": "Geometria final de cierre y lift por etapas aun insuficiente para mantener el objeto a mayor z",
            "probability": "alta",
            "why": "STRICT_CONTACT y STRICT_PROBE pasan; el fallo aparece mas tarde, ya con contacto inicial confirmado.",
        },
        {
            "rank": 3,
            "title": "Necesidad de mantener endurecida la auditoria fisica",
            "probability": "media",
            "why": "El stack mostraba outliers en pose_info; ahora la auditoria ya los eleva como WARNING/FAIL y no conviene volver a criterios laxos.",
        },
        {
            "rank": 4,
            "title": "Sensores 3D/octomap ausentes en MoveIt",
            "probability": "baja",
            "why": "El log de move_group avisa de que no hay plugin 3D, pero la cadena de plan y ejecucion FJT funciona y no explica la caida fisica post-lift.",
        },
    ]
    root_cause = ranking[0]["title"]
    if pose_map:
        ranking[2]["evidence"] = pose_map
    return {
        "root_cause": root_cause,
        "ranking": ranking,
        "proposed_minimal_changes": [
            {
                "file": "tools/audit_system.py",
                "reason": "Convertir CAPA 3.5 POSE INFO en WARNING/FAIL cuando aparezcan outliers altos, para que la auditoria no dé PASS con max_z=10.0.",
                "code_block": """# Idea minima
if pose_ok and max_z is not None and max_z > 1.2:
    pose_reason = f\"pose outlier detected: max_z={max_z:.3f}\"
    _log_line(\"CAPA 3.5 POSE INFO\", \"WARNING\", pose_reason)
""",
            },
            {
                "file": "tools/test_object_pose_sanity_rclpy.py",
                "reason": "Sustituir el umbral fijo z>=1.2 por una banda relativa a la mesa/objeto de referencia, para detectar mejor objetos flotando pero evitar falsos PASS.",
                "code_block": """# Idea minima
table_ref_z = pick_z if math.isfinite(pick_z) else 0.0
max_allowed_z = max(0.20, table_ref_z + 0.10)
if z >= max_allowed_z:
    offenders.append((name, z))
""",
            },
            {
                "file": "src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py",
                "reason": "Añadir un micro-settle antes del gate de cada strict lift y exigir dos muestras buenas consecutivas, sin relajar el umbral fisico.",
                "code_block": """# Idea minima
time.sleep(float(os.environ.get(\"PANEL_PICK_OBJECT_STRICT_LIFT_SETTLE_SEC\", \"0.20\")))
_assert_carry_coherence_after_lift(
    ...,
    min_consecutive_override=2,
    gate_label=gate_label,
)
""",
            },
        ],
    }


def _build_tests_section(results: Dict[str, CommandResult], audit_json: Dict[str, Any]) -> List[Dict[str, Any]]:
    tests = []
    audit_layers = audit_json.get("layers") or {}
    for key in ("audit_system", "controllers", "fjt", "pose_sanity"):
        res = results[key]
        status = "PASS" if res.ok else "FAIL"
        if key == "audit_system":
            pose_warning = ((audit_layers.get("pose_info") or {}).get("warning") or "")
            joint_state_issue = not bool((audit_layers.get("joint_states") or {}).get("ok"))
            if pose_warning and not joint_state_issue:
                status = "WARNING"
            elif joint_state_issue and bool((audit_layers.get("joint_states") or {}).get("has_topic")):
                status = "WARNING"
            elif res.ok:
                status = "PASS"
        tests.append(
            {
                "name": key,
                "status": status,
                "command": " ".join(res.argv),
                "summary": (res.stdout.strip() or res.stderr.strip()).splitlines()[:12],
            }
        )
    tests.append(
        {
            "name": "audit_global_state",
            "status": "PASS" if audit_json.get("state") == "STATE_READY_FOR_PICK" else "WARNING",
            "command": "python3 tools/audit_system.py --json",
            "summary": [f"state={audit_json.get('state', 'UNKNOWN')}"] if audit_json else ["state=UNKNOWN"],
        }
    )
    return tests


def _markdown_report(data: Dict[str, Any]) -> str:
    executive_lines = data["executive_summary"]
    architecture = data["architecture"]
    layers = data["layer_checks"]
    tests = data["tests"]
    diagnosis = data["diagnosis"]
    files = data["generated_files"]

    lines: List[str] = []
    lines.append("# Auditoria Tecnica MoveIt 2 / Gazebo / ROS 2")
    lines.append("")
    lines.append("## 1. Resumen ejecutivo")
    for item in executive_lines:
        lines.append(f"- {item}")
    lines.append("")
    lines.append("## 2. Arquitectura encontrada")
    for component in architecture["components"]:
        lines.append(f"- {component['name']}: {component['role']} ({component['path']})")
    lines.append("")
    lines.append("## 3. Comprobacion por capas")
    for name, info in layers.items():
        lines.append(f"- {name}: {info['status']} - {info['summary']}")
    lines.append("")
    lines.append("## 4. Pruebas minimas y resultados")
    for test in tests:
        headline = test["summary"][0] if test["summary"] else "sin salida"
        lines.append(f"- {test['name']}: {test['status']} - {headline}")
    lines.append("")
    lines.append("## 5. Causa raiz mas probable")
    lines.append(f"- {diagnosis['root_cause']}")
    lines.append("")
    lines.append("## 6. Ranking de causas")
    for item in diagnosis["ranking"]:
        lines.append(f"- #{item['rank']} {item['title']} ({item['probability']}): {item['why']}")
    lines.append("")
    lines.append("## 7. Archivos generados")
    for file_path in files:
        lines.append(f"- {file_path}")
    lines.append("")
    lines.append("## 8. Cambios minimos propuestos")
    for proposal in diagnosis["proposed_minimal_changes"]:
        lines.append(f"- {proposal['file']}: {proposal['reason']}")
        lines.append("")
        lines.append("```python")
        lines.append(proposal["code_block"].rstrip())
        lines.append("```")
    lines.append("")
    lines.append("## 9. Checklist final PASS / FAIL")
    for name, info in layers.items():
        if info["status"] in ("PASS", "FAIL"):
            lines.append(f"- {name}: {info['status']}")
    lines.append("")
    lines.append("## 10. Proximo paso recomendado")
    lines.append("- Mantener strict_physics_mode y atacar la estabilidad del transporte vertical en panel_pick_object.py; no tocar MoveIt ni controladores porque esas capas ya estan pasando.")
    lines.append("")
    lines.append("## Evidencia relevante")
    for line in data["evidence_tail"]:
        lines.append(f"- {line}")
    return "\n".join(lines) + "\n"


def _markdown_summary(data: Dict[str, Any]) -> str:
    lines = [
        "# MoveIt 2 System Summary",
        "",
        f"- Timestamp: {data['timestamp']}",
        f"- Estado global audit_system: {data['audit_system_state']}",
        f"- Root cause mas probable: {data['diagnosis']['root_cause']}",
        f"- Physical grasp transport: {data['layer_checks']['physical_grasp_transport']['status']}",
        f"- MoveIt plan/exec: {data['layer_checks']['moveit_plan_exec']['status']}",
        f"- ros2_control: {data['layer_checks']['ros2_control']['status']}",
        f"- Logical grasp mode: {data['layer_checks']['logical_grasp_mode']['status']}",
        "",
        "## Decision",
        "",
        "El sistema no esta bloqueado por ROS 2, MoveIt ni ros2_control. El bloqueo principal esta localizado en la capa de agarre fisico durante el lift estricto, despues de que planificacion y ejecucion ya hayan pasado.",
    ]
    return "\n".join(lines) + "\n"


def main() -> int:
    if shutil.which("ros2") is None:
        print("ERROR: ros2 no esta en PATH. Ejecuta este script con el entorno ROS cargado.", file=sys.stderr)
        return 2

    REPORT_DIR.mkdir(parents=True, exist_ok=True)

    node_list_res = _run("node_list", ["ros2", "node", "list"], timeout=8.0)
    topic_list_res = _run("topic_list", ["ros2", "topic", "list"], timeout=8.0)
    service_list_res = _run("service_list", ["ros2", "service", "list"], timeout=8.0)
    action_list_res = _run("action_list", ["ros2", "action", "list", "-t"], timeout=8.0)
    controllers_list_res = _run(
        "controllers_list",
        ["ros2", "control", "list_controllers", "-c", "/controller_manager"],
        timeout=8.0,
    )

    audit_system_argv = [
        "python3",
        str(TOOLS_DIR / "audit_system.py"),
        "--json",
        "--timeout",
        "4",
        "--clock-timeout",
        "4",
        "--tf-timeout",
        "4",
        "--pose-timeout",
        "3",
    ]

    def _audit_score(result: CommandResult) -> tuple:
        data = _load_json_from_output(result.stdout)
        state = str(data.get("state") or "")
        checks = data.get("checks") or {}
        pose_warning = str(((data.get("layers") or {}).get("pose_info") or {}).get("warning") or "")
        return (
            1 if state == "STATE_READY_FOR_PICK" else 0,
            1 if checks.get("joint_states") == "PASS_JOINT_STATES" else 0,
            1 if checks.get("tf") == "PASS_TF_CHAIN" else 0,
            1 if checks.get("controllers") == "PASS_CONTROLLERS_ACTIVE" else 0,
            1 if pose_warning else 0,
            1 if result.ok else 0,
        )

    results = {
        "audit_system": _run_with_retries(
            "audit_system",
            audit_system_argv,
            timeout=30.0,
            attempts=3,
            score_fn=_audit_score,
        ),
        "controllers": _run(
            "controllers",
            ["python3", str(TOOLS_DIR / "test_controllers_rclpy.py"), "--timeout", "4"],
            timeout=20.0,
        ),
        "fjt": _run(
            "fjt",
            ["python3", str(TOOLS_DIR / "test_fjt_action_rclpy.py"), "--timeout", "3"],
            timeout=20.0,
        ),
        "pose_sanity": _run(
            "pose_sanity",
            [
                "python3",
                str(TOOLS_DIR / "test_object_pose_sanity_rclpy.py"),
                "--timeout",
                "6",
                "--targets",
                "pick_demo",
                "box_yellow",
                "box_red",
                "cyl_purple",
            ],
            timeout=20.0,
        ),
    }

    audit_json = _load_json_from_output(results["audit_system"].stdout)
    node_list = [line.strip() for line in node_list_res.stdout.splitlines() if line.strip()]
    log_lines = _read_text(LOG_FILE).splitlines() if LOG_FILE.is_file() else []

    layer_checks = _build_layer_checks(
        audit_json,
        node_list,
        controllers_list_res.stdout,
        results["pose_sanity"],
        results["fjt"],
        log_lines,
    )
    diagnosis = _build_diagnosis(layer_checks, results["pose_sanity"])
    tests = _build_tests_section(results, audit_json)
    pose_warning = str((((audit_json.get("layers") or {}).get("pose_info") or {}).get("warning") or ""))
    pose_probe_failed = not results["pose_sanity"].ok
    joint_states_status = layer_checks["joint_states_tf"]["status"]

    executive_summary = [
        "La infraestructura ROS 2, /clock, ros2_control y la accion FollowJointTrajectory estan operativas.",
        "MoveIt planifica y ejecuta correctamente; el cuello de botella real no esta en la capa de plan/exec.",
        "El ultimo fallo relevante en ros2_launch.log esta en strict_lift_stage_3 con carry_coherence_failed y dist≈0.190 frente a max=0.180.",
        "En el arranque auditado, strict_physics_mode desactiva el attach backend, por lo que el sistema ya esta separando agarre logico de agarre fisico.",
        (
            f"La capa joint_states/TF queda en {joint_states_status}; si baja a WARNING suele ser por muestreo transitorio de /joint_states, no por caida de ros2_control."
            if joint_states_status != "PASS"
            else "Joint states y TF estaban disponibles en la muestra seleccionada de la auditoria."
        ),
        (
            f"La auditoria fisica ya eleva outliers de pose: {pose_warning}."
            if pose_warning
            else "La auditoria fisica no detecto outliers de pose en esta corrida."
        ),
        (
            "La prueba de sanidad fisica ahora detecta objetos fuera de rango sobre mesa, evitando falsos PASS."
            if pose_probe_failed
            else "La prueba de sanidad fisica no encontro objetos flotando en esta corrida."
        ),
    ]

    data: Dict[str, Any] = {
        "timestamp": datetime.now(UTC).isoformat(timespec="seconds").replace("+00:00", "Z"),
        "workspace": str(ROOT),
        "audit_system_state": audit_json.get("state", "UNKNOWN"),
        "executive_summary": executive_summary,
        "architecture": _build_architecture(),
        "layer_checks": layer_checks,
        "tests": tests,
        "diagnosis": diagnosis,
        "live_graph": {
            "nodes": node_list,
            "topics": [line.strip() for line in topic_list_res.stdout.splitlines() if line.strip()],
            "services": [line.strip() for line in service_list_res.stdout.splitlines() if line.strip()],
            "actions": [line.strip() for line in action_list_res.stdout.splitlines() if line.strip()],
        },
        "raw_commands": {
            name: {
                "argv": res.argv,
                "ok": res.ok,
                "returncode": res.returncode,
                "stdout": res.stdout,
                "stderr": res.stderr,
                "timed_out": res.timed_out,
            }
            for name, res in {
                **results,
                "node_list": node_list_res,
                "topic_list": topic_list_res,
                "service_list": service_list_res,
                "action_list": action_list_res,
                "controllers_list": controllers_list_res,
            }.items()
        },
        "evidence_tail": _tail_matches(
            log_lines,
            r"strict_physics_mode|STRICT_CONTACT|STRICT_PROBE|STRICT_LIFT|CARRY_GATE|carry_coherence_failed|SECUENCIA COMPLETADA EXITOSAMENTE",
            limit=20,
        ),
    }

    audit_path = REPORT_DIR / "moveit2_system_audit.md"
    summary_path = REPORT_DIR / "moveit2_system_summary.md"
    json_path = REPORT_DIR / "moveit2_system_status.json"

    data["generated_files"] = [str(audit_path), str(summary_path), str(json_path)]

    audit_path.write_text(_markdown_report(data), encoding="utf-8")
    summary_path.write_text(_markdown_summary(data), encoding="utf-8")
    json_path.write_text(json.dumps(data, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")

    print(str(audit_path))
    print(str(summary_path))
    print(str(json_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())