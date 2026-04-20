#!/usr/bin/env python3
from __future__ import annotations

import difflib
import os
import re
import subprocess
import sys
import textwrap
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class EnvEntry:
    name: str
    default: str
    source_label: str
    source_file: Path


@dataclass(frozen=True)
class ResolvedEnv:
    name: str
    group: str
    launch_default: str
    runtime_override: str
    panel_default: str
    historical_default: str


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def write_text(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")


def rel(root: Path, path: Path | None) -> str:
    if path is None:
        return "no disponible"
    try:
        return str(path.relative_to(root))
    except Exception:
        return str(path)


def capture(pattern: str, text: str, label: str, *, flags: int = re.S) -> str:
    match = re.search(pattern, text, flags)
    if not match:
        raise SystemExit(f"No se pudo extraer {label}")
    return match.group(1)


def capture_groups(pattern: str, text: str, label: str, *, flags: int = re.S) -> tuple[str, ...]:
    match = re.search(pattern, text, flags)
    if not match:
        raise SystemExit(f"No se pudo extraer {label}")
    return match.groups()


def first_matching_line(paths: list[Path], predicate) -> tuple[Path | None, str | None]:
    for path in paths:
        if not path.exists() or not path.is_file():
            continue
        try:
            with path.open(encoding="utf-8", errors="replace") as handle:
                for raw_line in handle:
                    line = raw_line.rstrip("\n")
                    if predicate(line):
                        return path, line.strip()
        except Exception:
            continue
    return None, None


def first_two_world_locked_samples(paths: list[Path]) -> tuple[Path | None, float | None]:
    for path in paths:
        if not path.exists() or not path.is_file():
            continue
        stamps: list[float] = []
        try:
            with path.open(encoding="utf-8", errors="replace") as handle:
                for raw_line in handle:
                    line = raw_line.rstrip("\n")
                    if "demo_transport_follow_tick object=pick_demo mode=world_locked" not in line:
                        continue
                    match = re.search(r"\[(\d+\.\d+)\]", line)
                    if match:
                        stamps.append(float(match.group(1)))
                    if len(stamps) >= 2:
                        return path, stamps[1] - stamps[0]
        except Exception:
            continue
    return None, None


def load_markdown_or_pdf(markdown_path: Path, pdf_path: Path) -> str:
    if markdown_path.exists():
        return read_text(markdown_path)
    if pdf_path.exists() and shutil_which("pdftotext"):
        try:
            result = subprocess.run(
                ["pdftotext", "-layout", str(pdf_path), "-"],
                check=True,
                capture_output=True,
                text=True,
            )
            return result.stdout
        except Exception:
            return ""
    return ""


def git_show_file(root: Path, path: Path) -> str:
    try:
        rel_path = str(path.relative_to(root))
    except Exception:
        return ""
    try:
        result = subprocess.run(
            ["git", "-C", str(root), "show", f"HEAD:{rel_path}"],
            check=True,
            capture_output=True,
            text=True,
        )
        return result.stdout
    except Exception:
        return ""


def shutil_which(cmd: str) -> str | None:
    for entry in os.environ.get("PATH", "").split(os.pathsep):
        candidate = Path(entry) / cmd
        if candidate.exists() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


def split_sections(text: str, level: int) -> list[tuple[str, str]]:
    heading = "#" * level
    pattern = re.compile(rf"(?m)^({re.escape(heading)})\s+(.*)$")
    matches = list(pattern.finditer(text))
    sections: list[tuple[str, str]] = []
    for index, match in enumerate(matches):
        start = match.start()
        end = len(text)
        for next_match in matches[index + 1 :]:
            if next_match.group(1) == heading:
                end = next_match.start()
                break
        title = match.group(2).strip()
        sections.append((title, text[start:end].strip()))
    return sections


def find_section_by_prefix(text: str, prefix: str, *, level: int = 2) -> str:
    for title, block in split_sections(text, level):
        if title.startswith(prefix):
            return block
    return ""


def strip_first_heading(block: str) -> str:
    lines = block.splitlines()
    if not lines:
        return ""
    return "\n".join(lines[1:]).lstrip()


def shift_headings(text: str, delta: int = 1) -> str:
    if not text.strip():
        return ""
    shifted_lines: list[str] = []
    in_fence = False
    for line in text.splitlines():
        if line.strip().startswith("```"):
            in_fence = not in_fence
            shifted_lines.append(line)
            continue
        if in_fence:
            shifted_lines.append(line)
            continue
        match = re.match(r"^(#{1,6})(\s+.*)$", line)
        if match:
            level = min(6, len(match.group(1)) + delta)
            shifted_lines.append("#" * level + match.group(2))
        else:
            shifted_lines.append(line)
    return "\n".join(shifted_lines).strip()


def preserved_block(label: str, block: str, *, shift: int = 1) -> str:
    if not block.strip():
        return f"### {label}\n\n- No disponible en las fuentes cargadas.\n"
    body = shift_headings(strip_first_heading(block), shift)
    return f"### {label}\n\n{body}\n"


def parse_package_info(path: Path) -> tuple[str, str]:
    root = ET.fromstring(read_text(path))
    name = (root.findtext("name") or path.parent.name).strip()
    description = (root.findtext("description") or "").strip()
    return name, description


def parse_console_scripts(path: Path) -> list[tuple[str, str]]:
    text = read_text(path)
    match = re.search(r"console_scripts[\"']\s*:\s*\[(.*?)\]", text, re.S)
    if not match:
        return []
    body = match.group(1)
    entries: list[tuple[str, str]] = []
    for raw in re.findall(r"[\"']([^\"']+)[\"']", body):
        if "=" not in raw:
            continue
        name, target = raw.split("=", 1)
        entries.append((name.strip(), target.strip()))
    return entries


def collect_env_entries(text: str, source_label: str, source_file: Path) -> list[EnvEntry]:
    entries: list[EnvEntry] = []
    for match in re.finditer(
        r'os\.environ\.get\(\s*["\']([A-Z][A-Z0-9_]+)["\']\s*,\s*["\']([^"\']*)["\']\s*\)',
        text,
    ):
        entries.append(EnvEntry(match.group(1), match.group(2).strip(), source_label, source_file))
    for match in re.finditer(
        r'^export\s+([A-Z][A-Z0-9_]+)=\"\$\{\1:-([^}]*)}\"',
        text,
        re.M,
    ):
        entries.append(EnvEntry(match.group(1), match.group(2).strip(), source_label, source_file))
    return entries


def dedupe_env_entries(entries: list[EnvEntry]) -> list[EnvEntry]:
    unique: list[EnvEntry] = []
    seen: set[tuple[str, str, str, str]] = set()
    for entry in entries:
        key = (entry.name, entry.default.strip(), entry.source_label, str(entry.source_file))
        if key in seen:
            continue
        seen.add(key)
        unique.append(EnvEntry(entry.name, entry.default.strip(), entry.source_label, entry.source_file))
    return unique


def parse_historical_env_defaults(text: str) -> dict[str, str]:
    defaults: dict[str, str] = {}
    for line in text.splitlines():
        match = re.match(r"^\|\s*`?([A-Z][A-Z0-9_]+)`?\s*\|\s*([^|]+?)\s*\|", line)
        if match:
            defaults.setdefault(match.group(1), match.group(2).strip())
    return defaults


def env_group(name: str) -> str:
    if name.startswith("GRASP_CONTACT") or "OBJECT_HEIGHT" in name or "TCP_OFFSET" in name:
        return "geometría y altura de agarre"
    if "APPROACH_COARSE" in name:
        return "APPROACH_COARSE"
    if "GRASP_DOWN" in name:
        return "GRASP_DOWN"
    if "ALIGN" in name:
        return "GRASP_ALIGN_IK"
    if "PRE_CLOSE" in name:
        return "PRE_CLOSE"
    if "CLOSE" in name and "PRE_CLOSE" not in name:
        return "CLOSE"
    if name.startswith("PANEL_PICK_DEMO_ATTACH"):
        return "ATTACH_GATE"
    if name.startswith("ATTACH_BACKEND"):
        return "backend attach/transporte"
    if "POSE_SOURCE" in name or "STALE" in name or "SNAPSHOT" in name:
        return "freshness / pose source"
    if "DIRECT_IK_RUNTIME_SETTLE" in name or "DIRECT_IK_" in name:
        return "settle IK directo"
    if name.startswith("PANEL_MOVEIT_BRIDGE"):
        return "MoveIt bridge"
    if "LIFT" in name or "CARRY" in name or "TRANSPORT" in name:
        return "LIFT / CARRY"
    return "otros"


def env_units(name: str, value: str) -> str:
    upper = name.upper()
    if upper.endswith("_M"):
        return "m"
    if upper.endswith("_SEC") or "TIMEOUT" in upper:
        return "s"
    if upper.endswith("_MS"):
        return "ms"
    if upper.endswith("_HZ"):
        return "Hz"
    if upper.endswith("_RAD"):
        return "rad"
    if upper.endswith("_SAMPLES") or "RETRIES" in upper or "ATTEMPTS" in upper:
        return "conteo"
    if value.lower() in {"0", "1", "true", "false"}:
        return "flag"
    return "según variable"


def env_effect(group: str) -> str:
    mapping = {
        "geometría y altura de agarre": "Ajusta alturas, offsets y contacto geométrico del pick.",
        "APPROACH_COARSE": "Controla la aproximación gruesa antes del descenso final.",
        "GRASP_DOWN": "Controla el descenso, segmentación IK y/o cartesian path del grasp.",
        "GRASP_ALIGN_IK": "Afina alineación XY/Z y compensación del residual geométrico.",
        "PRE_CLOSE": "Valida pose y permite realineación justo antes de cerrar.",
        "CLOSE": "Controla cierre del gripper y su confirmación por telemetría.",
        "ATTACH_GATE": "Controla la ventana temporal y los umbrales del attach lógico.",
        "LIFT / CARRY": "Controla validación física post-grasp y transporte con objeto.",
        "freshness / pose source": "Controla frescura, fuente y tolerancias de poses/TF.",
        "settle IK directo": "Controla settle y tolerancias del camino directo por IK.",
        "MoveIt bridge": "Controla timeouts, tolerancias y escalado del puente a MoveIt.",
        "backend attach/transporte": "Controla el backend de attach, follow y demo transport.",
    }
    return mapping.get(group, "Ajusta comportamiento runtime del pipeline.")


def env_risk(group: str) -> str:
    mapping = {
        "geometría y altura de agarre": "Un offset incorrecto desplaza el TCP y degrada el grasp.",
        "APPROACH_COARSE": "Tolerancias demasiado agresivas disparan abortos o saltos de fase.",
        "GRASP_DOWN": "Steps o tolerancias mal ajustados favorecen cambio de rama IK o colisión.",
        "GRASP_ALIGN_IK": "Puede dejar residual Z sin corregir o introducir sobrecorrección.",
        "PRE_CLOSE": "Permite cerrar fuera de objeto o repetir realineaciones inútiles.",
        "CLOSE": "Puede dejar CLOSE en PEND o confirmar falsamente un cierre débil.",
        "ATTACH_GATE": "Puede aprobar attach lógico sin contacto físico real, o bloquear attaches válidos.",
        "LIFT / CARRY": "Puede aceptar falsos positivos o abortar carries físicamente correctos.",
        "freshness / pose source": "Pose stale o mezcla de fuentes produce decisiones incoherentes.",
        "settle IK directo": "Un settle corto deja FK/TF desalineados respecto a la telemetría real.",
        "MoveIt bridge": "Time-outs o tolerancias erróneas degradan planificación y ejecución.",
        "backend attach/transporte": "Puede romper el seguimiento, producir stale soft follow o world_locked incoherente.",
    }
    return mapping.get(group, "Riesgo de divergencia entre intención, ejecución y telemetría.")


def env_inventory_markdown(entries: list[EnvEntry], historical_defaults: dict[str, str]) -> str:
    resolved = resolve_env_entries(entries, historical_defaults)
    grouped: dict[str, list[ResolvedEnv]] = {}
    for entry in resolved:
        grouped.setdefault(entry.group, []).append(entry)

    blocks: list[str] = []
    for group in [
        "geometría y altura de agarre",
        "APPROACH_COARSE",
        "GRASP_DOWN",
        "GRASP_ALIGN_IK",
        "PRE_CLOSE",
        "CLOSE",
        "ATTACH_GATE",
        "LIFT / CARRY",
        "freshness / pose source",
        "settle IK directo",
        "MoveIt bridge",
        "backend attach/transporte",
        "otros",
    ]:
        rows = sorted(grouped.get(group, []), key=lambda item: item.name)
        if not rows:
            continue
        lines = [
            f"#### {group}",
            "",
            "| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado | Discrepancia |",
            "|---|---|---|---|---|---|---|",
        ]
        for item in rows:
            state, discrepancy = env_state_and_discrepancy(item)
            lines.append(
                "| {name} | {launch} | {runtime} | {panel} | {historical} | {state} | {disc} |".format(
                    name=item.name,
                    launch=item.launch_default or "-",
                    runtime=item.runtime_override or "-",
                    panel=item.panel_default or "-",
                    historical=item.historical_default or "-",
                    state=state,
                    disc=discrepancy,
                )
            )
        blocks.append("\n".join(lines))
    return "\n\n".join(blocks)


def resolve_env_entries(entries: list[EnvEntry], historical_defaults: dict[str, str]) -> list[ResolvedEnv]:
    filtered = [
        entry
        for entry in dedupe_env_entries(entries)
        if entry.name.startswith(("PANEL_PICK_DEMO_", "ATTACH_BACKEND_", "PANEL_MOVEIT_BRIDGE_", "GRASP_CONTACT_"))
    ]
    names = sorted({entry.name for entry in filtered})
    resolved: list[ResolvedEnv] = []
    for name in names:
        launch_default = ""
        runtime_override = ""
        panel_default = ""
        for entry in filtered:
            if entry.name != name:
                continue
            if entry.source_label == "launch" and entry.default:
                launch_default = entry.default
            elif entry.source_label == "start_panel" and entry.default:
                runtime_override = entry.default
            elif entry.source_label == "panel" and entry.default:
                panel_default = entry.default
        resolved.append(
            ResolvedEnv(
                name=name,
                group=env_group(name),
                launch_default=launch_default,
                runtime_override=runtime_override,
                panel_default=panel_default,
                historical_default=historical_defaults.get(name, ""),
            )
        )
    return resolved


def env_state_and_discrepancy(entry: ResolvedEnv) -> tuple[str, str]:
    current_values = [value for value in [entry.launch_default, entry.runtime_override, entry.panel_default] if value]
    distinct_values = list(dict.fromkeys(current_values))
    discrepancies: list[str] = []
    if len(distinct_values) > 1:
        discrepancies.append("actual difiere entre fuentes")
    if entry.historical_default and entry.historical_default not in distinct_values:
        discrepancies.append(f"histórico={entry.historical_default}")
    if entry.runtime_override and entry.launch_default and entry.runtime_override != entry.launch_default:
        discrepancies.append("wrapper pisa launch")
    if entry.panel_default and entry.launch_default and entry.panel_default != entry.launch_default:
        discrepancies.append("panel usa fallback distinto")
    state = "alineado"
    if discrepancies:
        state = "discrepancia abierta"
    elif entry.runtime_override:
        state = "override runtime activo"
    elif entry.launch_default:
        state = "default launch"
    elif entry.panel_default:
        state = "fallback local panel"
    return state, "; ".join(dict.fromkeys(discrepancies)) if discrepancies else "sin discrepancia relevante"


def env_source_markdown(resolved: list[ResolvedEnv], title: str, names: list[str]) -> str:
    rows = [entry for entry in resolved if entry.name in names]
    if not rows:
        return f"### {title}\n\n- Sin variables extraídas para este bloque."
    lines = [
        f"### {title}",
        "",
        "| Variable | Launch | Runtime wrapper | Fallback panel | Histórico | Estado |",
        "|---|---|---|---|---|---|",
    ]
    for entry in rows:
        state, _ = env_state_and_discrepancy(entry)
        lines.append(
            "| {name} | {launch} | {runtime} | {panel} | {historical} | {state} |".format(
                name=entry.name,
                launch=entry.launch_default or "-",
                runtime=entry.runtime_override or "-",
                panel=entry.panel_default or "-",
                historical=entry.historical_default or "-",
                state=state,
            )
        )
    return "\n".join(lines)


def env_discrepancy_markdown(resolved: list[ResolvedEnv]) -> str:
    rows = []
    for entry in resolved:
        _, discrepancy = env_state_and_discrepancy(entry)
        if discrepancy == "sin discrepancia relevante":
            continue
        rows.append((entry.name, entry.group, entry.launch_default or "-", entry.runtime_override or "-", entry.panel_default or "-", entry.historical_default or "-", discrepancy))
    if not rows:
        return "### 7.4 Discrepancias de parámetros\n\n- No se detectaron discrepancias de parámetros."
    lines = [
        "### 7.4 Discrepancias de parámetros",
        "",
        "| Variable | Grupo | Launch | Runtime wrapper | Fallback panel | Histórico | Lectura |",
        "|---|---|---|---|---|---|---|",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return "\n".join(lines)


def markdown_table(rows: list[tuple[str, ...]], headers: tuple[str, ...]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "|" + "|".join(["---"] * len(headers)) + "|",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return "\n".join(lines)


def top_surface_from_model(model_pose_z: float, local_pose_z: float, size_z: float) -> float:
    return model_pose_z + local_pose_z + (size_z / 2.0)


def build_phase_profiles(
    *,
    basket_pose: tuple[str, str, str],
    post_grasp_timeout: str,
    post_grasp_min_obj: str,
    post_grasp_min_lift: str,
    post_grasp_max_tcp: str,
    home_timeout: str,
    home_min_obj: str,
    home_min_lift: str,
    home_max_tcp: str,
    attach_xy_tol: str,
    attach_rel_drift: str,
    carry_settle_sec: str,
) -> str:
    phases = [
        {
            "name": "HOME",
            "state": "histórico revalidado por labels actuales",
            "goal": "Llevar el robot a una postura segura de reposo antes de iniciar o cerrar ciclo.",
            "target": "Preset articular HOME / JOINT_TABLE_POSE_RAD.",
            "frame": "joint space",
            "method": "Preset articular vía joint_trajectory_controller.",
            "success": "Convergencia articular dentro de tolerancia.",
            "failure": "Timeout de movimiento o controlador no activo.",
            "skip": "Puede omitirse si ya está en HOME y la lógica de pre-check lo confirma.",
            "logs": "[PICK][HOME] o trazas DIRECT equivalentes.",
            "env": "PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC (si entra por MoveIt).",
            "notes": "Se conserva como fase histórica útil; la implementación actual sigue manejando HOME como estado seguro aunque el pipeline directo haya cambiado de detalle interno.",
        },
        {
            "name": "MESA",
            "state": "histórico no revalidado completamente en esta corrida",
            "goal": "Llevar el robot a la pose de imagen / pre-pick sobre la mesa.",
            "target": "Preset tipo JOINT_PICK_IMAGE_POSE_RAD o equivalente de captura.",
            "frame": "joint space",
            "method": "Preset articular previo a PICK_IMAGE / selección.",
            "success": "Robot estabilizado en pose de observación.",
            "failure": "Timeout, controladores inactivos o joints no convergentes.",
            "skip": "Puede integrarse con PICK_IMAGE o ser absorbida por la lógica actual del panel.",
            "logs": "Trazas con label MESA / PICK_IMAGE.",
            "env": "Sin variable exclusiva confirmada; depende del pipeline de observación.",
            "notes": "Se recupera desde 2026-04-18 porque sigue siendo útil para entender el flujo panel -> visión -> pick.",
        },
        {
            "name": "APPROACH_COARSE",
            "state": "actual confirmado",
            "goal": "Posicionar rg2_pinch_center sobre el objeto con margen seguro antes del descenso.",
            "target": "(x_obj, y_obj, z_obj + clearance) en base_link.",
            "frame": "base_link / rg2_pinch_center",
            "method": "IK directo con conversión a base_link_inertia y NEGATE_XY permanente.",
            "success": "Gate XY/Z de aproximación dentro de tolerancia.",
            "failure": "IK no converge, settle insuficiente o divergencia FK/TF-live.",
            "skip": "No suele omitirse; puede haber correcciones XY/Z intermedias.",
            "logs": "[PICK][DIRECT][APPROACH_COARSE] y gates asociados.",
            "env": "PANEL_PICK_DEMO_APPROACH_COARSE_*.",
            "notes": "Aquí es crítica la diferencia world vs base_link y la semántica base_link_inertia del solver DH.",
        },
        {
            "name": "GRASP_DOWN",
            "state": "actual confirmado",
            "goal": "Descender el TCP hasta la altura de grasp sin saltar de rama IK.",
            "target": "Altura de contacto sobre el cilindro en base_link.",
            "frame": "base_link / rg2_pinch_center",
            "method": "MoveIt cartesiano si está habilitado, con fallback a IK segmentado conservador.",
            "success": "Error XY/Z y dist3D dentro de umbral; sin cambio de rama crítico.",
            "failure": "IK fuera de tolerancia, cartesian path fallido o branch jump.",
            "skip": "No se omite salvo abortos tempranos del pipeline.",
            "logs": "[PICK][DIRECT][GRASP_DOWN], [GRASP_DOWN_CARTESIAN], [GRASP_DOWN_FALLBACK].",
            "env": "PANEL_PICK_DEMO_GRASP_DOWN_*.",
            "notes": "La mitigación de branch jump y los steps de 5 mm se preservan como parte del conocimiento histórico útil.",
        },
        {
            "name": "GRASP_ALIGN_IK",
            "state": "actual confirmado",
            "goal": "Compensar residual geométrico y afinar Z/XY antes de cerrar.",
            "target": "Pose de contacto con bias iterativo en Z.",
            "frame": "base_link",
            "method": "Loop IK con bias Z y criterios de salida XY/Z.",
            "success": "ALIGN_EXIT_XY/Z dentro de tolerancia y residual aceptable.",
            "failure": "Intentos agotados o sin mejora de residual.",
            "skip": "Puede omitirse si el target ya es alcanzable y el panel así lo permite.",
            "logs": "[PICK][DIRECT][GRASP_ALIGN_IK] attempt=... bias=...",
            "env": "PANEL_PICK_DEMO_ALIGN_*.",
            "notes": "Se enlaza con el bug histórico del residual DH/SDF en Z y con la semántica actual tool0 vs rg2_pinch_center.",
        },
        {
            "name": "PRE_CLOSE",
            "state": "actual confirmado",
            "goal": "Validar alineación final antes de mandar el cierre del gripper.",
            "target": "Pose actual TCP vs objeto inmediatamente antes de CLOSE.",
            "frame": "base_link",
            "method": "Lectura FK/TF + comparación contra el objeto; puede reintentar realineación.",
            "success": "XY y Z_err dentro de tolerancia.",
            "failure": "Error fuera de umbral tras reintentos.",
            "skip": "Puede omitirse si el pipeline marca reachable alignment suficiente.",
            "logs": "[PICK][DIRECT][PRE_CLOSE] xy_err=... z_err=...",
            "env": "PANEL_PICK_DEMO_PRE_CLOSE_*.",
            "notes": "Sigue siendo una fase distinta de CLOSE: valida pose, no confirma grasp.",
        },
        {
            "name": "CLOSE",
            "state": "actual confirmado",
            "goal": "Cerrar RG2 y confirmar que hubo movimiento y/o apertura medida consistente.",
            "target": "Comando a rg2_finger_joint1/2 hasta cierre objetivo.",
            "frame": "joint space (gripper)",
            "method": "Publicación a gripper_controller + espera de confirmación por joint_states.",
            "success": "Delta de joints y/o opening_sum dentro de heurística de confirmación.",
            "failure": "CLOSE en PEND, controlador inactivo o timeout de confirmación.",
            "skip": "No se omite en el ciclo de pick.",
            "logs": "[PICK][DIRECT][CLOSE], wait_start/wait_ok/wait_timeout.",
            "env": "PANEL_PICK_DEMO_CLOSE_* y PANEL_PICK_DEMO_GRIPPER_*.",
            "notes": "Mantener explícito el bug histórico CLOSE en PEND y que cerrar no demuestra por sí solo grasp físico estable.",
        },
        {
            "name": "ATTACH_GATE",
            "state": "actual confirmado",
            "goal": "Aprobar attach lógico sólo cuando proximidad, drift y cierre son coherentes.",
            "target": f"TCP-objeto dentro de {attach_xy_tol} m XY y drift relativo dentro de {attach_rel_drift} m.",
            "frame": "base_link / world según muestras del evaluador",
            "method": "AttachGateEvaluator con ventana temporal multi-fuente.",
            "success": "Distancia TCP-objeto, drift, cierre de gripper y backend OK dentro de ventana estable.",
            "failure": "AttachGateEvaluator FAIL o backend sin confirmar attach.",
            "skip": "No se omite; es gate de seguridad lógico.",
            "logs": "[ATTACH_GATE][CHECK|PASS|FAIL|WARN].",
            "env": "PANEL_PICK_DEMO_ATTACH_*.",
            "notes": "ATTACH_GATE correcto = attach lógico aprobado. No equivale a transporte físico confirmado; esa separación debe preservarse y enlazarse con LIFT/CARRY.",
        },
        {
            "name": "LIFT",
            "state": "actual confirmado",
            "goal": "Elevar el objeto lo suficiente para demostrar levantamiento real tras el grasp.",
            "target": "Subida corta post-grasp antes del carry largo.",
            "frame": "base_link",
            "method": "IK directo o ruta equivalente con validación física post-grasp.",
            "success": f"_validate_demo_carry(post_grasp_lift) con timeout={post_grasp_timeout}s, min_obj_move={post_grasp_min_obj} m, min_lift_delta={post_grasp_min_lift} m y max_tcp_dist={post_grasp_max_tcp} m.",
            "failure": "demo_carry_validation_failed, object_not_updated, carry_follow_lost o tcp_dist_above_max.",
            "skip": "No se omite si ATTACH_GATE pasó.",
            "logs": "[PICK][DIRECT][FINAL_TRACE] phase=CARRY event=wait_done ..., [PICK][DIRECT][PHYSICS] phase=post_grasp_lift ...",
            "env": f"PANEL_PICK_DEMO_CARRY_SETTLE_SEC={carry_settle_sec} y thresholds de carry/lift en panel.",
            "notes": "Aquí se decide el primer veredicto físico serio. El attach lógico puede existir y aun así el objeto quedarse inmóvil o descoherente respecto al TCP.",
        },
        {
            "name": "CARRY / TRANSPORT",
            "state": "actual confirmado",
            "goal": "Transportar el objeto manteniendo coherencia física entre pose del objeto y TCP.",
            "target": "Ruta desde pick hasta cesta / home intermedio según pipeline.",
            "frame": "base_link y validación cruzada con world",
            "method": "Backend follow_tcp como semántica base; pick_demo entra por demo_transport con world_locked por defecto.",
            "success": "Carry validation OK y seguimiento coherente del objeto con el TCP.",
            "failure": "carry_follow_lost, object_not_updated, best_lift_delta < 0, best_tcp_dist > máximo o stale_tcp_pose_soft_follow degradando el seguimiento.",
            "skip": "No se omite cuando el objeto sigue adjunto lógicamente y debe ir a entrega.",
            "logs": "[ATTACH_BACKEND] demo_transport_follow_tick ..., [PICK][DIRECT][FINAL_TRACE] phase=CARRY ...",
            "env": "ATTACH_BACKEND_* y PANEL_PICK_DEMO_CARRY_*.",
            "notes": "Mantener explícita la discrepancia entre metadata de fase, llamada real y telemetría; no simplificarla.",
        },
        {
            "name": "HOME_WITH_OBJECT",
            "state": "actual confirmado parcialmente",
            "goal": "Pasar por una postura segura con objeto antes o durante el transporte largo.",
            "target": "Pose segura intermedia de home con objeto aún transportado.",
            "frame": "joint space / base_link según implementación concreta.",
            "method": "Preset o IK intermedio con validación carry específica para home_with_object.",
            "success": f"_validate_demo_carry(home_with_object) con timeout={home_timeout}s, min_obj_move={home_min_obj} m, min_lift_delta={home_min_lift} m y max_tcp_dist default={home_max_tcp} m.",
            "failure": "Carry deja de ser coherente durante el retorno intermedio.",
            "skip": "Si la ruta actual salta directamente a cesta o la fase no aplica.",
            "logs": "Trazas home_with_object y validación carry asociada.",
            "env": "PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M y thresholds asociados.",
            "notes": "La fase existe en el código actual aunque el documento 2026-04-18 la dejaba más implícita que explícita.",
        },
        {
            "name": "CESTA",
            "state": "histórico recuperado + target actual inferido del world file",
            "goal": "Posicionar el TCP sobre la bandeja de depósito antes de soltar.",
            "target": f"bandeja_deposito en world≈({basket_pose[0]}, {basket_pose[1]}, {basket_pose[2]}), con altura efectiva según release del pipeline.",
            "frame": "world para geometría de escena; base_link para ejecución.",
            "method": "Preset/IK/MoveIt según ruta actual del panel.",
            "success": "TCP en zona de entrega sin perder coherencia del objeto.",
            "failure": "No converge la ruta o el objeto deja de seguir al TCP antes de soltar.",
            "skip": "Si se aborta antes del transporte final.",
            "logs": "[PICK][DIRECT][TRANSPORT] y trazas CESTA/CESTA_RELEASE.",
            "env": "Sin variable exclusiva confirmada en esta corrida; depende del pipeline de entrega.",
            "notes": "La geometría actual de la cesta se reconfirma desde worlds/ur5_mesa_objetos.sdf; los detalles finos de altura de release siguen parte históricos.",
        },
        {
            "name": "RELEASE / CESTA_RELEASE",
            "state": "histórico recuperado + actual parcialmente revalidado",
            "goal": "Abrir el gripper y desacoplar el objeto para dejarlo en la cesta.",
            "target": "Gripper abierto + detach lógico/físico.",
            "frame": "joint space (gripper) y backend attach/detach.",
            "method": "Comando de apertura + detach del backend/Gazebo.",
            "success": "Apertura confirmada y objeto liberado del backend de attach.",
            "failure": "Timeout de apertura, detach no procesado o objeto sigue attached.",
            "skip": "No se omite si el ciclo llega a la cesta.",
            "logs": "[PICK][DIRECT][CESTA_RELEASE], detach_request_received, gazebo_detach_applied=...",
            "env": "PANEL_PICK_DEMO_RELEASE_*.",
            "notes": "Conviene validarlo también con topics del backend actual, no sólo con la semántica legacy de gripper_anchor.",
        },
        {
            "name": "HOME_FINAL",
            "state": "histórico revalidado por labels actuales",
            "goal": "Retornar a postura segura tras release o tras abortar en fase tardía.",
            "target": "Preset HOME final.",
            "frame": "joint space",
            "method": "Preset articular final.",
            "success": "Robot estable y ciclo cerrado en estado seguro.",
            "failure": "Timeout o controlador no listo.",
            "skip": "Puede omitirse sólo en abortos tempranos con parada manual.",
            "logs": "Trazas HOME_FINAL / retorno a home.",
            "env": "Sin variable exclusiva confirmada; usa timeouts generales de movimiento.",
            "notes": "Se conserva por trazabilidad operativa y por cierre de ciclo completo.",
        },
    ]

    lines = [
        "### 6.1 Fases reconstruidas y anotadas",
        "",
        "Las siguientes fases se presentan como unión consistente entre la base 2026-04-18, el documento vigente 2026-04-20 y el código inspeccionado en esta ejecución.",
        "",
    ]
    for index, phase in enumerate(phases, start=1):
        lines.extend(
            [
                f"### 6.{index + 1} {phase['name']}",
                "",
                f"- Estado: {phase['state']}",
                f"- Objetivo: {phase['goal']}",
                f"- Target: {phase['target']}",
                f"- Frame: {phase['frame']}",
                f"- Método: {phase['method']}",
                f"- Criterio de éxito: {phase['success']}",
                f"- Criterio de fallo: {phase['failure']}",
                f"- Criterio de skip: {phase['skip']}",
                f"- Logs relevantes: {phase['logs']}",
                f"- Variables de entorno asociadas: {phase['env']}",
                f"- Observaciones y discrepancias: {phase['notes']}",
                "",
            ]
        )
    return "\n".join(lines).strip()


def main() -> None:
    if len(sys.argv) != 9:
        raise SystemExit("Uso: generar_base_conocimiento_tfm.py ROOT OUT_MD OUT_DIFF REF_PDF HIST_MD TMP_DIR DATE_TAG SOURCES_FILE")

    root = Path(sys.argv[1])
    out_md = Path(sys.argv[2])
    out_diff = Path(sys.argv[3])
    ref_pdf = Path(sys.argv[4])
    hist_md = Path(sys.argv[5])
    tmp_dir = Path(sys.argv[6])
    date_tag = sys.argv[7]
    sources_file = Path(sys.argv[8])

    global ROOT
    ROOT = root

    files = {
        "launch": root / "agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py",
        "panel": root / "agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py",
        "backend": root / "agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_backend.py",
        "urdf": root / "agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro",
        "sdf": root / "agarre_ros2_ws/models/ur5_rg2/model.sdf",
        "world": root / "agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf",
        "controllers": root / "agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml",
        "panel_settings": root / "agarre_ros2_ws/src/ur5_qt_panel/config/panel_settings.yaml",
        "attach_gate": root / "agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/attach_gate_evaluator.py",
        "world_tf": root / "agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py",
        "start_panel": root / "agarre_ros2_ws/scripts/start_panel_v2.sh",
        "pkg_bringup": root / "agarre_ros2_ws/src/ur5_bringup/package.xml",
        "pkg_description": root / "agarre_ros2_ws/src/ur5_description/package.xml",
        "pkg_moveit": root / "agarre_ros2_ws/src/ur5_moveit_config/package.xml",
        "pkg_panel": root / "agarre_ros2_ws/src/ur5_qt_panel/package.xml",
        "pkg_grasping": root / "agarre_ros2_ws/src/tfm_grasping/package.xml",
        "pkg_tools": root / "agarre_ros2_ws/src/ur5_tools/package.xml",
        "pkg_interfaces": root / "agarre_ros2_ws/src/ur5_panel_interfaces/package.xml",
        "setup_panel": root / "agarre_ros2_ws/src/ur5_qt_panel/setup.py",
        "setup_tools": root / "agarre_ros2_ws/src/ur5_tools/setup.py",
        "setup_grasping": root / "agarre_ros2_ws/src/tfm_grasping/setup.py",
    }
    for label, path in files.items():
        if not path.exists():
            raise SystemExit(f"No existe la fuente requerida {label}: {path}")

    base_dir = out_md.parent
    previous_output_text = read_text(out_md) if out_md.exists() else ""
    versioned_current_text = git_show_file(root, out_md)
    current_seed_md = out_md if versioned_current_text else None
    if not versioned_current_text:
        candidates = sorted(base_dir.glob("*_base_conocimiento_tecnica_TFM.md"))
        for candidate in reversed(candidates):
            candidate_text = read_text(candidate)
            if "Documento generado como base acumulativa y no sustitutiva." in candidate_text:
                continue
            current_seed_md = candidate
            versioned_current_text = candidate_text
            break
    if not versioned_current_text and previous_output_text and "Documento generado como base acumulativa y no sustitutiva." not in previous_output_text:
        current_seed_md = out_md
        versioned_current_text = previous_output_text

    launch_text = read_text(files["launch"])
    panel_text = read_text(files["panel"])
    backend_text = read_text(files["backend"])
    urdf_text = read_text(files["urdf"])
    sdf_text = read_text(files["sdf"])
    world_text = read_text(files["world"])
    controllers_text = read_text(files["controllers"])
    panel_settings_text = read_text(files["panel_settings"])
    attach_gate_text = read_text(files["attach_gate"])
    world_tf_text = read_text(files["world_tf"])
    start_panel_text = read_text(files["start_panel"])
    historical_text = load_markdown_or_pdf(hist_md, ref_pdf)
    current_seed_text = read_text(current_seed_md) if current_seed_md and current_seed_md.exists() else ""

    rg2_tcp_xyz = capture(
        r'<joint name="rg2_tcp_joint"[^>]*>.*?<origin xyz="([^"]+)"',
        urdf_text,
        "offset tool0 -> rg2_tcp",
    )
    rg2_pinch_xyz = capture(
        r'<joint name="rg2_pinch_center_joint"[^>]*>.*?<origin xyz="([^"]+)"',
        urdf_text,
        "offset tool0 -> rg2_pinch_center",
    )
    urdf_base_origin = capture(
        r'<origin xyz="([^"]+)" rpy="0 0 0"/>',
        urdf_text,
        "origen URDF world -> base_link",
    )
    sdf_hand_rel, sdf_hand_pose = capture_groups(
        r'<joint name="ur5_hand_joint"[^>]*>.*?<pose relative_to="([^"]+)">([^<]+)</pose>',
        sdf_text,
        "pose de ur5_hand_joint",
    )
    tool0_pose = capture(
        r'<joint name="end_effector_frame_fixed_joint" type="fixed">.*?<pose relative_to="wrist_3_link">([^<]+)</pose>',
        sdf_text,
        "pose de tool0 en SDF",
    )
    camera_wrist_pose = capture(
        r'<joint name="camera_wrist_joint" type="fixed">.*?<pose relative_to="rg2_hand">([^<]+)</pose>',
        sdf_text,
        "pose camera_wrist_link",
    )
    pick_demo_anchor_sdf = capture(
        r'<joint name="pick_demo_anchor_joint" type="fixed">.*?<pose relative_to="tool0">([^<]+)</pose>',
        sdf_text,
        "pose pick_demo_anchor en SDF",
    )
    gripper_tcp_z_offset = capture(
        r'^\s*gripper_tcp_z_offset\s*:\s*([-+0-9.eE]+)\s*$',
        panel_settings_text,
        "gripper_tcp_z_offset",
        flags=re.M,
    )
    runtime_pick_demo_anchor = str(max(0.0, float(rg2_tcp_xyz.split()[-1]) - float(gripper_tcp_z_offset)))

    attach_xy_tol = capture(
        r'"PANEL_PICK_DEMO_ATTACH_XY_TOL_M".*?os\.environ\.get\("PANEL_PICK_DEMO_ATTACH_XY_TOL_M", "([^"]+)"\)',
        launch_text,
        "PANEL_PICK_DEMO_ATTACH_XY_TOL_M",
    )
    attach_z_tol = capture(
        r'"PANEL_PICK_DEMO_ATTACH_Z_TOL_M".*?os\.environ\.get\("PANEL_PICK_DEMO_ATTACH_Z_TOL_M", "([^"]+)"\)',
        launch_text,
        "PANEL_PICK_DEMO_ATTACH_Z_TOL_M",
    )
    attach_follow_max = capture(
        r'"PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M".*?os\.environ\.get\("PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M", "([^"]+)"\)',
        launch_text,
        "PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M",
    )
    attach_rel_drift = capture(
        r'"PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M".*?os\.environ\.get\("PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M", "([^"]+)"\)',
        launch_text,
        "PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M",
    )
    attach_stable_window = capture(
        r'"PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC".*?os\.environ\.get\("PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC", "([^"]+)"\)',
        launch_text,
        "PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC",
    )
    attach_min_samples = capture(
        r'"PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES".*?os\.environ\.get\("PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES", "([^"]+)"\)',
        launch_text,
        "PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES",
    )
    attach_tf_visual_gap = capture(
        r'"PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M".*?os\.environ\.get\("PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M", "([^"]+)"\)',
        launch_text,
        "PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M",
    )
    gripper_closed_thr = capture(
        r'"PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M".*?os\.environ\.get\("PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M", "([^"]+)"\)',
        launch_text,
        "PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M",
    )
    attach_backend_mode = capture(
        r'DeclareLaunchArgument\(\s*"attach_backend_mode",\s*default_value=os\.environ\.get\("ATTACH_BACKEND_MODE", "([^"]+)"\)',
        launch_text,
        "ATTACH_BACKEND_MODE",
    )
    attach_backend_max_pose_age = capture(
        r'DeclareLaunchArgument\(\s*"attach_backend_max_pose_age_sec",\s*default_value=os\.environ\.get\("ATTACH_BACKEND_MAX_POSE_AGE_SEC", "([^"]+)"\)',
        launch_text,
        "ATTACH_BACKEND_MAX_POSE_AGE_SEC",
    )
    attach_backend_follow_rate = capture(
        r'DeclareLaunchArgument\(\s*"attach_backend_follow_rate_hz",\s*default_value=os\.environ\.get\("ATTACH_BACKEND_FOLLOW_RATE_HZ", "([^"]+)"\)',
        launch_text,
        "ATTACH_BACKEND_FOLLOW_RATE_HZ",
    )
    attach_backend_max_dist = capture(
        r'DeclareLaunchArgument\(\s*"attach_backend_max_dist_m",\s*default_value=os\.environ\.get\("ATTACH_BACKEND_MAX_DIST_M", "([^"]+)"\)',
        launch_text,
        "ATTACH_BACKEND_MAX_DIST_M",
    )
    demo_transport_objects = capture(
        r'ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS",\s*"([^"]+)"',
        launch_text,
        "ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS",
    )
    attach_settle_sec = capture(
        r'PANEL_PICK_DEMO_ATTACH_SETTLE_SEC",\s*"([^"]+)"',
        panel_text,
        "PANEL_PICK_DEMO_ATTACH_SETTLE_SEC",
    )
    carry_settle_sec = capture(
        r'PANEL_PICK_DEMO_CARRY_SETTLE_SEC",\s*"([^"]+)"',
        panel_text,
        "PANEL_PICK_DEMO_CARRY_SETTLE_SEC",
    )
    post_attach_hold_sec = capture(
        r'PANEL_PICK_DEMO_POST_ATTACH_HOLD_SEC",\s*"([^"]+)"',
        panel_text,
        "PANEL_PICK_DEMO_POST_ATTACH_HOLD_SEC",
    )
    carry_phase_min_obj, carry_phase_min_lift, carry_phase_max_tcp = capture_groups(
        r'_phase_begin\(\s*"CARRY".*?"min_obj_move_m":\s*([0-9.]+),\s*"min_lift_delta_m":\s*([0-9.]+),\s*"max_tcp_dist_m":\s*([0-9.]+)',
        panel_text,
        "metadata de fase CARRY",
    )
    post_grasp_timeout, post_grasp_min_obj, post_grasp_min_lift, post_grasp_max_tcp = capture_groups(
        r'_validate_demo_carry\(\s*initial_obj_world=initial_obj_world,\s*phase="post_grasp_lift",\s*timeout_sec=([0-9.]+),\s*min_obj_move_m=([0-9.]+),\s*min_lift_delta_m=([0-9.]+),\s*max_tcp_dist_m=([0-9.]+),',
        panel_text,
        "llamada real post_grasp_lift",
    )
    home_timeout, home_min_obj, home_min_lift = capture_groups(
        r'_validate_demo_carry\(\s*initial_obj_world=initial_obj_world,\s*phase="home_with_object",\s*timeout_sec=([0-9.]+),\s*min_obj_move_m=([0-9.]+),\s*min_lift_delta_m=([0-9.]+),',
        panel_text,
        "llamada real home_with_object",
    )
    home_max_tcp = capture(
        r'PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M", "([^"]+)"',
        panel_text,
        "PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M",
    )

    mesa_model = capture_groups(
        r'<model name="mesa_pro">.*?<pose>([^<]+)</pose>.*?<collision name="tablero_collision">\s*<pose>([^<]+)</pose>\s*<geometry><box><size>([^<]+)</size>',
        world_text,
        "mesa_pro",
    )
    pick_demo_world_pose = capture(
        r'<model name="pick_demo">.*?<pose>([^<]+)</pose>',
        world_text,
        "pose de pick_demo",
    )
    pick_demo_mass = capture(
        r'<model name="pick_demo">.*?<mass>([^<]+)</mass>',
        world_text,
        "masa de pick_demo",
    )
    pick_demo_radius = capture(
        r'<model name="pick_demo">.*?<cylinder><radius>([^<]+)</radius>',
        world_text,
        "radio de pick_demo",
    )
    pick_demo_length = capture(
        r'<model name="pick_demo">.*?<cylinder><radius>[^<]+</radius><length>([^<]+)</length>',
        world_text,
        "altura de pick_demo",
    )
    basket_pose = capture(
        r'<model name="bandeja_deposito">.*?<pose>([^<]+)</pose>',
        world_text,
        "pose de bandeja_deposito",
    )
    basket_base_size = capture(
        r'<model name="bandeja_deposito">.*?<collision name="base_c">\s*<geometry><box><size>([^<]+)</size>',
        world_text,
        "base de bandeja_deposito",
    )
    left_finger_limit = capture(
        r'<joint name="rg2_finger_joint1" type="revolute">.*?<upper>([^<]+)</upper>',
        sdf_text,
        "upper rg2_finger_joint1",
    )
    finger_friction = capture(
        r'<link name="rg2_leftfinger">.*?<mu>([^<]+)</mu>',
        sdf_text,
        "mu finger",
    )
    finger_kp = capture(
        r'<link name="rg2_leftfinger">.*?<kp>([^<]+)</kp>',
        sdf_text,
        "kp finger",
    )
    finger_kd = capture(
        r'<link name="rg2_leftfinger">.*?<kd>([^<]+)</kd>',
        sdf_text,
        "kd finger",
    )
    rg2_hand_mass = capture(
        r'<link name="rg2_hand">.*?<mass>([^<]+)</mass>',
        sdf_text,
        "masa rg2_hand",
    )
    left_finger_pose = capture(
        r'<joint name="rg2_finger_joint1" type="revolute">\s*<pose relative_to="rg2_hand">([^<]+)</pose>',
        sdf_text,
        "pose rg2_finger_joint1",
    )
    right_finger_pose = capture(
        r'<joint name="rg2_finger_joint2" type="revolute">\s*<pose relative_to="rg2_hand">([^<]+)</pose>',
        sdf_text,
        "pose rg2_finger_joint2",
    )

    mesa_model_pose = [float(v) for v in mesa_model[0].split()[:3]]
    mesa_board_pose = [float(v) for v in mesa_model[1].split()[:3]]
    mesa_board_size = [float(v) for v in mesa_model[2].split()[:3]]
    mesa_top_z = top_surface_from_model(mesa_model_pose[2], mesa_board_pose[2], mesa_board_size[2])
    basket_pose_parts = [float(v) for v in basket_pose.split()[:3]]
    basket_base_size_parts = [float(v) for v in basket_base_size.split()[:3]]
    basket_floor_top = top_surface_from_model(basket_pose_parts[2], 0.0, basket_base_size_parts[2])

    audit_doc = root / "auditoria/informe_fix_visual_grasp_20260419.md"
    reports_status = root / "reports/evidence/ros2/moveit2_system_status.json"
    audit_text = read_text(audit_doc) if audit_doc.exists() else ""
    reports_status_text = read_text(reports_status) if reports_status.exists() else ""

    helper_logs = sorted((root / "auditoria").glob("**/helper.log"))
    stack_logs = sorted((root / "auditoria").glob("**/stack.log")) + sorted((root / "historico").glob("stack_manual_*.log"))
    all_text_logs = helper_logs + stack_logs + sorted((root / "auditoria").glob("**/*.md"))

    static_fail_path, static_fail_line = first_matching_line(
        all_text_logs,
        lambda line: (
            "demo_carry_validation_failed phase=post_grasp_lift" in line
            and "best_obj_move=0.000" in line
            and "best_lift_delta=0.000" in line
        ),
    )
    follow_lost_path, follow_lost_line = first_matching_line(
        all_text_logs,
        lambda line: (
            "demo_carry_validation_failed phase=post_grasp_lift" in line
            and "carry_detail=carry_follow_lost" in line
            and "best_lift_delta=-" in line
        ),
    )
    never_moved_path, never_moved_line = first_matching_line(
        all_text_logs,
        lambda line: ("carry_follow_lost=true" in line and "detail=object_never_moved" in line),
    )
    stale_path, stale_line = first_matching_line(
        stack_logs,
        lambda line: "stale_tcp_pose_soft_follow" in line,
    )
    world_locked_path, world_locked_line = first_matching_line(
        stack_logs,
        lambda line: "demo_transport_follow_tick object=pick_demo mode=world_locked" in line,
    )
    world_locked_gap_path, world_locked_gap = first_two_world_locked_samples(stack_logs)
    report_follow_role = None
    if reports_status_text:
        match = re.search(r'"role": "([^"]*follow_tcp[^"]*)"', reports_status_text, re.S)
        if match:
            report_follow_role = match.group(1)

    audit_failure_snippet = None
    if audit_text:
        match = re.search(
            r"\[PICK\] ✗ Error: demo_carry_validation_failed.*?best_obj_move=0\.000.*?best_lift_delta=0\.000.*?best_tcp_dist=0\.113",
            audit_text,
            re.S,
        )
        if match:
            audit_failure_snippet = textwrap.dedent(
                """
                [PICK] ✗ Error: demo_carry_validation_failed
                  phase=post_grasp_lift
                  best_obj_move=0.000
                  best_lift_delta=0.000
                  best_tcp_dist=0.113
                """
            ).strip()

    follow_semantic = "confirmado" if "follow_confirmed_only_after_carry" in panel_text else "no confirmado"
    world_locked_gap_txt = "no confirmado en logs"
    if world_locked_gap is not None:
        world_locked_gap_txt = f"{world_locked_gap:.3f} s entre dos ticks consecutivos"

    hist_env_defaults = parse_historical_env_defaults(historical_text)
    env_entries = []
    env_entries.extend(collect_env_entries(launch_text, "launch", files["launch"]))
    env_entries.extend(collect_env_entries(start_panel_text, "start_panel", files["start_panel"]))
    env_entries.extend(collect_env_entries(panel_text, "panel", files["panel"]))
    resolved_env = resolve_env_entries(env_entries, hist_env_defaults)

    package_rows = []
    for key in [
        "pkg_bringup",
        "pkg_description",
        "pkg_moveit",
        "pkg_panel",
        "pkg_grasping",
        "pkg_tools",
        "pkg_interfaces",
    ]:
        name, description = parse_package_info(files[key])
        package_rows.append((f"`{name}`", f"`{rel(root, files[key].parent)}`", description or "Sin descripción declarada."))

    current_nodes = [
        ("robot_state_publisher", "built-in", "Publica robot_description y árbol TF semántico."),
        ("gz_sim", "built-in", "Motor de simulación Gazebo."),
        ("ros_gz_bridge", "built-in", "Puente Gazebo ↔ ROS 2 para clock, cámaras y pose/info."),
        ("joint_state_broadcaster", rel(root, files["controllers"]), "Publica /joint_states del robot y gripper."),
        ("joint_trajectory_controller", rel(root, files["controllers"]), "Ejecuta trayectorias del brazo UR5."),
        ("gripper_controller", rel(root, files["controllers"]), "Recibe comandos de apertura/cierre de RG2."),
    ]
    for setup_key in ["setup_tools", "setup_panel", "setup_grasping"]:
        for name, target in parse_console_scripts(files[setup_key]):
            current_nodes.append((name, rel(root, files[setup_key]), f"Entry point Python: {target}"))

    launch_rows = [
        ("`ur5_stack.launch.py`", f"`{rel(root, files['launch'])}`", "Launch principal: Gazebo, RSP, bridges, attach backend, scene sync, bridge MoveIt y panel."),
        ("`ur5_rsp.launch.py`", "`agarre_ros2_ws/src/ur5_bringup/launch/ur5_rsp.launch.py`", "Robot state publisher del UR5/RG2."),
        ("`ur5_ros2_control.launch.py`", "`agarre_ros2_ws/src/ur5_bringup/launch/ur5_ros2_control.launch.py`", "Bringup de ros2_control y controller_manager."),
        ("`ur5_moveit_bringup.launch.py`", "`agarre_ros2_ws/src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py`", "Bringup de MoveIt 2 / move_group."),
    ]

    file_control_rows = [
        ("Frames TF del robot", "`agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro`"),
        ("Modelo físico Gazebo", "`agarre_ros2_ws/models/ur5_rg2/model.sdf`"),
        ("Mundo de simulación", "`agarre_ros2_ws/worlds/ur5_mesa_objetos.sdf`"),
        ("Controladores ros2_control", "`agarre_ros2_ws/src/ur5_bringup/config/ur5_mock_controllers.yaml`"),
        ("Variables de entorno del pick demo", "`agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py`"),
        ("Overrides runtime del panel", "`agarre_ros2_ws/scripts/start_panel_v2.sh`"),
        ("Lógica de fases del pick", "`agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py`"),
        ("Gate de attach", "`agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/attach_gate_evaluator.py`"),
        ("Backend attach/transporte", "`agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_backend.py`"),
        ("TF world -> base_link", "`agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py`"),
        ("Geometría vertical del pick", "`agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_geometry.py`"),
        ("Interfaz Qt / orquestación", "`agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py`"),
    ]

    frame_rows = [
        ("`world`", "origen Gazebo", "—", "—", "Origen absoluto de simulación", "Mezclar world con base_link desplaza targets del pick."),
        ("`base_link`", "frame URDF publicado", "`world`", f"actual confirmado: {urdf_base_origin}", "Base del UR5 para IK, panel y MoveIt", "No asumir sólo offset Z: la traslación actual también incluye X=-0.85 m."),
        ("`base_link_inertia`", "frame cinemático DH", "`base_link`", "histórico/documentado: (0,0,0) + Rz(pi)", "Marco interno del solver IK/FK", "Si se ignora, reaparece el bug NEGATE_XY / simetría especular."),
        ("`cadena cinemática UR5`", "joints UR5", "`base_link_inertia`", "per DH / ur_description", "Shoulder -> wrist_3 del brazo", "No mezclar valores DH con poses world sin transformación previa."),
        ("`tool0`", "TCP semántico del brazo", "`wrist_3_link`", f"SDF visual actual: {tool0_pose}", "Padre semántico de RG2 y ancla de offset del gripper", "Confundir tool0 con el punto real de grasp desplaza el contacto 175 mm."),
        ("`rg2_base_link`", "frame URDF fijo", "`tool0`", "actual confirmado: 0 0 0", "Base semántica del gripper RG2", "No equivale al pinch center operativo."),
        ("`rg2_tcp`", "frame URDF fijo", "`tool0`", f"actual confirmado: {rg2_tcp_xyz}", "TCP virtual/semántico del gripper", "No confundir con geometría visible SDF del cuerpo RG2."),
        ("`rg2_pinch_center`", "frame URDF fijo", "`tool0`", f"actual confirmado: {rg2_pinch_xyz}", "Punto operacional de grasp y attach", "Es el frame correcto para lógica de pick; usar tool0 degrada Z y distancias."),
        ("`camera_wrist_link`", "frame SDF fijo", "`rg2_hand`", f"actual confirmado en SDF: {camera_wrist_pose}", "Cámara montada en la muñeca/gripper", "Es geometría SDF, no el TCP operacional."),
        ("`pick_demo_anchor`", "frame SDF fijo / runtime rewrite", "`tool0`", f"SDF fuente={pick_demo_anchor_sdf}; runtime actual efectivo={runtime_pick_demo_anchor} m por panel_settings gripper_tcp_z_offset={gripper_tcp_z_offset}", "Ancla auxiliar para attach backend / probes", "Si el offset runtime cambia, el ancla deja de coincidir con rg2_pinch_center aunque el SDF fuente quede estable."),
    ]

    frame_discrepancy_rows = [
        ("world -> base_link", f"actual confirmado: {urdf_base_origin}", "histórico simplificado: (0,0,0.850)", "La geometría actual incluye traslación en X; documentar sólo Z ya no es suficiente."),
        ("tool0 -> rg2_pinch_center", f"actual confirmado: {rg2_pinch_xyz}", "histórico: 0 0 0.175", "Sin discrepancia de valor; se conserva como confirmación actual."),
        ("SDF ur5_hand_joint", f"actual confirmado: relative_to={sdf_hand_rel}, pose={sdf_hand_pose}", "memorias previas con otros valores", "Priorizar el source tree actual; los valores previos quedan como históricos o desactualizados."),
        ("pick_demo_anchor", f"actual runtime efectivo: {runtime_pick_demo_anchor} m", f"histórico documentado: {pick_demo_anchor_sdf.split()[2]} m", "Hoy coincide porque gripper_tcp_z_offset=0.0, pero el launch mantiene la lógica de reescritura runtime y debe documentarse como tal."),
    ]

    geometry_rows = [
        ("Apertura máxima por finger", f"{left_finger_limit} rad", rel(root, files["sdf"])),
        ("Fricción fingers", f"mu={finger_friction}", rel(root, files["sdf"])),
        ("Stiffness contacto fingers", f"kp={finger_kp}", rel(root, files["sdf"])),
        ("Damping contacto fingers", f"kd={finger_kd}", rel(root, files["sdf"])),
        ("Masa cuerpo RG2 (rg2_hand)", rg2_hand_mass, rel(root, files["sdf"])),
        ("tool0 -> rg2_pinch_center", rg2_pinch_xyz, rel(root, files["urdf"])),
        ("Pose visual ur5_hand_joint", f"relative_to={sdf_hand_rel}, pose={sdf_hand_pose}", rel(root, files["sdf"])),
        ("Finger izquierdo respecto a rg2_hand", left_finger_pose, rel(root, files["sdf"])),
        ("Finger derecho respecto a rg2_hand", right_finger_pose, rel(root, files["sdf"])),
    ]

    workspace_rows = [
        ("Objeto pick_demo", f"cilindro radio={pick_demo_radius} m, longitud={pick_demo_length} m, masa={pick_demo_mass} kg", rel(root, files["world"])),
        ("Spawn pick_demo (world)", pick_demo_world_pose, rel(root, files["world"])),
        ("Mesa útil", f"centro world=({mesa_model_pose[0]:.3f}, {mesa_model_pose[1]:.3f}), tablero={mesa_board_size[0]:.3f} x {mesa_board_size[1]:.3f} x {mesa_board_size[2]:.3f} m", rel(root, files["world"])),
        ("Altura superficie mesa", f"actual confirmada ≈ {mesa_top_z:.3f} m en world", rel(root, files["world"])),
        ("Cesta / bandeja destino", f"pose world base={basket_pose}; superficie base≈{basket_floor_top:.3f} m", rel(root, files["world"])),
        ("Reach operativo UR5", "histórico/documentado previamente: ~0.85 m", rel(root, hist_md if hist_md.exists() else ref_pdf)),
        ("Residual geométrico", "actual: pendiente de validación runtime fina; histórico: residual DH/SDF mitigado vía bias loop", rel(root, files["panel"])),
    ]

    controller_rows = [
        ("`joint_state_broadcaster`", "joint_state_broadcaster/JointStateBroadcaster", "todos los joints", "`/joint_states`", "Actual confirmado en ur5_mock_controllers.yaml"),
        ("`joint_trajectory_controller`", "joint_trajectory_controller/JointTrajectoryController", "shoulder_pan .. wrist_3", "`/joint_trajectory_controller/joint_trajectory`", "Actual confirmado en ur5_mock_controllers.yaml"),
        ("`gripper_controller`", "forward_command_controller/ForwardCommandController", "rg2_finger_joint1/2", "`/gripper_controller/commands`", "Actual confirmado en ur5_mock_controllers.yaml"),
    ]

    topic_rows = [
        ("`/joint_states`", "PUB", "joint_state_broadcaster", "panel, bridge MoveIt, diagnósticos"),
        ("`/tf` y `/tf_static`", "PUB", "RSP, gz_pose_bridge, world_tf_publisher", "todos los consumidores TF"),
        ("`/desired_grasp/request`", "PUB", "panel", "ur5_moveit_bridge"),
        ("`/desired_grasp/result`", "PUB", "ur5_moveit_bridge", "panel"),
        ("`/gripper/<obj>/attach`", "SUB backend", "panel / caller", "gripper_attach_backend"),
        ("`/gripper/<obj>/detach`", "SUB backend", "panel / caller", "gripper_attach_backend"),
        ("`/gripper/<obj>/state`", "PUB backend", "gripper_attach_backend", "panel / diagnósticos"),
        ("`/drop_anchor/<obj>/attach|detach|state`", "PUB/SUB backend ↔ plugin Gazebo", "gripper_attach_backend / plugin detachable", "backend / Gazebo"),
        ("`/gripper_anchor/<obj>/attach|detach`", "PUB backend", "gripper_attach_backend", "plugin/ancla de tool anchor cuando aplica"),
        ("`/world/ur5_mesa_objetos/pose/info`", "PUB Gazebo bridge", "ros_gz_bridge", "world_tf_publisher y gripper_attach_backend"),
    ]

    phase_profiles_md = build_phase_profiles(
        basket_pose=(f"{basket_pose_parts[0]:.3f}", f"{basket_pose_parts[1]:.3f}", f"{basket_floor_top:.3f}"),
        post_grasp_timeout=post_grasp_timeout,
        post_grasp_min_obj=post_grasp_min_obj,
        post_grasp_min_lift=post_grasp_min_lift,
        post_grasp_max_tcp=post_grasp_max_tcp,
        home_timeout=home_timeout,
        home_min_obj=home_min_obj,
        home_min_lift=home_min_lift,
        home_max_tcp=home_max_tcp,
        attach_xy_tol=attach_xy_tol,
        attach_rel_drift=attach_rel_drift,
        carry_settle_sec=carry_settle_sec,
    )

    current_seed_text = versioned_current_text
    current_sections = {title: block for title, block in split_sections(current_seed_text, 2)}
    historical_sections = {title: block for title, block in split_sections(historical_text, 2)}

    sources_used = [
        *files.values(),
        current_seed_md if current_seed_md and current_seed_md.exists() else None,
        hist_md if hist_md.exists() else None,
        ref_pdf if ref_pdf.exists() else None,
        audit_doc if audit_doc.exists() else None,
        reports_status if reports_status.exists() else None,
        static_fail_path,
        follow_lost_path,
        never_moved_path,
        stale_path,
        world_locked_path,
        world_locked_gap_path,
    ]
    unique_sources = []
    seen = set()
    for path in sources_used:
        if path is None:
            continue
        resolved = str(path)
        if resolved in seen:
            continue
        seen.add(resolved)
        unique_sources.append(path)
    write_text(sources_file, "\n".join(rel(root, path) for path in unique_sources) + "\n")

    parts: list[str] = []
    parts.append("# Base de Conocimiento Técnica — Sistema UR5 + RG2 Pick & Place")
    parts.append("")
    parts.append(f"Fecha de generación: {date_tag}")
    parts.append("")
    parts.append("Documento generado como base acumulativa y no sustitutiva. Esta versión une el estado confirmado en código/logs vigentes, el contenido especializado ya presente en la versión actual y el detalle estructural recuperado de la base 2026-04-18.")
    parts.append("")
    parts.append("## 1. Resumen Ejecutivo")
    parts.append("")
    parts.extend(
        [
            "- Modo de generación: acumulativo; append, don’t replace.",
            "- Verdad runtime priorizada: código fuente inspeccionado en esta ejecución, seguido de logs/evidencias actuales y reports recientes.",
            "- Contenido vigente preservado explícitamente: attach lógico vs transporte físico, ATTACH_GATE vs CARRY, follow_tcp vs world_locked, stale_tcp_pose_soft_follow, discrepancias metadata/llamada real/telemetría y diagnósticos best_obj_move / best_lift_delta / best_tcp_dist.",
            "- Recuperación histórica: se reincorporan arquitectura extendida, tabla amplia de frames, geometría, flujo fase por fase, inventario amplio de variables, controladores/topics, bugs legacy y troubleshooting operativo de 2026-04-18.",
            f"- Discrepancias abiertas resaltadas: world->base_link actual={urdf_base_origin} frente a simplificaciones históricas; attach backend launch={attach_backend_max_dist} / wrapper runtime=0.06; max_pose_age launch={attach_backend_max_pose_age} / wrapper runtime=2.5.",
            "- Regla de trazabilidad aplicada: cuando una fuente histórica difiere de la actual, se conserva como histórico/documentado previamente en lugar de borrarla.",
        ]
    )
    parts.append("")
    if historical_sections.get("1. Resumen Ejecutivo del Sistema"):
        parts.append(preserved_block("Detalle histórico recuperado (2026-04-18)", historical_sections.get("1. Resumen Ejecutivo del Sistema", ""), shift=1).strip())
        parts.append("")

    parts.append("## 2. Fuentes Verificadas")
    parts.append("")
    parts.extend(
        [
            "### 2.1 Prioridad de fuentes aplicada",
            "",
            "1. Código fuente inspeccionado en esta ejecución.",
            "2. Logs y evidencias runtime locales actuales.",
            "3. Reports recientes del workspace.",
            "4. Base 2026-04-18 como referencia histórica estructural.",
            "",
            "### 2.2 Archivos y artefactos usados en esta generación",
            "",
        ]
    )
    for path in unique_sources:
        parts.append(f"- `{rel(root, path)}`")
    parts.append("")
    parts.append("### 2.3 Política de merge aplicada")
    parts.append("")
    parts.extend(
        [
            "- append, don’t replace",
            "- prefer current for runtime truth",
            "- preserve historical structure when still useful",
            "- annotate conflicts instead of dropping them",
            "- never compress specialized diagnostics into generic summaries",
        ]
    )
    parts.append("")
    parts.append("## 3. Arquitectura del Proyecto")
    parts.append("")
    parts.append("### 3.1 Paquetes ROS 2 del workspace y responsabilidad")
    parts.append("")
    parts.append(markdown_table(package_rows, ("Paquete", "Ruta", "Responsabilidad / descripción")))
    parts.append("")
    parts.append("### 3.2 Nodos principales y función")
    parts.append("")
    node_rows = [(f"`{name}`", f"`{source}`", desc) for name, source, desc in current_nodes]
    parts.append(markdown_table(node_rows, ("Nodo / entry point", "Fuente", "Función")))
    parts.append("")
    parts.append("### 3.3 Launch files relevantes y qué lanza cada uno")
    parts.append("")
    parts.append(markdown_table(launch_rows, ("Archivo", "Ruta", "Qué lanza")))
    parts.append("")
    parts.append("### 3.4 Qué archivo controla cada parte")
    parts.append("")
    parts.append(markdown_table(file_control_rows, ("Parte del sistema", "Archivo de control")))
    parts.append("")
    parts.append("### 3.5 Relación entre módulos y flujo de comandos")
    parts.append("")
    parts.extend(
        [
            "- panel_v2 orquesta interacción de usuario, estados UI y disparo del ciclo.",
            "- panel_pick_demo ejecuta fases, targets, settle, gates y validación física del carry.",
            "- ur5_kinematics aplica IK/FK directo en base_link_inertia; el panel traduce desde base_link con NEGATE_XY.",
            "- ur5_moveit_bridge publica/consume desired_grasp para el camino MoveIt.",
            "- joint_trajectory_controller y gripper_controller ejecutan los comandos físicos en Gazebo.",
            "- gripper_attach_backend decide attach_route_decision entre follow_tcp, tool_anchor y demo_transport/world_locked según objeto y configuración.",
        ]
    )
    parts.append("")
    parts.append("### 3.6 Flujo panel → IK → controller")
    parts.append("")
    parts.append("`panel_pick_demo.py` → `_move_tcp_direct()` / `_send_ik_motion()` → `ur5_kinematics.ik_ur5()` → publicación a `/joint_trajectory_controller/joint_trajectory` → `joint_trajectory_controller` → `/joint_states` → verificación de convergencia y settle.")
    parts.append("")
    parts.append("### 3.7 Flujo panel → MoveIt bridge → move_group → resultado")
    parts.append("")
    parts.append("`panel_pick_demo.py` → `/desired_grasp/request` → `ur5_moveit_bridge` → `move_group.computeCartesianPath()/execute()` → `/desired_grasp/result` → panel. El bridge se lanza ya en el stack principal y no sólo on-demand.")
    parts.append("")
    parts.append(preserved_block("Detalle estructural recuperado (2026-04-18)", historical_sections.get("2. Arquitectura del Proyecto", ""), shift=1).strip())
    parts.append("")

    parts.append("## 4. Frames y Offsets")
    parts.append("")
    parts.append("### 4.1 Tabla de frames y offsets consolidada")
    parts.append("")
    parts.append(markdown_table(frame_rows, ("Frame", "Tipo", "Padre", "Offset / estado", "Uso operativo", "Riesgos al mezclar frames")))
    parts.append("")
    parts.append("### 4.2 Discrepancias actuales vs históricas")
    parts.append("")
    parts.append(markdown_table(frame_discrepancy_rows, ("Elemento", "Valor actual confirmado", "Valor histórico / documentado", "Observación")))
    parts.append("")
    parts.extend(
        [
            "### 4.3 Observaciones obligatorias preservadas y reforzadas",
            "",
            f"- `tool0` vs `rg2_pinch_center`: el offset semántico actual confirmado sigue siendo {rg2_pinch_xyz}. La geometría visual SDF del gripper se modela por `ur5_hand_joint={sdf_hand_pose}` y no debe confundirse con el TCP semántico.",
            f"- `world` vs `base_link`: el estado actual confirmado no es sólo Z. La traslación usada por URDF/world file es `{urdf_base_origin}`.",
            "- `NEGATE_XY`: sigue siendo consecuencia del uso de `base_link_inertia` en el solver DH; no es una preferencia opcional ni debe tratarse como flag de runtime activa.",
            "- TCP semántico vs geometría visual SDF: la mano visible del SDF y el punto de grasp semántico viven en capas distintas; la segunda es la que manda para IK, attach y carry validation.",
        ]
    )
    parts.append("")
    parts.append(preserved_block("Detalle histórico recuperado (2026-04-18)", historical_sections.get("3. Frames y Offsets", ""), shift=1).strip())
    parts.append("")

    parts.append("## 5. Geometría del Gripper, Objeto y Workspace")
    parts.append("")
    parts.append("### 5.1 Geometría actual confirmada desde código y SDF")
    parts.append("")
    parts.append(markdown_table(geometry_rows, ("Elemento", "Valor actual confirmado", "Fuente")))
    parts.append("")
    parts.append("### 5.2 Objeto, mesa, cesta y restricciones del workspace")
    parts.append("")
    parts.append(markdown_table(workspace_rows, ("Elemento", "Valor / estado", "Fuente")))
    parts.append("")
    parts.extend(
        [
            "### 5.3 Estado de validación de geometría",
            "",
            "- `tool0 -> rg2_pinch_center`: actual confirmado en URDF.",
            "- `pick_demo`: actual confirmado en world file con radio=0.025 m y longitud=0.05 m.",
            "- Reach operativo UR5 ≈ 0.85 m: histórico no revalidado en esta corrida; se conserva por utilidad operativa.",
            "- Residual DH/SDF fino subcentimétrico: pendiente de validación runtime detallada; se mantiene el conocimiento histórico sobre bias loop y residual en Z.",
            "",
            "### 5.4 Notas de tolerancia y residual geométrico",
            "",
            "- La compensación geométrica fina sigue ocurriendo en GRASP_ALIGN_IK y no debe sustituirse por asumir que el SDF visible coincide exactamente con el TCP.",
            "- `gripper_tcp_z_offset` actual en `panel_settings.yaml` vale 0.0, pero el launch conserva la capacidad de reescribir `pick_demo_anchor` en runtime; se documenta como riesgo de deriva geométrica si ese valor vuelve a cambiar.",
            "- Cuando un dato aquí no ha podido revalidarse automáticamente, se mantiene como histórico/documentado o inferido desde documentación previa, en vez de eliminarse.",
        ]
    )
    parts.append("")
    parts.append(preserved_block("Detalle histórico recuperado (2026-04-18)", historical_sections.get("4. Geometría del Gripper y del Objeto", ""), shift=1).strip())
    parts.append("")

    parts.append("## 6. Flujo Completo del Pick")
    parts.append("")
    parts.append(phase_profiles_md)
    parts.append("")
    parts.append("### 6.99 Enlace obligatorio con la validación física post-grasp")
    parts.append("")
    parts.append("Las fases `ATTACH_GATE`, `LIFT`, `CARRY / TRANSPORT` y `HOME_WITH_OBJECT` deben interpretarse junto con el bloque especializado de validación física post-grasp / carry. `ATTACH_GATE` aprueba attach lógico; la confirmación física sólo llega cuando CARRY supera sus thresholds y telemetría coherente.")
    parts.append("")
    parts.append(preserved_block("Detalle histórico recuperado (2026-04-18)", historical_sections.get("5. Flujo Completo del Pick", ""), shift=1).strip())
    parts.append("")

    parts.append("## 7. Variables de Entorno y Parámetros Críticos")
    parts.append("")
    parts.extend(
        [
            "### 7.0 Cómo leer esta sección",
            "",
            "- `Launch`: default declarado en `ur5_stack.launch.py`.",
            "- `Runtime wrapper`: override exportado por `start_panel_v2.sh` antes de lanzar el panel/stack.",
            "- `Fallback panel`: valor local que usa `panel_pick_demo.py` si lee la variable directamente y no recibe otra cosa.",
            "- El inventario exhaustivo completo se conserva en el apéndice 15.5; aquí se muestran sólo parámetros de alto impacto operativo.",
        ]
    )
    parts.append("")
    parts.append(
        env_source_markdown(
            resolved_env,
            "7.1 Geometría, descenso y alineación",
            [
                "GRASP_CONTACT_Z_OFFSET_M",
                "PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_M",
                "PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M",
                "PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_Z_STEP_M",
                "PANEL_PICK_DEMO_GRASP_DOWN_USE_MOVEIT_CARTESIAN",
                "PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT",
                "PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL",
                "PANEL_PICK_DEMO_GRASP_DOWN_STRICT_XY_TOL_M",
                "PANEL_PICK_DEMO_GRASP_DOWN_STRICT_Z_TOL_M",
                "PANEL_PICK_DEMO_GRASP_DOWN_STRICT_DIST_TOL_M",
                "PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL",
                "PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M",
                "PANEL_PICK_DEMO_ALIGN_EXIT_Z_TOL_M",
                "PANEL_PICK_DEMO_ALIGN_Z_RESIDUAL_TOL_M",
                "PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M",
                "PANEL_PICK_DEMO_PRE_CLOSE_Z_ERR_TOL_M",
            ],
        )
    )
    parts.append("")
    parts.append(
        env_source_markdown(
            resolved_env,
            "7.2 Cierre, attach y carry",
            [
                "PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC",
                "PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM",
                "PANEL_PICK_DEMO_CLOSE_XY_TOL_M",
                "PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M",
                "PANEL_PICK_DEMO_ATTACH_XY_TOL_M",
                "PANEL_PICK_DEMO_ATTACH_Z_TOL_M",
                "PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M",
                "PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M",
                "PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC",
                "PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES",
                "PANEL_PICK_DEMO_CARRY_SETTLE_SEC",
                "PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M",
                "ATTACH_BACKEND_MODE",
                "ATTACH_BACKEND_MAX_POSE_AGE_SEC",
                "ATTACH_BACKEND_FOLLOW_RATE_HZ",
                "ATTACH_BACKEND_FOLLOW_BREAK_DIST_M",
                "ATTACH_BACKEND_MAX_DIST_M",
                "ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS",
            ],
        )
    )
    parts.append("")
    parts.append(
        env_source_markdown(
            resolved_env,
            "7.3 Frescura de pose y MoveIt bridge",
            [
                "PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC",
                "PANEL_PICK_DEMO_POSE_SOURCE_TOL_M",
                "PANEL_PICK_DEMO_PHASE_JUMP_TOL_M",
                "PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SEC",
                "PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_DELTA_M",
                "PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC",
                "PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC",
                "PANEL_MOVEIT_BRIDGE_JOINT_STATE_TIMEOUT_SEC",
                "PANEL_MOVEIT_BRIDGE_JOINT_STATE_MAX_AGE_SEC",
                "PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE",
                "PANEL_MOVEIT_BRIDGE_ACCEL_SCALE",
            ],
        )
    )
    parts.append("")
    parts.append(env_discrepancy_markdown(resolved_env))
    parts.append("")
    parts.append("### 7.98 Discrepancias explícitas que no deben perderse")
    parts.append("")
    parts.extend(
        [
            f"- `PANEL_PICK_DEMO_ATTACH_XY_TOL_M`: actual en launch={attach_xy_tol}; histórico 2026-04-18 documentado como 0.008 en la tabla anterior. Mantener ambos valores con trazabilidad.",
            f"- `ATTACH_BACKEND_MAX_POSE_AGE_SEC`: launch actual={attach_backend_max_pose_age}; wrapper `start_panel_v2.sh` exporta 2.5 por defecto.",
            f"- `ATTACH_BACKEND_MAX_DIST_M`: launch actual={attach_backend_max_dist}; wrapper `start_panel_v2.sh` exporta 0.06 por defecto.",
            f"- CARRY metadata vs llamada real: metadata CARRY=({carry_phase_min_obj}, {carry_phase_min_lift}, {carry_phase_max_tcp}) frente a llamada post_grasp_lift=({post_grasp_min_obj}, {post_grasp_min_lift}, {post_grasp_max_tcp}).",
        ]
    )
    parts.append("")
    parts.append("### 7.99 Nota sobre trazabilidad histórica")
    parts.append("")
    parts.append("- El bloque histórico extenso de variables ya no se inserta entero en el cuerpo principal para evitar ruido operativo.")
    parts.append("- Su contenido queda absorbido en la columna `Histórico` y en la columna `Discrepancia` del apéndice 15.5, donde se conserva el inventario exhaustivo actual con trazabilidad de overrides y defaults previos.")
    parts.append("")

    parts.append("## 8. Validación Física Post-Grasp / Carry")
    parts.append("")
    parts.extend(
        [
            "### 8.1 Estado actual confirmado",
            "",
            "- ATTACH_GATE correcto no equivale a grasp físico confirmado.",
            f"- `follow_confirmed_only_after_carry` en fuente actual: {follow_semantic}.",
            f"- `pick_demo` sigue entrando por `demo_transport` porque `ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS={demo_transport_objects}` y el backend activa `use_world_locked_pose=True` en esa rama.",
            f"- `attach_backend_mode` por launch sigue siendo `{attach_backend_mode}` y representa la semántica base para objetos no desviados a demo transport.",
            f"- `stale_tcp_pose_soft_follow` aparece cuando `tcp_age` supera `max_pose_age_sec={attach_backend_max_pose_age}`; los logs inspeccionados muestran separación efectiva entre ticks world_locked de {world_locked_gap_txt}.",
            "",
            "### 8.2 Diagnóstico físico por métricas best_*",
            "",
            f"- `best_obj_move` bajo umbral: el objeto no se movió lo suficiente desde su pose inicial. Evidencia actual: `{static_fail_line or 'no localizada en logs inspeccionados'}`.",
            f"- `best_lift_delta < 0`: el objeto cambió de pose, pero no acompañó el lift del TCP. Evidencia actual: `{follow_lost_line or 'no localizada en logs inspeccionados'}`.",
            "- `best_tcp_dist > máximo`: hubo movimiento, pero no coherente con un grasp estable respecto al TCP; esto invalida el carry aunque el objeto se haya desplazado.",
            f"- `object_not_updated / object_never_moved`: evidencia actual `{never_moved_line or 'no localizada en logs inspeccionados'}`.",
            "",
            "### 8.3 Discrepancias abiertas que deben quedar explícitas",
            "",
            f"- Metadata de fase CARRY: min_obj_move={carry_phase_min_obj}, min_lift_delta={carry_phase_min_lift}, max_tcp_dist={carry_phase_max_tcp}.",
            f"- Llamada real `_validate_demo_carry(post_grasp_lift)`: min_obj_move={post_grasp_min_obj}, min_lift_delta={post_grasp_min_lift}, max_tcp_dist={post_grasp_max_tcp}, timeout={post_grasp_timeout}.",
            "- La telemetría `FINAL_TRACE wait_done` sigue pudiendo cerrar con timeout 1.60 aunque la llamada real sea 3.0 s; esto es inconsistencia de observabilidad, no ruido cosmético.",
        ]
    )
    parts.append("")
    parts.append(preserved_block("Bloque vigente preservado íntegro (2026-04-20)", current_sections.get("7. Nueva Sección Obligatoria — Validación Física Post-Grasp / Carry", ""), shift=1).strip())
    parts.append("")

    parts.append("## 9. Controladores, Topics y Semántica de Attach")
    parts.append("")
    parts.append("### 9.1 Controladores ros2_control activos y función")
    parts.append("")
    parts.append(markdown_table(controller_rows, ("Controlador", "Tipo", "Joints", "Topic comando / salida", "Estado")))
    parts.append("")
    parts.append("### 9.2 Topics principales y quién publica / consume")
    parts.append("")
    parts.append(markdown_table(topic_rows, ("Topic / patrón", "Dirección", "Quién publica / expone", "Quién consume")))
    parts.append("")
    parts.append("### 9.3 Semántica actual de attach")
    parts.append("")
    parts.extend(
        [
            f"- Prefijo lógico principal actual del backend: `/gripper` (`gripper_prefix=/gripper`).",
            f"- Prefijo de ancla de tool-anchor: `/gripper_anchor` (`tool_anchor_prefix=/gripper_anchor`).",
            f"- Prefijo de drop-anchor: `/drop_anchor` (`drop_anchor_prefix=/drop_anchor`).",
            f"- Modo nominal global: `{attach_backend_mode}`; pero `pick_demo` entra por demo transport y usa `world_locked` por defecto dentro del backend actual.",
            "- Esto obliga a documentar por separado attach lógico, attach por tool anchor y transporte físico real.",
        ]
    )
    parts.append("")
    parts.append("### 9.4 Comandos de verificación runtime")
    parts.append("")
    parts.append("```bash\nros2 control list_controllers\nros2 topic echo /joint_states --once\nros2 run tf2_ros tf2_echo world base_link\nros2 run tf2_ros tf2_echo base_link rg2_pinch_center\nros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray \"data: [1.18, 1.18]\" --once\nros2 topic echo /gripper/pick_demo/state --once\nros2 topic echo /gripper_anchor/pick_demo/state --once\nros2 run tf2_ros tf2_monitor world base_link rg2_pinch_center\n```")
    parts.append("")
    parts.append(preserved_block("Detalle histórico recuperado (2026-04-18)", historical_sections.get("7. Controladores, Tópicos y Validaciones", ""), shift=1).strip())
    parts.append("")

    parts.append("## 10. Bugs Conocidos y Fixes Aplicados")
    parts.append("")
    parts.append("### 10.1 Bugs / hallazgos actuales ligados a carry y attach")
    parts.append("")
    current_bug_rows = [
        ("Attach lógico aprobado pero transporte físico fallido", "ATTACH_GATE puede pasar aunque CARRY falle", "Diseño deliberadamente separado entre attach lógico y confirmación física", "Código actual + logs FINAL_TRACE/CARRY", "Mantener separación explícita; no tratar ATTACH_GATE como éxito final", "Vigente / documentado"),
        ("`world_locked` puede arrastrar una referencia retrasada", "Objeto no sigue al TCP o queda incoherente", "Demo transport usa `use_world_locked_pose=True` para `pick_demo`", f"Log world_locked: {world_locked_line or 'no localizado'}", "Ajustar frescura de pose y no ocultar la discrepancia frente a follow_tcp", "Vigente / parcialmente mitigado"),
        ("`stale_tcp_pose_soft_follow` degrada carry", "Warnings de stale y carry_follow_lost", f"Pose TCP demasiado vieja respecto a max_pose_age={attach_backend_max_pose_age}", f"Log stale: {stale_line or 'no localizado'}", "Se elevó max pose age en wrapper runtime, pero sigue siendo riesgo abierto", "Vigente / riesgo abierto"),
        ("Metadata, llamada real y telemetría de CARRY divergen", "Timeouts y thresholds no coinciden según la fuente que se mire", "Valores codificados en sitios distintos del panel", "Inspección de panel_pick_demo.py + logs helper/stack", "Conservar discrepancia anotada en vez de resumirla", "Vigente / sin cierre"),
        ("Diagnóstico best_* mal interpretado", "Se concluye grasp o fallo con criterio insuficiente", "Se ignora la relación entre best_obj_move, best_lift_delta y best_tcp_dist", "Auditorías 2026-04-18/19 y documento actual", "Mantener criterios especializados y troubleshooting específico", "Vigente / documentado"),
    ]
    parts.append(markdown_table(current_bug_rows, ("Bug / hallazgo", "Síntoma", "Causa raíz", "Cómo se detectó", "Fix / tratamiento actual", "Estado")))
    parts.append("")
    parts.append(preserved_block("Detalle histórico recuperado (2026-04-18)", historical_sections.get("8. Bugs Conocidos y Fixes Aplicados", ""), shift=1).strip())
    parts.append("")

    parts.append("## 11. Guía de Troubleshooting")
    parts.append("")
    parts.append("### 11.1 Síntomas actuales específicos del carry y attach")
    parts.append("")
    troubleshooting_items = [
        ("`carry_validation` fallido", "Revisar `FINAL_TRACE phase=CARRY`, thresholds reales `_validate_demo_carry`, y si el fallo es `object_not_updated`, `carry_follow_lost` o `tcp_dist_above_max`.", "Código actual + helper/stack logs"),
        ("`object_not_updated` / `best_obj_move=0`", "Confirmar `/gripper/pick_demo/state`, `demo_transport_set_pose_ok` y si el objeto sigue sobre la mesa. Si nunca abandona la mesa, el attach fue sólo lógico.", static_fail_line or "No localizado en logs inspeccionados"),
        ("`carry_follow_lost` / `best_lift_delta < 0`", "Comparar pose objeto vs TCP durante LIFT/CARRY y buscar `stale_tcp_pose_soft_follow` y `world_locked` retrasado.", follow_lost_line or "No localizado en logs inspeccionados"),
        ("`best_tcp_dist > máximo`", "El objeto se mueve, pero no acompaña al TCP. Verificar offset ancla, follow mode, frescura de pose y thresholds reales de carry.", world_locked_line or "No localizado en logs inspeccionados"),
        ("`stale_tcp_pose_soft_follow`", f"Inspeccionar `/world/ur5_mesa_objetos/pose/info`, TF freshness y valores `ATTACH_BACKEND_MAX_POSE_AGE_SEC` launch={attach_backend_max_pose_age} / wrapper=2.5.", stale_line or "No localizado en logs inspeccionados"),
        ("CLOSE en PEND", "Verificar gripper_controller activo, joint_states del gripper y delta de cierre. No confundir cierre medido con attach/carry confirmado.", "Bug legacy aún relevante"),
        ("UI muestra poses incoherentes", "Separar FK base_link_inertia vs TF-live base_link y revisar world->base_link actual con X=-0.85 además de Z.", "world_tf_publisher + panel traces"),
    ]
    parts.append(markdown_table(troubleshooting_items, ("Síntoma", "Diagnóstico operativo", "Referencia")))
    parts.append("")
    parts.append(preserved_block("Detalle histórico recuperado (2026-04-18)", historical_sections.get("9. Guía de Troubleshooting", ""), shift=1).strip())
    parts.append("")

    parts.append("## 12. Estado Actual del Sistema")
    parts.append("")
    parts.extend(
        [
            "- Geometría semántica actual confirmada: `tool0 -> rg2_tcp = tool0 -> rg2_pinch_center = 0 0 0.175`.",
            f"- Geometría visual SDF actual confirmada: `ur5_hand_joint relative_to={sdf_hand_rel}, pose={sdf_hand_pose}`.",
            f"- `pick_demo_anchor` runtime actual: 0 0 {runtime_pick_demo_anchor} sobre `tool0` porque `gripper_tcp_z_offset={gripper_tcp_z_offset}`.",
            f"- Backend actual: modo launch `{attach_backend_mode}`, pero `pick_demo` en `{demo_transport_objects}` usa `world_locked` en demo transport.",
            f"- Validación carry post-grasp real: timeout={post_grasp_timeout}s, min_obj_move={post_grasp_min_obj}, min_lift_delta={post_grasp_min_lift}, max_tcp_dist={post_grasp_max_tcp}.",
        ]
    )
    parts.append("")
    parts.append("## 13. Riesgos Abiertos")
    parts.append("")
    parts.extend(
        [
            "- Riesgo de leer thresholds de CARRY desde metadata de fase y no desde la llamada efectiva.",
            "- Riesgo de asumir que ATTACH_GATE correcto implica carry físico correcto.",
            "- Riesgo de degradación por pose TCP stale cuando el backend entra en soft follow con referencia vieja.",
            "- Riesgo documental si se simplifica `world -> base_link` a sólo Z y se pierde el desplazamiento actual en X.",
        ]
    )
    parts.append("")
    parts.append("## 14. Próximos Pasos")
    parts.append("")
    parts.extend(
        [
            "- Revalidar estadísticamente el carry con múltiples corridas y registrar distribución de `best_obj_move`, `best_lift_delta` y `best_tcp_dist`.",
            "- Medir con más precisión la latencia efectiva de `demo_transport_follow_tick` frente a `follow_rate_hz` nominal.",
            "- Verificar si conviene unificar metadata, llamada real y telemetría de CARRY para reducir la discrepancia de observabilidad.",
            "- Mantener una tabla de cambios de defaults entre launch, wrapper runtime y documento histórico para no perder trazabilidad en futuras revisiones.",
        ]
    )
    parts.append("")
    hist_status = historical_sections.get("10. Estado Actual del Sistema", "")
    if hist_status:
        sub_sections = {title: block for title, block in split_sections(hist_status, 3)}
        if sub_sections.get("10.3 Próximos pasos recomendados"):
            parts.append(preserved_block("Próximos pasos recuperados (2026-04-18)", sub_sections.get("10.3 Próximos pasos recomendados", ""), shift=1).strip())
            parts.append("")

    parts.append("## 15. Apéndices")
    parts.append("")
    parts.append("### 15.1 Evidencia cruzada preservada del documento vigente")
    parts.append("")
    parts.append(preserved_block("Bloque vigente preservado (evidencia cruzada)", current_sections.get("8. Evidencia Cruzada desde Auditoría e Histórico", ""), shift=1).strip())
    parts.append("")
    parts.append("### 15.2 Índice de fuentes usadas por el generador")
    parts.append("")
    parts.append(f"- `{rel(root, sources_file)}`")
    parts.append(f"- `{rel(root, tmp_dir)}`")
    parts.append("")
    parts.append("### 15.3 Confirmación de merge")
    parts.append("")
    parts.extend(
        [
            "- Se preserva íntegramente la base vigente 2026-04-20 como snapshot estable en el apéndice 15.4.",
            "- Se reincorpora el detalle histórico faltante mediante bloques recuperados desde la base 2026-04-18 en Markdown.",
            "- Las discrepancias entre valores actuales e históricos se anotan como `valor actual confirmado`, `histórico/documentado previamente` o `riesgo / discrepancia abierta`.",
        ]
    )
    parts.append("")

    parts.append("### 15.4 Snapshot íntegro de la base vigente 2026-04-20")
    parts.append("")
    if current_seed_text.strip():
        parts.append(shift_headings(strip_first_heading(current_seed_text), 1))
    else:
        parts.append("- No se pudo cargar una semilla vigente versionada para preservación íntegra.")
    parts.append("")

    parts.append("### 15.5 Inventario exhaustivo de variables actuales")
    parts.append("")
    parts.append(env_inventory_markdown(env_entries, hist_env_defaults))
    parts.append("")

    main_md = "\n".join(part.rstrip() for part in parts if part is not None).strip() + "\n"

    previous_text = previous_output_text.splitlines() if previous_output_text else []
    current_text = main_md.splitlines()
    diff_lines = list(
        difflib.unified_diff(
            previous_text,
            current_text,
            fromfile=rel(root, out_md) if out_md.exists() else (rel(root, current_seed_md) if current_seed_md and current_seed_md.exists() else "documento_previo"),
            tofile=out_md.name,
            lineterm="",
        )
    )
    diff_md = "# Diff contra documento anterior\n\n" + (
        "```diff\n" + "\n".join(diff_lines[:320]) + "\n```\n"
        if diff_lines
        else "No se detectaron diferencias de texto frente al documento previo cargado.\n"
    )

    write_text(out_md, main_md)
    write_text(out_diff, diff_md)


if __name__ == "__main__":
    main()