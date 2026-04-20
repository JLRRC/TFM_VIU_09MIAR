#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# generar_base_conocimiento_tfm.sh
# Genera la base de conocimiento técnica actualizada del TFM
# usando únicamente código, auditorías y logs locales.
#
# Artefacto principal:
#   - YYYY-MM-DD_base_conocimiento_tecnica_TFM.md
#
# Artefactos adicionales:
#   - YYYY-MM-DD_diff_contra_documento_anterior.md
#   - PDF opcional si GENERAR_BASE_CONOCIMIENTO_PDF=1 y hay motor disponible
#
# Uso:
#   chmod +x generar_base_conocimiento_tfm.sh
#   ./generar_base_conocimiento_tfm.sh
# ============================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${ROOT_DIR}/agarre_ros2_ws"
REPORTS_DIR="${ROOT_DIR}/reports"
BASE_CONOCIMIENTO_DIR="${REPORTS_DIR}/BaseDeConocimiento"
DATE_TAG="$(date +%F)"

OUT_BASENAME="${DATE_TAG}_base_conocimiento_tecnica_TFM"
OUT_MD="${BASE_CONOCIMIENTO_DIR}/${OUT_BASENAME}.md"
OUT_PDF="${BASE_CONOCIMIENTO_DIR}/${OUT_BASENAME}.pdf"
OUT_DIFF_MD="${BASE_CONOCIMIENTO_DIR}/${DATE_TAG}_diff_contra_documento_anterior.md"

REF_PDF="${BASE_CONOCIMIENTO_DIR}/2026-04-18_base_conocimiento_tecnica_TFM.pdf"
if [[ ! -e "${REF_PDF}" ]]; then
    REF_PDF="${REPORTS_DIR}/2026-04-18_base_conocimiento_tecnica_TFM.pdf"
fi

TMP_DIR="${BASE_CONOCIMIENTO_DIR}/.tmp_base_conocimiento_${DATE_TAG}"
GEN_LOG="${TMP_DIR}/generacion.log"
SOURCES_FILE="${TMP_DIR}/fuentes_verificadas.txt"
HTML_TMP="${TMP_DIR}/${OUT_BASENAME}.html"

LAUNCH_FILE="${WS_DIR}/src/ur5_bringup/launch/ur5_stack.launch.py"
PANEL_FILE="${WS_DIR}/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py"
BACKEND_FILE="${WS_DIR}/src/ur5_tools/ur5_tools/gripper_attach_backend.py"
URDF_FILE="${WS_DIR}/src/ur5_description/urdf/ur5.urdf.xacro"
SDF_FILE="${WS_DIR}/models/ur5_rg2/model.sdf"

mkdir -p "${REPORTS_DIR}"
mkdir -p "${BASE_CONOCIMIENTO_DIR}"
mkdir -p "${TMP_DIR}"
: > "${GEN_LOG}"

log() {
  local msg="[BASE-CONOCIMIENTO] $*"
  printf '%s\n' "$msg"
  printf '%s\n' "$msg" >> "${GEN_LOG}"
}

warn() {
  local msg="[BASE-CONOCIMIENTO][WARN] $*"
  printf '%s\n' "$msg"
  printf '%s\n' "$msg" >> "${GEN_LOG}"
}

fail() {
  local msg="[BASE-CONOCIMIENTO][ERROR] $*"
  printf '%s\n' "$msg" >&2
  printf '%s\n' "$msg" >> "${GEN_LOG}"
  exit 1
}

check_path() {
  local p="$1"
  [[ -e "$p" ]] || fail "No existe: $p"
}

check_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "Falta dependencia: $1"
}

render_pdf() {
  if command -v pandoc >/dev/null 2>&1 && command -v xelatex >/dev/null 2>&1; then
    log "Renderizando PDF con pandoc + xelatex..."
    pandoc "${OUT_MD}" \
      -o "${OUT_PDF}" \
      --from gfm \
      --pdf-engine=xelatex \
      -V geometry:margin=2.2cm \
      -V fontsize=11pt \
      -V colorlinks=true
    return 0
  fi

  if command -v pandoc >/dev/null 2>&1 && command -v wkhtmltopdf >/dev/null 2>&1; then
    log "Renderizando PDF con pandoc + wkhtmltopdf..."
    pandoc "${OUT_MD}" -o "${HTML_TMP}" --from gfm
    wkhtmltopdf "${HTML_TMP}" "${OUT_PDF}"
    return 0
  fi

  return 1
}

log "Verificando rutas..."
check_path "${ROOT_DIR}"
check_path "${WS_DIR}"
check_path "${REPORTS_DIR}"
check_path "${BASE_CONOCIMIENTO_DIR}"
check_path "${LAUNCH_FILE}"
check_path "${PANEL_FILE}"
check_path "${BACKEND_FILE}"
check_path "${URDF_FILE}"
check_path "${SDF_FILE}"

if [[ ! -e "${REF_PDF}" ]]; then
  warn "No existe PDF de referencia: ${REF_PDF}. El diff se basará solo en fuentes locales disponibles."
fi

log "Verificando herramientas..."
check_cmd python3
check_cmd cp
check_cmd date

log "Generando documento técnico local en Markdown..."
python3 - "${ROOT_DIR}" "${OUT_MD}" "${OUT_DIFF_MD}" "${REF_PDF}" "${TMP_DIR}" "${DATE_TAG}" "${SOURCES_FILE}" <<'PY' || fail "Falló la generación local del Markdown"
from __future__ import annotations

import difflib
import re
import sys
import textwrap
from pathlib import Path


root = Path(sys.argv[1])
out_md = Path(sys.argv[2])
out_diff = Path(sys.argv[3])
ref_pdf = Path(sys.argv[4])
tmp_dir = Path(sys.argv[5])
date_tag = sys.argv[6]
sources_file = Path(sys.argv[7])


required_files = {
    "launch": root / "agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py",
    "panel": root / "agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py",
    "backend": root / "agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_backend.py",
    "urdf": root / "agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro",
    "sdf": root / "agarre_ros2_ws/models/ur5_rg2/model.sdf",
}

for label, path in required_files.items():
    if not path.exists():
        raise SystemExit(f"No existe la fuente requerida {label}: {path}")


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def rel(path: Path | None) -> str:
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


def strip_leading_doc_indent(text: str) -> str:
    lines = text.splitlines()
    return "\n".join(line[4:] if line.startswith("    ") else line for line in lines)


launch_text = read_text(required_files["launch"])
panel_text = read_text(required_files["panel"])
backend_text = read_text(required_files["backend"])
urdf_text = read_text(required_files["urdf"])
sdf_text = read_text(required_files["sdf"])

audit_doc = root / "auditoria/informe_fix_visual_grasp_20260419.md"
reports_status = root / "reports/evidence/ros2/moveit2_system_status.json"

audit_text = read_text(audit_doc) if audit_doc.exists() else ""
reports_status_text = read_text(reports_status) if reports_status.exists() else ""

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

sdf_hand_rel, sdf_hand_pose = capture_groups(
    r'<joint name="ur5_hand_joint"[^>]*>.*?<pose relative_to="([^"]+)">([^<]+)</pose>',
    sdf_text,
    "pose de ur5_hand_joint",
)

def launch_env_default(name: str) -> str:
    return capture(
        rf'"{re.escape(name)}".*?os\.environ\.get\("{re.escape(name)}", "([^"]+)"\)',
        launch_text,
        f"default de {name}",
    )


attach_xy_tol = launch_env_default("PANEL_PICK_DEMO_ATTACH_XY_TOL_M")
attach_z_tol = launch_env_default("PANEL_PICK_DEMO_ATTACH_Z_TOL_M")
attach_follow_max = launch_env_default("PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M")
attach_rel_drift = launch_env_default("PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M")
attach_stable_window = launch_env_default("PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC")
attach_min_samples = launch_env_default("PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES")
attach_tf_visual_gap = launch_env_default("PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M")
gripper_closed_thr = launch_env_default("PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M")

attach_backend_mode = capture(
    r'DeclareLaunchArgument\(\s*"attach_backend_mode",\s*default_value=os\.environ\.get\("ATTACH_BACKEND_MODE", "([^"]+)"\)',
    launch_text,
    "attach_backend_mode",
)
attach_backend_max_pose_age = capture(
    r'DeclareLaunchArgument\(\s*"attach_backend_max_pose_age_sec",\s*default_value=os\.environ\.get\("ATTACH_BACKEND_MAX_POSE_AGE_SEC", "([^"]+)"\)',
    launch_text,
    "attach_backend_max_pose_age_sec",
)
attach_backend_follow_rate = capture(
    r'DeclareLaunchArgument\(\s*"attach_backend_follow_rate_hz",\s*default_value=os\.environ\.get\("ATTACH_BACKEND_FOLLOW_RATE_HZ", "([^"]+)"\)',
    launch_text,
    "attach_backend_follow_rate_hz",
)
attach_backend_max_dist = capture(
    r'DeclareLaunchArgument\(\s*"attach_backend_max_dist_m",\s*default_value=os\.environ\.get\("ATTACH_BACKEND_MAX_DIST_M", "([^"]+)"\)',
    launch_text,
    "attach_backend_max_dist_m",
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
    "default de PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M",
)

carry_trace_start_timeout = capture(
    r'_final_phase_trace\(\s*"CARRY",\s*event="wait_start".*?timeout_sec="([^"]+)"',
    panel_text,
    "timeout trazado de inicio de CARRY",
)
carry_trace_done_timeout = capture(
    r'_final_phase_trace\(\s*"CARRY",\s*event="wait_done".*?timeout_sec="([^"]+)"',
    panel_text,
    "timeout trazado de cierre de CARRY",
)

report_follow_role = None
if reports_status_text:
    report_follow_role = capture(
        r'"role": "([^"]*follow_tcp[^"]*)"',
        reports_status_text,
        "rol follow_tcp en reports",
        flags=re.S,
    )

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

helper_logs = sorted((root / "auditoria").glob("**/helper.log"))
stack_logs = sorted((root / "auditoria").glob("**/stack.log")) + sorted(
    (root / "historico").glob("stack_manual_*.log")
)
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
    lambda line: (
        "carry_follow_lost=true" in line
        and "detail=object_never_moved" in line
    ),
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

previous_md_candidates = [
    path
    for path in sorted((root / "reports" / "BaseDeConocimiento").glob("*_base_conocimiento_tecnica_TFM.md"))
    + sorted(root.glob("*_base_conocimiento_tecnica_TFM.md"))
    + sorted((root / "reports").glob("*_base_conocimiento_tecnica_TFM.md"))
    if path.resolve() != out_md.resolve()
]
previous_md = previous_md_candidates[-1] if previous_md_candidates else None

sources_used = [
    required_files["launch"],
    required_files["panel"],
    required_files["backend"],
    required_files["urdf"],
    required_files["sdf"],
]
for candidate in [audit_doc if audit_doc.exists() else None, reports_status if reports_status.exists() else None, static_fail_path, follow_lost_path, never_moved_path, stale_path, world_locked_path, previous_md, ref_pdf if ref_pdf.exists() else None]:
    if candidate is not None and candidate not in sources_used:
        sources_used.append(candidate)

sources_file.write_text(
    "\n".join(rel(path) for path in sources_used),
    encoding="utf-8",
)

world_locked_gap_txt = "no confirmado en logs"
if world_locked_gap is not None:
    world_locked_gap_txt = f"{world_locked_gap:.3f} s entre dos ticks consecutivos del mismo log"

evidence_static = static_fail_line or "No se encontró una línea con best_obj_move=0.000 en los logs inspeccionados."
evidence_follow_lost = follow_lost_line or "No se encontró una línea con carry_detail=carry_follow_lost en los logs inspeccionados."
evidence_stale = stale_line or "No se encontró warning stale_tcp_pose_soft_follow en los logs inspeccionados."
evidence_world_locked = world_locked_line or "No se encontró demo_transport_follow_tick mode=world_locked en los logs inspeccionados."
evidence_never_moved = never_moved_line or "No se encontró la traza detail=object_never_moved en los logs inspeccionados."

follow_semantic = "confirmado" if "follow_confirmed_only_after_carry" in panel_text else "no confirmado"

main_md = strip_leading_doc_indent(textwrap.dedent(
    f"""
    # Base de Conocimiento Técnica — Sistema UR5 + RG2 Pick & Place

    Fecha de generación: {date_tag}

    Documento generado automáticamente a partir del estado real del workspace, sin depender de servicios externos. El artefacto principal de esta ejecución es Markdown; el PDF queda opcional y desactivado por defecto.

    ## 1. Resumen Ejecutivo del Sistema

    - Workspace inspeccionado: {root}
    - Launch principal revisado: {rel(required_files['launch'])}
    - Orquestador del pick revisado: {rel(required_files['panel'])}
    - Backend de attach/transporte revisado: {rel(required_files['backend'])}
    - Estado actual confirmado: el pipeline separa el attach lógico del transporte físico. ATTACH_GATE puede aprobar y dejar el objeto en estado lógico CARRIED, pero la confirmación física solo llega cuando CARRY pasa.
    - Artefacto principal generado por este script: {out_md.name}

    ## 2. Fuentes Verificadas

    ### 2.1 Código fuente inspeccionado

    - {rel(required_files['launch'])}
    - {rel(required_files['panel'])}
    - {rel(required_files['backend'])}
    - {rel(required_files['urdf'])}
    - {rel(required_files['sdf'])}

    ### 2.2 Auditorías, histórico y reports usados

    - {rel(audit_doc) if audit_doc.exists() else 'auditoria/informe_fix_visual_grasp_20260419.md no disponible'}
    - {rel(static_fail_path)}
    - {rel(follow_lost_path)}
    - {rel(stale_path)}
    - {rel(world_locked_path)}
    - {rel(reports_status) if reports_status.exists() else 'reports/evidence/ros2/moveit2_system_status.json no disponible'}
    - {rel(ref_pdf) if ref_pdf.exists() else 'reports/2026-04-18_base_conocimiento_tecnica_TFM.pdf no disponible'}

    ## 3. Arquitectura del Proyecto

    - El launch del stack publica el entorno de pick directo y lanza el backend de attach cuando launch_attach_backend=true.
    - El panel ejecuta la secuencia de grasp, cierre, attach, lift, carry, transporte a cesta y release.
    - El backend implementa dos comportamientos conceptuales distintos:
      - follow_tcp como modo general de attach cinemático.
      - demo_transport para pick_demo, con rama world_locked activa por defecto.
    - Según {rel(reports_status) if reports_status.exists() else 'reports'}, {report_follow_role or 'no confirmado en reports'}, lo que refuerza que follow_tcp sigue siendo la semántica base del backend fuera del caso pick_demo.

    ## 4. Frames y Offsets

    - Offset semántico tool0 -> rg2_tcp en URDF: {rg2_tcp_xyz}
    - Offset semántico tool0 -> rg2_pinch_center en URDF: {rg2_pinch_xyz}
    - Pose actual de ur5_hand_joint en SDF: relative_to={sdf_hand_rel}, pose={sdf_hand_pose}
    - Lectura operativa:
      - La cadena TF semántica sitúa tanto rg2_tcp como rg2_pinch_center a +0.175 m de tool0.
      - El modelo SDF sigue describiendo la mano visible desde wrist_3_link mediante ur5_hand_joint; esto debe tratarse como geometría visual de Gazebo, no como definición del TCP semántico.
      - La consistencia exacta entre geometría visible y TCP semántico no se recalcula de nuevo dentro de este script; por tanto cualquier afirmación visual fina debe considerarse pendiente de validación runtime si se necesita precisión subcentimétrica.

    ## 5. Flujo Completo del Pick

    El pipeline directo actual, consolidando panel y snapshots históricos, recorre estas fases lógicas:

    1. Aproximación y descenso al grasp.
    2. PRE_CLOSE y CLOSE de la pinza.
    3. ATTACH_GATE para aceptar el attach lógico con chequeo geométrico y ventana temporal.
    4. LIFT corto post-grasp.
    5. CARRY para validar movimiento físico observable del objeto.
    6. HOME_WITH_OBJECT y transporte hacia cesta si el carry fue válido.
    7. RELEASE y retorno final.

    El punto crítico no es el attach lógico, sino la transición ATTACH_GATE -> CARRY.

    ## 6. Variables de Entorno y Parámetros Críticos

    ### 6.1 ATTACH_GATE por defecto en el launch

    - PANEL_PICK_DEMO_ATTACH_XY_TOL_M = {attach_xy_tol}
    - PANEL_PICK_DEMO_ATTACH_Z_TOL_M = {attach_z_tol}
    - PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M = {attach_follow_max}
    - PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M = {attach_rel_drift}
    - PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC = {attach_stable_window}
    - PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES = {attach_min_samples}
    - PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M = {attach_tf_visual_gap}
    - PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M = {gripper_closed_thr}
    - PANEL_PICK_DEMO_ATTACH_SETTLE_SEC en panel = {attach_settle_sec}
    - PANEL_PICK_DEMO_POST_ATTACH_HOLD_SEC en panel = {post_attach_hold_sec}

    ### 6.2 Backend de attach/transporte

    - attach_backend_mode por defecto en el launch = {attach_backend_mode}
    - ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS por defecto = {demo_transport_objects}
    - attach_backend_max_pose_age_sec por defecto = {attach_backend_max_pose_age}
    - attach_backend_follow_rate_hz por defecto = {attach_backend_follow_rate}
    - attach_backend_max_dist_m por defecto = {attach_backend_max_dist}
    - El código del backend marca pick_demo como demo_transport y, al activarlo, fija use_world_locked_pose=True.
    - Esto significa que pick_demo no sigue la misma ruta que el resto de objetos cuando entra por demo_transport; el modo nominal del backend sigue siendo follow_tcp, pero el objeto demo entra por world_locked salvo reconfiguración explícita.

    ## 7. Nueva Sección Obligatoria — Validación Física Post-Grasp / Carry

    ### 7.1 ATTACH_GATE correcto vs carry_validation fallido

    - ATTACH_GATE correcto no equivale a grasp físico confirmado.
    - En el código actual, tras ATTACH_GATE se deja follow_confirmed=false y se registra la nota follow_confirmed_only_after_carry. Estado de esta afirmación en fuente: {follow_semantic}.
    - La confirmación física solo se marca cuando _validate_demo_carry(...) retorna OK durante CARRY.
    - Por tanto:
      - ATTACH_GATE correcto = attach lógico aceptado, objeto publicado/kinemático disponible, proximidad validada en ventana temporal.
      - carry_validation fallido = el objeto no demuestra transporte físico suficiente con respecto al TCP y/o a la elevación esperada.

    ### 7.2 Timeout específico y thresholds activos en post_grasp_lift

    #### Llamada real activa en panel_pick_demo.py para phase=post_grasp_lift

    - timeout_sec = {post_grasp_timeout}
    - min_obj_move_m = {post_grasp_min_obj}
    - min_lift_delta_m = {post_grasp_min_lift}
    - max_tcp_dist_m = {post_grasp_max_tcp}
    - live_world_fn = _fresh_gazebo_object_world
    - Espera previa de asentamiento del carry: PANEL_PICK_DEMO_CARRY_SETTLE_SEC = {carry_settle_sec}

    #### Llamada real activa en panel_pick_demo.py para phase=home_with_object

    - timeout_sec = {home_timeout}
    - min_obj_move_m = {home_min_obj}
    - min_lift_delta_m = {home_min_lift}
    - max_tcp_dist_m = env PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M, default {home_max_tcp}

    #### Discrepancias internas todavía presentes en el código

    - Metadata de _phase_begin("CARRY"): min_obj_move_m={carry_phase_min_obj}, min_lift_delta_m={carry_phase_min_lift}, max_tcp_dist_m={carry_phase_max_tcp}
    - Llamada real a _validate_demo_carry para post_grasp_lift: min_obj_move_m={post_grasp_min_obj}, min_lift_delta_m={post_grasp_min_lift}, max_tcp_dist_m={post_grasp_max_tcp}
    - FINAL_TRACE de inicio de CARRY usa timeout={carry_trace_start_timeout}
    - FINAL_TRACE de cierre de CARRY sigue codificando timeout={carry_trace_done_timeout}
    - Conclusión: hoy existen discrepancias entre metadata de fase, comentarios adyacentes, llamada efectiva y timeout trazado en cierre.

    ### 7.3 Modos de transporte y seguimiento

    - follow_tcp:
      - Es el modo base del backend por launch.
      - Sigue el TCP con offset relativo para objetos que no entran por demo_transport.
      - También es la semántica que describe {rel(reports_status) if reports_status.exists() else 'reports'}.
    - world_locked:
      - Se activa para pick_demo porque ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS incluye pick_demo y _activate_demo_transport_attachment fija use_world_locked_pose=True.
      - En esta rama el backend calcula desired = tcp + world_offset fijo y mantiene la orientación world_q* almacenada en el attach.
      - Evidencia literal:

        ```text
        {evidence_world_locked}
        ```

    ### 7.4 Efecto de world_locked cuando el objeto no sigue realmente al TCP

    - Si la pose del TCP llega vieja o el backend no actualiza la pose del objeto a tiempo, world_locked sigue publicando ticks sobre una referencia retrasada.
    - En ese escenario puede verse attach lógico correcto pero carry físico fallido.
    - El efecto práctico observado es uno de estos dos:
      - El objeto nunca abandona la mesa: best_obj_move=0.000 y best_lift_delta=0.000.
      - El objeto sí cambia de pose, pero no acompaña el lift del TCP: best_lift_delta<0 y best_tcp_dist crece por encima del máximo.

    ### 7.5 Impacto de stale_tcp_pose_soft_follow y relación con latencia/ventana de validación

    - El backend emite stale_tcp_pose_soft_follow cuando tcp_age supera attach_backend_max_pose_age_sec={attach_backend_max_pose_age}, pero aún no llega al hard_age calculado en el propio backend.
    - Mientras no se alcance el hard_age, la ruta de soft-follow sigue intentando mover el objeto con una pose de TCP envejecida.
    - Evidencia literal:

        ```text
        {evidence_stale}
        ```

    - La propia lógica del panel añade una espera previa CARRY_SETTLE de {carry_settle_sec} s y comenta que el backend puede actualizar a ritmo efectivo cercano a 1 Hz en headless, a pesar de que el parámetro nominal attach_backend_follow_rate_hz está en {attach_backend_follow_rate} Hz.
    - En los logs de world_locked inspeccionados, la separación observada entre dos ticks consecutivos es {world_locked_gap_txt}, lo que refuerza que la latencia efectiva del transporte puede dominar la ventana de validación física.
    - Relación operativa confirmada:
      - TCP fresco -> menor riesgo de falso stale y de arrastrar una referencia retrasada.
      - Backend lento o stale -> mayor probabilidad de que CARRY evalúe la pose del objeto antes de que world_locked/follow_tcp haya reflejado el lift real.
      - Ventana de validación corta + TCP viejo -> más fallos del tipo object_not_updated o carry_follow_lost.

    ### 7.6 Discrepancia entre timeout configurado y timeout observado en logs

    - Timeout real pasado a _validate_demo_carry en post_grasp_lift: {post_grasp_timeout} s.
    - Timeout trazado al inicio de CARRY: {carry_trace_start_timeout} s.
    - Timeout trazado al cierre de CARRY en el código: {carry_trace_done_timeout} s.
    - En helper.log históricos recientes reaparece ese cierre con timeout=1.60 incluso cuando el código actual llama con 3.0 s.
    - Esto debe tratarse como inconsistencia abierta de telemetría, no como un simple detalle cosmético, porque dificulta correlacionar código y ejecución real.

    ### 7.7 Criterios de diagnóstico solicitados

    - Si best_obj_move < umbral:
      - Diagnóstico primario: el objeto no se ha movido lo suficiente desde su pose inicial.
      - Caso extremo: object_not_updated, típico cuando el objeto permanece sobre la mesa.
      - Evidencia:

        ```text
        {evidence_static}
        ```

      - Evidencia complementaria:

        ```text
        {evidence_never_moved}
        ```

    - Si best_lift_delta < 0:
      - Diagnóstico primario: el objeto cambió de pose, pero no acompañó el lift del TCP; la elevación respecto a la referencia inicial fue negativa.
      - Suele venir combinado con carry_follow_lost y/o tcp_dist_above_max.
      - Evidencia:

        ```text
        {evidence_follow_lost}
        ```

    - Si best_tcp_dist > máximo:
      - Diagnóstico primario: el objeto quedó demasiado lejos del TCP durante el tramo que debería demostrar transporte físico coherente.
      - Esto invalida el carry incluso si hubo movimiento del objeto, porque el movimiento no es coherente con un grasp estable.

    ### 7.8 Separación conceptual pedida

    - CAPA 1 = geometría / unión visual
      - Incluye URDF, SDF, meshes, offsets visuales y consistencia tool0/TCP/gripper visible.
      - Puede dar una pinza visualmente razonable aunque todavía no exista prueba física de transporte.
    - CAPA 2 = attach / carry / seguimiento TCP / validación física
      - Incluye ATTACH_GATE, attach backend, demo_transport/follow_tcp, stale_tcp_pose_soft_follow y _validate_demo_carry.
      - Es la capa que decide si el objeto fue realmente transportado con el TCP o si solo hubo attach lógico/kinemático.

    ## 8. Evidencia Cruzada desde Auditoría e Histórico

    ### 8.1 Síntesis en auditoría

    {"```text\n" + audit_failure_snippet + "\n```" if audit_failure_snippet else '- No se encontró en auditoría una síntesis textual exacta con las métricas best_obj_move/best_lift_delta/best_tcp_dist.'}

    ### 8.2 Patrones confirmados en logs

    - Patrón A: objeto inmóvil tras lift lógico.
    - Patrón B: objeto con movimiento pero sin lift coherente y alejamiento respecto al TCP.
    - Patrón C: warnings stale_tcp_pose_soft_follow intercalados con world_locked, señal de que la frescura de la pose del TCP influye directamente en el carry observado.

    ## 9. Topics y Semántica de Attach

    - El backend publica y consume topics con el patrón /gripper/<objeto>/attach, /gripper/<objeto>/detach y /gripper/<objeto>/state.
    - La decisión de ruta del attach distingue entre demo_transport, tool_anchor y follow_tcp.
    - Para pick_demo, la configuración actual del launch lo encamina por demo_transport.

    ## 10. Estado Actual del Sistema

    - Confirmado en código:
      - Markdown debe ser el artefacto principal del script de base de conocimiento.
      - ATTACH_GATE y CARRY representan capas distintas y no deben confundirse.
      - pick_demo entra por demo_transport y usa world_locked por defecto.
      - Hay incoherencias internas entre la telemetría y la llamada real de carry validation.
    - Confirmado en logs/auditoría:
      - Existen fallos repetidos de post_grasp_lift con objeto inmóvil.
      - Existen fallos repetidos con best_lift_delta negativo y tcp_dist_above_max.
      - stale_tcp_pose_soft_follow aparece en histórico reciente con edades superiores a max_pose_age_sec.
    - Pendiente de validación adicional si se necesita cierre definitivo:
      - Medida estadística completa de latencia backend por campaña, no solo muestras puntuales.
      - Revalidación visual actual fina de la geometría visible RG2 frente al TCP semántico en la rama vigente.

    ## 11. Riesgos Abiertos

    - Riesgo de interpretación errónea si se usan los valores de metadata de CARRY en vez de la llamada real a _validate_demo_carry.
    - Riesgo de telemetría inconsistente mientras FINAL_TRACE siga cerrando con timeout={carry_trace_done_timeout} para CARRY.
    - Riesgo de falsos diagnósticos si se observa solo ATTACH_GATE y no se revisa la validación física posterior.
    - Riesgo de degradación por frescura insuficiente del TCP cuando aparecen warnings stale_tcp_pose_soft_follow.

    ## 12. Apéndice de Fuentes Usadas

    El índice completo de fuentes usadas por este generador queda en:

    - {rel(sources_file)}
    - {rel(tmp_dir / 'generacion.log')}
    """
).strip()) + "\n"

if previous_md is not None and previous_md.exists():
    previous_text = read_text(previous_md).splitlines()
    current_text = main_md.splitlines()
    diff_lines = list(
        difflib.unified_diff(
            previous_text,
            current_text,
            fromfile=rel(previous_md),
            tofile=out_md.name,
            lineterm="",
        )
    )
    short_diff = diff_lines[:240]
    diff_md = "# Diff contra documento anterior\n\n" + (
        "```diff\n" + "\n".join(short_diff) + "\n```\n"
        if short_diff
        else "No se detectaron diferencias de texto frente al markdown anterior.\n"
    )
else:
    diff_md = textwrap.dedent(
        f"""
        # Diff contra documento anterior

        - Referencia documental disponible en el workspace: {rel(ref_pdf) if ref_pdf.exists() else 'no disponible'}
        - No existe un markdown anterior de base de conocimiento dentro del workspace para diff línea a línea.
        - Diferencias confirmadas en esta generación:
                    - El artefacto principal pasa a generarse dentro de reports/BaseDeConocimiento.
          - El script deja de depender de claude para generar el documento.
          - El PDF queda opcional y desactivado por defecto.
          - Se añade una subsección explícita de validación física post-grasp / carry basada en código y logs reales.
          - Se documenta que pick_demo entra por demo_transport con mode=world_locked aunque el attach_backend_mode por defecto siga siendo follow_tcp.
          - Se documenta la discrepancia actual entre timeout real de carry, metadata de fase y timeout observado en trazas de cierre.
        """
    ).strip() + "\n"

out_md.write_text(main_md, encoding="utf-8")
out_diff.write_text(diff_md, encoding="utf-8")
PY

[[ -s "${OUT_MD}" ]] || fail "No se generó el documento principal en Markdown"
[[ -s "${OUT_DIFF_MD}" ]] || fail "No se generó el diff en Markdown"

PDF_REQUESTED="${GENERAR_BASE_CONOCIMIENTO_PDF:-0}"
PDF_REQUESTED="$(printf '%s' "${PDF_REQUESTED}" | tr '[:upper:]' '[:lower:]')"

case "${PDF_REQUESTED}" in
  1|true|yes|on)
    if ! render_pdf; then
      fail "PDF solicitado pero no hay motor disponible. Instala: pandoc + xelatex (o pandoc + wkhtmltopdf)"
    fi
    [[ -s "${OUT_PDF}" ]] || fail "No se generó el PDF solicitado"
    ;;
  *)
    log "PDF desactivado por defecto. Usa GENERAR_BASE_CONOCIMIENTO_PDF=1 para habilitarlo."
    rm -f "${OUT_PDF}" "${HTML_TMP}"
    ;;
esac

log "Artefactos finales generados en ${BASE_CONOCIMIENTO_DIR}..."

log "Proceso terminado"
echo
echo "Generados:"
echo "  - ${OUT_MD}"
echo "  - ${OUT_DIFF_MD}"
if [[ -s "${OUT_PDF}" ]]; then
  echo "  - ${OUT_PDF}"
fi
echo "  - ${GEN_LOG}"
echo "  - ${SOURCES_FILE}"
echo
echo "Artefactos finales en reports/BaseDeConocimiento:"
echo "  - ${OUT_MD}"
echo "  - ${OUT_DIFF_MD}"
if [[ -s "${OUT_PDF}" ]]; then
    echo "  - ${OUT_PDF}"
fi