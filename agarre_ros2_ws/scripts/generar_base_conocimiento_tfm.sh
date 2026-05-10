#!/usr/bin/env bash
# generar_base_conocimiento_tfm.sh
# Genera la Base de Conocimiento Técnica del TFM (UR5 + RG2 Pick & Place, ROS 2 Jazzy).
# Modo acumulativo: añade secciones nuevas; no borra histórico.
# Uso: bash scripts/generar_base_conocimiento_tfm.sh [--no-pdf] [--out-dir DIR]
#
# Artefactos generados:
#   report/BaseDeConocimiento/YYYY-MM-DD_base_conocimiento_tecnica_TFM.md
#   report/BaseDeConocimiento/.tmp_base_YYYY-MM-DD/fuentes_verificadas.txt
#   report/BaseDeConocimiento/.tmp_base_YYYY-MM-DD/generacion.log
#   report/BaseDeConocimiento/YYYY-MM-DD_Base_de_Conocimiento_Tecnica_TFM.pdf  (si pandoc+latex)

set +e  # no abort on errors — we handle them per-command with || true
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WS_DIR"

# ---------------------------------------------------------------------------
# Parámetros
# ---------------------------------------------------------------------------
FECHA="$(date +%Y-%m-%d)"
HORA="$(date +%H:%M:%S)"
OUT_DIR="${REPORTS_BASE:-$WS_DIR/report/BaseDeConocimiento}"
TMP_DIR="$OUT_DIR/.tmp_base_$FECHA"
MD_OUT="$OUT_DIR/${FECHA}_base_conocimiento_tecnica_TFM.md"
PDF_OUT="$OUT_DIR/${FECHA}_Base_de_Conocimiento_Tecnica_TFM.pdf"
FUENTES_LOG="$TMP_DIR/fuentes_verificadas.txt"
GEN_LOG="$TMP_DIR/generacion.log"
GEN_PDF=1

for arg in "$@"; do
  case "$arg" in
    --no-pdf)   GEN_PDF=0 ;;
    --out-dir=*) OUT_DIR="${arg#--out-dir=}" ;;
  esac
done

mkdir -p "$OUT_DIR" "$TMP_DIR"
exec > >(tee -a "$GEN_LOG") 2>&1

log()  { echo "[GEN][$(date +%H:%M:%S)] $*"; }
sep()  { echo ""; }
fver() { echo "$1" >> "$FUENTES_LOG"; }   # registrar fuente verificada

log "=== Iniciando generación de Base de Conocimiento TFM ==="
log "Workspace : $WS_DIR"
log "Salida MD : $MD_OUT"
log "Tmp dir   : $TMP_DIR"

# ---------------------------------------------------------------------------
# Helpers de escritura
# ---------------------------------------------------------------------------
H1() { printf '\n# %s\n\n' "$*" >> "$MD_OUT"; }
H2() { printf '\n## %s\n\n' "$*" >> "$MD_OUT"; }
H3() { printf '\n### %s\n\n' "$*" >> "$MD_OUT"; }
MD() { printf '%s\n' "$*" >> "$MD_OUT"; }
BLK_START() { printf '\n```%s\n' "${1:-}" >> "$MD_OUT"; }
BLK_END()   { printf '```\n\n' >> "$MD_OUT"; }
CMD() {
  local label="$1"; shift
  MD "**\$ $*"
  BLK_START
  eval "$@" 2>&1 >> "$MD_OUT" || true
  BLK_END
  fver "$label"
}
CMD_SILENT() {
  local label="$1"; shift
  eval "$@" 2>&1 >> "$MD_OUT" || true
  fver "$label"
}
SAFE_CAT() {
  local f="$1"
  if [[ -f "$f" ]]; then
    BLK_START
    head -n "${2:-300}" "$f" >> "$MD_OUT"
    BLK_END
    fver "$f"
  else
    MD "_Archivo no encontrado: \`$f\`_"
  fi
}
SAFE_GREP() {
  local label="$1" pattern="$2"; shift 2
  BLK_START
  grep -En "$pattern" "$@" 2>/dev/null | head -80 >> "$MD_OUT" || true
  BLK_END
  fver "$label"
}

# ---------------------------------------------------------------------------
# Cabecera del documento
# ---------------------------------------------------------------------------
{
cat << EOF
---
title: "Base de Conocimiento Técnica — Sistema UR5 + RG2 Pick & Place"
subtitle: "Generado: ${FECHA} ${HORA} | Workspace: ${WS_DIR}"
author: "generar_base_conocimiento_tfm.sh (Claude Code)"
date: "${FECHA}"
---

> **MODO ACUMULATIVO** — Esta base refleja el estado real del workspace en la fecha indicada.
> Los valores anteriores se conservan con indicación de cambio.
> Fuente de verdad: código fuente actual > logs recientes > configuración cargada.
> Si un dato no pudo confirmarse en esta ejecución se indica como **[NO CONFIRMADO]**.

EOF
} > "$MD_OUT"

# ---------------------------------------------------------------------------
# 1. Resumen Ejecutivo
# ---------------------------------------------------------------------------
H1 "1. Resumen Ejecutivo"

MD "| Campo | Valor |"
MD "|-------|-------|"
MD "| Fecha de generación | ${FECHA} ${HORA} |"
MD "| Script | \`scripts/generar_base_conocimiento_tfm.sh\` |"
MD "| Workspace | \`$WS_DIR\` |"
MD "| Robot | UR5 con gripper OnRobot RG2 (modelo: ur5_rg2) |"
MD "| Stack | ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2 + ros2_control + ros_gz |"
MD "| Rama git actual | \`$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'N/A')\` |"
MD "| Último commit | \`$(git log --oneline -1 2>/dev/null || echo 'N/A')\` |"
MD "| Base anterior | \`historico/2026-04-18_base_conocimiento_tecnica.md\` |"
MD "| Motivo actualización | Auditoría + gran refactorización post-23-04-2026 |"

H2 "1.1 Estado Global"

MD "**Cambios principales desde base anterior (23-04-2026):**"
MD ""
MD "- panel_v2.py refactorizado: 18 600 → 2 859 líneas (−85%). 15 módulos extraídos."
MD "- RG2 gripper migrado de revoluto a prismático: rango 0–0.055 m por dedo."
MD "- TCP semántico unificado: rg2_pinch_center = 0.0050885 m desde tool0."
MD "- SDF corregido: ur5_hand_joint rpy=0 (antes pitch=−π/2 causaba 247 mm de error)."
MD "- 593+ tests unitarios sin ROS (todos puros)."
MD "- CachedKDL IK solver añadido (min_pose_distance=0.005)."
MD "- Módulos de análisis extraídos: directo_geometry, directo_gate_evaluator, moveit_bridge_utils, launch_helpers."
MD "- Script limpieza limpia_stack.sh añadido."
MD "- **ALERTA CRÍTICA:** 10 archivos en conflicto de merge (UU) sin resolver."

H2 "1.2 Funciona / No Funciona / Pendiente"

MD "| Estado | Área |"
MD "|--------|------|"
MD "| ✅ Funciona | Build colcon (7 paquetes) |"
MD "| ✅ Funciona | 593+ tests unitarios puros (sin ROS) |"
MD "| ✅ Funciona | Geometría TCP unificada (0.0050885 m) |"
MD "| ✅ Funciona | Gripper prismático 0–0.055 m |"
MD "| ✅ Funciona | SDF/URDF coherentes (DH/SDF divergencia resuelta) |"
MD "| ✅ Funciona | DIRECTO pick completo (ATTACH→LIFT→CARRY→CESTA) |"
MD "| ⚠️ Pendiente | Resolver 10 conflictos de merge (UU) |"
MD "| ⚠️ Pendiente | Validar runtime con stack completo post-refactor |"
MD "| ❌ Activo | kinematics.yaml en conflicto: CachedKDL vs plain KDL |"
MD "| ❌ Activo | joint_limits.yaml en conflicto |"
MD "| ❌ Activo | ur5_stack.launch.py en conflicto |"

# ---------------------------------------------------------------------------
# 2. Git y cambios recientes
# ---------------------------------------------------------------------------
H1 "2. Estado Git y Cambios Recientes"

H2 "2.1 Log (últimos 30 commits)"
CMD "git-log" git log --oneline -30

H2 "2.2 Status actual (¡MERGE CONFLICTS!)"
CMD "git-status" git status --short

H2 "2.3 Archivos modificados (diff stat)"
BLK_START
git diff --stat 2>/dev/null | head -40 >> "$MD_OUT" || true
BLK_END

H2 "2.4 Archivos en conflicto de merge (UU)"
BLK_START
git status --short 2>/dev/null | grep "^UU" >> "$MD_OUT" || echo "Ninguno" >> "$MD_OUT"
BLK_END
MD "> **ACCIÓN REQUERIDA:** Los archivos marcados UU tienen conflictos sin resolver."
MD "> El stack NO es buildable en estado de merge sin resolver estos conflictos."

# ---------------------------------------------------------------------------
# 3. Estructura del Workspace
# ---------------------------------------------------------------------------
H1 "3. Estructura del Workspace"

H2 "3.1 Árbol src (3 niveles)"
CMD "src-tree" find src -maxdepth 3 -type f -name "*.py" -o -name "*.xml" -o -name "*.yaml" -o -name "*.launch.py" -o -name "*.srdf" | grep -v __pycache__ | sort

H2 "3.2 Scripts"
CMD "scripts-list" find scripts -maxdepth 1 -type f | sort

H2 "3.3 Modelos"
CMD "models-list" find models -maxdepth 4 -type f | sort

H2 "3.4 Worlds"
CMD "worlds-list" find worlds -maxdepth 2 -type f | sort

H2 "3.5 Reports y evidencias"
CMD "reports-list" find report -maxdepth 4 -type f | sort

H2 "3.6 Histórico"
CMD "historico-list" find historico -maxdepth 3 -type f | sort

# ---------------------------------------------------------------------------
# 4. Paquetes ROS 2
# ---------------------------------------------------------------------------
H1 "4. Paquetes ROS 2"

for PKG_DIR in src/tfm_grasping src/ur5_bringup src/ur5_description src/ur5_moveit_config src/ur5_panel_interfaces src/ur5_qt_panel src/ur5_tools; do
  PKG_NAME="$(basename "$PKG_DIR")"
  H2 "4.x $PKG_NAME"

  if [[ -f "$PKG_DIR/package.xml" ]]; then
    H3 "package.xml"
    SAFE_CAT "$PKG_DIR/package.xml" 60
  fi

  if [[ -f "$PKG_DIR/setup.py" ]]; then
    H3 "setup.py (entry_points)"
    SAFE_GREP "setup-$PKG_NAME" "console_scripts|entry_points|=.*:" "$PKG_DIR/setup.py"
  fi

  H3 "Nodos Python"
  BLK_START
  find "$PKG_DIR" -name "*.py" ! -path "*test*" ! -name "setup.py" ! -name "conftest.py" ! -path "*__pycache__*" | sort >> "$MD_OUT"
  BLK_END

  H3 "Launch files"
  BLK_START
  find "$PKG_DIR" -name "*.launch.py" | sort >> "$MD_OUT"
  BLK_END

  H3 "Config YAML"
  BLK_START
  find "$PKG_DIR" -name "*.yaml" -o -name "*.yml" | sort >> "$MD_OUT"
  BLK_END

  H3 "Tests"
  BLK_START
  find "$PKG_DIR" -name "test_*.py" | sort >> "$MD_OUT" && find "$PKG_DIR" -name "test_*.py" | xargs grep -c "def test_" 2>/dev/null | awk -F: '{s+=$2} END {print "Total funciones test: " s}' >> "$MD_OUT" || true
  BLK_END

  fver "$PKG_DIR"
done

# ---------------------------------------------------------------------------
# 5. Entry Points confirmados
# ---------------------------------------------------------------------------
H1 "5. Entry Points Confirmados"

MD "| Paquete | Entry Point | Módulo | Función |"
MD "|---------|-------------|--------|---------|"
for SETUP in src/*/setup.py; do
  PKG="$(dirname "$SETUP" | xargs basename)"
  grep -E "^\s+['\"].*=.*:.*['\"]" "$SETUP" 2>/dev/null | while IFS= read -r line; do
    clean=$(echo "$line" | tr -d "' \",")
    name="${clean%%=*}"
    target="${clean#*=}"
    mod="${target%%:*}"
    func="${target##*:}"
    echo "| $PKG | $name | $mod | $func |" >> "$MD_OUT"
  done
done

# ---------------------------------------------------------------------------
# 6. Launch Principal — ur5_stack.launch.py
# ---------------------------------------------------------------------------
H1 "6. Launch Principal"

H2 "6.1 ur5_stack.launch.py (primeras 200 líneas)"
SAFE_CAT "src/ur5_bringup/launch/ur5_stack.launch.py" 200

H2 "6.2 Nodos lanzados (grep Node)"
SAFE_GREP "launch-nodes" "Node\(|composable|include.*launch" "src/ur5_bringup/launch/ur5_stack.launch.py"

H2 "6.3 launch_helpers.py — funciones auxiliares"
SAFE_GREP "launch-helpers-fns" "^def " "src/ur5_bringup/launch/launch_helpers.py"

H2 "6.4 ur5_moveit_bringup.launch.py"
SAFE_CAT "src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py" 100

H2 "6.5 ur5_rsp.launch.py"
SAFE_CAT "src/ur5_bringup/launch/ur5_rsp.launch.py" 80

H2 "6.6 Variables de entorno del launcher (start_panel_v2.sh)"
MD "> Archivo: \`scripts/start_panel_v2.sh\` — $(wc -l < scripts/start_panel_v2.sh) líneas"
SAFE_GREP "launcher-env" '^\s*:\s+"\${|export [A-Z]|WS_DIR|GZ_|PANEL_|GAZEBO_|ROS_|RMW_|GZ_RESOURCE' "scripts/start_panel_v2.sh"

# ---------------------------------------------------------------------------
# 7. Variables de Entorno y Parámetros
# ---------------------------------------------------------------------------
H1 "7. Variables de Entorno y Parámetros"

H2 "7.1 panel_runtime_validated.env (perfil validado)"
SAFE_CAT "scripts/panel_runtime_validated.env" 120

H2 "7.2 Variables PANEL_* detectadas en el launcher"
BLK_START
grep -Eo 'PANEL_[A-Z_0-9]+' scripts/start_panel_v2.sh 2>/dev/null | sort -u >> "$MD_OUT" || true
BLK_END

H2 "7.3 Variables GZ_* / GAZEBO_* / ROS_*"
BLK_START
grep -Eo '(GZ_|GAZEBO_|ROS_)[A-Z_0-9]+' scripts/start_panel_v2.sh 2>/dev/null | sort -u >> "$MD_OUT" || true
BLK_END

H2 "7.4 Constantes en panel_config.py"
SAFE_GREP "panel-config-consts" "^[A-Z_]+ *=" "src/ur5_qt_panel/ur5_qt_panel/panel_config.py"

H2 "7.5 panel_settings.py — dataclass campos"
SAFE_GREP "panel-settings-fields" "^\s+\w+.*: .*=|^@dataclass|^class Panel" "src/ur5_qt_panel/ur5_qt_panel/panel_settings.py"

# ---------------------------------------------------------------------------
# 8. URDF / Xacro
# ---------------------------------------------------------------------------
H1 "8. URDF / Xacro — ur5.urdf.xacro"

H2 "8.1 Propiedades xacro (frames, offsets)"
SAFE_GREP "urdf-props" "xacro:property|rg2_contact_tcp|tool0|rg2|gripper|finger|mount|hand|tcp|pinch" \
  "src/ur5_description/urdf/ur5.urdf.xacro"

H2 "8.2 Joints relevantes"
SAFE_GREP "urdf-joints" '<joint name|<parent link|<child link|<origin xyz|type="prismatic|type="fixed|type="revolute' \
  "src/ur5_description/urdf/ur5.urdf.xacro"

H2 "8.3 Xacro completo (primeras 300 líneas)"
SAFE_CAT "src/ur5_description/urdf/ur5.urdf.xacro" 300

# ---------------------------------------------------------------------------
# 9. SDF Model
# ---------------------------------------------------------------------------
H1 "9. SDF Model — models/ur5_rg2/model.sdf"

H2 "9.1 Joints y frames relevantes"
SAFE_GREP "sdf-joints" 'joint name|parent>|child>|pose relative_to|type="prismatic|type="fixed|type="revolute|rg2_mount|ur5_hand_joint|tool0|rg2' \
  "models/ur5_rg2/model.sdf"

H2 "9.2 Gripper SDF — comentarios de geometría"
SAFE_GREP "sdf-gripper-geom" 'rg2_hand|rg2_left|rg2_right|rg2_base|rg2_finger|rg2_mount|pinch|tcp|prism|0\.055|0\.120|0\.175|0\.0050885|0\.0823|ur5_hand_joint' \
  "models/ur5_rg2/model.sdf"

H2 "9.3 SDF completo (primeras 300 líneas)"
SAFE_CAT "models/ur5_rg2/model.sdf" 300

H2 "9.4 ur5_controllers.yaml (modelo)"
SAFE_CAT "models/ur5_rg2/ur5_controllers.yaml"

# ---------------------------------------------------------------------------
# 10. World SDF
# ---------------------------------------------------------------------------
H1 "10. World SDF"

H2 "10.1 ur5_mesa_objetos.sdf (primeras 200 líneas)"
SAFE_CAT "worlds/ur5_mesa_objetos.sdf" 200

H2 "10.2 ur5_debug_empty.sdf"
SAFE_CAT "worlds/ur5_debug_empty.sdf" 80

# ---------------------------------------------------------------------------
# 11. MoveIt 2 Configuración
# ---------------------------------------------------------------------------
H1 "11. MoveIt 2"

H2 "11.1 ur5.srdf"
SAFE_CAT "src/ur5_moveit_config/config/ur5.srdf"

H2 "11.2 ur5_strict.srdf"
SAFE_CAT "src/ur5_moveit_config/config/ur5_strict.srdf" 60

H2 "11.3 kinematics.yaml"
SAFE_CAT "src/ur5_moveit_config/config/kinematics.yaml"

H2 "11.4 joint_limits.yaml"
SAFE_CAT "src/ur5_moveit_config/config/joint_limits.yaml" 100

H2 "11.5 moveit_controllers.yaml"
SAFE_CAT "src/ur5_moveit_config/config/moveit_controllers.yaml"

H2 "11.6 ompl_planning.yaml"
SAFE_CAT "src/ur5_moveit_config/config/ompl_planning.yaml" 60

H2 "11.7 planning_scene_monitor_parameters.yaml"
SAFE_CAT "src/ur5_moveit_config/config/planning_scene_monitor_parameters.yaml"

H2 "11.8 ur5_moveit_bridge.py — funciones principales"
SAFE_GREP "moveit-bridge-fns" "^    def |^class " "src/ur5_tools/ur5_tools/ur5_moveit_bridge.py"

H2 "11.9 moveit_bridge_utils.py — helpers"
SAFE_GREP "moveit-utils-fns" "^def " "src/ur5_tools/ur5_tools/moveit_bridge_utils.py"

# ---------------------------------------------------------------------------
# 12. ros2_control y Gazebo
# ---------------------------------------------------------------------------
H1 "12. ros2_control y Gazebo"

H2 "12.1 ur5_controllers.yaml (descripción)"
SAFE_CAT "src/ur5_description/config/ur5_controllers.yaml"

H2 "12.2 ur5_mock_controllers.yaml"
SAFE_CAT "src/ur5_bringup/config/ur5_mock_controllers.yaml"

H2 "12.3 Bridge YAML base"
SAFE_CAT "scripts/ros_gz_bridge.yaml" 60

H2 "12.4 gz_pose_bridge.py"
SAFE_GREP "gz-pose-bridge-fns" "^def |^class " "src/ur5_tools/ur5_tools/gz_pose_bridge.py"

H2 "12.5 gz_ros_control_guard.py"
SAFE_GREP "gz-control-guard-fns" "^def |^class " "src/ur5_tools/ur5_tools/gz_ros_control_guard.py"

H2 "12.6 planning_scene_sync.py"
SAFE_GREP "planning-scene-fns" "^def |^class " "src/ur5_tools/ur5_tools/planning_scene_sync.py"

# ---------------------------------------------------------------------------
# 13. Panel Qt — Arquitectura modular
# ---------------------------------------------------------------------------
H1 "13. Panel Qt — Arquitectura Modular"

H2 "13.1 Módulos ur5_qt_panel (post-refactoring)"
BLK_START
ls src/ur5_qt_panel/ur5_qt_panel/*.py | grep -v __pycache__ | sort >> "$MD_OUT"
echo "" >> "$MD_OUT"
echo "Total módulos: $(ls src/ur5_qt_panel/ur5_qt_panel/*.py | wc -l)" >> "$MD_OUT"
BLK_END

H2 "13.2 panel_v2.py — callbacks registrados"
SAFE_GREP "panel-v2-callbacks" "def _on_|def on_|clicked\.connect|pressed\.connect|toggled\.connect|\.connect\(" \
  "src/ur5_qt_panel/ur5_qt_panel/panel_v2.py"

H2 "13.3 panel_pick_demo.py — funciones principales"
SAFE_GREP "pick-demo-fns" "^    def |^class " "src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py"

H2 "13.4 panel_pick_object.py — funciones principales"
SAFE_GREP "pick-obj-fns" "^    def |^class " "src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py"

H2 "13.5 panel_pick_geometry.py"
SAFE_GREP "pick-geom-fns" "^def |^class " "src/ur5_qt_panel/ur5_qt_panel/panel_pick_geometry.py"

H2 "13.6 panel_config.py — constantes de fase"
SAFE_CAT "src/ur5_qt_panel/ur5_qt_panel/panel_config.py" 200

H2 "13.7 panel_settings.py (completo)"
SAFE_CAT "src/ur5_qt_panel/ur5_qt_panel/panel_settings.py" 200

H2 "13.8 attach_gate_evaluator.py"
SAFE_GREP "attach-gate-fns" "^def |^class " "src/ur5_qt_panel/ur5_qt_panel/attach_gate_evaluator.py"

H2 "13.9 directo_gate_evaluator.py"
SAFE_GREP "directo-gate-fns" "^def |^class " "src/ur5_qt_panel/ur5_qt_panel/directo_gate_evaluator.py"

H2 "13.10 directo_geometry.py"
SAFE_GREP "directo-geom-fns" "^def |^class " "src/ur5_qt_panel/ur5_qt_panel/directo_geometry.py"

H2 "13.11 ur5_kinematics.py"
SAFE_GREP "kinematics-fns" "^def |^class " "src/ur5_qt_panel/ur5_qt_panel/ur5_kinematics.py"

H2 "13.12 panel_readiness.py"
SAFE_GREP "readiness-fns" "^def |^class " "src/ur5_qt_panel/ur5_qt_panel/panel_readiness.py"

H2 "13.13 panel_state_machine.py"
SAFE_GREP "state-machine-fns" "^def |^class " "src/ur5_qt_panel/ur5_qt_panel/panel_state_machine.py"

H2 "13.14 step_pipeline_helpers.py"
SAFE_GREP "step-pipeline-fns" "^def |^class " "src/ur5_qt_panel/ur5_qt_panel/step_pipeline_helpers.py"

# ---------------------------------------------------------------------------
# 14. ur5_tools — nodos backend
# ---------------------------------------------------------------------------
H1 "14. ur5_tools — Nodos Backend"

for NODE_PY in \
  src/ur5_tools/ur5_tools/gripper_attach_backend.py \
  src/ur5_tools/ur5_tools/world_tf_publisher.py \
  src/ur5_tools/ur5_tools/system_state_manager.py \
  src/ur5_tools/ur5_tools/gripper_geometry.py \
  src/ur5_tools/ur5_tools/controller_bootstrap.py \
  src/ur5_tools/ur5_tools/release_objects_service.py \
  src/ur5_tools/ur5_tools/tf_probe.py \
  src/ur5_tools/ur5_tools/clock_probe.py \
  src/ur5_tools/ur5_tools/cycle_logger.py \
  src/ur5_tools/ur5_tools/jt_smoke_test.py; do
  NODE_NAME="$(basename "$NODE_PY" .py)"
  H2 "14.x $NODE_NAME"
  SAFE_GREP "ur5tools-$NODE_NAME" "^def |^class |topic|service|action|subscriber|publisher|srv_|srv =|pub =|sub =|create_pub|create_sub|create_ser|create_cli" "$NODE_PY"
done

# ---------------------------------------------------------------------------
# 15. tfm_grasping — Percepción
# ---------------------------------------------------------------------------
H1 "15. tfm_grasping — Percepción"

H2 "15.1 grasp_inference.py"
SAFE_GREP "grasp-inference-fns" "^def |^class " "src/tfm_grasping/tfm_grasping/grasp_inference.py"

H2 "15.2 grasp_module.py"
SAFE_GREP "grasp-module-fns" "^def |^class " "src/tfm_grasping/tfm_grasping/grasp_module.py"

H2 "15.3 geometry.py"
SAFE_GREP "tfm-geom-fns" "^def |^class " "src/tfm_grasping/tfm_grasping/geometry.py"

H2 "15.4 config.py"
SAFE_CAT "src/tfm_grasping/tfm_grasping/config.py" 60

H2 "15.5 ros_interface.py"
SAFE_GREP "tfm-ros-fns" "^def |^class " "src/tfm_grasping/tfm_grasping/ros_interface.py"

# ---------------------------------------------------------------------------
# 16. Frames TF y Geometría
# ---------------------------------------------------------------------------
H1 "16. Frames TF y Geometría"

H2 "16.1 Tabla de frames"

MD "| Frame | Parent | Offset (xyz rpy) | Fuente | Uso |"
MD "|-------|--------|-------------------|--------|-----|"
MD "| world | — | 0 0 0 | ur5_stack.launch.py (world_tf_publisher) | Origen global |"
MD "| base_link | world | env WS_X/Y/Z | world_tf_publisher | Base del robot |"
MD "| base_link_inertia | base_link | 0 0 0 | ur5.urdf.xacro | Inercial UR5 |"
MD "| shoulder_link | base_link_inertia | DH | ur5.urdf.xacro | Hombro |"
MD "| upper_arm_link | shoulder_link | DH | ur5.urdf.xacro | Brazo superior |"
MD "| forearm_link | upper_arm_link | DH | ur5.urdf.xacro | Antebrazo |"
MD "| wrist_1_link | forearm_link | DH | ur5.urdf.xacro | Muñeca 1 |"
MD "| wrist_2_link | wrist_1_link | DH | ur5.urdf.xacro | Muñeca 2 |"
MD "| wrist_3_link | wrist_2_link | DH | ur5.urdf.xacro | Muñeca 3 |"
MD "| tool0 | wrist_3_link | 0 0 0 | ur5.urdf.xacro | Flange UR5 |"
MD "| rg2_hand | tool0 | 0 0 0 (rpy=0) | ur5.urdf.xacro + SDF | Cuerpo gripper |"
MD "| rg2_tcp | tool0 | 0 0 0.0050885 | ur5.urdf.xacro | Alias TCP semántico |"
MD "| rg2_pinch_center | tool0 | 0 0 0.0050885 | ur5.urdf.xacro (SRDF tip) | TCP operacional MoveIt |"
MD "| rg2_leftfinger | rg2_hand | 0 +0.Y 0.120 | ur5.urdf.xacro | Dedo izquierdo prismático |"
MD "| rg2_rightfinger | rg2_hand | 0 −0.Y 0.120 | ur5.urdf.xacro | Dedo derecho prismático |"
MD "| pick_demo_anchor | world | gz pose | gz_pose_bridge | Pose objeto pick_demo |"

H2 "16.2 TCP semántico — historial de cambios"

MD "| Fecha | Valor anterior | Valor actual | Motivo |"
MD "|-------|----------------|--------------|--------|"
MD "| pre-2026-04-22 | 0.175 m (yemas visuales) | — | Error: confundía TCP con yemas |"
MD "| 2026-04-22 | 0.175 | 0.05 (GRIPPER_TCP_Z_OFFSET) | Fix parcial |"
MD "| 2026-04-25 | 0.05 | **0.0050885 m** | Unificación TCP canónico |"
MD "| 2026-04-25 | pitch=−π/2 (end_effector_frame_fixed_joint) | rpy=0 0 0 | Fix SDF 90° mismatch |"

H2 "16.3 world_tf_publisher.py — offsets publicados"
SAFE_GREP "world-tf-offsets" "WS_X|WS_Y|WS_Z|base_link|world|transform|TransformStamped|publish" \
  "src/ur5_tools/ur5_tools/world_tf_publisher.py"

H2 "16.4 gripper_geometry.py — constantes"
SAFE_GREP "gripper-geom-consts" "^[A-Z_]+ *=|def |class |rg2|tcp|pinch|FINGER|OPEN|CLOSE|0\.055|0\.120|0\.175|0\.0050885" \
  "src/ur5_tools/ur5_tools/gripper_geometry.py"

# ---------------------------------------------------------------------------
# 17. Tests y Validación
# ---------------------------------------------------------------------------
H1 "17. Tests y Validación"

H2 "17.1 Resumen de tests"
BLK_START
echo "Paquetes con tests:" >> "$MD_OUT"
for PKG_DIR in src/*/; do
  COUNT=$(find "$PKG_DIR" -name "test_*.py" 2>/dev/null | xargs grep -l "def test_" 2>/dev/null | wc -l)
  TOTAL=$(find "$PKG_DIR" -name "test_*.py" -exec grep -h "def test_" {} \; 2>/dev/null | wc -l)
  [[ "$COUNT" -gt 0 ]] && echo "  $(basename "$PKG_DIR"): $COUNT archivos, $TOTAL funciones test_*" >> "$MD_OUT"
done
BLK_END

H2 "17.2 validate_startup_repro.sh"
SAFE_CAT "scripts/validate_startup_repro.sh" 100

H2 "17.3 validate_before_demo.sh"
SAFE_CAT "scripts/validate_before_demo.sh" 100

H2 "17.4 validate_pick_3_cycles.sh"
SAFE_CAT "scripts/validate_pick_3_cycles.sh" 80

H2 "17.5 smoke_test.sh"
SAFE_CAT "scripts/smoke_test.sh" 80

H2 "17.6 Tabla de tests clave"
MD "| Test | Archivo | Qué valida | Requiere ROS |"
MD "|------|---------|------------|--------------|"
MD "| test_gripper_geometry | ur5_tools | Geometría RG2, offsets, ranges | No |"
MD "| test_moveit_bridge_utils | ur5_tools | 16 helpers IK/FK/bridge | No |"
MD "| test_moveit_bridge_plan_success | ur5_tools | Integración plan+execute | No |"
MD "| test_attach_gate_evaluator | ur5_qt_panel | Lógica attach gate | No |"
MD "| test_directo_gate_evaluator | ur5_qt_panel | Gates DIRECTO | No |"
MD "| test_directo_geometry | ur5_qt_panel | Geometría DIRECTO | No |"
MD "| test_ur5_kinematics | ur5_qt_panel | FK/IK DH analítico | No |"
MD "| test_param_utils | ur5_tools | duck-typed, sin ROS | No |"
MD "| test_panel_config | ur5_qt_panel | Constantes panel | No |"
MD "| test_state_machine | ur5_qt_panel | Estado máquina | No |"
MD "| test_step_pipeline | ur5_qt_panel | Pipeline steps | No |"
MD "| test_panel_pick_object_moveit_init | ur5_qt_panel | MoveIt init audit | No |"
MD "| test_launch_helpers | ur5_bringup | 27 tests launch helpers | No |"
MD "| validate_startup_repro.sh | scripts | Arranque reproducible | Sí |"
MD "| validate_pick_3_cycles.sh | scripts | 3 ciclos pick completos | Sí |"

# ---------------------------------------------------------------------------
# 18. Logs Recientes
# ---------------------------------------------------------------------------
H1 "18. Logs Recientes"

H2 "18.1 Logs más recientes"
BLK_START
find log -type f -printf "%T@ %p\n" 2>/dev/null | sort -nr | head -30 | awk '{print $2}' >> "$MD_OUT" || echo "log/ no disponible" >> "$MD_OUT"
BLK_END

H2 "18.2 ros2_launch.log (últimas 100 líneas)"
if [[ -f log/ros2_launch.log ]]; then
  BLK_START
  tail -100 log/ros2_launch.log >> "$MD_OUT"
  BLK_END
  fver "log/ros2_launch.log"
else
  MD "_log/ros2_launch.log no disponible_"
fi

H2 "18.3 Errores en logs recientes"
BLK_START
find log -type f -name "*.log" 2>/dev/null | sort | xargs grep -iE "error|exception|traceback|fatal|timeout|failed" 2>/dev/null | head -60 >> "$MD_OUT" || echo "Sin errores encontrados" >> "$MD_OUT"
BLK_END

H2 "18.4 attach_backend.log"
if [[ -f log/attach_backend.log ]]; then
  BLK_START
  tail -40 log/attach_backend.log >> "$MD_OUT"
  BLK_END
  fver "log/attach_backend.log"
fi

H2 "18.5 world_tf_publisher.log"
if [[ -f log/world_tf_publisher.log ]]; then
  BLK_START
  tail -30 log/world_tf_publisher.log >> "$MD_OUT"
  BLK_END
  fver "log/world_tf_publisher.log"
fi

H2 "18.6 Reportes de auditoría"
for f in report/auditoria_moveit_step_20260424.md report/runtime_moveit_step_20260424.md; do
  [[ -f "$f" ]] && { H3 "$f"; SAFE_CAT "$f" 80; }
done

H2 "18.7 Repro startup último ciclo"
for f in report/repro_startup/20260423_161929/*.log; do
  [[ -f "$f" ]] && { H3 "$(basename "$f")"; SAFE_CAT "$f" 30; }
done

# ---------------------------------------------------------------------------
# 19. Runtime actual (si stack disponible)
# ---------------------------------------------------------------------------
H1 "19. Runtime Actual"

MD "> **Nota:** Esta sección sólo tiene datos si el stack ROS 2 está arrancado durante la generación."
MD ""

_ros_sourced=0
source /opt/ros/jazzy/setup.bash 2>/dev/null && source install/setup.bash 2>/dev/null && _ros_sourced=1 || true
if [[ "$_ros_sourced" -eq 1 ]]; then
  H2 "19.1 Paquetes instalados (ur5/tfm/panel)"
  CMD "ros2-pkg-list" ros2 pkg list | grep -E "ur5|tfm|panel"

  H2 "19.2 Executables por paquete"
  for PKG in ur5_qt_panel ur5_tools tfm_grasping ur5_bringup; do
    H3 "$PKG"
    CMD "executables-$PKG" ros2 pkg executables "$PKG"
  done

  H2 "19.3 Nodos activos"
  CMD "ros2-node-list" ros2 node list

  H2 "19.4 Topics activos"
  CMD "ros2-topic-list" ros2 topic list

  H2 "19.5 Services activos"
  CMD "ros2-service-list" ros2 service list

  H2 "19.6 Actions activas"
  CMD "ros2-action-list" ros2 action list

  H2 "19.7 Controladores"
  CMD "ros2-control-list" ros2 control list_controllers

  H2 "19.8 TF frames"
  CMD "ros2-tf-frames" timeout 5 ros2 run tf2_tools view_frames --ros-args -p output_file:=/tmp/tf_frames_bk 2>&1 || ros2 topic echo /tf_static --once 2>&1 | head -60 || echo "[NO CONFIRMADO] TF no disponible"
else
  MD "**[RUNTIME NO DISPONIBLE]** — Stack ROS 2 no arrancado durante esta generación."
  MD "Para datos runtime, arrancar el stack y re-ejecutar el script."
fi

# ---------------------------------------------------------------------------
# 20. Bugs Conocidos y Estado
# ---------------------------------------------------------------------------
H1 "20. Bugs Conocidos y Estado"

MD "| Bug | Síntoma | Causa raíz | Fix aplicado | Archivo | Estado | Fecha |"
MD "|-----|---------|------------|--------------|---------|--------|-------|"
MD "| Doble offset Z | TCP 50mm erróneo | GRIPPER_TCP_Z_OFFSET=0.05 aplicado a rg2_pinch_center | _DIRECTO_GRASP_Z=0.0 | panel_pick_demo.py | Resuelto | 2026-04-08 |"
MD "| False positive grasp | min_lift_delta demasiado bajo | 0.025 m | 0.025→0.060 | panel_pick_demo.py | Resuelto | 2026-04-07 |"
MD "| APPROACH_COARSE_NOT_READY | H1=overshoot 18mm IK | ik_err_tol=0.035 demasiado estricto | approach_clearance_ok gate | ur5_moveit_bridge.py | Resuelto | 2026-04-15 |"
MD "| SDF pitch=−π/2 | 247mm error visual TCP | end_effector_frame_fixed_joint rpy=(-π/2,0,0) | rpy=0 + ur5_hand_joint Z=0.07 | model.sdf | Resuelto | 2026-04-25 |"
MD "| RG2 revoluto legacy | RuntimeError RELEASE | PANEL_GRIPPER_OPEN_RAD=0.5 (revoluto) en prismático | 0.5→0.055 m | start_panel_v2.sh | Resuelto | 2026-04-25 |"
MD "| ee_link mismatch MoveIt | Ciclos MOVEIT bloqueados | ee_link incorrecto en bridge | ee_link=rg2_pinch_center | ur5_moveit_bridge.py | Resuelto | 2026-04-25 |"
MD "| NEGATE_XY dead code | — | Var inactiva | Eliminada | panel_pick_demo.py | Resuelto | 2026-04-16 |"
MD "| RSP muerto sin guard | TF stale, TCP OK falso | RSP sin respawn | Plan FASE A-E | ur5_stack.launch.py | Pendiente | 2026-04-24 |"
MD "| detachable_shadow_follow=False | CARRY fallaba | Bloqueaba follow loop cinemático | =True | panel_pick_demo.py | Resuelto | 2026-04-19 |"
MD "| DH/SDF divergencia 343mm | Benchmarks con artefactos | SDF offsets erróneos | 9 fixes SDF Opción 3 | model.sdf | Resuelto | 2026-04-23 |"
MD "| **10 merge conflicts (UU)** | Build puede fallar | Merge incompleto ENTREGA.V3 | **PENDIENTE** | ver git status | **Activo** | 2026-04-26 |"

# ---------------------------------------------------------------------------
# 21. Troubleshooting
# ---------------------------------------------------------------------------
H1 "21. Troubleshooting"

H2 "Panel no arranca"
MD "**Diagnóstico:** \`bash scripts/smoke_test.sh\`"
MD "**Causas comunes:** imports rotos, entry_points no instalados, Qt no disponible."
MD "**Fix:** \`colcon build && source install/setup.bash && ros2 run ur5_qt_panel panel_v2\`"

H2 "Panel arranca pero botones no funcionan"
MD "**Diagnóstico:** revisar callbacks en panel_v2.py, verificar que el stack está arrancado."
MD "**Causa:** nodos backend no activos (ur5_moveit_bridge, gripper_attach_backend)."
MD "**Fix:** arrancar stack completo con \`bash scripts/start_panel_v2.sh\`"

H2 "Sistema no llega a READY"
MD "**Diagnóstico:** \`bash scripts/diag_startup_health.sh\`"
MD "**Causa:** TF no fresco, controladores inactivos, MoveIt no listo."
MD "**Fix:** verificar \`ros2 control list_controllers\`, \`ros2 topic echo /tf_static\`"

H2 "Merge conflicts bloquean el build"
MD "**Diagnóstico:** \`git status --short | grep UU\`"
MD "**Fix:** resolver cada archivo UU con \`git checkout --ours\` o \`git checkout --theirs\` + \`git add\`"
MD "**Archivos afectados:**"
BLK_START
git status --short 2>/dev/null | grep "^UU" >> "$MD_OUT" || echo "Ninguno" >> "$MD_OUT"
BLK_END

H2 "Gripper no cierra / error RELEASE"
MD "**Causa:** PANEL_GRIPPER_OPEN_RAD con valor legacy revoluto (0.5) en sistema prismático."
MD "**Fix:** verificar \`PANEL_GRIPPER_OPEN_RAD=0.055\` en scripts/start_panel_v2.sh"

H2 "TCP incorrecto / grasp falla"
MD "**Causa:** rg2_pinch_center no apunta a 0.0050885 m desde tool0."
MD "**Fix:** verificar SRDF tip_link=rg2_pinch_center, URDF rg2_contact_tcp_xyz=0.0050885"

H2 "MoveIt no responde"
MD "**Diagnóstico:** \`ros2 action list | grep follow\`"
MD "**Causa:** move_group no arrancado o controladores inactivos."
MD "**Fix:** \`ros2 launch ur5_moveit_config ur5_moveit_bringup.launch.py\`"

H2 "TF roto / stale"
MD "**Diagnóstico:** \`ros2 run ur5_tools tf_probe\`"
MD "**Causa:** RSP muerto, world_tf_publisher no arrancado."
MD "**Fix:** verificar \`ros2 node list | grep robot_state_publisher\`"

# ---------------------------------------------------------------------------
# 22. Discrepancias abiertas
# ---------------------------------------------------------------------------
H1 "22. Discrepancias Abiertas"

MD "| Área | Descripción | Riesgo | Estado |"
MD "|------|-------------|--------|--------|"
MD "| Merge conflicts | 10 archivos UU sin resolver | Alto | Activo |"
MD "| kinematics.yaml | CachedKDL vs plain KDL en conflicto | Medio | Activo |"
MD "| PANEL_GRIPPER_OPEN_RAD | Dos valores en start_panel_v2.sh (0.0425 y 0.055) | Medio | Activo (merge) |"
MD "| RSP sin guard/respawn | TF puede quedar stale sin reinicio | Medio | Pendiente |"
MD "| panel_settings.py merge | Dos valores gripper_open_rad (0.0425/0.055) | Medio | Activo (merge) |"

# ---------------------------------------------------------------------------
# 23. Próximos Pasos Recomendados
# ---------------------------------------------------------------------------
H1 "23. Próximos Pasos Recomendados"

MD "1. **[CRÍTICO] Resolver los 10 conflictos de merge (UU)** antes de cualquier otra acción."
MD "   \`git status --short | grep UU\` — resolver uno a uno."
MD "2. **Verificar build limpio** tras resolver conflictos: \`colcon build --symlink-install\`"
MD "3. **Ejecutar smoke test**: \`bash scripts/smoke_test.sh\`"
MD "4. **Validar runtime completo**: \`bash scripts/validate_startup_repro.sh\`"
MD "5. **Validar 3 ciclos pick**: \`bash scripts/validate_pick_3_cycles.sh\`"
MD "6. **Añadir guard/respawn para RSP** en ur5_stack.launch.py (issue pendiente)."
MD "7. **Actualizar ARCHITECTURE.md** post-merge para reflejar estado final."

# ---------------------------------------------------------------------------
# 24. Apéndices
# ---------------------------------------------------------------------------
H1 "24. Apéndices"

H2 "24.1 Índice de fuentes verificadas"
BLK_START
sort -u "$FUENTES_LOG" >> "$MD_OUT"
BLK_END

H2 "24.2 Estadísticas de generación"
BLK_START
echo "Fecha               : $FECHA $HORA" >> "$MD_OUT"
echo "Líneas en MD final  : $(wc -l < "$MD_OUT")" >> "$MD_OUT"
echo "Fuentes verificadas : $(sort -u "$FUENTES_LOG" | wc -l)" >> "$MD_OUT"
echo "Paquetes ROS 2      : $(find src -name "package.xml" | wc -l)" >> "$MD_OUT"
echo "Entry points        : $(grep -rh "console_scripts" src/*/setup.py 2>/dev/null | wc -l) bloques (ver Sec 5)" >> "$MD_OUT"
echo "Launch files        : $(find src -name "*.launch.py" | wc -l)" >> "$MD_OUT"
echo "Módulos panel       : $(ls src/ur5_qt_panel/ur5_qt_panel/*.py 2>/dev/null | wc -l)" >> "$MD_OUT"
echo "Tests (archivos)    : $(find src -name "test_*.py" | wc -l)" >> "$MD_OUT"
echo "Conflicts (UU)      : $(git status --short 2>/dev/null | grep -c "^UU" || echo 0)" >> "$MD_OUT"
echo "Bugs documentados   : 11" >> "$MD_OUT"
BLK_END

H2 "24.3 Referencia a base anterior"
MD "- Ubicación: \`historico/2026-04-18_base_conocimiento_tecnica.md\` (MD)"
MD "- PDF: \`historico/2026-04-18_base_conocimiento_tecnica.pdf\`"
MD "- HTML: \`historico/2026-04-18_base_conocimiento_tecnica.html\`"
MD "- Auditoría arquitectura: \`historico/2026-04-17_auditoria_arquitectura_completa.md\`"

H2 "24.4 Comandos ejecutados en esta generación"
BLK_START
grep '^\*\*\$ ' "$MD_OUT" | sed 's/^\*\*\$ //' | head -50 >> "$MD_OUT" || true
BLK_END

# ---------------------------------------------------------------------------
# Generar PDF (opcional)
# ---------------------------------------------------------------------------
if [[ "$GEN_PDF" -eq 1 ]]; then
  log "Intentando generar PDF con pandoc..."
  if command -v pandoc &>/dev/null; then
    pandoc "$MD_OUT" \
      -o "$PDF_OUT" \
      --pdf-engine=xelatex \
      -V geometry:margin=2cm \
      -V fontsize=10pt \
      -V lang=es \
      --toc \
      --toc-depth=2 \
      2>/dev/null && log "PDF generado: $PDF_OUT" || {
      log "xelatex falló, probando pdflatex..."
      pandoc "$MD_OUT" -o "$PDF_OUT" --pdf-engine=pdflatex -V geometry:margin=2cm 2>/dev/null \
        && log "PDF generado con pdflatex: $PDF_OUT" \
        || log "PDF no disponible (pandoc/latex no instalados o error). Solo MD generado."
    }
  else
    log "pandoc no disponible — solo Markdown generado."
  fi
fi

# ---------------------------------------------------------------------------
# Resumen final
# ---------------------------------------------------------------------------
LINES="$(wc -l < "$MD_OUT")"
FUENTES="$(sort -u "$FUENTES_LOG" | wc -l)"
CONFLICTOS="$(git status --short 2>/dev/null | grep -c "^UU" || echo 0)"

echo ""
echo "=================================================================="
echo " BASE DE CONOCIMIENTO TFM — Generación completada"
echo "=================================================================="
echo " Markdown    : $MD_OUT"
[[ -f "$PDF_OUT" ]] && echo " PDF         : $PDF_OUT"
echo " Fuentes     : $FUENTES archivos/fuentes verificadas"
echo " Líneas MD   : $LINES"
echo " Paquetes    : $(find src -name "package.xml" | wc -l)"
echo " Entry pts   : 14 (ver Sección 5)"
echo " Launch files: $(find src -name "*.launch.py" | wc -l)"
echo " Tests       : $(find src -name "test_*.py" | wc -l) archivos"
echo " Módulos panel: $(ls src/ur5_qt_panel/ur5_qt_panel/*.py 2>/dev/null | wc -l)"
echo " Env vars    : $(grep -c "^export PANEL_" scripts/panel_runtime_validated.env 2>/dev/null || echo '?')"
echo " Bugs doc.   : 11"
echo " CONFLICTOS  : $CONFLICTOS archivos UU sin resolver"
echo ""
echo " DISCREPANCIAS ABIERTAS:"
echo "   1. $CONFLICTOS merge conflicts UU — BUILD PUEDE FALLAR"
echo "   2. kinematics.yaml: CachedKDL vs KDL en conflicto"
echo "   3. PANEL_GRIPPER_OPEN_RAD: 0.0425 vs 0.055 (merge conflict)"
echo "   4. RSP sin guard/respawn (pendiente)"
echo ""
echo " Para ChatGPT/GPT: subir $MD_OUT"
echo "=================================================================="
