#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# generar_base_conocimiento_tfm.sh
# Genera el documento técnico actualizado del TFM y deja
# una copia en la raíz y otra en /reports
#
# Requisitos:
#   - Claude Code CLI disponible como: claude
#   - pandoc + xelatex   (recomendado)
#     o pandoc + wkhtmltopdf
#
# Uso:
#   chmod +x generar_base_conocimiento_tfm.sh
#   ./generar_base_conocimiento_tfm.sh
# ============================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${ROOT_DIR}/agarre_ros2_ws"
REPORTS_DIR="${ROOT_DIR}/reports"
DATE_TAG="$(date +%F)"

OUT_BASENAME="${DATE_TAG}_base_conocimiento_tecnica_TFM"
OUT_MD="${ROOT_DIR}/${OUT_BASENAME}.md"
OUT_PDF="${ROOT_DIR}/${OUT_BASENAME}.pdf"
OUT_DIFF_MD="${ROOT_DIR}/${DATE_TAG}_diff_contra_documento_anterior.md"

COPIA_PDF_REPORTS="${REPORTS_DIR}/${OUT_BASENAME}.pdf"
COPIA_MD_REPORTS="${REPORTS_DIR}/${OUT_BASENAME}.md"
COPIA_DIFF_REPORTS="${REPORTS_DIR}/${DATE_TAG}_diff_contra_documento_anterior.md"

REF_PDF="${REPORTS_DIR}/2026-04-18_base_conocimiento_tecnica_TFM.pdf"

TMP_DIR="${ROOT_DIR}/.tmp_base_conocimiento_${DATE_TAG}"
PROMPT_FILE="${TMP_DIR}/prompt_base_conocimiento.txt"
RAW_AI_OUT="${TMP_DIR}/raw_ai_output.md"
HTML_TMP="${TMP_DIR}/${OUT_BASENAME}.html"

mkdir -p "${TMP_DIR}"
mkdir -p "${REPORTS_DIR}"

log() {
  echo "[BASE-CONOCIMIENTO] $*"
}

fail() {
  echo "[BASE-CONOCIMIENTO][ERROR] $*" >&2
  exit 1
}

check_path() {
  local p="$1"
  [[ -e "$p" ]] || fail "No existe: $p"
}

check_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "Falta dependencia: $1"
}

log "Verificando rutas..."
check_path "${ROOT_DIR}"
check_path "${WS_DIR}"
check_path "${REPORTS_DIR}"
check_path "${REF_PDF}"

log "Verificando herramientas..."
check_cmd claude
check_cmd python3
check_cmd find
check_cmd sed
check_cmd awk
check_cmd grep
check_cmd cp

cat > "${PROMPT_FILE}" <<EOF
Eres un ingeniero senior de robótica especializado en ROS 2 Jazzy, Gazebo Sim, MoveIt 2, ros2_control, ros_gz y documentación técnica de sistemas pick-and-place.

Tu tarea es RECONSTRUIR Y ACTUALIZAR por completo el documento técnico del proyecto a partir del estado REAL del workspace, usando como plantilla y referencia estructural el documento existente:

${REF_PDF}

OBJETIVO
Generar una nueva versión actualizada, precisa y utilizable del documento “Base de Conocimiento Técnica — Sistema UR5 + RG2 Pick & Place”, reflejando el estado real actual del proyecto y corrigiendo cualquier desfase respecto al código, launch files, parámetros, frames, offsets, controladores, bugs, fixes y estado operativo.

CONTEXTO FIJO
- Root del proyecto: ${ROOT_DIR}
- Workspace: ${WS_DIR}
- ROS 2 Jazzy
- Gazebo Sim moderno
- MoveIt 2
- Robot: UR5 + OnRobot RG2
- Sistema en simulación
- Archivo clave URDF/Xacro:
  ${WS_DIR}/src/ur5_description/urdf/ur5.urdf.xacro

INSTRUCCIÓN PRINCIPAL
NO rehagas el documento de memoria.
NO copies ciegamente el PDF anterior.
Debes usar el PDF anterior SOLO como estructura base y revalidar TODO contra el código fuente real actual.

MÉTODO OBLIGATORIO
1. Inspecciona y resume el estado actual de:
   - src/ur5_bringup/
   - src/ur5_description/
   - src/ur5_moveit_config/
   - src/ur5_qt_panel/
   - src/tfm_grasping/
   - src/ur5_tools/
   - models/ur5_rg2/
   - worlds/
   - scripts/

2. Verifica expresamente:
   - launch principal y sublaunches relevantes
   - nodos realmente usados
   - controladores activos y configuración
   - URDF/Xacro y SDF del robot
   - frames TF y offsets exactos
   - lógica real del pick_demo
   - variables de entorno vigentes
   - cambios recientes relevantes
   - bugs corregidos y bugs aún abiertos
   - estado real actual: resuelto / pendiente / hipótesis abiertas

3. Compara contra el PDF anterior y detecta:
   - offsets que hayan cambiado
   - thresholds que hayan cambiado
   - rutas de archivo que hayan cambiado
   - launch args nuevos o eliminados
   - variables nuevas o deprecadas
   - fixes nuevos no documentados
   - bugs del PDF anterior que ya no aplican
   - secciones que deban reescribirse

4. Genera DOS salidas:
   A) Documento principal completo en Markdown
   B) Diff corto contra el documento anterior

REGLAS DE CALIDAD
- No inventes nada
- Si una afirmación no está soportada por el código actual, no la pongas
- Si hay incertidumbre, márcala explícitamente
- Distingue siempre entre:
  - URDF/Xacro
  - SDF/Gazebo
  - TF semántico
  - geometría visual
  - lógica de panel
  - backend attach
  - MoveIt
  - ros2_control
- Separa claramente:
  - CAPA 1: geometría visual
  - CAPA 2: frames / TCP / offsets
  - CAPA 3: lógica de pick
  - CAPA 4: attach backend
  - CAPA 5: validación física

VALIDACIONES OBLIGATORIAS
Debes verificar específicamente:
- offset tool0 -> rg2_tcp
- offset tool0 -> rg2_pinch_center
- posición de ur5_hand_joint en model.sdf
- consistencia visual URDF/SDF
- mundo usado por el launch principal
- attach_backend_max_dist_m vigente
- target Z de grasp
- approach_coarse extra Z
- segment step de GRASP_DOWN
- tolerancias de ALIGN / PRE_CLOSE / ATTACH
- min_lift_delta vigente
- controlador del gripper
- topics de attach/detach
- frame operacional real del grasp

ESTRUCTURA OBLIGATORIA DEL DOCUMENTO PRINCIPAL
1. Resumen Ejecutivo del Sistema
2. Arquitectura del Proyecto
3. Frames y Offsets
4. Geometría del Gripper y del Objeto
5. Flujo Completo del Pick
6. Variables de Entorno y Parámetros Críticos
7. Controladores, Topics y Validaciones
8. Bugs Conocidos, Fixes Aplicados y Regresiones Detectadas
9. Guía de Troubleshooting
10. Estado Actual del Sistema
11. Diferencias respecto al documento anterior
12. Apéndices
13. Resumen de cambios desde la versión anterior
14. Riesgos abiertos

FORMATO DE SALIDA OBLIGATORIO
Devuelve exactamente este bloque, en este orden:

=====BEGIN_MAIN_MD=====
[aquí el documento principal completo en markdown]
=====END_MAIN_MD=====

=====BEGIN_DIFF_MD=====
[aquí el diff corto contra el documento anterior en markdown]
=====END_DIFF_MD=====

RUTAS ÚTILES
- Root: ${ROOT_DIR}
- Workspace: ${WS_DIR}
- PDF de referencia: ${REF_PDF}

Empieza inspeccionando el workspace real y reconstruye el documento actualizado completo.
EOF

log "Lanzando generación con Claude..."
claude -p "$(cat "${PROMPT_FILE}")" > "${RAW_AI_OUT}" || fail "Falló la ejecución de Claude"

log "Extrayendo documento principal..."
awk '
/^=====BEGIN_MAIN_MD=====$/ {flag=1; next}
/^=====END_MAIN_MD=====$/   {flag=0}
flag
' "${RAW_AI_OUT}" > "${OUT_MD}"

log "Extrayendo diff..."
awk '
/^=====BEGIN_DIFF_MD=====$/ {flag=1; next}
/^=====END_DIFF_MD=====$/   {flag=0}
flag
' "${RAW_AI_OUT}" > "${OUT_DIFF_MD}"

[[ -s "${OUT_MD}" ]] || fail "No se pudo extraer el documento principal"
[[ -s "${OUT_DIFF_MD}" ]] || fail "No se pudo extraer el diff"

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

if ! render_pdf; then
  fail "No hay motor de PDF disponible. Instala: pandoc + xelatex  (o pandoc + wkhtmltopdf)"
fi

[[ -s "${OUT_PDF}" ]] || fail "No se generó el PDF"

log "Copiando artefactos a reports..."
cp -f "${OUT_PDF}" "${COPIA_PDF_REPORTS}"
cp -f "${OUT_MD}" "${COPIA_MD_REPORTS}"
cp -f "${OUT_DIFF_MD}" "${COPIA_DIFF_REPORTS}"

log "Proceso terminado"
echo
echo "Generados:"
echo "  - ${OUT_MD}"
echo "  - ${OUT_PDF}"
echo "  - ${OUT_DIFF_MD}"
echo
echo "Copias en reports:"
echo "  - ${COPIA_MD_REPORTS}"
echo "  - ${COPIA_PDF_REPORTS}"
echo "  - ${COPIA_DIFF_REPORTS}"