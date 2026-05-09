#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# generar_base_conocimiento_tfm.sh
# Genera la base de conocimiento técnica acumulativa del TFM
# preservando el bloque vigente y reincorporando detalle útil
# de la base histórica.
# ============================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${ROOT_DIR}/agarre_ros2_ws"
REPORTS_DIR="${ROOT_DIR}/report"
BASE_CONOCIMIENTO_DIR="${REPORTS_DIR}/BaseDeConocimiento"
DATE_TAG="$(date +%F)"

OUT_BASENAME="${DATE_TAG}_base_conocimiento_tecnica_TFM"
OUT_MD="${BASE_CONOCIMIENTO_DIR}/${OUT_BASENAME}.md"
OUT_PDF="${BASE_CONOCIMIENTO_DIR}/${OUT_BASENAME}.pdf"
OUT_DIFF_MD="${BASE_CONOCIMIENTO_DIR}/${DATE_TAG}_diff_contra_documento_anterior.md"

REF_PDF="${BASE_CONOCIMIENTO_DIR}/2026-04-18_base_conocimiento_tecnica_TFM.pdf"
HIST_MD="${WS_DIR}/historico/2026-04-18_base_conocimiento_tecnica.md"
PY_HELPER="${ROOT_DIR}/generar_base_conocimiento_tfm.py"

TMP_DIR="${BASE_CONOCIMIENTO_DIR}/.tmp_base_conocimiento_${DATE_TAG}"
GEN_LOG="${TMP_DIR}/generacion.log"
SOURCES_FILE="${TMP_DIR}/fuentes_verificadas.txt"
HTML_TMP="${TMP_DIR}/${OUT_BASENAME}.html"

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

log "Verificando rutas y dependencias..."
check_path "${ROOT_DIR}"
check_path "${WS_DIR}"
check_path "${REPORTS_DIR}"
check_path "${PY_HELPER}"
check_path "${WS_DIR}/src/ur5_bringup/launch/ur5_stack.launch.py"
check_path "${WS_DIR}/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py"
check_path "${WS_DIR}/src/ur5_tools/ur5_tools/gripper_attach_backend.py"
check_path "${WS_DIR}/src/ur5_tools/ur5_tools/gripper_geometry.py"
check_path "${WS_DIR}/src/ur5_description/urdf/ur5.urdf.xacro"
check_path "${WS_DIR}/models/ur5_rg2/model.sdf"
check_path "${WS_DIR}/worlds/ur5_mesa_objetos.sdf"
check_path "${WS_DIR}/src/ur5_bringup/config/ur5_mock_controllers.yaml"
check_path "${WS_DIR}/src/ur5_qt_panel/config/panel_settings.yaml"
check_path "${WS_DIR}/scripts/start_panel_v2.sh"
check_path "${WS_DIR}/scripts/panel_runtime_validated.env"
check_path "${WS_DIR}/scripts/validate_startup_repro.sh"

if [[ ! -e "${HIST_MD}" ]]; then
  warn "No existe la base histórica Markdown ${HIST_MD}; se intentará fallback a PDF si está disponible."
fi
if [[ ! -e "${REF_PDF}" ]]; then
  warn "No existe PDF histórico de referencia en ${REF_PDF}."
fi

check_cmd python3
check_cmd cp
check_cmd date

log "Generando documento técnico acumulativo en Markdown..."
python3 "${PY_HELPER}" \
  "${ROOT_DIR}" \
  "${OUT_MD}" \
  "${OUT_DIFF_MD}" \
  "${REF_PDF}" \
  "${HIST_MD}" \
  "${TMP_DIR}" \
  "${DATE_TAG}" \
  "${SOURCES_FILE}" \
  || fail "Falló la generación acumulativa del Markdown"

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

log "Artefactos finales generados en ${BASE_CONOCIMIENTO_DIR}"
echo
echo "Generados:"
echo "  - ${OUT_MD}"
echo "  - ${OUT_DIFF_MD}"
if [[ -s "${OUT_PDF}" ]]; then
  echo "  - ${OUT_PDF}"
fi
echo "  - ${GEN_LOG}"
echo "  - ${SOURCES_FILE}"
