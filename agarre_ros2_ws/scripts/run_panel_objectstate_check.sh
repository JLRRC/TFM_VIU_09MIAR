#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/run_panel_objectstate_check.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

ROOT="/home/laboratorio/TFM"
PANEL_PYTHONPATH="${ROOT}/agarre_ros2_ws/src/ur5_qt_panel"
LOG_DIR="${ROOT}/agarre_ros2_ws/log/object_state"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${PANEL_LOG_FILE:-${LOG_DIR}/panel_objectstate_${STAMP}.log}"

mkdir -p "${LOG_DIR}"

cat <<'EOF'
Panel ObjectState check
1) Ensure ROS 2/Gazebo are sourced and running as usual.
2) The panel will open; run:
   - TEST ROBOT (should NOT move objects)
   - Pick demo (pick_demo)
3) Close the panel to finish verification.
EOF

echo "Logging to: ${LOG_FILE}"

PYTHON_BIN="${PYTHON_BIN:-}"
if [[ -z "${PYTHON_BIN}" ]]; then
  CANDIDATES=()
  for candidate in \
    "${ROOT}/agarre_inteligente/.venv-tfm/bin/python" \
    "${ROOT}/agarre_inteligente/venv/bin/python" \
    "${ROOT}/agarre_inteligente/.venv/bin/python" \
    "${ROOT}/.venv-tfm/bin/python"; do
    if [[ -x "$candidate" ]]; then
      CANDIDATES+=("$candidate")
    fi
  done
  if command -v python3 >/dev/null 2>&1; then
    CANDIDATES+=("python3")
  fi
  if command -v python >/dev/null 2>&1; then
    CANDIDATES+=("python")
  fi
  for cand in "${CANDIDATES[@]}"; do
    if "${cand}" - <<'PY' >/dev/null 2>&1; then
import PyQt5  # noqa: F401
PY
      PYTHON_BIN="${cand}"
      break
    fi
  done
  if [[ -z "${PYTHON_BIN}" ]]; then
    echo "ERROR: no python with PyQt5 found. Set PYTHON_BIN explicitly." >&2
    exit 127
  fi
fi

PANEL_SKIP_LAUNCH="${PANEL_SKIP_LAUNCH:-}"
PANEL_LAUNCH_CMD="${PANEL_LAUNCH_CMD:-}"
if [[ -z "${PANEL_LAUNCH_CMD}" ]]; then
  if [[ -f "/opt/ros/jazzy/setup.bash" ]] && [[ -f "${ROOT}/agarre_ros2_ws/install/setup.bash" ]]; then
    PANEL_LAUNCH_CMD="bash -lc 'source /opt/ros/jazzy/setup.bash && source ${ROOT}/agarre_ros2_ws/install/setup.bash && ros2 run ur5_qt_panel panel_v2'"
  fi
fi

if [[ -z "${PANEL_SKIP_LAUNCH}" ]]; then
  if [[ -n "${PANEL_LAUNCH_CMD}" ]]; then
    eval "${PANEL_LAUNCH_CMD}" 2>&1 | tee "${LOG_FILE}"
  else
    PYTHONPATH="${PANEL_PYTHONPATH}" \
    "${PYTHON_BIN}" -m ur5_qt_panel.panel_v2 2>&1 | tee "${LOG_FILE}"
  fi
else
  echo "PANEL_SKIP_LAUNCH=1: using existing log only."
fi

echo
echo "Post-run checks:"
"${PYTHON_BIN}" - <<PY
import re
from pathlib import Path

log_path = Path("${LOG_FILE}")
text = log_path.read_text(encoding="utf-8", errors="ignore")

def found(pattern: str) -> bool:
    return re.search(pattern, text) is not None

checks = [
    ("TEST read-only ON", r"\\[OBJECTS\\]\\[TEST\\] read-only on"),
    ("TEST read-only OFF", r"\\[OBJECTS\\]\\[TEST\\] read-only off"),
    ("ObjectState ON_TABLE->GRASPED", r"\\[OBJECTS\\] state pick_demo -> GRASPED"),
    ("ObjectState GRASPED->CARRIED", r"\\[OBJECTS\\] state pick_demo -> CARRIED"),
    ("ObjectState CARRIED->RELEASED", r"\\[OBJECTS\\] state pick_demo -> RELEASED"),
]

ok = True
for label, pattern in checks:
    hit = found(pattern)
    status = "OK" if hit else "MISS"
    print(f"{status}: {label}")
    if not hit:
        ok = False

if not ok:
    print("WARN: Missing expected log lines. Verify TEST/pick demo was executed.")
PY
