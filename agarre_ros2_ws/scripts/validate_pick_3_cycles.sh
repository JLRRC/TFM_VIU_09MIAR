#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/validate_pick_3_cycles.sh
# Contenido: Wrapper thin que delega en el pytest E2E equivalente.
#
# Antes (pre-refactor F4.3): ~100 lineas de bash con un loop por
# ciclo, calls a ros2 service, parsing del log y manejo de errores.
#
# Ahora: simplemente exporta PICK_E2E_LIVE=1 (y opcionalmente
# PICK_VALIDATE_CYCLES) e invoca el test pytest definido en
# src/ur5_bringup/test/test_e2e_pick_cycles.py. Toda la logica
# (lifecycle del stack, calls a servicios, parsing del log,
# resumenes y skip behaviour) vive ahora en Python -> testeable
# en aislamiento y reusable como `pytest -k pick` desde IDE.
#
# Uso:
#   ./scripts/validate_pick_3_cycles.sh           # 3 ciclos (default)
#   ./scripts/validate_pick_3_cycles.sh 5         # 5 ciclos
#   PICK_VALIDATE_REQUIRE_PASS=0 \
#     ./scripts/validate_pick_3_cycles.sh         # warning-only mode
#
# Variables exportadas via env (todas opcionales salvo CYCLES posicional):
#   PICK_VALIDATE_CYCLES        — N ciclos (default 3, override $1)
#   PICK_VALIDATE_WAIT_LOOPS    — segundos de wait por ciclo (default 600)
#   PICK_VALIDATE_REQUIRE_PASS  — 0 = warning-only, 1 = fail si no PASS

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CYCLES="${1:-${PICK_VALIDATE_CYCLES:-3}}"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

export PICK_E2E_LIVE=1
export PICK_VALIDATE_CYCLES="${CYCLES}"
export PICK_VALIDATE_WAIT_LOOPS="${PICK_VALIDATE_WAIT_LOOPS:-600}"

cd "$ROOT_DIR"

# F7 audit (2026-05-10): KPIs por ciclo escritos a JSON para análisis
# offline y artifact CI. La ruta puede sobrescribirse vía
# PICK_VALIDATE_KPI_FILE (default: log/pick_kpi_<timestamp>.json).
KPI_FILE_DEFAULT="$ROOT_DIR/log/pick_kpi_$(date +%Y%m%d_%H%M%S).json"
export PICK_VALIDATE_KPI_FILE="${PICK_VALIDATE_KPI_FILE:-$KPI_FILE_DEFAULT}"
mkdir -p "$(dirname "$PICK_VALIDATE_KPI_FILE")"

EXIT_CODE=0
python3 -m pytest \
    src/ur5_bringup/test/test_e2e_pick_cycles.py \
    -v -s --no-header || EXIT_CODE=$?

# F7 audit: resumen JSON aggregable.
if [[ -f "$PICK_VALIDATE_KPI_FILE" ]]; then
    echo "[validate_pick_3_cycles] KPI JSON: $PICK_VALIDATE_KPI_FILE"
    python3 -c "
import json, sys
try:
    data = json.load(open('$PICK_VALIDATE_KPI_FILE'))
    n = len(data.get('cycles', []))
    ok = sum(1 for c in data.get('cycles', []) if c.get('success'))
    avg = sum(float(c.get('duration_sec', 0)) for c in data.get('cycles', [])) / max(n, 1)
    print(f'[validate_pick_3_cycles] cycles={n} success={ok}/{n} avg_duration_sec={avg:.1f}')
except Exception as exc:
    print(f'[validate_pick_3_cycles] kpi parse failed: {exc}', file=sys.stderr)
"
fi

exit $EXIT_CODE
