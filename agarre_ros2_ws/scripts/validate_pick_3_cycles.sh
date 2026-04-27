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
exec python3 -m pytest \
    src/ur5_bringup/test/test_e2e_pick_cycles.py \
    -v -s --no-header
