#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/ci_local.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

echo "[CI_LOCAL] workspace=$ROOT_DIR"

RUFF_BIN="ruff"
PYTEST_BIN="pytest"
for candidate in \
  "/home/laboratorio/TFM/agarre_inteligente/.venv-tfm/bin" \
  "/home/laboratorio/TFM/agarre_inteligente/venv/bin" \
  "/home/laboratorio/TFM/agarre_inteligente/.venv/bin" \
  "/home/laboratorio/TFM/.venv-tfm/bin"; do
  if [[ -x "$candidate/ruff" ]]; then
    RUFF_BIN="$candidate/ruff"
    break
  fi
done
for candidate in \
  "/home/laboratorio/TFM/agarre_inteligente/.venv-tfm/bin" \
  "/home/laboratorio/TFM/agarre_inteligente/venv/bin" \
  "/home/laboratorio/TFM/agarre_inteligente/.venv/bin" \
  "/home/laboratorio/TFM/.venv-tfm/bin"; do
  if [[ -x "$candidate/pytest" ]]; then
    PYTEST_BIN="$candidate/pytest"
    break
  fi
done

if ! command -v "$RUFF_BIN" >/dev/null 2>&1; then
  echo "[CI_LOCAL][ERROR] ruff no disponible en PATH"
  exit 1
fi

if ! command -v "$PYTEST_BIN" >/dev/null 2>&1; then
  echo "[CI_LOCAL][ERROR] pytest no disponible en PATH"
  exit 1
fi

echo "[CI_LOCAL] ruff check"
"$RUFF_BIN" check src

echo "[CI_LOCAL] pytest"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$PYTEST_BIN" --import-mode=importlib -q src/ur5_qt_panel/test src/ur5_tools/test

echo "[CI_LOCAL] OK"
