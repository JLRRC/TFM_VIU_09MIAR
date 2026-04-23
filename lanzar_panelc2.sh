#!/usr/bin/env bash
# lanzar_panelc2.sh — entrypoint operativo canonico del panel.
# La repeticion manual final de las pruebas debe ejecutarse con este launcher.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/lanzar_panelv2.sh" "$@"
