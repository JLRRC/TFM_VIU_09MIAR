#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/repro_release_objects_idempotent.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_FILE="${1:-$ROOT_DIR/auditoria/panel_run.log}"

echo "[REPRO] 1) Ejecuta el panel en otra terminal:"
echo "  cd $ROOT_DIR && ./scripts/start_panel_v2.sh"
echo
echo "[REPRO] 2) Pulsa 'Soltar objetos' 3 veces seguidas."
echo
echo "[REPRO] 3) Verifica logs en: $LOG_FILE"
if [[ ! -f "$LOG_FILE" ]]; then
  echo "[WARN] No existe el log: $LOG_FILE"
  echo "       Pasa el path del log como argumento:"
  echo "       $0 /ruta/a/tu/panel_run.log"
  exit 1
fi

echo
echo "[CHECK] Debe NO aparecer duplicación/errores de detach:"
rg -n "link\\(1\\)|Joint\\(1\\)|Could not find the original skeleton|detaching joint \\[fixed\\(" "$LOG_FILE" || true

echo
echo "[CHECK] Debe NO intentar limpiar drop_anchor(>2):"
rg -n "drop_anchor\\(([3-9]|[1-9][0-9]+)\\)" "$LOG_FILE" || true

echo
echo "[CHECK] Si service success=false, debe verse fallo (sin exito falso):"
rg -n "release_objects service success=false|❌ Soltar objetos falló|✅ Objetos soltados" "$LOG_FILE" || true

echo
echo "[CHECK] Si service success=true, debe verse reset/drop:"
rg -n "release_objects service success=true|global reset applied owners=NONE|objetos liberados|Objetos soltados" "$LOG_FILE" || true
