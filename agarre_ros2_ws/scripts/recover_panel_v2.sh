#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/recover_panel_v2.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Lanza el trigger remoto de Recover del panel_v2.
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
RECOVER_SERVICE="${PANEL_RECOVER_TRIGGER_SERVICE:-/panel/recover}"

echo "[RECOVER_PANEL_V2] WS_DIR=${WS_DIR}"
echo "[RECOVER_PANEL_V2] RECOVER_SERVICE=${RECOVER_SERVICE}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
  source "${WS_DIR}/install/setup.bash"
fi
set -u

for _ in $(seq 1 20); do
  if ros2 node info /panel_superpro >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done

if ! ros2 node info /panel_superpro >/dev/null 2>&1; then
  echo "[RECOVER_PANEL_V2] ERROR: /panel_superpro no disponible" >&2
  exit 1
fi

ros2 service call "${RECOVER_SERVICE}" std_srvs/srv/Trigger "{}"
