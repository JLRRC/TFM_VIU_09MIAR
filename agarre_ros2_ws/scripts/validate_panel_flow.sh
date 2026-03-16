#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/validate_panel_flow.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

log() { echo "[VALIDATE] $*"; }
warn() { echo "[VALIDATE][WARN] $*" >&2; }
fail=0

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi
if [[ -n "${WS_DIR:-}" && -f "${WS_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "${WS_DIR}/install/setup.bash"
  set -u
elif [[ -f "$(pwd)/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "$(pwd)/install/setup.bash"
  set -u
fi

if ! command -v ros2 >/dev/null 2>&1; then
  warn "ros2 no está disponible en PATH."
  exit 1
fi

if command -v rg >/dev/null 2>&1; then
  filter() { rg --no-line-number "$1"; }
else
  filter() { grep -nE "$1"; }
fi

log "Comprobando /clock..."
if ros2 topic list | filter "^/clock$" >/dev/null 2>&1; then
  if timeout 4.0 ros2 topic echo --once /clock --qos-reliability best_effort >/dev/null 2>&1; then
    log "OK /clock publica."
  elif timeout 4.0 ros2 topic echo --once /clock >/dev/null 2>&1; then
    log "OK /clock publica [qos default]."
  elif timeout 4.0 ros2 topic hz /clock >/dev/null 2>&1; then
    log "OK /clock activo (hz)."
  else
    warn "Existe /clock pero no publica (timeout)."
    fail=1
  fi
else
  warn "/clock no existe."
  fail=1
fi

log "Comprobando /world/*/pose/info..."
pose_topics="$(ros2 topic list | filter "^/world/.*/pose/info$" || true)"
if [[ -n "${pose_topics}" ]]; then
  log "OK pose/info detectado:"
  echo "${pose_topics}" | sed "s/^/[VALIDATE]   /"
  pose_topic_first="$(echo "${pose_topics}" | head -n 1 | sed 's/^[0-9]\+://')"
  if timeout 6.0 ros2 topic echo --once "${pose_topic_first}" --qos-reliability best_effort >/dev/null 2>&1; then
    log "OK pose/info publica (${pose_topic_first})."
  elif timeout 6.0 ros2 topic echo --once "${pose_topic_first}" >/dev/null 2>&1; then
    log "OK pose/info publica (${pose_topic_first}) [qos default]."
  else
    warn "pose/info no publica (timeout)."
    fail=1
  fi
else
  warn "No se detecta /world/*/pose/info."
  fail=1
fi

log "Comprobando nodos clave..."
nodes="$(ros2 node list 2>/dev/null || true)"
echo "${nodes}" | filter "parameter_bridge|ros_gz_bridge" >/dev/null 2>&1 \
  && log "OK bridge node detectado." \
  || warn "No se detecta node de bridge."

echo "${nodes}" | filter "world_tf_publisher" >/dev/null 2>&1 \
  && log "OK world_tf_publisher detectado." \
  || warn "No se detecta world_tf_publisher."

echo "${nodes}" | filter "system_state_manager" >/dev/null 2>&1 \
  && log "OK system_state_manager detectado." \
  || warn "No se detecta system_state_manager."

log "Comprobando TF world->base_link..."
if command -v ros2 >/dev/null 2>&1 && command -v timeout >/dev/null 2>&1; then
  set +e
  tf_out="$(timeout 4.0 ros2 run tf2_ros tf2_echo world base_link --ros-args -p use_sim_time:=true 2>/dev/null | head -n 6)"
  set -e
  if echo "${tf_out}" | filter "Translation:" >/dev/null 2>&1; then
    log "OK TF world->base_link."
  else
    warn "TF world->base_link no disponible (timeout)."
    fail=1
  fi
else
  warn "No se pudo validar TF world->base_link (falta timeout o ros2)."
  fail=1
fi

log "Comprobando TF base_link->tool0..."
if command -v ros2 >/dev/null 2>&1 && command -v timeout >/dev/null 2>&1; then
  set +e
  tf_out="$(timeout 4.0 ros2 run tf2_ros tf2_echo base_link tool0 --ros-args -p use_sim_time:=true 2>/dev/null | head -n 6)"
  set -e
  if echo "${tf_out}" | filter "Translation:" >/dev/null 2>&1; then
    log "OK TF base_link->tool0."
  else
    warn "TF base_link->tool0 no disponible (timeout)."
  fi
else
  warn "No se pudo validar TF base_link->tool0 (falta timeout o ros2)."
fi

log "Comprobando cámara..."
camera_topic="${PANEL_CAMERA_TOPIC:-/camera_overhead/image}"
camera_required="${PANEL_CAMERA_REQUIRED:-0}"
if ros2 topic list | filter "^${camera_topic}$" >/dev/null 2>&1; then
  if timeout 5.0 ros2 topic echo --once "${camera_topic}" --qos-reliability best_effort >/dev/null 2>&1; then
    log "OK cámara publica (${camera_topic})."
  elif timeout 5.0 ros2 topic echo --once "${camera_topic}" >/dev/null 2>&1; then
    log "OK cámara publica (${camera_topic}) [qos default]."
  else
    if [[ "$camera_required" == "1" ]]; then
      warn "Cámara requerida no publica (${camera_topic})."
      fail=1
    else
      warn "Cámara no publica (${camera_topic}) [no requerida]."
    fi
  fi
else
  if [[ "$camera_required" == "1" ]]; then
    warn "No existe topic de cámara requerida (${camera_topic})."
    fail=1
  else
    warn "No existe topic de cámara (${camera_topic}) [no requerida]."
  fi
fi

log "Comprobando controller_manager..."
CM_PATH="${PANEL_CONTROLLER_MANAGER:-/controller_manager}"
if ! ros2 service list | filter "${CM_PATH}/list_controllers$" >/dev/null 2>&1; then
  CM_DISCOVERED="$(ros2 service list | filter "/controller_manager/list_controllers$" | head -n 1 | sed 's#/list_controllers$##')"
  if [ -n "${CM_DISCOVERED}" ]; then
    CM_PATH="${CM_DISCOVERED}"
  fi
fi
if ros2 service list | filter "${CM_PATH}/list_controllers$" >/dev/null 2>&1; then
  if command -v ros2 >/dev/null 2>&1; then
    if ros2 control list_controllers -c "${CM_PATH}" >/dev/null 2>&1; then
      log "OK ros2 control list_controllers (${CM_PATH})."
    else
      warn "ros2 control list_controllers fallo (${CM_PATH})."
    fi
  else
    log "Servicio controller_manager detectado (${CM_PATH})."
  fi
else
  warn "No se detecta ${CM_PATH}/list_controllers."
fi

log "Comprobando /system_state..."
if ros2 topic list | filter "^/system_state$" >/dev/null 2>&1; then
  if timeout 4.0 ros2 topic echo --once /system_state >/dev/null 2>&1; then
    log "OK /system_state publica."
  else
    warn "/system_state existe pero no publica (timeout)."
  fi
else
  warn "/system_state no existe."
fi

log "Comprobando /system_diag..."
if ros2 topic list | filter "^/system_diag$" >/dev/null 2>&1; then
  if timeout 4.0 ros2 topic echo --once /system_diag >/dev/null 2>&1; then
    log "OK /system_diag publica."
  else
    warn "/system_diag existe pero no publica (timeout)."
  fi
else
  warn "/system_diag no existe."
fi

log "Comprobando TF con tf_probe..."
if command -v ros2 >/dev/null 2>&1; then
  set +e
  tf_probe_out="$(timeout 6.0 ros2 run ur5_tools tf_probe --ros-args -p use_sim_time:=true -p startup_grace_sec:=2.0 -p timeout_sec:=0.5 2>&1 | head -n 40)"
  set -e
  if echo "${tf_probe_out}" | filter "TF ok world->base_link" >/dev/null 2>&1 \
    && echo "${tf_probe_out}" | filter "TF ok base_link->tool0" >/dev/null 2>&1; then
    log "OK tf_probe (world->base_link, base_link->tool0)."
  else
    warn "tf_probe reporta TF inestable o faltante."
  fi
else
  warn "No se pudo ejecutar tf_probe (ros2 no disponible)."
fi

log "Comprobando JointTrajectory con jt_smoke_test..."
if command -v ros2 >/dev/null 2>&1; then
  if timeout 8.0 ros2 run ur5_tools jt_smoke_test --ros-args -p use_sim_time:=true >/dev/null 2>&1; then
    log "OK jt_smoke_test (joint_states cambian)."
  else
    warn "jt_smoke_test falló (no hay movimiento real)."
  fi
else
  warn "No se pudo ejecutar jt_smoke_test (ros2 no disponible)."
fi

log "Validación básica completada."
exit "${fail}"
