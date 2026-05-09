#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/validate_startup_repro.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="/home/laboratorio/TFM/agarre_ros2_ws"
CYCLES="${1:-3}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$WS_DIR/report/repro_startup/$STAMP"
mkdir -p "$OUT_DIR"

cd "$WS_DIR"

echo "[REPRO] ws=$WS_DIR" | tee -a "$OUT_DIR/summary.log"
echo "[REPRO] cycles=$CYCLES" | tee -a "$OUT_DIR/summary.log"
echo "[REPRO] out=$OUT_DIR" | tee -a "$OUT_DIR/summary.log"

fails=0

source_ros() {
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$WS_DIR/install/setup.bash"
  set -u
}

run_pipeline_check() {
  source_ros
  local expected_action="${PANEL_EXPECTED_TRAJ_ACTION:-/joint_trajectory_controller/follow_joint_trajectory}"
  local controller_manager="${PANEL_CONTROLLER_MANAGER:-/controller_manager}"
  local nodes actions controllers joints result="PASS"
  local joint_regex="shoulder_pan_joint|shoulder_lift_joint|elbow_joint|wrist_1_joint|wrist_2_joint|wrist_3_joint"

  nodes="$(ros2 node list 2>/dev/null | grep '^/' | wc -l | tr -d ' ' || true)"
  actions="$(ros2 action list 2>/dev/null | grep -c "^${expected_action}$" || true)"
  controllers=0
  for _ in $(seq 1 10); do
    controllers="$(ros2 control list_controllers --controller-manager "$controller_manager" 2>/dev/null | grep -Ec '[[:space:]]active$' || true)"
    [[ "$controllers" =~ ^[0-9]+$ ]] || controllers=0
    if [[ "$controllers" -ge 3 ]]; then
      break
    fi
    sleep 1
  done

  local js_msg
  js_msg="$(timeout 8s ros2 topic echo --once /joint_states 2>/dev/null || true)"
  joints="$(printf '%s\n' "$js_msg" | grep -Eo "$joint_regex" | sort -u | wc -l | tr -d ' ' || true)"

  echo "NODES=$nodes"
  echo "ACTION_AVAILABLE=$actions ($expected_action)"
  echo "ACTIVE_CONTROLLERS=$controllers"
  echo "UR5_JOINTS_IN_JOINT_STATES=$joints"

  [[ "$nodes" -ge 20 ]] || result="FAIL"
  [[ "$actions" -ge 1 ]] || result="FAIL"
  [[ "$controllers" -ge 3 ]] || result="FAIL"
  [[ "$joints" -eq 6 ]] || result="FAIL"

  echo "RESULTADO: $result"
  [[ "$result" == "PASS" ]]
}

capture_diag_check() {
  local output_json="$1"
  source_ros
  python3 ./scripts/capture_system_diag.py \
    --timeout 15.0 \
    --require-geometry-ok \
    --require-state READY \
    --output "$output_json"
}

for i in $(seq 1 "$CYCLES"); do
  echo "" | tee -a "$OUT_DIR/summary.log"
  echo "=== CYCLE $i START ===" | tee -a "$OUT_DIR/summary.log"

  ./scripts/stop_panel_v2.sh > "$OUT_DIR/cycle${i}_stop.log" 2>&1 || true
  ./scripts/start_panel_v2.sh --bg > "$OUT_DIR/cycle${i}_start.log" 2>&1

  sleep 10

  diag_ok=0
  if capture_diag_check "$OUT_DIR/cycle${i}_system_diag.json" > "$OUT_DIR/cycle${i}_system_diag.log" 2>&1; then
    diag_ok=1
  fi

  pipeline_ok=0
  if run_pipeline_check > "$OUT_DIR/cycle${i}_pipeline.log" 2>&1; then
    pipeline_ok=1
  fi

  if [[ "$diag_ok" -eq 1 && "$pipeline_ok" -eq 1 ]]; then
    ./scripts/stop_panel_v2.sh > "$OUT_DIR/cycle${i}_post_stop.log" 2>&1 || true
    source_ros
    post_nodes="$(ros2 node list 2>/dev/null | grep '^/' | wc -l | tr -d ' ' || true)"
    post_nodes="${post_nodes:-0}"
    echo "POST_STOP_NODES=$post_nodes" >> "$OUT_DIR/cycle${i}_pipeline.log"

    if [[ "$post_nodes" -eq 0 ]]; then
      echo "[REPRO] cycle=$i PASS" | tee -a "$OUT_DIR/summary.log"
    else
      echo "[REPRO] cycle=$i FAIL (post_stop_nodes=$post_nodes)" | tee -a "$OUT_DIR/summary.log"
      fails=$((fails+1))
    fi
  else
    echo "[REPRO] cycle=$i FAIL (diag_ok=$diag_ok pipeline_ok=$pipeline_ok)" | tee -a "$OUT_DIR/summary.log"
    fails=$((fails+1))
  fi

  tail -n 8 "$OUT_DIR/cycle${i}_system_diag.log" >> "$OUT_DIR/summary.log"
  tail -n 8 "$OUT_DIR/cycle${i}_pipeline.log" >> "$OUT_DIR/summary.log"
done

echo "" | tee -a "$OUT_DIR/summary.log"
if [[ "$fails" -eq 0 ]]; then
  echo "[REPRO] RESULT=PASS fails=0/$CYCLES" | tee -a "$OUT_DIR/summary.log"
  exit 0
fi

echo "[REPRO] RESULT=FAIL fails=$fails/$CYCLES" | tee -a "$OUT_DIR/summary.log"
exit 2
