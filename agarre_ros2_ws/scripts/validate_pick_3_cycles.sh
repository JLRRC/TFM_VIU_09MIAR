#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CYCLES="${1:-${PICK_VALIDATE_CYCLES:-3}}"
WAIT_LOOPS="${PICK_VALIDATE_WAIT_LOOPS:-600}"
WAIT_SLEEP_SEC="${PICK_VALIDATE_WAIT_SLEEP_SEC:-1}"
SUMMARY_FILE="$ROOT_DIR/auditoria/pick_${CYCLES}_cycles_summary_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$ROOT_DIR/auditoria"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

run_cycle() {
  local cycle="$1"
  local log_file="$ROOT_DIR/log/ros2_launch.log"
  local start_line=1
  echo "=== CYCLE ${cycle} START ===" | tee -a "$SUMMARY_FILE"
  cd "$ROOT_DIR"
  ./scripts/stop_panel_v2.sh >/dev/null 2>&1 || true
  sleep 4
  PANEL_COLD_BOOT=1 PANEL_FORCE_OFFSCREEN=1 PANEL_START_STACK=1 PANEL_LAUNCH_MOVEIT=1 MOVEIT_MODE=move_group PANEL_AUTO_BRIDGE=0 ./scripts/start_panel_v2.sh --bg >/dev/null 2>&1

  if [[ -f "$log_file" ]]; then
    start_line=$(( $(wc -l < "$log_file") + 1 ))
  fi

  for _ in $(seq 1 50); do
    if ros2 service list 2>/dev/null | grep -q '^/panel/pick_object$' && ros2 service list 2>/dev/null | grep -q '^/panel/select_object$'; then
      break
    fi
    sleep 1
  done

  for _ in $(seq 1 40); do
    if tail -n "+${start_line}" "$log_file" | grep -q 'STATE READY (Sistema listo)'; then
      if ! tail -n "+${start_line}" "$log_file" | grep -q 'ERROR_FATAL'; then
        sleep 3
        break
      fi
    fi
    sleep 1
  done

  ros2 service call /panel/select_object ur5_panel_interfaces/srv/SelectObject "{name: pick_demo}" >/dev/null 2>&1
  sleep 1
  ros2 service call /panel/pick_object std_srvs/srv/Trigger "{}" >/dev/null 2>&1

  local result="TIMEOUT"
  for _ in $(seq 1 "$WAIT_LOOPS"); do
    if tail -n "+${start_line}" "$log_file" | grep -q 'SECUENCIA COMPLETADA EXITOSAMENTE'; then
      result="PASS"
      break
    fi
    if tail -n "+${start_line}" "$log_file" | grep -Eq 'carry_coherence_failed|\[PICK_OBJ\]\[ABORT\]|Error en pick objeto|\[PICK_OBJ\]\[FAIL_CLASS\]'; then
      result="FAIL"
      break
    fi
    sleep "$WAIT_SLEEP_SEC"
  done

  echo "CYCLE_${cycle}=${result}" | tee -a "$SUMMARY_FILE"
  tail -n "+${start_line}" "$log_file" | grep -nE 'SECUENCIA COMPLETADA EXITOSAMENTE|carry_coherence_failed|\[PICK_OBJ\]\[ABORT\]|Error en pick objeto|\[PICK_OBJ\]\[FAIL_CLASS\]' | tail -n 20 | tee -a "$SUMMARY_FILE" || true
  if [[ "$result" == "TIMEOUT" ]]; then
    tail -n "+${start_line}" "$log_file" | tail -n 40 | tee -a "$SUMMARY_FILE" || true
  fi
  ./scripts/stop_panel_v2.sh >/dev/null 2>&1 || true
  sleep 4
  echo "=== CYCLE ${cycle} END ===" | tee -a "$SUMMARY_FILE"
}

pass_count=0
for cycle in $(seq 1 "$CYCLES"); do
  run_cycle "$cycle"
  if grep -q "CYCLE_${cycle}=PASS" "$SUMMARY_FILE"; then
    pass_count=$((pass_count + 1))
  fi
done

echo "SUMMARY PASS=${pass_count}/${CYCLES}" | tee -a "$SUMMARY_FILE"
echo "SUMMARY_FILE=$SUMMARY_FILE" | tee -a "$SUMMARY_FILE"
