#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_ID="${1:-$ROOT_DIR/../auditoria/spatial_$(date +%Y%m%d)/moveit_lift_validation_${STAMP}}"
WAIT_READY_SEC="${WAIT_READY_SEC:-80}"
WAIT_LIFT_SEC="${WAIT_LIFT_SEC:-220}"
CAPTURE_INTERVAL_SEC="${CAPTURE_INTERVAL_SEC:-0.25}"
CAPTURE_TIMEOUT_SEC="${CAPTURE_TIMEOUT_SEC:-120}"
TRACE_SAMPLE_HZ="${TRACE_SAMPLE_HZ:-25}"

mkdir -p "$RUN_ID/camera_frames"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

LOG_FILE="$ROOT_DIR/log/ros2_launch.log"
CAP_PID=""
TRACE_PID=""

cleanup() {
  if [[ -n "$TRACE_PID" ]]; then
    kill "$TRACE_PID" >/dev/null 2>&1 || true
    wait "$TRACE_PID" >/dev/null 2>&1 || true
  fi
  if [[ -n "$CAP_PID" ]]; then
    wait "$CAP_PID" >/dev/null 2>&1 || true
  fi
  "$ROOT_DIR/scripts/stop_panel_v2.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

cd "$ROOT_DIR"
"$ROOT_DIR/scripts/stop_panel_v2.sh" >/dev/null 2>&1 || true
sleep 4

export PANEL_COLD_BOOT="${PANEL_COLD_BOOT:-0}"
export PANEL_FORCE_OFFSCREEN=1
export PANEL_GZ_GUI=0
export PANEL_START_STACK=1
export PANEL_LAUNCH_MOVEIT=1
export MOVEIT_MODE="${MOVEIT_MODE:-move_group}"
export PANEL_AUTO_BRIDGE=0
export PANEL_AUTO_RUN_PICK_DEMO=0
export ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS=,
export PANEL_PICK_OBJECT_RETURN_TO_MESA=0
export PANEL_PICK_OBJECT_HOME_BEFORE_CESTA=0

"$ROOT_DIR/scripts/start_panel_v2.sh" --bg >"$RUN_ID/start_bg.log" 2>&1

start_line=1
if [[ -f "$LOG_FILE" ]]; then
  start_line=$(( $(wc -l <"$LOG_FILE") + 1 ))
fi

for _ in $(seq 1 "$WAIT_READY_SEC"); do
  if ros2 service list 2>/dev/null | grep -q '^/panel/pick_demo$' &&
     ros2 service list 2>/dev/null | grep -q '^/panel/select_object$'; then
    break
  fi
  sleep 1
done

for _ in $(seq 1 "$WAIT_READY_SEC"); do
  if [[ -f "$LOG_FILE" ]] &&
     tail -n +"$start_line" "$LOG_FILE" | grep -q 'STATE READY (Sistema listo)'; then
    sleep 2
    break
  fi
  sleep 1
done

python3 "$ROOT_DIR/scripts/capture_camera_frames.py" \
  --output "$RUN_ID/camera_frames" \
  --interval "$CAPTURE_INTERVAL_SEC" \
  --timeout "$CAPTURE_TIMEOUT_SEC" \
  >"$RUN_ID/capture.log" 2>&1 &
CAP_PID=$!

python3 "$ROOT_DIR/scripts/grasp_audit_trace_capture.py" \
  --output "$RUN_ID/trace.jsonl" \
  --label "$(basename "$RUN_ID")" \
  --sample-hz "$TRACE_SAMPLE_HZ" \
  --frames world base_link tool0 rg2_tcp rg2_pinch_center \
  >"$RUN_ID/trace.log" 2>&1 &
TRACE_PID=$!

select_reply="$(ros2 service call /panel/select_object ur5_panel_interfaces/srv/SelectObject '{name: pick_demo}' 2>&1 || true)"
printf '%s\n' "$select_reply" >"$RUN_ID/select_call.log"

pick_reply="$(ros2 service call /panel/pick_object std_srvs/srv/Trigger '{}' 2>&1 || true)"
printf '%s\n' "$pick_reply" >"$RUN_ID/pick_call.log"

result="TIMEOUT"
for _ in $(seq 1 "$WAIT_LIFT_SEC"); do
  if [[ -f "$LOG_FILE" ]] &&
     tail -n +"$start_line" "$LOG_FILE" | grep -q '\[MOVEIT2\]\[STEP\] label=LIFT state=ok'; then
    result="LIFT_OK"
    break
  fi
  if [[ -f "$LOG_FILE" ]] &&
     tail -n +"$start_line" "$LOG_FILE" | grep -Eq 'carry_coherence_failed|\[PICK_OBJ\]\[ABORT\]|\[PICK_OBJ\]\[FAIL_CLASS\]|Error en pick objeto'; then
    result="FAIL"
    break
  fi
  sleep 1
done

sleep 3

{
  echo "RUN_ID=$RUN_ID"
  echo "RESULT=$result"
  echo "START_LINE=$start_line"
} | tee "$RUN_ID/result.env"
