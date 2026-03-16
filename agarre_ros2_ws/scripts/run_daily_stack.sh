#!/usr/bin/env bash
set -euo pipefail

WS_DIR="/home/laboratorio/TFM/agarre_ros2_ws"
SCRIPTS_DIR="$WS_DIR/scripts"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_daily_stack.sh start [wait_sec]
  ./scripts/run_daily_stack.sh status
  ./scripts/run_daily_stack.sh stop

Commands:
  start   Starts stack in background, runs health and pipeline checks.
  status  Shows panel/launch status.
  stop    Stops running stack.

Examples:
  ./scripts/run_daily_stack.sh start
  ./scripts/run_daily_stack.sh start 12
  ./scripts/run_daily_stack.sh status
  ./scripts/run_daily_stack.sh stop
EOF
}

source_env() {
  set +u
  source /opt/ros/jazzy/setup.bash
  if [[ -f "$WS_DIR/install/setup.bash" ]]; then
    source "$WS_DIR/install/setup.bash"
  fi
  set -u

  export VISION_DIR="${VISION_DIR:-/home/laboratorio/TFM/agarre_inteligente}"
  export INFER_CKPT="${INFER_CKPT:-$VISION_DIR/experiments/EXP1_SIMPLE_RGB/seed_0/checkpoints/best.pth}"
  export GZ_SIM_RESOURCE_PATH="${WS_DIR}/models:${WS_DIR}/worlds:${WS_DIR}/install:/opt/ros/jazzy/share:${GZ_SIM_RESOURCE_PATH:-}"
}

start_flow() {
  local wait_sec="${1:-10}"
  echo "[DAILY] workspace: $WS_DIR"
  echo "[DAILY] wait_sec: $wait_sec"

  source_env

  cd "$WS_DIR"
  "$SCRIPTS_DIR/start_panel_v2.sh" --bg

  echo "[DAILY] waiting ${wait_sec}s before checks..."
  sleep "$wait_sec"

  echo "[DAILY] running quick status..."
  "$SCRIPTS_DIR/status_panel_v2.sh" || true

  echo "[DAILY] running startup health check..."
  "$SCRIPTS_DIR/diag_startup_health.sh" || true

  echo "[DAILY] running pipeline check..."
  if "$WS_DIR/check_pipeline.sh"; then
    echo "[DAILY] RESULT: PASS"
    return 0
  fi

  echo "[DAILY] RESULT: FAIL"
  return 1
}

status_flow() {
  cd "$WS_DIR"
  "$SCRIPTS_DIR/status_panel_v2.sh"
}

stop_flow() {
  cd "$WS_DIR"
  "$SCRIPTS_DIR/stop_panel_v2.sh"
}

cmd="${1:-start}"
case "$cmd" in
  start)
    start_flow "${2:-10}"
    ;;
  status)
    status_flow
    ;;
  stop)
    stop_flow
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "Unknown command: $cmd"
    usage
    exit 2
    ;;
esac
