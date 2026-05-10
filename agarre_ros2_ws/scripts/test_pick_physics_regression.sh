#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/test_pick_physics_regression.sh
# Contenido: regresion live del pick fisico pick_demo en Gazebo.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

LIVE="${TFM_PHYSICAL_PICK_LIVE:-0}"
ASSUME_STACK="${TFM_PICK_ASSUME_STACK:-0}"
RESET_OBJECT="${TFM_PICK_RESET_OBJECT:-1}"
TIMEOUT_SEC="${TFM_PICK_TIMEOUT_SEC:-420}"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT_DIR="${TFM_PICK_REGRESSION_DIR:-$ROOT_DIR/reports/pick_physics_regression_$STAMP}"
STACK_LOG="$REPORT_DIR/stack.log"
RESULT_JSON="$REPORT_DIR/pick_result.json"
METRICS_JSON="$REPORT_DIR/physical_metrics.json"
STACK_PID=""

if [[ "$LIVE" != "1" ]]; then
  echo "[PICK_PHYSICS_REGRESSION][SKIP] Set TFM_PHYSICAL_PICK_LIVE=1 to run Gazebo physical pick."
  exit 0
fi

mkdir -p "$REPORT_DIR"

set +u
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

cleanup() {
  if [[ -n "$STACK_PID" ]]; then
    kill "$STACK_PID" >/dev/null 2>&1 || true
    wait "$STACK_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

wait_for_ready() {
  local deadline=$((SECONDS + 180))
  while (( SECONDS < deadline )); do
    if timeout 5 ros2 action list 2>/dev/null | grep -q '^/pick_place$' \
      && timeout 5 ros2 action list 2>/dev/null | grep -q '^/orchestrator/plan_to_pose$' \
      && timeout 5 ros2 topic echo /joint_states --once 2>/dev/null | grep -q 'position'; then
      return 0
    fi
    sleep 2
  done
  return 1
}

if [[ "$ASSUME_STACK" != "1" ]]; then
  if pgrep -af 'ros2 launch ur5_bringup ur5_stack.launch.py|gz sim -s -r .*/world_runtime.sdf' >/dev/null; then
    echo "[PICK_PHYSICS_REGRESSION][ERROR] Existing stack detected. Stop it or set TFM_PICK_ASSUME_STACK=1."
    pgrep -af 'ros2 launch ur5_bringup ur5_stack.launch.py|gz sim -s -r .*/world_runtime.sdf' || true
    exit 2
  fi
  export PANEL_CAMERA_REQUIRED=0
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
  ros2 launch ur5_bringup ur5_stack.launch.py \
    launch_panel:=false moveit_mode:=move_group headless:=true use_sim_time:=true \
    > "$STACK_LOG" 2>&1 &
  STACK_PID="$!"
fi

if ! wait_for_ready; then
  echo "[PICK_PHYSICS_REGRESSION][ERROR] Stack did not become ready. See $STACK_LOG"
  exit 3
fi

if [[ "$RESET_OBJECT" == "1" ]]; then
  GZ_PID="$(pgrep -f 'gz sim -s -r .*/world_runtime.sdf' | head -n 1 || true)"
  if [[ -n "$GZ_PID" && -r "/proc/$GZ_PID/environ" ]]; then
    GZ_PARTITION_VALUE="$(tr '\0' '\n' < "/proc/$GZ_PID/environ" | awk -F= '$1=="GZ_PARTITION" {print $2; exit}')"
    export GZ_PARTITION="${GZ_PARTITION_VALUE:-${GZ_PARTITION:-}}"
  fi
  if command -v gz >/dev/null 2>&1; then
    gz service -s /world/ur5_mesa_objetos/set_pose/blocking \
      --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
      --req 'name: "pick_demo" position { x: -0.42 y: 0.0 z: 0.876 } orientation { w: 1.0 }' \
      > "$REPORT_DIR/reset_pick_demo.txt" 2>&1 || true
  fi
fi

python3 scripts/run_single_pick_pickdemo.py \
  --object pick_demo \
  --timeout "$TIMEOUT_SEC" \
  --out "$RESULT_JSON" \
  2>&1 | tee "$REPORT_DIR/pick_runner.log"

python3 - "$RESULT_JSON" "${STACK_LOG:-$REPORT_DIR/pick_runner.log}" "$METRICS_JSON" <<'PY'
import json
import math
import re
import sys
from pathlib import Path

result_path = Path(sys.argv[1])
log_path = Path(sys.argv[2])
metrics_path = Path(sys.argv[3])
result = json.loads(result_path.read_text())
if result.get("verdict") != "SUCCESS" or result.get("result_success") is not True:
    raise SystemExit(f"pick action did not succeed: {result.get('result_reason')}")

text = log_path.read_text(errors="replace") if log_path.exists() else ""
follow_re = re.compile(
    r"demo_transport_follow_tick .* desired=\((-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)\) "
    r"tcp=\((-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)\)"
)
samples = []
for match in follow_re.finditer(text):
    desired = tuple(float(match.group(i)) for i in range(1, 4))
    tcp = tuple(float(match.group(i)) for i in range(4, 7))
    samples.append((desired, tcp))

if not samples:
    raise SystemExit("no Gazebo follow ticks found; action success alone is not physical evidence")

initial_z = samples[0][0][2]
best_lift_delta = max(sample[0][2] - initial_z for sample in samples)
best_tcp_dist = min(
    math.dist(sample[0], sample[1])
    for sample in samples
)
attach_ok = "attach_route_decision object=pick_demo" in text and "geometry_ok=true" in text
done_ok = "result success=True" in text or result.get("result_success") is True
if best_lift_delta < 0.05:
    raise SystemExit(f"object did not lift enough: best_lift_delta={best_lift_delta:.4f}m")
if best_tcp_dist > 0.05:
    raise SystemExit(f"object did not stay near TCP: best_tcp_dist={best_tcp_dist:.4f}m")
if not attach_ok:
    raise SystemExit("attach gate was not confirmed in stack log")
if "carry_follow_lost" in text:
    raise SystemExit("carry follow was lost during transport")
if not done_ok:
    raise SystemExit("DONE/result success not confirmed")

metrics = {
    "best_lift_delta_m": best_lift_delta,
    "best_tcp_dist_m": best_tcp_dist,
    "follow_tick_count": len(samples),
    "result_json": str(result_path),
    "source_log": str(log_path),
}
metrics_path.write_text(json.dumps(metrics, indent=2) + "\n")
print(json.dumps(metrics, indent=2))
PY

echo "[PICK_PHYSICS_REGRESSION] OK evidence=$REPORT_DIR"