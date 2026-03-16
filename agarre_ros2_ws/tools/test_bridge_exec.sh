#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/tools/test_bridge_exec.sh
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
set -u

POSE_X="${1:-0.43}"
POSE_Y="${2:-0.00}"
POSE_Z="${3:-0.075}"
POSE_TOPIC="${POSE_TOPIC:-/desired_grasp}"
RESULT_TOPIC="${RESULT_TOPIC:-/desired_grasp/result}"
TIMEOUT_SEC="${TIMEOUT_SEC:-15}"

tmp_result="$(mktemp)"
cleanup() {
  rm -f "$tmp_result"
}
trap cleanup EXIT

run_cmd() {
  local title="$1"
  shift
  echo
  echo "===== ${title} ====="
  echo "+ $*"
  "$@"
  local rc=$?
  echo "[RC] ${rc}"
  return $rc
}

collect_diag() {
  run_cmd "Controllers list" timeout 6s ros2 control list_controllers || true
  run_cmd "MoveIt nodes" timeout 6s bash -lc 'ros2 node list | grep -E "move_group|moveit|bridge" || true' || true
  run_cmd "Trajectory topic info" timeout 6s ros2 topic info /joint_trajectory_controller/joint_trajectory -v || true
  run_cmd "FollowJointTrajectory actions" timeout 6s bash -lc 'ros2 action list -t | grep FollowJointTrajectory || true' || true
}

echo "[TEST] bridge exec started: $(date -Iseconds)"
echo "[TEST] pose_topic=${POSE_TOPIC} result_topic=${RESULT_TOPIC}"
echo "[TEST] target base_link=(${POSE_X}, ${POSE_Y}, ${POSE_Z})"

pose_subs="$(ros2 topic info "${POSE_TOPIC}" -v 2>/dev/null | awk '/Subscribers:/ {print $2; exit}')"
if [[ -z "${pose_subs}" ]]; then
  pose_subs=0
fi
if [[ "${pose_subs}" -le 0 ]]; then
  echo "[FAIL] ${POSE_TOPIC} sin subscriptores (pose_subs=${pose_subs})"
  collect_diag
  exit 1
fi

run_cmd "Publish PRE_GRASP test pose" \
  timeout 5s ros2 topic pub --once "${POSE_TOPIC}" geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: ${POSE_X}, y: ${POSE_Y}, z: ${POSE_Z}}, orientation: {x: 0.0, y: 0.7071068, z: 0.7071068, w: 0.0}}}"

echo
echo "===== Wait result ====="
echo "+ timeout ${TIMEOUT_SEC}s ros2 topic echo --once ${RESULT_TOPIC}"
if ! timeout "${TIMEOUT_SEC}s" ros2 topic echo --once "${RESULT_TOPIC}" > "${tmp_result}"; then
  echo "[FAIL] timeout esperando ${RESULT_TOPIC}"
  collect_diag
  exit 1
fi

cat "${tmp_result}"

result_eval="$(
python3 - "$tmp_result" <<'PY'
import ast
import json
import pathlib
import re
import sys

text = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace")
m = re.search(r"data:\s*(.+)", text)
if not m:
    print("status=unknown")
    sys.exit(0)
raw = m.group(1).strip()
try:
    payload = ast.literal_eval(raw)
    data = json.loads(payload) if isinstance(payload, str) else {}
except Exception:
    print("status=parse_error")
    sys.exit(0)
success = bool(data.get("success", False))
plan_ok = bool(data.get("plan_ok", False))
exec_ok = bool(data.get("exec_ok", False))
message = str(data.get("message", ""))
print(f"status={'ok' if success else 'fail'}")
print(f"plan_ok={str(plan_ok).lower()}")
print(f"exec_ok={str(exec_ok).lower()}")
print(f"message={message}")
PY
)"
echo
echo "===== Result summary ====="
echo "${result_eval}"

if grep -q "status=ok" <<< "${result_eval}"; then
  echo "[PASS] bridge execution success"
  exit 0
fi

echo "[FAIL] bridge execution failed; collecting diagnostics"
collect_diag
exit 1
