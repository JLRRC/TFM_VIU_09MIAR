#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/tools/test_bridge_result.sh
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
set -u

POSE_X="${1:-0.43}"
POSE_Y="${2:-0.00}"
POSE_Z="${3:-0.075}"
POSE_TOPIC="${POSE_TOPIC:-/desired_grasp}"
RESULT_TOPIC="${RESULT_TOPIC:-/desired_grasp/result}"
TIMEOUT_SEC="${TIMEOUT_SEC:-5}"

TMP_RESULT="$(mktemp)"
PROBE_LOG="$(mktemp)"
PROBE_PID=""
cleanup() {
  if [[ -n "${PROBE_PID}" ]] && kill -0 "${PROBE_PID}" 2>/dev/null; then
    kill -TERM "${PROBE_PID}" 2>/dev/null || true
    sleep 1
    kill -KILL "${PROBE_PID}" 2>/dev/null || true
  fi
  rm -f "${TMP_RESULT}" "${PROBE_LOG}"
}
trap cleanup EXIT

diag_on_fail() {
  echo
  echo "===== Diagnostics ====="
  echo "+ ros2 topic info ${POSE_TOPIC} -v"
  timeout -k 1s 4s ros2 topic info "${POSE_TOPIC}" -v || true
  echo "+ ros2 topic info ${RESULT_TOPIC} -v"
  timeout -k 1s 4s ros2 topic info "${RESULT_TOPIC}" -v || true
  echo "+ ros2 node list | grep -E \"bridge|move_group|moveit\""
  timeout -k 1s 4s bash -lc 'ros2 node list | grep -E "bridge|move_group|moveit" || true' || true
}

echo "[TEST] bridge-result started: $(date -Iseconds)"
echo "[TEST] pose_topic=${POSE_TOPIC} result_topic=${RESULT_TOPIC} timeout=${TIMEOUT_SEC}s"
echo "[TEST] target base_link=(${POSE_X}, ${POSE_Y}, ${POSE_Z})"

STAMP_NS="$(date +%s%N)"
echo "[TEST] stamp_ns=${STAMP_NS}"

python3 - "${POSE_TOPIC}" "${RESULT_TOPIC}" "${POSE_X}" "${POSE_Y}" "${POSE_Z}" "${TIMEOUT_SEC}" "${STAMP_NS}" "${TMP_RESULT}" >"${PROBE_LOG}" 2>&1 <<'PY' &
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Probe(Node):
    def __init__(self, pose_topic: str, result_topic: str):
        super().__init__("test_bridge_result_probe")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pose_topic = pose_topic
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, qos)
        self.result_sub = self.create_subscription(String, result_topic, self._cb, qos)
        self.result_raw = ""

    def _cb(self, msg: String) -> None:
        self.result_raw = msg.data


def main() -> int:
    pose_topic = sys.argv[1]
    result_topic = sys.argv[2]
    pose_x = float(sys.argv[3])
    pose_y = float(sys.argv[4])
    pose_z = float(sys.argv[5])
    timeout_sec = float(sys.argv[6])
    stamp_ns = int(sys.argv[7])
    out_path = sys.argv[8]

    rclpy.init()
    node = Probe(pose_topic, result_topic)

    # Wait briefly for a consumer of /desired_grasp.
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.count_subscribers(node.pose_topic) > 0:
            break
    if node.count_subscribers(node.pose_topic) <= 0:
        print(f"[FAIL] sin subscriptores en {pose_topic}", flush=True)
        return 2

    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.header.stamp.sec = stamp_ns // 1_000_000_000
    pose.header.stamp.nanosec = stamp_ns % 1_000_000_000
    pose.pose.position.x = pose_x
    pose.pose.position.y = pose_y
    pose.pose.position.z = pose_z
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.7071068
    pose.pose.orientation.z = 0.7071068
    pose.pose.orientation.w = 0.0
    node.pose_pub.publish(pose)

    start = time.monotonic()
    while (time.monotonic() - start) < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.result_raw:
            break
    if not node.result_raw:
        print(f"[FAIL] timeout esperando {result_topic}", flush=True)
        return 3

    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write(f"data: {node.result_raw!r}\\n")
    print(f"[TEST] result_raw={node.result_raw}", flush=True)
    return 0


rc = 99
try:
    rc = main()
except Exception as exc:
    print(f"[FAIL] probe exception: {exc}", flush=True)
    rc = 9
finally:
    # Avoid hangs in rclpy.shutdown seen under heavy FastDDS churn.
    os._exit(rc)
PY
PROBE_PID=$!

PROBE_TIMEOUT=$((TIMEOUT_SEC + 10))
DEADLINE=$((SECONDS + PROBE_TIMEOUT))
PROBE_RC=0
while kill -0 "${PROBE_PID}" 2>/dev/null; do
  if (( SECONDS >= DEADLINE )); then
    echo "[FAIL] probe Python no finalizo en ${PROBE_TIMEOUT}s"
    kill -TERM "${PROBE_PID}" 2>/dev/null || true
    sleep 1
    kill -KILL "${PROBE_PID}" 2>/dev/null || true
    PROBE_RC=124
    break
  fi
  sleep 1
done
if [[ "${PROBE_RC}" -eq 0 ]]; then
  wait "${PROBE_PID}" || PROBE_RC=$?
fi
cat "${PROBE_LOG}"
if [[ "${PROBE_RC}" -ne 0 ]]; then
  diag_on_fail
  exit 1
fi

cat "${TMP_RESULT}"
python3 - "${TMP_RESULT}" "${STAMP_NS}" <<'PY'
import ast
import json
import pathlib
import re
import sys

text = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace")
expected = int(sys.argv[2])
m = re.search(r"data:\s*(.+)", text)
if not m:
    print("[FAIL] payload result ausente")
    sys.exit(1)
raw = m.group(1).strip()
try:
    payload = ast.literal_eval(raw)
    data = json.loads(payload) if isinstance(payload, str) else {}
except Exception as exc:
    print(f"[FAIL] payload invalido: {exc}")
    sys.exit(1)
print(
    "[RESULT] "
    f"request_id={data.get('request_id')} success={data.get('success')} "
    f"plan_ok={data.get('plan_ok')} exec_ok={data.get('exec_ok')} "
    f"target_stamp_ns={data.get('target_stamp_ns')} message={data.get('message')}"
)
got_stamp = int(data.get("target_stamp_ns", 0) or 0)
if got_stamp != expected:
    print(f"[WARN] stamp mismatch expected={expected} got={got_stamp}")
PY
