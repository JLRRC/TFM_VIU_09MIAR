#!/usr/bin/env bash
# run_directo_validation.sh — Orquestador de corrida fresca de DIRECTO.
# Lanza: panel (con runner offscreen) + capture + benchmark y recoge resultados.
#
# Uso:
#   ./scripts/run_directo_validation.sh
#
# Variables de entorno opcionales:
#   DIRECTO_TIMEOUT_SEC  — timeout total del runner (default: 300s)
#   CAPTURE_TIMEOUT_SEC  — timeout del capturador (default: 300s)
#   OUT_DIR              — directorio de salida (default: auto)

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
AUDIT_DIR="$WS_DIR/../auditoria_spatial_$(date +%Y%m%d)"
RUN_ID="directo_validation_$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_DIR:-$AUDIT_DIR/$RUN_ID}"

mkdir -p "$OUT_DIR"

# Cargar entorno ROS 2 base + overlay del workspace.
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
    source "$WS_DIR/install/setup.bash"
fi

DIRECTO_TIMEOUT_SEC="${DIRECTO_TIMEOUT_SEC:-600}"
CAPTURE_TIMEOUT_SEC="${CAPTURE_TIMEOUT_SEC:-300}"

WORLD="${GZ_WORLD:-ur5_mesa_objetos}"
POSE_TOPIC="/world/${WORLD}/pose/info"
TRACE_FILE="$OUT_DIR/directo_trace.jsonl"
HELPER_LOG="$OUT_DIR/helper.log"
CAPTURE_LOG="$OUT_DIR/capture.log"
BENCHMARK_JSON="$OUT_DIR/benchmark.json"
BENCHMARK_STDOUT_JSON="$OUT_DIR/benchmark_stdout.json"
SUMMARY="$OUT_DIR/orchestrator_summary.txt"

echo "[ORCH] Corrida DIRECTO: $RUN_ID"
echo "[ORCH] Salida: $OUT_DIR"

# --- Limpiar procesos Gazebo/bridge residuales de corridas anteriores ---
echo "[ORCH] Limpiando procesos residuales..."
pkill -f "gz sim" 2>/dev/null || true
pkill -f "gz_server" 2>/dev/null || true
pkill -f "ros_gz_bridge" 2>/dev/null || true
pkill -f "parameter_bridge" 2>/dev/null || true
pkill -f "ros2_control_node" 2>/dev/null || true
pkill -f "controller_manager" 2>/dev/null || true
pkill -f "spawner" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
sleep 4
# Limpiar shared memory residual de FastDDS/CycloneDDS (evita 'Failed init_port' errors)
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
echo "[ORCH] Limpieza completada"

# --- Lanzar el runner del panel offscreen ---
env \
    QT_QPA_PLATFORM=offscreen \
    PANEL_FORCE_OFFSCREEN=1 \
    PANEL_START_STACK=0 \
    PANEL_SKIP_CLEANUP=1 \
    PANEL_CAMERA_REQUIRED=0 \
    PANEL_FATAL_STOPS_ALL=0 \
    PANEL_GZ_HEALTH_FREEZE_SEC=30 \
    PANEL_ALLOW_UNSETTLED_ON_TIMEOUT=1 \
    PANEL_TF_INIT_GRACE_SEC=300 \
    PANEL_PICK_DEMO_MAX_PROMOTED_STABLE_AGE_SEC=300 \
    PANEL_PICK_DEMO_MOVE_SEC=15 \
    PANEL_PICK_DEMO_STEP_TIMEOUT_EXTRA_SEC=60 \
    PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC=120 \
    PANEL_PICK_DEMO_IK_SEED_JOINTS=0.0,-1.5708,0.0,-1.5708,0.0,0.0 \
    DIRECTO_CLICK_DELAY_MS=8000 \
    "DIRECTO_EXIT_AFTER_MS=$(( DIRECTO_TIMEOUT_SEC * 1000 ))" \
    DIRECTO_RETRY_MS=5000 \
    DIRECTO_MAX_ATTEMPTS=90 \
    python3 "$SCRIPT_DIR/run_directo_button_offscreen.py" \
    >"$HELPER_LOG" 2>&1 &
HELPER_PID=$!
echo "[ORCH] Panel runner PID=$HELPER_PID"

# --- Lanzar el capturador en paralelo ---
env \
    python3 "$SCRIPT_DIR/grasp_audit_trace_capture.py" \
        --label DIRECTO \
        --output "$TRACE_FILE" \
        --pose-topic "$POSE_TOPIC" \
    >"$CAPTURE_LOG" 2>&1 &
CAPTURE_PID=$!
echo "[ORCH] Capture PID=$CAPTURE_PID"

# --- Esperar al runner del panel ---
wait "$HELPER_PID" && HELPER_RC=0 || HELPER_RC=$?
echo "[ORCH] Runner terminó rc=$HELPER_RC"

# Dar unos segundos más al capturador para que cierre limpiamente
sleep 3
kill "$CAPTURE_PID" 2>/dev/null || true
wait "$CAPTURE_PID" && CAPTURE_RC=0 || CAPTURE_RC=$?
echo "[ORCH] Capture terminó rc=$CAPTURE_RC"

# --- Ejecutar el benchmark ---
python3 "$SCRIPT_DIR/grasp_audit_benchmark.py" \
    --run "DIRECTO:directo:$TRACE_FILE" \
    --json-out "$BENCHMARK_JSON" \
    >"$BENCHMARK_STDOUT_JSON" 2>&1 && BENCHMARK_RC=0 || BENCHMARK_RC=$?
echo "[ORCH] Benchmark rc=$BENCHMARK_RC"

# --- Resumen ---
{
    echo "helper_final_rc=$HELPER_RC"
    echo "capture_rc=$CAPTURE_RC"
    echo "benchmark_rc=$BENCHMARK_RC"
} > "$SUMMARY"

echo "[ORCH] Resumen guardado en $SUMMARY"
echo "[ORCH] Logs: $OUT_DIR"
echo "helper_final_rc=$HELPER_RC capture_rc=$CAPTURE_RC benchmark_rc=$BENCHMARK_RC"
