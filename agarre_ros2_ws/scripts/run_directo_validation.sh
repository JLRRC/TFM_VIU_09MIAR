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
AUDIT_ROOT="${AUDIT_ROOT:-$WS_DIR/../auditoria}"
AUDIT_DIR="${AUDIT_DIR:-$AUDIT_ROOT/spatial_$(date +%Y%m%d)}"
RUN_ID="directo_validation_$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_DIR:-$AUDIT_DIR/$RUN_ID}"
RUNTIME_PROFILE="${RUNTIME_PROFILE:-$WS_DIR/scripts/panel_runtime_validated.env}"

mkdir -p "$OUT_DIR"

# Cargar entorno ROS 2 base + overlay del workspace.
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
    source "$WS_DIR/install/setup.bash"
fi
if [[ ! -f "$RUNTIME_PROFILE" ]]; then
    echo "[ORCH][ERROR] Falta el perfil runtime validado: $RUNTIME_PROFILE"
    exit 1
fi
# shellcheck disable=SC1090
source "$RUNTIME_PROFILE"
export PANEL_RUNTIME_PROFILE_PATH="$RUNTIME_PROFILE"

DIRECTO_TIMEOUT_SEC="${DIRECTO_TIMEOUT_SEC:-600}"
CAPTURE_TIMEOUT_SEC="${CAPTURE_TIMEOUT_SEC:-300}"
CANONICAL_LAUNCHER="${CANONICAL_LAUNCHER:-./lanzar_panelc2.sh}"

WORLD="${GZ_WORLD:-ur5_mesa_objetos}"
POSE_TOPIC="/world/${WORLD}/pose/info"
TRACE_FILE="$OUT_DIR/directo_trace.jsonl"
HELPER_LOG="$OUT_DIR/helper.log"
CAPTURE_LOG="$OUT_DIR/capture.log"
BENCHMARK_JSON="$OUT_DIR/benchmark.json"
BENCHMARK_STDOUT_JSON="$OUT_DIR/benchmark_stdout.json"
SUMMARY="$OUT_DIR/orchestrator_summary.txt"
STACK_STARTUP_MAX_ATTEMPTS="${STACK_STARTUP_MAX_ATTEMPTS:-3}"
SYSTEM_DIAG_TIMEOUT_SEC="${SYSTEM_DIAG_TIMEOUT_SEC:-30}"

echo "[ORCH] Corrida DIRECTO: $RUN_ID"
echo "[ORCH] Salida: $OUT_DIR"
echo "[ORCH] Perfil runtime: $PANEL_RUNTIME_VALIDATED_PROFILE ($RUNTIME_PROFILE)"

cleanup_residual_processes() {
    echo "[ORCH] Limpiando procesos residuales..."
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "gz_server" 2>/dev/null || true
    pkill -f "ros_gz_bridge" 2>/dev/null || true
    pkill -f "parameter_bridge" 2>/dev/null || true
    pkill -f "ros2_control_node" 2>/dev/null || true
    pkill -f "controller_manager" 2>/dev/null || true
    pkill -f "spawner" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "world_tf_publisher" 2>/dev/null || true
    # Residuales críticos entre corridas: gripper_attach_backend y move_group
    # pueden interceptar llamadas de release/attach del ciclo siguiente.
    pkill -f "gripper_attach_backend" 2>/dev/null || true
    pkill -f "controller_bootstrap" 2>/dev/null || true
    pkill -f "planning_scene_sync" 2>/dev/null || true
    pkill -f "system_state_manager" 2>/dev/null || true
    pkill -f "release_objects_service" 2>/dev/null || true
    pkill -f "ur5_qt_panel" 2>/dev/null || true
    pkill -f "panel_v2.py" 2>/dev/null || true
    pkill -f "ros2 run ur5_qt_panel panel_v2" 2>/dev/null || true
    pkill -f "move_group" 2>/dev/null || true
    pkill -f "gz-transport-topic" 2>/dev/null || true
    pkill -f "gz_pose_bridge" 2>/dev/null || true
    pkill -f "grasp_audit_trace_capture" 2>/dev/null || true
    pkill -f "run_directo_button_offscreen" 2>/dev/null || true
    pkill -f "capture_camera_frames" 2>/dev/null || true
    sleep 4
    # Limpiar shared memory residual de FastDDS/CycloneDDS (evita 'Failed init_port' errors)
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    echo "[ORCH] Limpieza completada"
}

cleanup_residual_processes

# --- Fijar GZ_PARTITION para que stack y runner offscreen usen el mismo bus gz-transport ---
export GZ_PARTITION="${GZ_PARTITION:-ur5pro_validation}"
export PANEL_CAMERA_REQUIRED=0
# Mantener cámaras en mundo headless para obtener evidencia visual real.
# Sin PANEL_KEEP_CAMERAS=1, el launch elimina todos los modelos de cámara del SDF.
export PANEL_KEEP_CAMERAS=1
CAMERA_LOG="$OUT_DIR/camera_capture.log"
CAMERA_FRAMES_DIR="$OUT_DIR/camera_frames"
VISUAL_SMOKE_DIR="$OUT_DIR/visual_smoke"
VISUAL_SMOKE_LOG="$OUT_DIR/visual_smoke.log"
VISUAL_MANIFEST="$VISUAL_SMOKE_DIR/visual_capture_manifest.json"
mkdir -p "$CAMERA_FRAMES_DIR"
mkdir -p "$VISUAL_SMOKE_DIR"

STACK_PID=""
STACK_LOG=""
SYSTEM_DIAG_JSON="$OUT_DIR/system_diag_startup.json"
SYSTEM_DIAG_LOG="$OUT_DIR/system_diag_capture.log"

start_stack_attempt() {
    local attempt="$1"
    local stack_log="$OUT_DIR/stack_attempt${attempt}.log"
    local system_diag_json="$OUT_DIR/system_diag_startup_attempt${attempt}.json"
    local system_diag_log="$OUT_DIR/system_diag_capture_attempt${attempt}.log"
    rm -f "$stack_log" "$system_diag_json" "$system_diag_log"
    echo "[ORCH] Lanzando stack completo (Gazebo + MoveIt2 + controllers) intento=${attempt}/${STACK_STARTUP_MAX_ATTEMPTS} GZ_PARTITION=$GZ_PARTITION"
    ros2 launch ur5_bringup ur5_stack.launch.py \
        headless:=true \
        launch_panel:=false \
        launch_moveit:=true \
        moveit_mode:=move_group \
        camera_required:=false \
        >"$stack_log" 2>&1 &
    STACK_PID=$!
    STACK_LOG="$stack_log"
    echo "[ORCH] Stack PID=$STACK_PID log=$STACK_LOG"

    MOVEIT_WAIT_SEC=300
    MOVEIT_WAIT_START=$(date +%s)
    echo "[ORCH] Esperando 'You can start planning now!' en stack.log (timeout=${MOVEIT_WAIT_SEC}s)..."
    until grep -q "You can start planning now" "$STACK_LOG" 2>/dev/null; do
        NOW=$(date +%s)
        ELAPSED=$(( NOW - MOVEIT_WAIT_START ))
        if (( ELAPSED >= MOVEIT_WAIT_SEC )); then
            echo "[ORCH][WARN] Timeout: move_group no publicó readiness en ${MOVEIT_WAIT_SEC}s (intento ${attempt})"
            return 1
        fi
        sleep 3
    done
    echo "[ORCH] move_group listo ($(( $(date +%s) - MOVEIT_WAIT_START ))s desde stack launch)"

    GAZEBO_WAIT_SEC=120
    GAZEBO_WAIT_START=$(date +%s)
    echo "[ORCH] Esperando datos en /world/${WORLD}/pose/info (timeout=${GAZEBO_WAIT_SEC}s)..."
    until timeout 8s ros2 topic echo --once "/world/${WORLD}/pose/info" >/dev/null 2>&1; do
        NOW=$(date +%s)
        ELAPSED=$(( NOW - GAZEBO_WAIT_START ))
        if (( ELAPSED >= GAZEBO_WAIT_SEC )); then
            echo "[ORCH][WARN] Gazebo pose/info sin datos tras ${GAZEBO_WAIT_SEC}s — continuando de todos modos"
            break
        fi
        sleep 5
    done
    echo "[ORCH] Gazebo pose/info activo ($(( $(date +%s) - GAZEBO_WAIT_START ))s)"
    # Extra grace para que system_state_manager publique GAZEBO_READY
    sleep 15

    echo "[ORCH] Capturando self-check geométrico de arranque..."
    if ! python3 "$SCRIPT_DIR/capture_system_diag.py" \
        --timeout "$SYSTEM_DIAG_TIMEOUT_SEC" \
        --output "$system_diag_json" \
        --require-geometry-ok \
        --require-state READY \
        >"$system_diag_log" 2>&1; then
        echo "[ORCH][WARN] /system_diag no llegó a READY con geometry_ok=true (intento ${attempt})"
        return 1
    fi
    cp "$system_diag_json" "$SYSTEM_DIAG_JSON"
    cp "$system_diag_log" "$SYSTEM_DIAG_LOG"
    ln -sfn "$(basename "$STACK_LOG")" "$OUT_DIR/stack.log"
    echo "[ORCH] Self-check geométrico OK snapshot=$SYSTEM_DIAG_JSON"
    return 0
}

startup_ok=0
for startup_attempt in $(seq 1 "$STACK_STARTUP_MAX_ATTEMPTS"); do
    if start_stack_attempt "$startup_attempt"; then
        startup_ok=1
        break
    fi
    if [[ -n "${STACK_PID:-}" ]]; then
        kill "$STACK_PID" 2>/dev/null || true
        wait "$STACK_PID" 2>/dev/null || true
    fi
    cleanup_residual_processes
done

if [[ "$startup_ok" != "1" ]]; then
    echo "[ORCH][ERROR] Stack no alcanzó READY geométrico tras ${STACK_STARTUP_MAX_ATTEMPTS} intentos"
    exit 1
fi

# --- Lanzar el runner del panel offscreen ---
env \
    GZ_PARTITION="$GZ_PARTITION" \
    PANEL_DIRECT_DEBUG_ROOT="/home/laboratorio/TFM/historico" \
    PANEL_MANAGED=1 \
    QT_QPA_PLATFORM=offscreen \
    PANEL_FORCE_OFFSCREEN=1 \
    PANEL_START_STACK=0 \
    PANEL_MOVEIT_REQUIRED=1 \
    PANEL_MOVEIT_MODE=move_group \
    PANEL_AUTO_BRIDGE=1 \
    PANEL_SKIP_CLEANUP=1 \
    PANEL_CAMERA_REQUIRED=0 \
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

# --- Lanzar captura de cámara en paralelo ---
env \
    python3 "$SCRIPT_DIR/capture_camera_frames.py" \
        --output "$CAMERA_FRAMES_DIR" \
        --interval 2.0 \
        --timeout "$(( DIRECTO_TIMEOUT_SEC + 60 ))" \
    >"$CAMERA_LOG" 2>&1 &
CAMERA_PID=$!
echo "[ORCH] Camera capture PID=$CAMERA_PID frames_dir=$CAMERA_FRAMES_DIR"

# --- Lanzar smoke visual dirigido ---
env \
    python3 "$SCRIPT_DIR/directo_visual_capture.py" \
        --output-dir "$VISUAL_SMOKE_DIR" \
        --helper-log "$HELPER_LOG" \
        --pose-topic "$POSE_TOPIC" \
        --camera-topics /camera_debug_top/image /camera_overhead/image \
    >"$VISUAL_SMOKE_LOG" 2>&1 &
VISUAL_SMOKE_PID=$!
echo "[ORCH] Visual smoke PID=$VISUAL_SMOKE_PID output_dir=$VISUAL_SMOKE_DIR"

# --- Esperar al runner del panel ---
echo "[ORCH][SHUTDOWN] helper_wait_begin"
wait "$HELPER_PID" && HELPER_RC=0 || HELPER_RC=$?
echo "[ORCH][SHUTDOWN] helper_wait_end rc=$HELPER_RC"
echo "[ORCH] Runner terminó rc=$HELPER_RC"

# Dar unos segundos más al capturador para que cierre limpiamente
sleep 3
echo "[ORCH][SHUTDOWN] capture_stop_begin pid=$CAPTURE_PID"
kill "$CAPTURE_PID" 2>/dev/null || true
wait "$CAPTURE_PID" && CAPTURE_RC=0 || CAPTURE_RC=$?
echo "[ORCH][SHUTDOWN] capture_stop_end rc=$CAPTURE_RC"
echo "[ORCH] Capture terminó rc=$CAPTURE_RC"

# Parar captura de cámara
kill "$CAMERA_PID" 2>/dev/null || true
wait "$CAMERA_PID" 2>/dev/null || true
echo "[ORCH] Camera capture terminado. Frames: $(ls "$CAMERA_FRAMES_DIR" 2>/dev/null | wc -l)"

# Parar smoke visual
kill "$VISUAL_SMOKE_PID" 2>/dev/null || true
wait "$VISUAL_SMOKE_PID" 2>/dev/null || true
echo "[ORCH] Visual smoke terminado. Artefactos: $(ls "$VISUAL_SMOKE_DIR" 2>/dev/null | wc -l)"
VISUAL_SMOKE_COMPLETE=0
if [[ -f "$VISUAL_MANIFEST" ]] \
    && python3 - "$VISUAL_MANIFEST" <<'PY'
import json
import sys

required = {"pre_grasp", "grasp_confirmed", "lift_with_object", "basket_drop"}
path = sys.argv[1]
with open(path, "r", encoding="utf-8") as handle:
    payload = json.load(handle)
if not isinstance(payload, dict):
    raise SystemExit(1)
raise SystemExit(0 if required.issubset(payload.keys()) else 1)
PY
then
    VISUAL_SMOKE_COMPLETE=1
fi
echo "[ORCH] Smoke visual completo=$VISUAL_SMOKE_COMPLETE launcher=$CANONICAL_LAUNCHER"

# --- Apagar stack ---
echo "[ORCH][SHUTDOWN] stack_stop_begin pid=$STACK_PID"
kill "$STACK_PID" 2>/dev/null || true
wait "$STACK_PID" && STACK_RC=0 || STACK_RC=$?
echo "[ORCH][SHUTDOWN] stack_stop_end rc=$STACK_RC"

# --- Ejecutar el benchmark ---
python3 "$SCRIPT_DIR/grasp_audit_benchmark.py" \
    --run "DIRECTO:directo:$TRACE_FILE" \
    --json-out "$BENCHMARK_JSON" \
    >"$BENCHMARK_STDOUT_JSON" 2>&1 && BENCHMARK_RC=0 || BENCHMARK_RC=$?
echo "[ORCH] Benchmark rc=$BENCHMARK_RC"

# --- Resumen ---
{
    echo "canonical_launcher=$CANONICAL_LAUNCHER"
    echo "runtime_profile=$RUNTIME_PROFILE"
    echo "runtime_profile_name=${PANEL_RUNTIME_VALIDATED_PROFILE:-unknown}"
    echo "helper_final_rc=$HELPER_RC"
    echo "capture_rc=$CAPTURE_RC"
    echo "benchmark_rc=$BENCHMARK_RC"
    echo "stack_rc=${STACK_RC:-n/a}"
    echo "visual_smoke_dir=$VISUAL_SMOKE_DIR"
    echo "visual_smoke_complete=$VISUAL_SMOKE_COMPLETE"
    echo "system_diag_json=$SYSTEM_DIAG_JSON"
} > "$SUMMARY"

echo "[ORCH] Resumen guardado en $SUMMARY"
echo "[ORCH] Logs: $OUT_DIR"
FINAL_RC="$HELPER_RC"
if [[ "$VISUAL_SMOKE_COMPLETE" -ne 1 ]]; then
    FINAL_RC=1
fi
if [[ "$BENCHMARK_RC" -ne 0 ]]; then
    FINAL_RC=1
fi
echo "[ORCH][SHUTDOWN] final_rc=$FINAL_RC"
echo "canonical_launcher=$CANONICAL_LAUNCHER helper_final_rc=$HELPER_RC capture_rc=$CAPTURE_RC benchmark_rc=$BENCHMARK_RC stack_rc=${STACK_RC:-n/a} visual_smoke_complete=$VISUAL_SMOKE_COMPLETE"
exit "$FINAL_RC"
