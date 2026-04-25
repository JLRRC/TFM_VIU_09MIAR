#!/usr/bin/env bash
# =============================================================================
# log_report.sh — Genera informe de fases desde los logs de sesión.
# Uso:  ./log_report.sh [LOG_DIR]
#       LOG_DIR por defecto: visual_autopick_ros_ws/log/runtime/latest
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_LOG="$SCRIPT_DIR/visual_autopick_ros_ws/log/runtime/latest"
LOG_DIR="${1:-$WS_LOG}"

if [ ! -d "$LOG_DIR" ]; then
    echo "ERROR: directorio de logs no encontrado: $LOG_DIR"
    echo "Uso: $0 [LOG_DIR]"
    exit 1
fi

REPORT="$LOG_DIR/report_$(date +%H-%M-%S).txt"
STACK="$LOG_DIR/stack.log"
PANEL="$LOG_DIR/panel.log"
TRACE_FULL="$LOG_DIR/trace_full.log"
TRACE_PHASES="$LOG_DIR/trace_phases.log"

{
echo "========================================================================"
echo "  INFORME DE TRAZAS — TFM Agarre Inteligente"
echo "  Generado: $(date)"
echo "  Log dir : $LOG_DIR"
echo "========================================================================"

# ============================================================
# 1. RESUMEN DE SESION
# ============================================================
echo ""
echo "── 1. RESUMEN DE SESION ─────────────────────────────────────────────────"
for f in "$STACK" "$PANEL" "$TRACE_FULL" "$TRACE_PHASES"; do
    if [ -f "$f" ]; then
        lines=$(wc -l < "$f" 2>/dev/null || echo 0)
        size=$(du -sh "$f" 2>/dev/null | cut -f1)
        echo "  $(basename $f): $lines lineas ($size)"
    else
        echo "  $(basename $f): NO ENCONTRADO"
    fi
done

# ============================================================
# 2. MODO DIRECTO — fases detectadas
# ============================================================
echo ""
echo "── 2. MODO DIRECTO ──────────────────────────────────────────────────────"
if [ -f "$STACK" ]; then
    grep -E "\[PICK\]\[DIRECT\]\[(ROUTE|PANEL_TRACE|ALCANCE_MONITOR|DIVERGENCE|IK_SEED|CYCLE_REF|GRIPPER|GRASP_DOWN|ABORT|RECOVERY|SELECT)\]" \
        "$STACK" 2>/dev/null | head -200 || echo "  (sin trazas DIRECTO en stack.log)"
else
    echo "  stack.log no disponible"
fi

# ============================================================
# 3. MODO MOVEIT — fases detectadas
# ============================================================
echo ""
echo "── 3. MODO MOVEIT ───────────────────────────────────────────────────────"
if [ -f "$STACK" ]; then
    grep -E "\[PICK\]\[MOVEIT\]\[|MOVE_REQUEST|MOVE_RESULT|\[BRIDGE\]\[(RECV|DISPATCH_START|EXEC_START|RESULT_PUBLISHED)\]" \
        "$STACK" 2>/dev/null | head -200 || echo "  (sin trazas MOVEIT en stack.log)"
fi
if [ -f "$PANEL" ]; then
    grep -E "VISUAL_AUTOPICK.*MOVE_(REQUEST|RESULT)|MODE=MOVEIT|PHASE_(START|END)" \
        "$PANEL" 2>/dev/null | head -100 || true
fi

# ============================================================
# 4. MODO AGARRE — fases detectadas
# ============================================================
echo ""
echo "── 4. MODO AGARRE (TFM) ─────────────────────────────────────────────────"
if [ -f "$STACK" ]; then
    grep -E "\[PICK\]\[MOVEIT\]\[STEP\]|\[PICK\]\[MOVEIT\]\[INIT\]|\[PICK\]\[MOVEIT\]\[SELECT\]|\[PICK\]\[MOVEIT\]\[TCP_BEFORE\]|\[PICK\]\[MOVEIT\]\[REQUEST\]|\[PICK\]\[MOVEIT\]\[EXEC_RESULT\]" \
        "$STACK" 2>/dev/null | head -200 || echo "  (sin trazas AGARRE)"
fi

# ============================================================
# 5. GRIPPER — todas las acciones
# ============================================================
echo ""
echo "── 5. GRIPPER ───────────────────────────────────────────────────────────"
if [ -f "$STACK" ]; then
    grep -E "\[GRIPPER\]|\[BTN\].*gripper|gripper.*close|gripper.*open|GRIPPER_CMD" \
        "$STACK" 2>/dev/null | head -100 || true
fi
if [ -f "$TRACE_PHASES" ]; then
    grep -E "GRIPPER_CMD" "$TRACE_PHASES" 2>/dev/null | head -50 || true
fi

# ============================================================
# 6. TF — divergencias y problemas
# ============================================================
echo ""
echo "── 6. TF / DIVERGENCIAS ─────────────────────────────────────────────────"
if [ -f "$STACK" ]; then
    grep -E "\[DIVERGENCE\]|\[TF\].*STALE|\[TF\].*FAIL|\[TF_FRESHNESS\]|\[GEOM_AUDIT\]|TF zero stamp|TF ok|TF stale" \
        "$STACK" 2>/dev/null | head -100 || true
fi
if [ -f "$TRACE_FULL" ]; then
    grep -E "\[DH_VS_TF\].*DIVERGE|\[TF\].*age=[1-9]" \
        "$TRACE_FULL" 2>/dev/null | head -50 || true
fi

# ============================================================
# 7. DH FK vs TF — divergencias
# ============================================================
echo ""
echo "── 7. DH FK vs TF (divergencias > 5mm) ─────────────────────────────────"
if [ -f "$TRACE_FULL" ]; then
    grep -E "\[DH_VS_TF\]" "$TRACE_FULL" 2>/dev/null | \
        awk -F'dist_m=' '{if(NF>1 && $2+0 > 0.005) print}' | head -100 || true
fi

# ============================================================
# 8. SDF Gazebo — objeto pick_demo
# ============================================================
echo ""
echo "── 8. SDF / GAZEBO — pick_demo ──────────────────────────────────────────"
if [ -f "$STACK" ]; then
    grep -E "pick_demo.*pos=|POSE_INFO.*pick_demo|\[SDF\].*pick_demo" \
        "$STACK" 2>/dev/null | tail -20 || true
fi
if [ -f "$TRACE_FULL" ]; then
    grep -E "\[SDF_SNAPSHOT\].*pick_demo|\[SDF\].*pick_demo" \
        "$TRACE_FULL" 2>/dev/null | tail -20 || true
fi

# ============================================================
# 9. ERRORES Y WARNINGS
# ============================================================
echo ""
echo "── 9. ERRORES Y WARNINGS ────────────────────────────────────────────────"
for f in "$STACK" "$PANEL" "$TRACE_FULL"; do
    if [ -f "$f" ]; then
        echo "  -- $(basename $f) --"
        grep -iE "\[ERROR\]|\[WARN\]|FAIL|ABORT|TIMEOUT|STALE|DIVERGE" \
            "$f" 2>/dev/null | grep -v "^#" | head -50 || true
    fi
done

# ============================================================
# 10. VISUAL AUTOPICK — fases del nuevo nodo
# ============================================================
echo ""
echo "── 10. VISUAL AUTOPICK — fases ──────────────────────────────────────────"
if [ -f "$PANEL" ]; then
    grep -E "VISUAL_AUTOPICK|PHASE|MOVE_REQUEST|MOVE_RESULT|GRIPPER|DONE|FAIL|SNAPSHOT" \
        "$PANEL" 2>/dev/null | head -200 || echo "  (panel.log vacio o sin trazas)"
fi

# ============================================================
# 11. SNAPSHOTS DE ESTADO COMPLETO
# ============================================================
echo ""
echo "── 11. SNAPSHOTS COMPLETOS (ultimos 3) ──────────────────────────────────"
if [ -f "$TRACE_FULL" ]; then
    grep -n "^.*\[SNAPSHOT:" "$TRACE_FULL" 2>/dev/null | tail -3 | while IFS=: read -r linenum rest; do
        echo "--- Snapshot linea $linenum ---"
        sed -n "${linenum},$((linenum+30))p" "$TRACE_FULL" 2>/dev/null || true
        echo ""
    done
fi

echo ""
echo "========================================================================"
echo "  FIN DEL INFORME"
echo "  Archivos completos en: $LOG_DIR"
echo "    trace_full.log   — todo"
echo "    trace_phases.log — solo fases"
echo "    trace_joints.csv — joints en CSV"
echo "    trace_sdf.csv    — SDF/Gazebo en CSV"
echo "    trace_tf.csv     — TF en CSV"
echo "========================================================================"

} 2>&1 | tee "$REPORT"

echo ""
echo "Informe guardado en: $REPORT"
