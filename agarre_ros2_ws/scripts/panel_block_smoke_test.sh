#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/panel_block_smoke_test.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -u

AUDIT_ROOT="/home/laboratorio/TFM/reports/panel_audit"
LOG_DIR="$AUDIT_ROOT/logs"
ART_DIR="$AUDIT_ROOT/artifacts"
FIG_DIR="$AUDIT_ROOT/figures"

mkdir -p "$LOG_DIR" "$ART_DIR" "$FIG_DIR"

LOG_FILE="$LOG_DIR/panel_block_smoke_test.log"
{
  echo "[PANEL_AUDIT] START $(date -Iseconds)"
  echo "[PANEL_AUDIT] cwd=$(pwd)"
  uname -a
  echo ""
  echo "[PANEL_AUDIT] python3=$(command -v python3 || true)"
  python3 --version || true
  echo ""
  if command -v ros2 >/dev/null 2>&1; then
    echo "[PANEL_AUDIT] ros2 found"
    ros2 --help | head -n 2 || true
    if command -v timeout >/dev/null 2>&1; then
      timeout 5 ros2 node list || true
      timeout 5 ros2 topic list || true
    else
      ros2 node list || true
      ros2 topic list || true
    fi
  else
    echo "[PANEL_AUDIT] ros2 not found"
  fi
  echo ""
  echo "[PANEL_AUDIT] checkpoint index"
  python3 - <<'PY'
import json
from pathlib import Path

root = Path('/home/laboratorio/TFM')
audit = root / 'reports' / 'panel_audit' / 'artifacts' / 'checkpoints_index.json'
roots = [
    root / 'agarre_inteligente' / 'experiments',
    root / 'agarre_inteligente' / 'experiments_smoke',
    root / 'agarre_inteligente' / 'experiments' / '_artefactos',
]
patterns = ('**/checkpoints/best.pth', '**/checkpoints/*.pth', '**/*.ckpt')
entries = []
seen = set()
for r in roots:
    if not r.exists():
        continue
    for pat in patterns:
        for p in r.rglob(pat):
            if p.is_file() and p not in seen:
                seen.add(p)
                try:
                    stat = p.stat()
                    size = stat.st_size
                    mtime = stat.st_mtime
                except OSError:
                    size = None
                    mtime = None
                entries.append({
                    'path': str(p),
                    'rel': str(p.relative_to(root)),
                    'size_bytes': size,
                    'mtime': mtime,
                })
entries.sort(key=lambda x: x['path'])
out = {'root': str(root), 'count': len(entries), 'entries': entries}
audit.parent.mkdir(parents=True, exist_ok=True)
audit.write_text(json.dumps(out, indent=2), encoding='utf-8')
print(f"checkpoints_index.json entries={len(entries)}")
PY
  echo ""
  echo "[PANEL_AUDIT] tfm_smoketest"
  PYTHONPATH=/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel:/home/laboratorio/TFM/agarre_ros2_ws/src/tfm_grasping \
    python3 /home/laboratorio/TFM/agarre_ros2_ws/scripts/tfm_smoketest.py || true
  echo ""
  echo "[PANEL_AUDIT] NOTE: UI-driven actions (apply/infer/visualize/execute/reset) require running panel_v2."
  echo "[PANEL_AUDIT] END $(date -Iseconds)"
} | tee -a "$LOG_FILE"
