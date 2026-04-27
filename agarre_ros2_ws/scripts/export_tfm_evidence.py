#!/usr/bin/env python3
"""Exporta la evidencia mas reciente del bloque TFM a report/evidence/ros2."""

from __future__ import annotations

import argparse
import json
import shutil
from datetime import datetime
from pathlib import Path
from typing import Iterable


def _safe_copy(src: Path, dst: Path) -> bool:
    if not src.exists() or not src.is_file():
        return False
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return True


def _iter_existing(paths: Iterable[Path]) -> list[Path]:
    return [path for path in paths if path.exists() and path.is_file()]


def _read_session(session_path: Path) -> dict:
    if not session_path.exists():
        return {}
    try:
        return json.loads(session_path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _build_stamp(session: dict) -> str:
    raw = str(session.get("timestamp") or "").strip()
    if raw:
        try:
            dt = datetime.fromisoformat(raw)
            return dt.strftime("%Y%m%d_%H%M%S")
        except Exception:
            pass
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _write_manifest(out_dir: Path, session: dict, copied: list[str]) -> None:
    manifest = out_dir / "README.md"
    lines = [
        "# Export de evidencia TFM",
        "",
        f"Fecha de exportacion: {datetime.now().isoformat(timespec='seconds')}",
        "",
        "## Resumen de sesion",
        "",
        f"- selection_policy: `{session.get('selection_policy', '--')}`",
        f"- postprocess_policy: `{session.get('postprocess_policy', '--')}`",
        f"- experiment: `{session.get('experiment', '--')}`",
        f"- experiment_base: `{session.get('experiment_base', '--')}`",
        f"- seed: `{session.get('seed', '--')}`",
        f"- model: `{session.get('model', '--')}`",
        f"- modality: `{session.get('modality', '--')}`",
        f"- val_success_pct: `{session.get('val_success_pct', '--')}`",
        f"- val_iou: `{session.get('val_iou', '--')}`",
        f"- weights_path: `{session.get('weights_path', '--')}`",
        "",
        "## Ficheros copiados",
        "",
    ]
    if copied:
        lines.extend(f"- `{name}`" for name in copied)
    else:
        lines.append("- Ninguno")
    manifest.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--audit-root", default="auditoria/panel_audit")
    ap.add_argument("--out-root", default="report/evidence/ros2/tfm_session_exports")
    args = ap.parse_args()

    audit_root = Path(args.audit_root)
    out_root = Path(args.out_root)

    session_path = audit_root / "artifacts" / "tfm_session_last.json"
    session = _read_session(session_path)
    stamp = _build_stamp(session)
    out_dir = out_root / stamp
    out_dir.mkdir(parents=True, exist_ok=True)

    candidates = _iter_existing(
        [
            session_path,
            audit_root / "artifacts" / "checkpoints_index.json",
            audit_root / "artifacts" / "grasp_last.json",
            audit_root / "artifacts" / "grasp_rect_last.json",
            audit_root / "artifacts" / "tfm_moveit_canonical_last.json",
            audit_root / "logs" / "apply_experiment.log",
            audit_root / "logs" / "infer.log",
            audit_root / "logs" / "visualize.log",
            audit_root / "logs" / "execute.log",
            audit_root / "figures" / "overlay_last.png",
        ]
    )

    copied: list[str] = []
    for src in candidates:
        rel = src.relative_to(audit_root)
        if _safe_copy(src, out_dir / rel):
            copied.append(str(rel))

    _write_manifest(out_dir, session, copied)
    print(f"[OK] Evidencia exportada a {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
